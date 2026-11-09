// SPDX-License-Identifier: MIT
/* Capture YUV420M and encode HEVC through DMA-BUF, without copying pixels.
 * Camera buffers stay out of the capture queue until the encoder returns them.
 */
#include "video/internal.h"
#include "video/video.h"
#include "camera/image.h"
#include "backends/ar9311/camera.h"
#include "app/state.h"
#include "app/settings.h"
#include "common/clock.h"
#include "radio/radio.h"
#include "kestrel_air.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define BUFFERS 5   // also caps left_frames below the drop thread's 6 (video/video.c)
#define PLANES 3
#define CAPTURE V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
#define OUTPUT V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
#define MMAP V4L2_MEMORY_MMAP
#define DMABUF V4L2_MEMORY_DMABUF

struct camera_plane { int fd; unsigned int size; };
struct coded_buffer { void *data; unsigned int size; };
static struct camera_plane camera[BUFFERS][PLANES];
static struct coded_buffer coded[BUFFERS];
static int capture_fd = -1, encoder_fd = -1;
static int capture_running, output_running, encoder_running;
static int held[BUFFERS];
static int enabled, warmup;
static atomic_int refresh_enabled;
static unsigned long submitted, received, returned;
static uint64_t progress;
int tune[T_N];

static int xioctl(int fd, unsigned long request, void *arg)
{
	int ret;
	do { ret = ioctl(fd, request, arg); } while (ret < 0 && errno == EINTR);
	return ret;
}

static void cleanup(void)
{
	image_pause();
	enum v4l2_buf_type type = CAPTURE;
	if (capture_running) xioctl(capture_fd, VIDIOC_STREAMOFF, &type);
	type = OUTPUT;
	if (output_running) xioctl(encoder_fd, VIDIOC_STREAMOFF, &type);
	type = CAPTURE;
	if (encoder_running) xioctl(encoder_fd, VIDIOC_STREAMOFF, &type);
	for (unsigned int i = 0; i < BUFFERS; i++) {
		if (coded[i].data && coded[i].data != MAP_FAILED)
			munmap(coded[i].data, coded[i].size);
		for (unsigned int j = 0; j < PLANES; j++)
			if (camera[i][j].fd >= 0) { close(camera[i][j].fd); camera[i][j].fd = -1; }
	}
	if (encoder_fd >= 0) close(encoder_fd);
	if (capture_fd >= 0) close(capture_fd);
	capture_fd = encoder_fd = -1;
    capture_running = output_running = encoder_running = 0;
    memset(coded, 0, sizeof(coded));
    memset(held, 0, sizeof(held));
    venc_running = shared.venc_running = 0;
    atomic_store(&refresh_enabled, 0);
    left_frames = 0;
}

static void fail(const char *what)
{
	perror(what);
	exit(EXIT_FAILURE);
}

// A capture or encoder fault while streaming tears the channel down here and
// the read loop makes a new one (video/video.c), so a passing fault costs a
// moment of video, not the process (the supervisor gives up after 5 short
// runs). More than RECOVER_MAX in a minute is no passing fault: exit. Called
// with venc_mtx held, as every user of the fds holds it.
#define RECOVER_MAX 5
static int recover(const char *what)
{
	static uint64_t since;
	static int count;
	perror(what);
	uint64_t now = mono_us();
	if (!since || now - since > 60000000) { since = now; count = 0; }
	if (++count > RECOVER_MAX) {
		fprintf(stderr, "video: %d camera/encoder faults in a minute\n", count);
		exit(EXIT_FAILURE);
	}
	printf("video: %s - a new encoder channel (%d this minute)\n", what, count);
	cleanup();
	return -1;
}

static void checked(int fd, unsigned long request, void *arg, const char *what)
{
	if (xioctl(fd, request, arg)) fail(what);
}

static struct v4l2_buffer buffer(unsigned int type, unsigned int memory,
		unsigned int index, unsigned int planes, struct v4l2_plane *p)
{
	return (struct v4l2_buffer) {
		.type = type, .memory = memory, .index = index,
		.length = planes, .m.planes = p,
	};
}

static int queue_camera(unsigned int index)
{
	struct v4l2_plane p[PLANES] = {0};
	struct v4l2_buffer b = buffer(CAPTURE, MMAP, index, PLANES, p);
	return xioctl(capture_fd, VIDIOC_QBUF, &b);
}

static void request_buffers(int fd, unsigned int type, unsigned int memory)
{
	struct v4l2_requestbuffers r = {
		.type = type, .memory = memory, .count = BUFFERS,
	};
	checked(fd, VIDIOC_REQBUFS, &r, "REQBUFS");
	if (r.count < BUFFERS) { errno = ENOMEM; fail("buffer count"); }
}

static int set_control(unsigned int id, int value)
{
	struct v4l2_control c = { .id = id, .value = value };
	return xioctl(encoder_fd, VIDIOC_S_CTRL, &c);
}

static void control(unsigned int id, int value)
{
	if (set_control(id, value)) fail("encoder control");
}

static int intra_refresh(void)
{
	struct v4l2_queryctrl q = { .id = V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD };
	if (xioctl(encoder_fd, VIDIOC_QUERYCTRL, &q)) {
		if (errno == EINVAL) return 0; /* Older fpvOS: retain periodic IDRs. */
		fail("encoder refresh query");
	}
	if (q.flags & V4L2_CTRL_FLAG_DISABLED) return 0;
	/* One CTU row per picture at 1080p: a complete sweep in 17 pictures.
	 * Keep explicit keyframe requests for joining and error recovery.
	 */
	control(V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD, 17);
	return 1;
}


int venc_create(int kbps) {
    static int initialized;
    char camera_path[128], encoder_path[128];
    if (!initialized) {
        for (unsigned i=0;i<BUFFERS;i++) for(unsigned j=0;j<PLANES;j++) camera[i][j].fd=-1;
        atexit(cleanup);
        initialized=1;
    }
    if (ar9311_node("ar-cvisp",camera_path,sizeof(camera_path)) ||
        ar9311_node("C&M Wave5 VPU encoder",encoder_path,sizeof(encoder_path))) return -1;
    capture_fd=open(camera_path,O_RDWR|O_NONBLOCK|O_CLOEXEC);
    encoder_fd=open(encoder_path,O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if(capture_fd<0 || encoder_fd<0) fail("open camera/encoder");
    kbps=kbps<128?128:kbps>20000?20000:kbps;
	struct v4l2_format cf = { .type = CAPTURE };
	checked(capture_fd, VIDIOC_G_FMT, &cf, "camera G_FMT");
	struct v4l2_pix_format_mplane *f = &cf.fmt.pix_mp;
	if (f->pixelformat != V4L2_PIX_FMT_YUV420M || f->num_planes != PLANES) {
		fprintf(stderr, "camera must provide three-plane YUV420M\n");
		errno = EINVAL; fail("unsupported camera/encoder format");
	}
	request_buffers(capture_fd, CAPTURE, MMAP);
	for (unsigned int i = 0; i < BUFFERS; i++) {
		struct v4l2_plane p[PLANES] = {0};
		struct v4l2_buffer b = buffer(CAPTURE, MMAP, i, PLANES, p);
		checked(capture_fd, VIDIOC_QUERYBUF, &b, "camera QUERYBUF");
		for (unsigned int j = 0; j < PLANES; j++) {
			struct v4l2_exportbuffer x = {
				.type = CAPTURE, .index = i, .plane = j,
				.flags = O_CLOEXEC | O_RDWR,
			};
			checked(capture_fd, VIDIOC_EXPBUF, &x, "camera EXPBUF");
			camera[i][j].fd = x.fd;
			camera[i][j].size = p[j].length;
		}
		if (queue_camera(i)) fail("camera QBUF");
	}

	struct v4l2_format ef = cf;
	ef.type = OUTPUT;
	for (unsigned int j = 0; j < PLANES; j++)
		ef.fmt.pix_mp.plane_fmt[j].sizeimage = camera[0][j].size;
	checked(encoder_fd, VIDIOC_S_FMT, &ef, "encoder source S_FMT");
	for (unsigned int j = 0; j < PLANES; j++) {
		if (ef.fmt.pix_mp.plane_fmt[j].bytesperline != f->plane_fmt[j].bytesperline ||
		    ef.fmt.pix_mp.plane_fmt[j].sizeimage > camera[0][j].size) {
			fprintf(stderr, "encoder substituted stride or requires a larger source buffer\n");
			errno = EINVAL; fail("unsupported camera/encoder format");
		}
	}
	ef = (struct v4l2_format) { .type = CAPTURE };
	ef.fmt.pix_mp = (struct v4l2_pix_format_mplane) {
		.width = f->width, .height = f->height, .pixelformat = V4L2_PIX_FMT_HEVC,
		.field = V4L2_FIELD_NONE, .num_planes = 1,
	};
	checked(encoder_fd, VIDIOC_S_FMT, &ef, "encoder HEVC S_FMT");
	if (ef.fmt.pix_mp.width != f->width || ef.fmt.pix_mp.height != f->height ||
	    ef.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_HEVC || ef.fmt.pix_mp.num_planes != 1) {
		fprintf(stderr, "encoder substituted coded format\n");
		errno = EINVAL; fail("unsupported camera/encoder format");
	}
	struct v4l2_streamparm sp = { .type = OUTPUT };
	sp.parm.output.timeperframe = (struct v4l2_fract) {1, 60};
	checked(encoder_fd, VIDIOC_S_PARM, &sp, "encoder frame rate");
	control(V4L2_CID_MPEG_VIDEO_HEVC_LEVEL, V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1);
	control(V4L2_CID_MPEG_VIDEO_BITRATE, kbps * 1000);
	int refresh = intra_refresh();
	atomic_store(&refresh_enabled, refresh);
	control(V4L2_CID_MPEG_VIDEO_GOP_SIZE, refresh ? 0 : 60);
	control(V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 1);
	control(V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE, 1);
	/* Wave5 passes this control to firmware in milliseconds. The 1000 ms
	 * default allows large IDRs to stall the radio once per GOP. Keep its
	 * rate-control reservoir small without buffering frames at the receiver.
	 */
	control(V4L2_CID_MPEG_VIDEO_VBV_SIZE, 17);
	control(V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP, 35);
	control(V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR, 1);
	control(V4L2_CID_MPEG_VIDEO_AU_DELIMITER, 0);
	request_buffers(encoder_fd, OUTPUT, DMABUF);
	request_buffers(encoder_fd, CAPTURE, MMAP);
	for (unsigned int i = 0; i < BUFFERS; i++) {
		struct v4l2_plane p = {0};
		struct v4l2_buffer b = buffer(CAPTURE, MMAP, i, 1, &p);
		checked(encoder_fd, VIDIOC_QUERYBUF, &b, "coded QUERYBUF");
		coded[i].size = p.length;
		coded[i].data = mmap(NULL, p.length, PROT_READ, MAP_SHARED,
				encoder_fd, p.m.mem_offset);
		if (coded[i].data == MAP_FAILED) fail("coded mmap");
		checked(encoder_fd, VIDIOC_QBUF, &b, "coded QBUF");
	}

    if (f->width != (unsigned)width || f->height != (unsigned)height) {
        errno=EINVAL; fail("capture dimensions");
    }
    venc_kbps=kbps; venc_fps=60;
    submitted=received=returned=0;
    enabled=1; warmup=8; progress=mono_us();
    printf("video: AR9311 DMA-BUF encoder %dx%d@60 %dkbps, complete frames\n",width,height,kbps);
    printf("video: %s, Wave5 VBV 17ms\n", refresh ? "intra refresh 17 pictures, on-demand IDR" : "periodic IDR fallback (no intra refresh control)");
    return 0;
}
void venc_start(void) {
    enum v4l2_buf_type ct=CAPTURE,ot=OUTPUT;
    checked(capture_fd,VIDIOC_STREAMON,&ct,"camera STREAMON"); capture_running=1;
    checked(encoder_fd,VIDIOC_STREAMON,&ot,"encoder OUTPUT STREAMON"); output_running=1;
    checked(encoder_fd,VIDIOC_STREAMON,&ct,"encoder CAPTURE STREAMON"); encoder_running=1;
    venc_running=shared.venc_running=1;
}
void venc_stop_destroy(void) {
    pthread_mutex_lock(&venc_mtx);
    cleanup();
    pthread_mutex_unlock(&venc_mtx);
}
void venc_input_enable(int on) { enabled=on; }
static int get_frame(uint8_t *dst,uint32_t room,vframe *out) {
    struct v4l2_plane p[PLANES]={0};
    struct v4l2_buffer b=buffer(OUTPUT,DMABUF,0,PLANES,p);
    while(xioctl(encoder_fd,VIDIOC_DQBUF,&b)==0) {
        if(b.index>=BUFFERS || !held[b.index] || (b.flags&V4L2_BUF_FLAG_ERROR)) {
            errno=EIO; return recover("encoder source return");
        }
        held[b.index]=0;
        if(queue_camera(b.index)) return recover("camera QBUF");
        returned++; progress=mono_us();
        memset(p,0,sizeof(p)); b=buffer(OUTPUT,DMABUF,0,PLANES,p);
    }
    if(errno!=EAGAIN) return recover("source DQBUF");
    memset(p,0,sizeof(p)); b=buffer(CAPTURE,MMAP,0,1,p);
    if(xioctl(encoder_fd,VIDIOC_DQBUF,&b)==0) {
        if(b.index>=BUFFERS || p[0].bytesused>coded[b.index].size ||
           p[0].bytesused<=p[0].data_offset || (b.flags&V4L2_BUF_FLAG_ERROR)) {
            errno=EIO; return recover("encoded frame");
        }
        size_t n=p[0].bytesused-p[0].data_offset;
        if(n>room) { errno=EMSGSIZE; return recover("encoded frame exceeds packet capacity"); }
        memcpy(dst,(char *)coded[b.index].data+p[0].data_offset,n);
        out->len=n; out->type=(b.flags&V4L2_BUF_FLAG_KEYFRAME)?1:0;
        out->frame_end=1;
        out->pts=(uint64_t)b.timestamp.tv_sec*1000000+b.timestamp.tv_usec;
        if(xioctl(encoder_fd,VIDIOC_QBUF,&b)) return recover("coded requeue");
        received++; left_frames=submitted-received; progress=mono_us();
        return 1;
    }
    if(errno!=EAGAIN) return recover("coded DQBUF");
    memset(p,0,sizeof(p)); b=buffer(CAPTURE,MMAP,0,PLANES,p);
    if(xioctl(capture_fd,VIDIOC_DQBUF,&b)==0) {
        if(b.index>=BUFFERS || held[b.index] || (b.flags&V4L2_BUF_FLAG_ERROR)) {
            errno=EIO; return recover("camera buffer");
        }
        unsigned index=b.index;
        if(warmup || !enabled) {
            if(queue_camera(index)) return recover("camera QBUF");
            if(warmup && !--warmup && ar9311_controls_start()) {
                errno=EIO; return recover("camera automatic controls");
            }
            progress=mono_us();
            return 0;
        }
        struct timeval stamp=b.timestamp;
        memset(p,0,sizeof(p)); b=buffer(OUTPUT,DMABUF,index,PLANES,p);
        b.timestamp=stamp;
        for(unsigned j=0;j<PLANES;j++) {
            p[j].m.fd=camera[index][j].fd;
            p[j].length=p[j].bytesused=camera[index][j].size;
        }
        if(xioctl(encoder_fd,VIDIOC_QBUF,&b)) return recover("encoder DMA-BUF QBUF");
        held[index]=1; submitted++; left_frames=submitted-received;
    } else if(errno!=EAGAIN) return recover("camera DQBUF");
    if(!warmup && ar9311_controls_alive()) { errno=EIO; return recover("camera controller exited"); }
    if(mono_us()-progress>5000000) { errno=ETIMEDOUT; return recover("camera/encoder stalled"); }
    return 0;
}
int stream_get(uint8_t *dst,uint32_t room,vframe *out,int timeout_ms) {
    uint64_t until=mono_us()+(uint64_t)timeout_ms*1000;
    int r=0;
    pthread_mutex_lock(&venc_mtx);
    while(venc_running) {
        r=get_frame(dst,room,out);
        if(r || mono_us()>=until) break;
        struct pollfd pf[2]={{capture_fd,POLLIN,0},{encoder_fd,POLLIN,0}};
        if(poll(pf,2,2)<0 && errno!=EINTR) { r=recover("video poll"); break; }
    }
    pthread_mutex_unlock(&venc_mtx);
    return r;
}
void tune_poll(void) { }
void roi_update(int on) { (void)on; }
int video_low_delay_lines(void) { return 0; }
int lat_info(void) { return 0; } // Capture timing not calibrated to the radio clock yet
int video_tune_armed(void) { return 0; }
int video_tune_pwr(void) { return -1; }   // no tune file here: the chosen TX power
int video_tune_pathb(void) { return 1; }  // and both TX paths, as in flight
int video_tgt_fps(void) { return fps; }
int video_kbps(void) { return venc_kbps; }
uint8_t video_features(void) {
    return (radio_bw40_enabled() ? KA_FEAT_MAXBW : 0) |
           (atomic_load(&refresh_enabled) ? KA_FEAT_INTRA_REFRESH : 0);
}
void set_venc_bitrate(int f,int kbps,int reason) {
    (void)f; (void)reason;
    kbps=kbps<128?128:kbps>20000?20000:kbps;
    pthread_mutex_lock(&venc_mtx);
    if(venc_running && kbps!=venc_kbps) {
        if(set_control(V4L2_CID_MPEG_VIDEO_BITRATE,kbps*1000)) recover("encoder bitrate");
        else venc_kbps=kbps;
    }
    pthread_mutex_unlock(&venc_mtx);
}
void video_set_bitrate(int kbps) { set_venc_bitrate(0,kbps,0); }
void venc_down(int reason) { set_venc_bitrate(0,venc_kbps*8/10,reason); }
void video_scale_bitrate(int num,int den) {
    if(den>0 && (fc_bitmap&4)) set_venc_bitrate(0,venc_kbps*num/den,2);
}
void video_enable_idr(int on) {
    pthread_mutex_lock(&venc_mtx);
    if(on && venc_running && set_control(V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,0)) recover("encoder keyframe");
    pthread_mutex_unlock(&venc_mtx);
}
