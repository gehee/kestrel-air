// SPDX-License-Identifier: MIT
/* Capture YUV420M and encode HEVC through DMA-BUF, without copying pixels.
 * Camera buffers stay out of the capture queue until the encoder returns them.
 */
#include "video/internal.h"
#include "video/video.h"
#include "video/slices.h"
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
static unsigned c_stride[PLANES];   // the camera planes' bytes a line
static int capture_fd = -1;
static int capture_running;

// The encoders: one for the whole picture, or one a strip (KA_CMD_STRIPS),
// each a channel of its own on the WAVE5 that reads its rows straight out of
// the camera's buffer (each plane's data_offset, which fpvOS's Wave5 patch
// honours). A strip is encoded as soon as its last row has landed, while the
// camera still writes the rows under it, so most of a picture is on the radio
// before its bottom is encoded: the WAVE5 hands a picture's bitstream out
// only at its end (0.9 ms + 6.3 us a row on the bench), slices or not. The
// ground joins the strips again and shows a picture once all of them are in.
#define STRIPS_MAX 4
struct strip {
    int fd, output_running, encoder_running;
    unsigned y, h;                       // its rows of the picture
    struct coded_buffer coded[BUFFERS];
};
static struct strip strip[STRIPS_MAX] = {{.fd = -1}, {.fd = -1}, {.fd = -1}, {.fd = -1}};
static int nstrips = 1;                  // encoders now: 1 (whole pictures), or 2-4 strips
// Each strip codes strip_ov rows past each of its joins (KA_STRIP_OVERLAP: 16
// by default, 0-32 in eights), which the ground cuts off again (header byte
// KA_H_STRIP_OV): no join is at the edge a strip was coded to, where nothing
// is predicted from beyond and nothing deblocked across - the faint line two
// strips made there. Stock hides its seam the same way (2 x 16 rows).
static unsigned strip_ov;
static volatile int strips_wanted;       // asked for (KA_CMD_STRIPS), from the next channel
static unsigned held[BUFFERS];           // bit k: strip k's encoder has the camera buffer
static unsigned out_strip;               // the strip taken from its encoder next
static uint64_t out_pts;                 // ...of this picture
static int enabled, warmup;
static unsigned long submitted, received, returned;
static uint64_t progress;
static uint32_t capture_lag_us;   // row 0's capture -> the capture driver's stamp
static int slices;                // a whole picture's slices (KA_SLICE_COUNT)

// Encoding before the picture is all in memory (the capture driver at depth 1,
// which hands a buffer over as its frame starts to land): a mark in the luma
// row KA_EARLY_ROW (768) is overwritten when the camera has written that far,
// and only then does the buffer go to the encoder. The encoder reads rows about
// twice as fast as the camera writes them (8 ms against 15.6 for the picture),
// so from there it reaches the bottom after the last row lands: 5.5 ms sooner
// than depth 2. With an exposure toggled every 100 ms, 600 pictures came out
// whole from rows 704 and 768, half of them torn from 576 (the encoder caught
// up with the camera). A buffer goes back to the camera only once its last
// row has landed too, by the clock (the rows land at the line rate, the last
// 15.6 ms after the stamp): queued while still being written - as warm-up and
// a disabled input do - its late rows land on the mark set for its next
// frame, and from then on every mark is already overwritten at hand-over.
// The stock app does the same with marks, in two strips. 0: off (depth 2 and
// up, where a buffer comes whole, a frame later).
static const uint8_t early_mark[8] = {0x01, 0xfe, 0x02, 0xfd, 0x03, 0xfc, 0x04, 0xfb};
static uint8_t *camera_y[BUFFERS];
static unsigned requeue;          // buffers done with, their last row still to land
static uint64_t landed_at[BUFFERS];   // when each buffer's last row has landed (mono us)
static uint32_t land_us;          // the stamp -> its frame's last row landed
static unsigned early_n, early_past;  // pictures released at their row, and already past it
static uint64_t early_wait_us[STRIPS_MAX];
static size_t camera_y_size[BUFFERS];
static unsigned y_stride;
// The marked rows: a whole picture's KA_EARLY_ROW, or each strip's last row.
// None (nmarks 0) where a buffer comes whole (depth 2 and up).
static int early_row, nmarks, mark_row[STRIPS_MAX];
static int waiting = -1, waiting_next;   // the buffer whose rows are landing, its strip next
static struct timeval waiting_stamp;
static volatile uint8_t *early_at(unsigned index, int k) { return camera_y[index] + (size_t)mark_row[k] * y_stride; }
static int early_landed(unsigned index, int k) {
    volatile uint8_t *p = early_at(index, k);
    for (int i = 0; i < 8; i++) if (p[i] != early_mark[i]) return 1;
    return 0;
}

// A coded picture handed out a slice at a time - each VCL NAL unit, the
// parameter sets with the first. The encoder gives the whole picture at once,
// but sent in slices the top of it can be decoded and lit on the goggle while
// the rest is still in the air. It is copied out of the encoder's buffer
// first, in one go: that buffer is uncached, and looking for the slices in it
// a byte at a time took 6 ms a picture.
static struct {
    int on, key;
    unsigned strip;     // the strip it is (strips)
    uint8_t *data;
    size_t size, len, at;
    uint64_t pts;
} pic;
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
	for (int k = 0; k < STRIPS_MAX; k++) {
		struct strip *s = &strip[k];
		type = OUTPUT;
		if (s->output_running) xioctl(s->fd, VIDIOC_STREAMOFF, &type);
		type = CAPTURE;
		if (s->encoder_running) xioctl(s->fd, VIDIOC_STREAMOFF, &type);
		for (unsigned int i = 0; i < BUFFERS; i++)
			if (s->coded[i].data && s->coded[i].data != MAP_FAILED)
				munmap(s->coded[i].data, s->coded[i].size);
		if (s->fd >= 0) close(s->fd);
		memset(s, 0, sizeof(*s));
		s->fd = -1;
	}
	for (unsigned int i = 0; i < BUFFERS; i++) {
		if (camera_y[i]) { munmap(camera_y[i], camera_y_size[i]); camera_y[i] = NULL; }
		for (unsigned int j = 0; j < PLANES; j++)
			if (camera[i][j].fd >= 0) { close(camera[i][j].fd); camera[i][j].fd = -1; }
	}
	if (capture_fd >= 0) close(capture_fd);
	capture_fd = -1;
    capture_running = 0;
    memset(held, 0, sizeof(held));
    pic.on = 0;
    out_strip = 0;
    waiting = -1;
    requeue = 0;
    venc_running = shared.venc_running = 0;
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
	if (nmarks) {
		for (int k = 0; k < nmarks; k++) {
			volatile uint8_t *m = early_at(index, k);
			for (int i = 0; i < 8; i++) m[i] = early_mark[i];
		}
		__sync_synchronize();
	}
	struct v4l2_plane p[PLANES] = {0};
	struct v4l2_buffer b = buffer(CAPTURE, MMAP, index, PLANES, p);
	return xioctl(capture_fd, VIDIOC_QBUF, &b);
}

// Done with a camera buffer: back to the camera now, or once its frame has
// all landed (early mode).
static int give_back(unsigned index)
{
	if (nmarks && mono_us() < landed_at[index]) {
		requeue |= 1u << index;
		return 0;
	}
	return queue_camera(index);
}

static void request_buffers(int fd, unsigned int type, unsigned int memory)
{
	struct v4l2_requestbuffers r = {
		.type = type, .memory = memory, .count = BUFFERS,
	};
	checked(fd, VIDIOC_REQBUFS, &r, "REQBUFS");
	if (r.count < BUFFERS) { errno = ENOMEM; fail("buffer count"); }
}

static int set_control(int fd, unsigned int id, int value)
{
	struct v4l2_control c = { .id = id, .value = value };
	return xioctl(fd, VIDIOC_S_CTRL, &c);
}

static void control(int fd, unsigned int id, int value)
{
	if (set_control(fd, id, value)) fail("encoder control");
}

// Intra refresh (KA_IR above 0, the default): each picture codes a band of
// CTU rows intra, so keyframes come only when the ground asks for one
// (video_enable_idr) and no picture is much larger than the others. The WAVE5
// moves its band one CTU row a picture whatever the band's width: a sweep is
// the picture's CTU rows, 17 at 1080p (0.28 s), and KA_IR, KA_IR_MODE and
// KA_IR_QP pick nothing here. Constrained intra prediction keeps refreshed
// rows from predicting off rows that are not yet: a ground joining mid-stream
// sees an exact picture after one sweep (17-19 pictures on the bench), not
// after 67. A kernel without fpvOS's Wave5 intra refresh patch refuses the
// control; the stream then keeps periodic keyframes and says so.
//
// Strips refresh by CTU columns (fpvOS's Wave5 patch 0017): by rows each
// strip's band is 2 CTU rows whatever its height, 4 strips coding half of
// every picture intra; by columns the band is 2 columns of each strip, 2 of
// the whole picture together (6.7% at 1920 wide, against 11.8% for whole
// pictures by rows) whatever the number of strips. A sweep is the picture's
// 30 columns (0.5 s); a lost picture is cleaned up after one, as by rows. A
// kernel without the patch: strips by rows, as before.
#define V4L2_CID_WAVE5_INTRA_REFRESH_COLUMNS (V4L2_CID_CODEC_BASE | 0x1f00)
static int ir_on, ir_kernel = 1, ir_columns = 1;
static int ir_wanted(void) { return env_int("KA_IR", 2) > 0; }


// A strip's share of the bitrate: its share of the rows coded (the overlaps
// counted in each strip that codes them).
static int strip_kbps(int kbps, int k) {
	if (nstrips < 2) return kbps;
	unsigned coded = 0;
	for (int i = 0; i < nstrips; i++) coded += strip[i].h;
	int v = (int)((int64_t)kbps * strip[k].h / (coded ? coded : 1));
	return v < 64 ? 64 : v;
}

// One encoder: the whole picture (k 0 alone) or strip k, `kbps` its share.
static void encoder_create(int k, const char *path, const struct v4l2_format *cf, int kbps) {
	struct strip *s = &strip[k];
	const struct v4l2_pix_format_mplane *f = &cf->fmt.pix_mp;
	s->fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (s->fd < 0) fail("open encoder");
	struct v4l2_format ef = *cf;
	ef.type = OUTPUT;
	ef.fmt.pix_mp.height = s->h;
	for (unsigned int j = 0; j < PLANES; j++)
		ef.fmt.pix_mp.plane_fmt[j].sizeimage = nstrips == 1 ? camera[0][j].size :
			(j ? (s->h + 1) / 2 : s->h) * c_stride[j];
	checked(s->fd, VIDIOC_S_FMT, &ef, "encoder source S_FMT");
	for (unsigned int j = 0; j < PLANES; j++) {
		// What it reads from its rows on must be in the camera's buffer.
		const unsigned off = (j ? s->y / 2 : s->y) * c_stride[j];
		if (ef.fmt.pix_mp.plane_fmt[j].bytesperline != f->plane_fmt[j].bytesperline ||
		    off + ef.fmt.pix_mp.plane_fmt[j].sizeimage > camera[0][j].size) {
			fprintf(stderr, "encoder substituted stride or requires a larger source buffer\n");
			errno = EINVAL; fail("unsupported camera/encoder format");
		}
	}
	ef = (struct v4l2_format) { .type = CAPTURE };
	ef.fmt.pix_mp = (struct v4l2_pix_format_mplane) {
		.width = f->width, .height = s->h, .pixelformat = V4L2_PIX_FMT_HEVC,
		.field = V4L2_FIELD_NONE, .num_planes = 1,
	};
	checked(s->fd, VIDIOC_S_FMT, &ef, "encoder HEVC S_FMT");
	if (ef.fmt.pix_mp.width != f->width || ef.fmt.pix_mp.height != s->h ||
	    ef.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_HEVC || ef.fmt.pix_mp.num_planes != 1) {
		fprintf(stderr, "encoder substituted coded format\n");
		errno = EINVAL; fail("unsupported camera/encoder format");
	}
	struct v4l2_streamparm sp = { .type = OUTPUT };
	sp.parm.output.timeperframe = (struct v4l2_fract) {1, 60};
	checked(s->fd, VIDIOC_S_PARM, &sp, "encoder frame rate");
	control(s->fd, V4L2_CID_MPEG_VIDEO_HEVC_LEVEL, V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1);
	control(s->fd, V4L2_CID_MPEG_VIDEO_BITRATE, kbps * 1000);
	if (!k) ir_on = ir_wanted() && ir_kernel;
	// The band sweeps the encoder's own CTU rows (a strip's in fewer
	// pictures), or a strip's CTU columns.
	int ir_period = (int)(s->h + 63) / 64;
	if (ir_on && nstrips > 1 && ir_columns) {
		if (set_control(s->fd, V4L2_CID_WAVE5_INTRA_REFRESH_COLUMNS, 1) == 0) {
			ir_period = (int)(f->width + 63) / 64;
		} else {
			if (k) fail("intra refresh by columns on a strip");
			perror("intra refresh by columns (a kernel without fpvOS's Wave5 patch 0017), strips by rows");
			ir_columns = 0;
		}
	}
	if (ir_on && set_control(s->fd, V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD, ir_period)) {
		if (k) fail("intra refresh on a strip");
		perror("intra refresh (a kernel without fpvOS's Wave5 patch), periodic keyframes");
		ir_kernel = ir_on = 0;
	}
	control(s->fd, V4L2_CID_MPEG_VIDEO_GOP_SIZE, ir_on ? 0 : 60);
	if (ir_on) control(s->fd, V4L2_CID_MPEG_VIDEO_HEVC_CONST_INTRA_PRED, 1);
	control(s->fd, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 1);
	// Rate control by blocks over a 50 ms buffer: every picture near its share.
	// With picture-level control over the driver's 1 s buffer, one picture in
	// a few came out at 33-45 KiB whatever the bitrate, each over the 1.2 x
	// average that cuts it by a fifth (video/video.c): 34 cuts in 45 s, the
	// bitrate down to 3.8 Mbit/s of 12. Either alone: the buffer 37-40 cuts,
	// blocks 15 (8.7 Mbit/s); both: none, 12 Mbit/s, largest 1.2 x average.
	control(s->fd, V4L2_CID_MPEG_VIDEO_VBV_SIZE, env_int("KA_VBV_MS", 50));
	control(s->fd, V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE, env_int("KA_CU_RC", 1) != 0);
	control(s->fd, V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP, 35);
	control(s->fd, V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR, 1);
	// Whole pictures: slices of whole CTU rows (64 px), 4 at 1080p60 - 5, 5,
	// 5 and 2 rows. A strip is one slice.
	if (nstrips == 1) {
		slices = env_int("KA_SLICE_COUNT", slice_count_default(f->height, 60));
		const int rows = (int)(f->height + 63) / 64, per = (rows + (slices > 1 ? slices : 1) - 1) / (slices > 1 ? slices : 1);
		if (slices > 1) {
			control(s->fd, V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE, V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB);
			control(s->fd, V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB, per * (int)((f->width + 63) / 64));
		}
	}
	control(s->fd, V4L2_CID_MPEG_VIDEO_AU_DELIMITER, 0);
	request_buffers(s->fd, OUTPUT, DMABUF);
	request_buffers(s->fd, CAPTURE, MMAP);
	for (unsigned int i = 0; i < BUFFERS; i++) {
		struct v4l2_plane p = {0};
		struct v4l2_buffer b = buffer(CAPTURE, MMAP, i, 1, &p);
		checked(s->fd, VIDIOC_QUERYBUF, &b, "coded QUERYBUF");
		s->coded[i].size = p.length;
		s->coded[i].data = mmap(NULL, p.length, PROT_READ, MAP_SHARED,
				s->fd, p.m.mem_offset);
		if (s->coded[i].data == MAP_FAILED) fail("coded mmap");
		checked(s->fd, VIDIOC_QBUF, &b, "coded QBUF");
	}
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
    if(capture_fd<0) fail("open camera");
    kbps=kbps<128?128:kbps>40000?40000:kbps;   // the encoder's own range, as the Lite's; radio_tgt_bitrate keeps to the link and the goggle's cap
	struct v4l2_format cf = { .type = CAPTURE };
	checked(capture_fd, VIDIOC_G_FMT, &cf, "camera G_FMT");
	struct v4l2_pix_format_mplane *f = &cf.fmt.pix_mp;
	if (f->pixelformat != V4L2_PIX_FMT_YUV420M || f->num_planes != PLANES) {
		fprintf(stderr, "camera must provide three-plane YUV420M\n");
		errno = EINVAL; fail("unsupported camera/encoder format");
	}
	for (unsigned int j = 0; j < PLANES; j++) c_stride[j] = f->plane_fmt[j].bytesperline;
	// The strips: the picture's CTU rows (64) shared out, the top ones a row
	// more where they do not divide - 5, 4, 4 and 4 rows at 1080p (the last
	// 248 lines), 6, 6 and 5, or 9 and 8 - each joined to the next on a CTU
	// row and coded strip_ov rows past each of its joins: 0-336, 304-592,
	// 560-848 and 816-1080 at 4.
	{
		const unsigned rows = (f->height + 63) / 64;
		nstrips = strips_wanted > 1 ? strips_wanted : 1;
		if (nstrips > STRIPS_MAX) nstrips = STRIPS_MAX;
		if ((unsigned)nstrips > rows) nstrips = (int)rows;
		const int ov = env_int("KA_STRIP_OVERLAP", 16);
		strip_ov = nstrips > 1 && ov > 0 && ov <= 32 && ov % 8 == 0 ? (unsigned)ov : 0;
		unsigned y = 0;
		for (int k = 0; k < nstrips; k++) {
			const unsigned r = rows / (unsigned)nstrips + ((unsigned)k < rows % (unsigned)nstrips);
			const unsigned end = k == nstrips - 1 ? f->height : y + r * 64;
			strip[k].y = k ? y - strip_ov : 0;
			strip[k].h = (k == nstrips - 1 ? end : end + strip_ov) - strip[k].y;
			y += r * 64;
		}
	}
	{
		unsigned depth = 0;
		FILE *d = fopen("/sys/module/ar_cvisp/parameters/depth", "r");
		if (d) { if (fscanf(d, "%u", &depth) != 1) depth = 0; fclose(d); }
		early_row = depth == 1 ? env_int("KA_EARLY_ROW", 768) : 0;
		if (early_row < 0 || early_row >= (int)f->height) early_row = 0;
		uint32_t line = 0, rows = 0, exp = 0;
		if (early_row && ar9311_sensor_timing(&line, &rows, &exp)) early_row = 0;
		land_us = (uint32_t)((uint64_t)line * rows / 1000000) + 200;   // 16.2 ms at 1080p60
		memset(landed_at, 0, sizeof(landed_at));
		y_stride = f->plane_fmt[0].bytesperline;
		// The marks: a whole picture goes at KA_EARLY_ROW, a strip once its
		// own last row is in.
		nmarks = !early_row ? 0 : nstrips == 1 ? 1 : nstrips;
		for (int k = 0; k < nmarks; k++)
			mark_row[k] = nstrips == 1 ? early_row : (int)(strip[k].y + strip[k].h - 1);
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
		if (nmarks) {
			camera_y_size[i] = p[0].length;
			camera_y[i] = mmap(NULL, p[0].length, PROT_READ | PROT_WRITE, MAP_SHARED,
					   capture_fd, p[0].m.mem_offset);
			if (camera_y[i] == MAP_FAILED) { camera_y[i] = NULL; fail("camera mmap"); }
		}
		if (queue_camera(i)) fail("camera QBUF");
	}
	for (int k = 0; k < nstrips; k++) encoder_create(k, encoder_path, &cf, strip_kbps(kbps, k));

    if (f->width != (unsigned)width || f->height != (unsigned)height) {
        errno=EINVAL; fail("capture dimensions");
    }
    capture_lag_us=ar9311_capture_lag_us();
    venc_kbps=kbps; venc_fps=60;
    submitted=received=returned=0;
    out_strip=0;
    early_n=early_past=0; memset(early_wait_us,0,sizeof(early_wait_us));   // this channel's own
    enabled=1; warmup=8; progress=mono_us();
    if (nstrips == 1)
        printf("video: AR9311 DMA-BUF encoder %dx%d@60 %dkbps, %d slices a picture, %s, capture stamps %u us late\n",
               width,height,kbps,slices>1?slices:1,ir_on?"intra refresh by CTU rows":"periodic keyframes",capture_lag_us);
    else {
        char rows[64]; int at=0;
        for (int k=0;k<nstrips;k++) at+=snprintf(rows+at,sizeof(rows)-(size_t)at,"%s%u",k?"/":"",strip[k].h);
        printf("video: AR9311 DMA-BUF encoders %dx%d@60 %dkbps in %d strips of %s rows (%u past each join), %s, capture stamps %u us late\n",
               width,height,kbps,nstrips,rows,strip_ov,
               !ir_on?"periodic keyframes":ir_columns?"intra refresh by CTU columns":"intra refresh by CTU rows",capture_lag_us);
    }
    return 0;
}
void venc_start(void) {
    enum v4l2_buf_type ct=CAPTURE,ot=OUTPUT;
    checked(capture_fd,VIDIOC_STREAMON,&ct,"camera STREAMON"); capture_running=1;
    for (int k=0;k<nstrips;k++) {
        checked(strip[k].fd,VIDIOC_STREAMON,&ot,"encoder OUTPUT STREAMON"); strip[k].output_running=1;
        checked(strip[k].fd,VIDIOC_STREAMON,&ct,"encoder CAPTURE STREAMON"); strip[k].encoder_running=1;
    }
    venc_running=shared.venc_running=1;
}
void venc_stop_destroy(void) {
    pthread_mutex_lock(&venc_mtx);
    cleanup();
    pthread_mutex_unlock(&venc_mtx);
}
void venc_input_enable(int on) { enabled=on; }
// Where the slice starting at `at` ends: at the start code of the next VCL NAL
// unit after its own (the parameter sets before it are part of it), or at the
// picture's end.
static size_t slice_end(const uint8_t *d, size_t len, size_t at) {
    int vcl = 0;
    for (size_t i = at; i + 3 < len; i++) {
        if (d[i] || d[i + 1] || d[i + 2] != 1) continue;
        const size_t start = i > at && !d[i - 1] ? i - 1 : i;
        if (((d[i + 3] >> 1) & 0x3f) < 32) {
            if (vcl) return start;
            vcl = 1;
        }
        i += 2;
    }
    return len;
}
static int next_slice(uint8_t *dst, uint32_t room, vframe *out) {
    const size_t end = slice_end(pic.data, pic.len, pic.at), n = end - pic.at;
    if (n > room) { errno = EMSGSIZE; return recover("encoded slice exceeds packet capacity"); }
    memcpy(dst, pic.data + pic.at, n);
    const int done = end >= pic.len;
    out->len = (uint32_t)n; out->type = pic.key; out->pts = pic.pts;
    // In strips the picture ends with its last one.
    out->frame_end = done && pic.strip == (unsigned)nstrips - 1;
    out->strip = nstrips > 1;
    out->strip_ov = nstrips > 1 ? (int)strip_ov : 0;
    pic.at = end;
    if (done) pic.on = 0;
    return 1;
}
// The camera and the encoder's input are serviced first on every call, a
// picture's slices still to go or not: handing four slices out one by one
// (each waits for the radio) used to hold the next picture's encode back, by
// 18 ms at times.
// A finished picture (or the next strip) from its encoder, its first slice
// into dst: 1, or 0 when none is ready.
static int take_coded(uint8_t *dst,uint32_t room,vframe *out) {
    struct strip *s=&strip[out_strip];
    struct v4l2_plane p[1]={0};
    struct v4l2_buffer b=buffer(CAPTURE,MMAP,0,1,p);
    if(xioctl(s->fd,VIDIOC_DQBUF,&b)) return errno==EAGAIN ? 0 : recover("coded DQBUF");
    if(b.index>=BUFFERS || p[0].bytesused>s->coded[b.index].size ||
       p[0].bytesused<=p[0].data_offset || (b.flags&V4L2_BUF_FLAG_ERROR)) {
        errno=EIO; return recover("encoded frame");
    }
    pic.len=p[0].bytesused-p[0].data_offset; pic.at=0;
    if(pic.len>pic.size) {
        uint8_t *d=realloc(pic.data,pic.len);
        if(!d) { errno=ENOMEM; return recover("coded picture copy"); }
        pic.data=d; pic.size=pic.len;
    }
    memcpy(pic.data,(const uint8_t *)s->coded[b.index].data+p[0].data_offset,pic.len);
    pic.key=(b.flags&V4L2_BUF_FLAG_KEYFRAME)?1:0;
    // Row 0's capture, as the stamps of the HiSilicon backend are: the
    // capture driver stamps its buffers when it hands them over.
    pic.pts=(uint64_t)b.timestamp.tv_sec*1000000+b.timestamp.tv_usec-capture_lag_us;
    if(xioctl(s->fd,VIDIOC_QBUF,&b)) return recover("coded requeue");
    // A picture's strips in order, all of one capture: a strip of another
    // would show two moments on the ground's screen at once.
    if(out_strip && pic.pts!=out_pts) { errno=EIO; return recover("strips out of step"); }
    out_pts=pic.pts; pic.strip=out_strip;
    out_strip=(out_strip+1)%(unsigned)nstrips;
    pic.on=1;
    if(!out_strip) { received++; left_frames=submitted-received; }
    progress=mono_us();
    return next_slice(dst,room,out);
}
// A camera buffer to encoder k: the whole of it, or strip k's rows of each
// plane (the chroma planes' rows half as many).
static int submit_camera(unsigned index, struct timeval stamp, int k) {
    const struct strip *s=&strip[k];
    struct v4l2_plane p[PLANES]={0};
    struct v4l2_buffer b=buffer(OUTPUT,DMABUF,index,PLANES,p);
    b.timestamp=stamp;
    for(unsigned j=0;j<PLANES;j++) {
        p[j].m.fd=camera[index][j].fd;
        p[j].length=p[j].bytesused=camera[index][j].size;
        if(nstrips>1) {
            p[j].data_offset=(j?s->y/2:s->y)*c_stride[j];
            p[j].bytesused=p[j].data_offset+(j?(s->h+1)/2:s->h)*c_stride[j];
        }
    }
    if(xioctl(s->fd,VIDIOC_QBUF,&b)) return recover("encoder DMA-BUF QBUF");
    held[index]|=1u<<k;
    if(!k) { submitted++; left_frames=submitted-received; }
    return 0;
}
// The landing buffer's strips whose rows are in, each to its encoder - all of
// them (force) once a newer buffer has come. Then the buffer is done waiting.
static int release_landed(int force) {
    const uint64_t st=(uint64_t)waiting_stamp.tv_sec*1000000+waiting_stamp.tv_usec;
    while(waiting>=0 && waiting_next<nstrips) {
        const int k=waiting_next;
        if(!force && k<nmarks && !early_landed((unsigned)waiting,k)) return 0;
        const uint64_t now=mono_us();
        if(k<nmarks) early_wait_us[k]+=now>st?now-st:0;
        if(submit_camera((unsigned)waiting,waiting_stamp,k)) return -1;
        waiting_next++;
    }
    if(waiting<0) return 0;
    waiting=-1;
    if(nmarks && ++early_n%300==0) {
        if(nstrips==1)
            printf("video: early start at row %d: %u pictures, released %.2f ms after the stamp on average, "
                   "%u already past it at hand-over\n",early_row,early_n,early_wait_us[0]/1000.0/early_n,early_past);
        else {
            char at[64]; int n=0;
            for(int k=0;k<nstrips;k++) n+=snprintf(at+n,sizeof(at)-(size_t)n,"%s%.2f",k?"/":"",early_wait_us[k]/1000.0/early_n);
            printf("video: strips released %s ms after the stamp on average (%u pictures), "
                   "%u already past their first at hand-over\n",at,early_n,early_past);
        }
        early_n=0; early_past=0; memset(early_wait_us,0,sizeof(early_wait_us));
    }
    return 0;
}
static int get_frame(uint8_t *dst,uint32_t room,vframe *out) {
    struct v4l2_plane p[PLANES]={0};
    struct v4l2_buffer b;
    // Back to the camera once every encoder has done with it - and none of
    // its strips is still to go (the landing buffer).
    for(int k=0;k<nstrips;k++) {
        memset(p,0,sizeof(p)); b=buffer(OUTPUT,DMABUF,0,PLANES,p);
        while(xioctl(strip[k].fd,VIDIOC_DQBUF,&b)==0) {
            if(b.index>=BUFFERS || !(held[b.index]>>k&1) || (b.flags&V4L2_BUF_FLAG_ERROR)) {
                errno=EIO; return recover("encoder source return");
            }
            held[b.index]&=~(1u<<k);
            if(!held[b.index] && waiting!=(int)b.index) {
                if(give_back(b.index)) return recover("camera QBUF");
                returned++;
            }
            progress=mono_us();
            memset(p,0,sizeof(p)); b=buffer(OUTPUT,DMABUF,0,PLANES,p);
        }
        if(errno!=EAGAIN) return recover("source DQBUF");
    }
    for(unsigned i=0;requeue && i<BUFFERS;i++)
        if((requeue>>i&1) && mono_us()>=landed_at[i]) {
            requeue&=~(1u<<i);
            if(queue_camera(i)) return recover("camera QBUF");
        }
    if(!pic.on) {
        int r=take_coded(dst,room,out);
        if(r) return r;
    }
    memset(p,0,sizeof(p)); b=buffer(CAPTURE,MMAP,0,PLANES,p);
    if(xioctl(capture_fd,VIDIOC_DQBUF,&b)==0) {
        if(b.index>=BUFFERS || held[b.index] || (b.flags&V4L2_BUF_FLAG_ERROR)) {
            errno=EIO; return recover("camera buffer");
        }
        unsigned index=b.index;
        landed_at[index]=(uint64_t)b.timestamp.tv_sec*1000000+b.timestamp.tv_usec+land_us;
        if(warmup || !enabled) {
            if(give_back(index)) return recover("camera QBUF");
            if(warmup && !--warmup && ar9311_controls_start()) {
                errno=EIO; return recover("camera automatic controls");
            }
            progress=mono_us();
        } else {
            // One landing at a time; a newer one sends the older's strips on.
            // A buffer done with by every encoder while it waited (an early
            // strip back before the next went in) goes back once it is done.
            if(waiting>=0) {
                const int w=waiting;
                if(release_landed(1)) return -1;
                if(!held[w]) { if(give_back((unsigned)w)) return recover("camera QBUF"); returned++; }
            }
            if(nmarks && early_landed(index,0)) early_past++;   // its first mark already gone: should not happen
            waiting=(int)index; waiting_stamp=b.timestamp; waiting_next=0;
        }
    } else if(errno!=EAGAIN) return recover("camera DQBUF");
    if(waiting>=0) {
        const int w=waiting;
        if(release_landed(0)) return -1;
        if(waiting<0 && !held[w]) { if(give_back((unsigned)w)) return recover("camera QBUF"); returned++; }
    }
    if(!warmup && ar9311_controls_alive()) { errno=EIO; return recover("camera controller exited"); }
    if(pic.on) return next_slice(dst,room,out);
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
        struct pollfd pf[1+STRIPS_MAX]={{capture_fd,POLLIN,0}};
        for(int k=0;k<nstrips;k++) pf[1+k]=(struct pollfd){strip[k].fd,POLLIN,0};
        // 0.2 ms while a buffer waits for its rows to land, else 2 ms
        struct timespec ts={0,waiting>=0 || requeue?200000:2000000};
        if(ppoll(pf,(nfds_t)(1+nstrips),&ts,NULL)<0 && errno!=EINTR) { r=recover("video poll"); break; }
    }
    pthread_mutex_unlock(&venc_mtx);
    return r;
}
void tune_poll(void) { }
void roi_update(int on) { (void)on; }
int video_low_delay_lines(void) { return 0; }
// The air-side times and the radio clock at encoder out, to 1/256 ms
// (KA_FEAT_LATINFO, KA_FEAT_APCLOCK): the capture stamps are row 0's capture
// by the measured timing (ar9311_capture_lag_us), so the ground reads them on
// its own clock as on the Lite.
int lat_info(void) { static int v = -1; if (v < 0) v = env_int("KA_LATINFO", 1) != 0; return v; }
int video_tune_armed(void) { return 0; }
int video_tune_pwr(void) { return -1; }   // no tune file here: the chosen TX power
int video_tune_pathb(void) { return 1; }  // and both TX paths, as in flight
int video_tgt_fps(void) { return fps; }
int video_kbps(void) { return venc_kbps; }
// Before the first channel, what the setting asks for: a refused control
// takes the bit back with the next header.
// The power levels too (150, 200 and 316 mW, and their AUTO): the ground
// offers them, and the P1 holds anything above its 23 dBm there (radio/power.c).
uint8_t video_features(void) {
    return (uint8_t)((lat_info() ? KA_FEAT_LATINFO | KA_FEAT_APCLOCK : 0) |
                     (radio_bw40_enabled()?KA_FEAT_MAXBW:0) |
                     (ir_wanted() && ir_kernel ? KA_FEAT_INTRA_REFRESH : 0) | KA_FEAT_PWRLVL);
}
void set_venc_bitrate(int f,int kbps,int reason) {
    (void)f; (void)reason;
    kbps=kbps<128?128:kbps>40000?40000:kbps;   // the encoder's own range, as the Lite's; radio_tgt_bitrate keeps to the link and the goggle's cap
    pthread_mutex_lock(&venc_mtx);
    if(venc_running && kbps!=venc_kbps) {
        int bad=0;
        for(int k=0;k<nstrips && !bad;k++) bad=set_control(strip[k].fd,V4L2_CID_MPEG_VIDEO_BITRATE,strip_kbps(kbps,k)*1000);
        if(bad) recover("encoder bitrate");
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
    // Every strip: the ground decodes each on its own.
    for(int k=0;on && venc_running && k<nstrips;k++)
        if(set_control(strip[k].fd,V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,0)) { recover("encoder keyframe"); break; }
    pthread_mutex_unlock(&venc_mtx);
}
// KA_CMD_STRIPS (and the link going down: 0): taken at the next channel,
// which this asks the read loop for, between pictures.
void video_set_strips(int n) {
    n=n<2?1:n>STRIPS_MAX?STRIPS_MAX:n;
    const int was=strips_wanted>1?strips_wanted:1;
    strips_wanted=n;
    if(n==was) return;
    printf("video: %s\n",n>1?"strips asked for":"whole pictures again");
    if(venc_running) venc_recreate=1;
}
void video_strips(int *now,int *max) {
    *now=venc_running?nstrips:(strips_wanted>1?strips_wanted:1);
    *max=STRIPS_MAX;
}
