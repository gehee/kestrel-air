// Source AE/AWB services shipped by fpvOS. They are owned by this capture
// session, started after warm-up, and stopped before capture is torn down.
#include "camera/image.h"
#include "app/config.h"
#include "backends/ar9311/camera.h"
#include "backends/ar9311/wb.h"
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>
#include <sys/socket.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
unsigned image_max_exposure_us;
static pid_t ae, awb;
static int ae_fd=-1, awb_fd=-1;
static pthread_mutex_t controls_lock=PTHREAD_MUTEX_INITIALIZER;
static void pause_locked(void);

// The AE's exposure cap, which it reads at start and again on a SIGHUP: the
// goggle's VIDEO > Max Exposure applies live, from the AE's operating point.
#define CAP_FILE "/run/kestrel-air/max-exposure-us"

static void write_num(const char *path, long v) {
    FILE *f = fopen(path, "w");
    if (!f) { fprintf(stderr, "camera: %s: %m\n", path); return; }
    fprintf(f, "%ld\n", v);
    if (fclose(f)) fprintf(stderr, "camera: %s: %m\n", path);
}
static void write_cap(void) { write_num(CAP_FILE, image_max_exposure_us); }
int image_max_exposure_live(void) { return 1; }
// The camera settings it applies, by the goggle's names, for its info
// (cam_ctrl): Rotate, the sensor's mirror and flip (camera_set_sensor_angle);
// EV, the AE's target, over its control socket; White Balance over the AWB
// service's: Auto, Lock current (wb_lock) or a kelvin preset (wb), locked at
// the tuning file's reference white points for it (wb.c); Saturation and
// Contrast, the ISP's (fpvOS ar-isp 0009); 3D DNR and Sharpness, its de3d and
// lnr strengths (0010).
const char *image_settings(void) { return "rotate,ev,wb,wb_lock,sat,contrast,sharpness,3dnr"; }
// The EV goes over the AE's control socket (ar9311_camera_set); the cap
// through its file, read again on a SIGHUP.
int image_set_max_exposure(unsigned us) {
    if (us > 1000000) return -1;
    pthread_mutex_lock(&controls_lock);
    if (us != image_max_exposure_us) {
        image_max_exposure_us = us;
        printf("camera: exposure capped at %u us%s\n", us, us ? "" : " - none, the tuning's own");
        write_cap();
        if (ae > 0) kill(ae, SIGHUP);
    }
    pthread_mutex_unlock(&controls_lock);
    return 0;
}

static void stop(pid_t *p) {
    if (*p <= 0) return;
    kill(*p, SIGTERM);
    for (int i = 0; i < 100; i++) {
        int r = waitpid(*p, NULL, WNOHANG);
        if (r == *p || (r < 0 && errno == ECHILD)) { *p = 0; return; }
        usleep(10000);
    }
    kill(*p, SIGKILL);
    while (waitpid(*p, NULL, 0) < 0 && errno == EINTR) { }
    *p = 0;
}
static void pause_locked(void) {
    if (awb_fd>=0) close(awb_fd);
    if (ae_fd>=0) close(ae_fd);
    awb_fd=ae_fd=-1;
    stop(&awb); stop(&ae);
}
void image_pause(void) {
    pthread_mutex_lock(&controls_lock); pause_locked(); pthread_mutex_unlock(&controls_lock);
}
static int spawn_controller(pid_t *pid, int *fd, char **argv) {
    int pair[2], e;
    if (socketpair(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0,pair)) return -1;
    // Move the child's endpoint above fd 3 before dup2, including the case
    // where stdin/out were closed by the caller.
    int child=fcntl(pair[1],F_DUPFD_CLOEXEC,4);
    close(pair[1]);
    if (child<0) {close(pair[0]);return -1;}
    posix_spawn_file_actions_t actions;
    e=posix_spawn_file_actions_init(&actions);
    if(!e) {
        e=posix_spawn_file_actions_addclose(&actions,pair[0]);
        if(!e)e=posix_spawn_file_actions_adddup2(&actions,child,3);
        if(!e)e=posix_spawn_file_actions_addclose(&actions,child);
        if(!e)e=posix_spawnp(pid,argv[0],&actions,NULL,argv,environ);
        posix_spawn_file_actions_destroy(&actions);
    }
    close(child);
    if(e) { close(pair[0]);*pid=0;errno=e;return -1; }
    *fd=pair[0];return 0;
}
static int request(int *fd, int value, int red, int blue, int *actual_red, int *actual_blue) {
    char command[64], reply[64], trailing;
    int n=red?snprintf(command,sizeof(command),"SET %d %d %d",value,red,blue):
              snprintf(command,sizeof(command),"SET %d",value);
    if (*fd<0) return -1;
    if (send(*fd,command,n,MSG_NOSIGNAL)!=n) goto failed;
    struct pollfd p={.fd=*fd,.events=POLLIN};
    int ready;
    do {ready=poll(&p,1,1000);} while(ready<0 && errno==EINTR);
    if (ready<=0 || !(p.revents&POLLIN)) goto failed;
    n=recv(*fd,reply,sizeof(reply)-1,MSG_TRUNC);
    if (n<=0 || n>=(int)sizeof(reply)-1) goto failed;
    reply[n]=0;
    int applied,r,b;
    if (sscanf(reply,"OK %d %d %d%c",&applied,&r,&b,&trailing)!=3 || applied!=value) goto failed;
    if(actual_red)*actual_red=r;
    if(actual_blue)*actual_blue=b;
    return 0;
failed:
    // Never let a late acknowledgement satisfy a subsequent command.
    close(*fd);*fd=-1;
    fprintf(stderr,"camera: control failed or timed out; restarting the capture session\n");
    return -1;
}
// White Balance as the AWB service's gains: a kelvin preset's from the
// tuning file's reference white points (wb.c, the vendor's lookup), locked
// there as Lock current's are; Lock current's as saved. -1 where there are
// none (no white points for it, or no valid saved lock).
static int wb_gains(int wb, int *red, int *blue) {
    *red=*blue=0;
    if(wb>0) {
        unsigned r,b;
        const char *tuning=getenv("KA_CAMERA_TUNING");
        if(!tuning || ar9311_wb_gains_file(tuning,wb,&r,&b)) {
            fprintf(stderr,"camera: no white points in the tuning file for %d K\n",wb);
            return -1;
        }
        *red=(int)r;*blue=(int)b;
    } else if(wb==-1) {
        *red=cfg_get("wb_lock_red",0);*blue=cfg_get("wb_lock_blue",0);
    } else return 0;
    if(*red<128 || *red>1024 || *blue<128 || *blue>1024) {*red=*blue=0;return -1;}
    return 0;
}
static int apply_saved(void) {
    int ev=cfg_get("ev_x10",0), wb=(int16_t)cfg_get("cct",0);
    if(ev < -10 || ev > 10) ev=0;
    if(wb != -1 && (wb < 1000 || wb > 20000)) wb=0;
    int red=0,blue=0;
    if(wb_gains(wb,&red,&blue)) wb=0;   // an older lock without gains: Auto
    if(request(&ae_fd,ev,0,0,NULL,NULL) || request(&awb_fd,wb?-1:0,red,blue,NULL,NULL))return -1;
    cfg_set("ev_x10",ev);cfg_set("cct",wb);
    return 0;
}
int ar9311_camera_set(unsigned id, int value) {
    if ((id==2 && (value < -10 || value > 10)) ||
        (id==5 && value!=0 && value!=-1 && (value < 1000 || value > 20000)) || (id!=2 && id!=5)) return -1;
    pthread_mutex_lock(&controls_lock);
    int red=0,blue=0,set_red=0,set_blue=0,want=value,result=0;
    // A kelvin preset: locked at the tuning's gains for it (SET -1 red blue).
    if(id==5 && value>0) {result=wb_gains(value,&set_red,&set_blue);want=-1;}
    if(!result) result=request(id==2?&ae_fd:&awb_fd,want,set_red,set_blue,&red,&blue);
    if(!result) {
        cfg_set(id==2?"ev_x10":"cct",value);
        if(id==5 && value==-1) {cfg_set("wb_lock_red",red);cfg_set("wb_lock_blue",blue);}
    }
    pthread_mutex_unlock(&controls_lock);
    // The flash write after: the services are free again by then.
    if(!result) {
        result=cfg_save();
        printf("camera: applied %s=%d red=%d blue=%d persistence=%s\n",
               id==2?"ev_x10":"white_balance",value,red,blue,result?"failed":"saved");
    }
    return result;
}
static int gain(const char *key, int *out) {
    const char *v = getenv(key);
    char *end;
    if (!v || !*v) return -1;
    long n = strtol(v, &end, 10);
    if (*end || n < 128 || n > 1024) return -1;
    *out = (int)n;
    return 0;
}
int image_start(int fps, int angle) {
    (void)fps; (void)angle;
    const char *t = getenv("KA_CAMERA_TUNING"), *order = getenv("KA_WB_ORDER");
    int r, b;
    if (!t || access(t, R_OK) || gain("KA_WB_RED", &r) || gain("KA_WB_BLUE", &b) ||
        !order || (strcmp(order,"grbg") && strcmp(order,"rggb") && strcmp(order,"bggr") && strcmp(order,"gbrg"))) {
        fprintf(stderr, "camera: launcher must supply tuning and verified white-balance settings\n");
        return -1;
    }
    if (image_max_exposure_us) printf("camera: exposure capped at %u us, the gain making up the rest\n", image_max_exposure_us);
    puts("camera: source AE/AWB; Rotate, EV, White Balance, Saturation, Contrast, Sharpness and 3D DNR from the goggle");
    return 0;
}
static int start_locked(void) {
    char sensor[128];
    int red, blue;
    if (ae || awb) return ae_fd>=0 && awb_fd>=0 ? 0 : -1;
    if (ar9311_node("sc231ai", sensor, sizeof(sensor)) ||
        gain("KA_WB_RED", &red) || gain("KA_WB_BLUE", &blue)) return -1;
    FILE *f = fopen("/sys/kernel/debug/ar-isp/white_balance", "w");
    if (!f) return -1;
    int ok = fprintf(f, "%u\n", (unsigned)red * 65536u + (unsigned)blue) > 0;
    if (fclose(f)) ok = 0;
    if (!ok) return -1;
    // --max-exposure-us: the AE holds the exposure there and raises the gain
    // instead (its table goes to 10 ms before any gain, 16.5 ms in the dark).
    // Through the file, which a new cap rewrites (image_set_max_exposure).
    write_cap();
    char *a[] = {"ar9311-aed", "--sensor-device", sensor, "--tuning",
        getenv("KA_CAMERA_TUNING"), "--stats", "300", "--control-fd", "3",
        "--max-exposure-file", CAP_FILE, NULL};
    char *b[] = {"ar9311-awbd", getenv("KA_WB_ORDER"), getenv("KA_WB_RED"), getenv("KA_WB_BLUE"), "--control-fd", "3", NULL};
    if(spawn_controller(&ae,&ae_fd,a) || spawn_controller(&awb,&awb_fd,b) || apply_saved()) {
        pause_locked();return -1;
    }
    return 0;
}
int ar9311_controls_start(void) {
    pthread_mutex_lock(&controls_lock);int r=start_locked();pthread_mutex_unlock(&controls_lock);return r;
}
static int alive_locked(void) {
    if(ae_fd<0 || awb_fd<0)return -1;

    if (ae <= 0 || awb <= 0) return -1;
    pid_t r;
    do { r = waitpid(ae, NULL, WNOHANG); } while (r < 0 && errno == EINTR);
    if (r != 0) { ae = 0; return -1; }
    if (awb <= 0) return 0;
    do { r = waitpid(awb, NULL, WNOHANG); } while (r < 0 && errno == EINTR);
    if (r != 0) { awb = 0; return -1; }
    return 0;
}
// The encoder's, each picture: never waits for a camera command on its way
// to a service and back (up to 1 s each) - one under way is a service alive.
// A command that failed closes its socket, which the next check finds.
int ar9311_controls_alive(void) {
    if(pthread_mutex_trylock(&controls_lock)) return 0;
    int r=alive_locked();pthread_mutex_unlock(&controls_lock);return r;
}
void image_restart(int fps, int angle) { (void)fps; (void)angle; }
// The exposure the AE service set on the sensor, in us; the rest is not read.
void image_get_info(image_isp_info *i) {
    memset(i, 0, sizeof(*i));
    uint32_t line, rows, exp;
    if (!ar9311_sensor_timing(&line, &rows, &exp)) i->exp_time_us = (uint32_t)(((uint64_t)line * exp + 500000) / 1000000);
}
// Before the setters below: the saved EV and White Balance over the services'
// sockets, then the ISP's own settings.
void image_set_sat(int v);
void image_set_contrast(int v);
void image_set_sharpness(int v);
void image_set_3dnr(int v);
void image_apply_all(void) {
    pthread_mutex_lock(&controls_lock);
    if(ae && awb && apply_saved()) fprintf(stderr,"camera: saved controls could not be applied\n");
    pthread_mutex_unlock(&controls_lock);
    image_set_sat(cfg_get("sat", 0));
    image_set_contrast(cfg_get("contrast", 0));
    image_set_sharpness(cfg_get("sharpness", 0));
    image_set_3dnr(cfg_get("cam_3dnr_strength", 5));
}
// The goggle's Saturation and Contrast, 0 (Auto) or 1..10 with 5 the
// tuning's, as the Lite maps them (camera/image.c sat_map, contrast_map):
// percent of the tuning's for fpvOS ar-isp's debugfs saturation (chroma rows
// of its colour-space matrix) and contrast (its gamma curve about mid-scale).
// The ISP keeps them across stream restarts.
static const int sat_pct[11] = { 100, 20, 60, 80, 90, 100, 110, 120, 140, 160, 180 };
static const int contrast_pct[11] = { 100, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150 };
static void isp_set(const char *name, const int *pct, int v) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/kernel/debug/ar-isp/%s", name);
    v = v >= 0 && v <= 10 ? v : 0;
    printf("camera: %s %d (%d%% of the tuning's)\n", name, v, pct[v]);
    write_num(path, pct[v]);
}
void image_set_sat(int v) { isp_set("saturation", sat_pct, v); }
void image_set_contrast(int v) { isp_set("contrast", contrast_pct, v); }
// The goggle's 3D DNR (Off 0, Low 2, Mid 3, High 4, Auto 5): the ISP's de3d
// strength pair, 0..100 with 50 the tuning's (ar-isp module parameters, the
// vendor's strength law). They take effect at its next ladder re-latch, forced
// here only while capture runs (the AE is up): unconfigured, the ISP must not
// be written, and a stream start applies them anyway.
void image_set_3dnr(int v) {
    int s = v == 0 ? 0 : v == 2 ? 30 : v == 4 ? 70 : 50;
    printf("camera: 3D noise reduction %d (de3d strength %d, 50 the tuning's)\n", v, s);
    write_num("/sys/module/ar_isp/parameters/de3d_strength", s);
    write_num("/sys/module/ar_isp/parameters/de3d_strength9", s);
    if (ae > 0) write_num("/sys/kernel/debug/ar-isp/ladders", 1);
}
// The goggle's Sharpness, 0 (Auto) or 1..10 with 5 the tuning's: the ISP's lnr
// strength (fpvOS ar-isp 0010, the vendor's Sharpness law, 50 the tuning's;
// above it the fields climb toward 511 whatever the gain, so the top stays
// mild), at its next re-latch as the 3D DNR.
static const int sharp_strength[11] = { 50, 10, 20, 30, 40, 50, 52, 54, 56, 58, 60 };
void image_set_sharpness(int v) {
    v = v >= 0 && v <= 10 ? v : 0;
    printf("camera: sharpness %d (lnr strength %d, 50 the tuning's)\n", v, sharp_strength[v]);
    write_num("/sys/module/ar_isp/parameters/lnr_strength", sharp_strength[v]);
    if (ae > 0) write_num("/sys/kernel/debug/ar-isp/ladders", 1);
}
void image_set_ev(int v) { (void)ar9311_camera_set(2,v); }
void image_set_awb(int v) { (void)ar9311_camera_set(5,v); }
#define UNSUPPORTED(name) void image_set_##name(int v) { (void)v; fprintf(stderr,"camera: " #name " control unsupported\n"); }
UNSUPPORTED(scene)
UNSUPPORTED(2dnr)
void image_low_power(int on) { (void)on; } // RF standby does not alter the validated sensor mode
