// Source AE/AWB services shipped by fpvOS. They are owned by this capture
// session, started after warm-up, and stopped before capture is torn down.
#include "camera/image.h"
#include "backends/ar9311/camera.h"
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>
#include <sys/socket.h>
#include "app/config.h"
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
static void write_cap(void) {
    FILE *f = fopen(CAP_FILE, "w");
    if (!f) { fprintf(stderr, "camera: %s: %m\n", CAP_FILE); return; }
    fprintf(f, "%u\n", image_max_exposure_us);
    if (fclose(f)) fprintf(stderr, "camera: %s: %m\n", CAP_FILE);
}
int image_max_exposure_live(void) { return 1; }
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
static int apply_saved(void) {
    int ev=cfg_get("ev_x10",0), wb=(int16_t)cfg_get("cct",0);
    if(ev < -10 || ev > 10) ev=0;
    if(wb != -1) wb=0;
    int red=0,blue=0;
    if(wb==-1) {
        red=cfg_get("wb_lock_red",0);blue=cfg_get("wb_lock_blue",0);
        if(red<128 || red>1024 || blue<128 || blue>1024) {wb=0;red=blue=0;}
    }
    if(request(&ae_fd,ev,0,0,NULL,NULL) || request(&awb_fd,wb,red,blue,NULL,NULL))return -1;
    cfg_set("ev_x10",ev);cfg_set("cct",wb);
    return 0;
}
int ar9311_camera_set(unsigned id, int value) {
    if ((id==2 && (value < -10 || value > 10)) ||
        (id==5 && value!=0 && value!=-1) || (id!=2 && id!=5)) return -1;
    pthread_mutex_lock(&controls_lock);
    int red=0,blue=0;
    int result=request(id==2?&ae_fd:&awb_fd,value,0,0,&red,&blue);
    if(!result) {
        cfg_set(id==2?"ev_x10":"cct",value);
        if(id==5 && value==-1) {cfg_set("wb_lock_red",red);cfg_set("wb_lock_blue",blue);}
        result=cfg_save();
        printf("camera: applied %s=%d red=%d blue=%d persistence=%s\n",
               id==2?"ev_x10":"white_balance",value,red,blue,result?"failed":"saved");
    }
    pthread_mutex_unlock(&controls_lock);
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
    puts("camera: source AE/AWB; remote EV and auto/locked white balance");
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
    do { r = waitpid(awb, NULL, WNOHANG); } while (r < 0 && errno == EINTR);
    if (r != 0) { awb = 0; return -1; }
    return 0;
}
int ar9311_controls_alive(void) {
    pthread_mutex_lock(&controls_lock);int r=alive_locked();pthread_mutex_unlock(&controls_lock);return r;
}
void image_restart(int fps, int angle) { (void)fps; (void)angle; }
// The exposure the AE service set on the sensor, in us; the rest is not read.
void image_get_info(image_isp_info *i) {
    memset(i, 0, sizeof(*i));
    uint32_t line, rows, exp;
    if (!ar9311_sensor_timing(&line, &rows, &exp)) i->exp_time_us = (uint32_t)(((uint64_t)line * exp + 500000) / 1000000);
}
void image_apply_all(void) {
    pthread_mutex_lock(&controls_lock);
    if(ae && awb && apply_saved()) fprintf(stderr,"camera: saved controls could not be applied\n");
    pthread_mutex_unlock(&controls_lock);
}
void image_set_ev(int v) { (void)ar9311_camera_set(2,v); }
void image_set_awb(int v) { (void)ar9311_camera_set(5,v); }
#define UNSUPPORTED(name) void image_set_##name(int v) { (void)v; fprintf(stderr,"camera: " #name " control unsupported\n"); }
UNSUPPORTED(scene)
UNSUPPORTED(sat)
UNSUPPORTED(sharpness)
UNSUPPORTED(contrast)
UNSUPPORTED(3dnr)
UNSUPPORTED(2dnr)
void image_low_power(int on) { (void)on; } // RF standby does not alter the validated sensor mode
