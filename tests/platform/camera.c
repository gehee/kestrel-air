// Exercise the real native camera transaction against a private fake daemon.
#define _GNU_SOURCE
#include <assert.h>
#include "../../src/backends/ar9311/image.c"
int ar9311_node(const char *name,char *path,size_t size){(void)name;(void)path;(void)size;return -1;}
int ar9311_sensor_timing(uint32_t *line,uint32_t *rows,uint32_t *exposure){(void)line;(void)rows;(void)exposure;return -1;}
static pid_t fake(int *fd,const char *expected,const char *response) {
    int pair[2];assert(!socketpair(AF_UNIX,SOCK_SEQPACKET,0,pair));
    pid_t p=fork();assert(p>=0);
    if(!p) {
        close(pair[0]);char buf[64];int n=recv(pair[1],buf,sizeof(buf)-1,0);
        assert(n>0);buf[n]=0;assert(!strcmp(buf,expected));
        assert(send(pair[1],response,strlen(response),MSG_NOSIGNAL)==(ssize_t)strlen(response));
        close(pair[1]);_exit(0);
    }
    close(pair[1]);*fd=pair[0];return p;
}
static void reap(pid_t p){int status;assert(waitpid(p,&status,0)==p && WIFEXITED(status) && !WEXITSTATUS(status));}
int main(void) {
    char directory[]=".local/tests/camera-XXXXXX", path[128];
    assert(mkdtemp(directory));
    snprintf(path,sizeof(path),"%s/settings.json",directory);
    assert(!setenv("KA_CONFIG",path,1));

    assert(ar9311_camera_set(2,11)<0);
    assert(ar9311_camera_set(5,5200)<0);
    pid_t p=fake(&ae_fd,"SET -7","OK -7 0 0");
    assert(!ar9311_camera_set(2,-7) && cfg_get("ev_x10",0)==-7);reap(p);close(ae_fd);
    p=fake(&awb_fd,"SET -1","OK -1 400 700");
    assert(!ar9311_camera_set(5,-1) && cfg_get("cct",0)==-1 && cfg_get("wb_lock_red",0)==400 && cfg_get("wb_lock_blue",0)==700);reap(p);close(awb_fd);
    p=fake(&ae_fd,"SET 10","ERR 10 0 0");
    assert(ar9311_camera_set(2,10)<0 && cfg_get("ev_x10",0)==-7 && ae_fd==-1);reap(p);
    p=fake(&ae_fd,"SET 0","OK 10 0 0");
    assert(ar9311_camera_set(2,0)<0 && cfg_get("ev_x10",0)==-7 && ae_fd==-1);reap(p);
    awb_fd=-1;
    // Use the real config schema and serializer: unknown keys silently drop.
    cfg_set("wb_lock_red",0);cfg_set("wb_lock_blue",0);cfg_set("cct",0);cfg_set("ev_x10",0);
    assert(!cfg_load());
    assert(cfg_get("wb_lock_red",0)==400 && cfg_get("wb_lock_blue",0)==700);
    assert(cfg_get("cct",0)==-1 && cfg_get("ev_x10",0)==-7);
    p=fake(&ae_fd,"SET -7","OK -7 0 0");
    pid_t w=fake(&awb_fd,"SET -1 400 700","OK -1 400 700");
    assert(!apply_saved());reap(p);reap(w);close(ae_fd);close(awb_fd);
    unlink(path);rmdir(directory);

    puts("Native camera transactions preserve settings on failure and save applied lock gains");
}
