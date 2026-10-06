// The rest of the board, as the stock air app drives it: the pairing
// button, the two LEDs, and the debug command port.
//
// - Button: /sys/class/gpio/gpio0/value, active low. Any press shorter than
//   8 s pairs on release; holding 8 s sends "40 00 01" to the upgrade daemon
//   on 127.0.0.1:6444 (and keeps timing). MSP 190 and debug command 0x86 ask
//   for pairing too, through a flag this thread services.
// - LEDs: red gpio63, green gpio8, a 100 ms state machine: radio init failed,
//   SoC over 90 C, pairing, no link, encoder stalled, all well.
// - Debug commands on UDP 4486 (the subset that does something here).
//
// The GPIOs are exported and set to in/out by the launcher, as stock's is.

#include "unit/board.h"
#include "video/video.h"
#include "app/state.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "radio/radio.h"
#include "app/config.h"
#include "common/clock.h"

#define KEY_GPIO   "/sys/class/gpio/gpio0/value"
#define LED_R      63
#define LED_G      8
#define USER_CFG   "/factory/user_cfg.json"

static volatile int manu_trig;
static volatile int pairing;

// ---- LEDs -----------------------------------------------------------------

static void gpio_set(int gpio, int on) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        printf("board: cannot open %s\n", path);
        return;
    }
    if (write(fd, on ? "1" : "0", 1) < 0) { /* nothing to do */ }
    close(fd);
}

static void led(int r, int g) {
    gpio_set(LED_R, r);
    gpio_set(LED_G, g);
}

void board_leds_boot(void) { led(0, 1); }

// Timer 1, every 100 ms: the first matching state wins.
static void *led_thread(void *arg) {
    (void)arg;
    int T = 0, TG = 0, B = 0, P = 0;
    for (;;) {
        usleep(100000);
        int down = !radio_connected(), stalled = video_stream_flag == 0;
        if (radio_init_failed) {                  // 2 s red blinking, 1 s dark, 1 s green
            if (P <= 1) {
                gpio_set(LED_R, ~B & 1);
                gpio_set(LED_G, 0);
                T = 0;
                if (++B == 10) { B = 0; P++; }
            } else if (++T >= 10) {
                T = 0;
                gpio_set(LED_R, 0);
                gpio_set(LED_G, B == 0);
                B ^= 1;
                if (B == 0) P = 0;
            }
        } else if (shared.cpu_hot) {              // red and green alternating, 200 ms
            if (++T >= 2) {
                T = 0;
                if (TG) led(1, 0); else led(0, 1);
                TG ^= 1;
            }
        } else if (pairing) {                  // solid red
            led(1, 0);
        } else if (down) {                     // green blinking 200 ms
            if (++T >= 2) { T = 0; led(0, TG); TG ^= 1; }
        } else if (stalled) {                  // green blinking 1 s
            if (++T >= 10) { T = 0; led(0, TG); TG ^= 1; }
        } else {                               // solid green
            led(0, 1);
        }
    }
    return NULL;
}

// ---- pairing --------------------------------------------------------------

void board_trigger_pairing(void) {
    puts("board: pairing asked for by the flight controller");
    manu_trig = 1;
}

// The paired ground's address, saved only if it changed.
static void save_peer_mac(const uint8_t m[4]) {
    int old[4];
    if (cfg_json_ints(USER_CFG, "bb_mac_addr_0", old, 4) == 4 && (old[0] & 0xff) == m[0] &&
        (old[1] & 0xff) == m[1] && (old[2] & 0xff) == m[2] && (old[3] & 0xff) == m[3]) {
        puts("board: the same ground as before, not saved");
        return;
    }
    char arr[64];
    snprintf(arr, sizeof(arr), "[\"0x%02X\", \"0x%02X\", \"0x%02X\", \"0x%02X\"]", m[0], m[1], m[2], m[3]);
    cfg_json_set(USER_CFG, "save_candidate_position", "0");
    printf("board: saved ground %d, mac=[%02X %02X %02X %02X]\n", 0, m[0], m[1], m[2], m[3]);
    cfg_json_set(USER_CFG, "bb_mac_addr_0", arr);
}

static void do_match(void) {
    uint8_t mac[4];
    pairing = 1;
    int r = radio_match(30000, mac);
    pairing = 0;
    if (r == 0) {
        printf("board: paired, mac=[%02X %02X %02X %02X]\n", mac[0], mac[1], mac[2], mac[3]);
        save_peer_mac(mac);
    } else {
        puts("board: pairing failed");
    }
}

// The stock upgrade daemon listens on 6444.
static void upgrade_key_send(void) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a;
    const uint8_t msg[3] = { 0x40, 0x00, 0x01 };
    if (s < 0) return;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = htons(6444);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sendto(s, msg, sizeof(msg), 0, (struct sockaddr *)&a, sizeof(a));
    close(s);
}

static int key_read(int fd, char *c) {
    lseek(fd, 0, SEEK_SET);
    return (int)read(fd, c, 1);
}

static void *key_thread(void *arg) {
    (void)arg;
    char c;
    int fd = open(KEY_GPIO, O_RDONLY);
    if (fd <= 0) {
        puts("board: cannot open the key input");
        return NULL;
    }
    for (;;) {
        if (manu_trig) {
            puts("board: pairing asked for");
            do_match();
            usleep(10000);
            manu_trig = 0;
        }
        int n = key_read(fd, &c);
        if (n <= 0) { printf("board: key read failed (1, %d)\n", n); break; }
        if (c == '1') { usleep(20000); continue; }             // released
        if ((n = key_read(fd, &c)) <= 0) { puts("board: key read failed (2)"); break; }
        if (c == '1') continue;                                // a glitch
        uint32_t t0 = wall_ms();
        printf("board: key %c\n", c);
        for (;;) {
            if (key_read(fd, &c) <= 0) { puts("board: key read failed (3)"); usleep(30000); break; }
            uint32_t dt = wall_ms() - t0;
            if (c != '0') {                                    // released: pair
                printf("board: key %c held %u ms: pairing\n", c, dt);
                do_match();
                usleep(30000);
                break;
            }
            if (dt >= 8000) {                                  // held 8 s: upgrade
                upgrade_key_send();
                printf("board: key held %u ms: upgrade mode\n", dt);
                usleep(30000);
                break;
            }
            usleep(10000);
        }
    }
    close(fd);
    return NULL;
}

// ---- debug commands (UDP 4486) --------------------------------------------

static void *dbg_thread(void *arg) {
    (void)arg;
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a;
    uint8_t b[1024];
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = htons(4486);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (s < 0 || bind(s, (struct sockaddr *)&a, sizeof(a)) < 0) {
        perror("dbg cmd bind");
        return NULL;
    }
    for (;;) {
        int n = (int)recv(s, b, sizeof(b), 0);
        if (n <= 0) continue;
        switch (b[0]) {
        case 0x82:
            printf("board: mcs %d, throughput %d, connected %d, relay %d\n", radio_mcs(), radio_throughput(),
                   radio_connected(), radio_relay_mode());
            break;
        case 0x85:
            if (n >= 3) video_set_bitrate(b[1] | b[2] << 8);
            break;
        case 0x86:
            manu_trig = 1;
            break;
        default:
            printf("board: debug command 0x%02x not handled\n", b[0]);
            break;
        }
    }
    return NULL;
}

int board_start(void) {
    pthread_t t;
    if (pthread_create(&t, NULL, key_thread, NULL)) return -1;
    pthread_detach(t);
    if (pthread_create(&t, NULL, led_thread, NULL)) return -1;
    pthread_detach(t);
    if (pthread_create(&t, NULL, dbg_thread, NULL)) return -1;
    pthread_detach(t);
    return 0;
}
