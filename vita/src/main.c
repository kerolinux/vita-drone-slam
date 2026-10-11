/* Vita Drone Controller
 *
 * Sends the Vita's sticks and buttons to bridge.py on a PC over UDP, and shows the bridge's telemetry.
 * See ../README.md and ../../vitalink.py for the message format.
 *
 * Controls
 *   left stick   up/down = throttle, left/right = yaw
 *   right stick  up/down = pitch,    left/right = roll
 *   TRIANGLE     hold 0.6 s = takeoff          CROSS   = land
 *   CIRCLE       = EMERGENCY STOP (motors cut)
 *   SQUARE       hold 1 s = calibrate (drone on a level surface, not flying)
 *   D-pad        up/down = pitch trim, left/right = roll trim
 *   L / R        yaw trim left / right         hold SELECT 0.8 s = reset trim
 *   START        cycle stick range 1 / 2 / 3 (starts at 1, the smallest)
 *   L + R + START + SELECT together = quit
 */
#include <psp2/ctrl.h>
#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/net/net.h>
#include <psp2/sysmodule.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debugScreen.h"
#include "protocol.h"

#define printf psvDebugScreenPrintf

#define DEFAULT_PC_IP "192.168.0.53"
#define CONFIG_PATH "ux0:data/vitadrone/config.txt" /* one line: the PC's IP address */
#define PC_PORT 9000
#define LOCAL_PORT 9001
#define NET_MEM_SIZE (1 * 1024 * 1024)
#define SEND_PERIOD_US 20000   /* 50 control packets per second */
#define DRAW_PERIOD_US 100000  /* 10 screen updates per second */
#define LINK_TIMEOUT_US 1500000

typedef struct {
    int down;
    uint64_t since;
} HoldButton;

/* 1 once the button has been held for need_us microseconds. */
static int held_for(HoldButton *h, int is_down, uint64_t now, uint64_t need_us) {
    if (!is_down) {
        h->down = 0;
        return 0;
    }
    if (!h->down) {
        h->down = 1;
        h->since = now;
    }
    return (now - h->since) >= need_us;
}

static void load_config(char *ip, size_t size) {
    snprintf(ip, size, "%s", DEFAULT_PC_IP);
    FILE *f = fopen(CONFIG_PATH, "r");
    if (!f) return;
    char line[64];
    if (fgets(line, sizeof line, f)) {
        size_t n = strlen(line);
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r' || line[n - 1] == ' ')) line[--n] = 0;
        if (n > 0) snprintf(ip, size, "%s", line);
    }
    fclose(f);
}

/* Returns the UDP socket (>= 0) or a negative error code. */
static int net_setup(const char *pc_ip, SceNetSockaddrIn *pc) {
    sceSysmoduleLoadModule(SCE_SYSMODULE_NET);

    SceNetInitParam param;
    param.memory = malloc(NET_MEM_SIZE);
    if (!param.memory) return -1;
    memset(param.memory, 0, NET_MEM_SIZE);
    param.size = NET_MEM_SIZE;
    param.flags = 0;
    if (sceNetInit(&param) < 0) return -2;

    int sock = sceNetSocket("vitadrone", SCE_NET_AF_INET, SCE_NET_SOCK_DGRAM, 0);
    if (sock < 0) return -3;

    SceNetSockaddrIn local;
    memset(&local, 0, sizeof local);
    local.sin_family = SCE_NET_AF_INET;
    local.sin_addr.s_addr = sceNetHtonl(SCE_NET_INADDR_ANY);
    local.sin_port = sceNetHtons(LOCAL_PORT);
    if (sceNetBind(sock, (SceNetSockaddr *)&local, sizeof local) < 0) return -4;

    int nonblocking = 1;
    sceNetSetsockopt(sock, SCE_NET_SOL_SOCKET, SCE_NET_SO_NBIO, &nonblocking, sizeof nonblocking);

    memset(pc, 0, sizeof *pc);
    pc->sin_family = SCE_NET_AF_INET;
    pc->sin_port = sceNetHtons(PC_PORT);
    if (sceNetInetPton(SCE_NET_AF_INET, pc_ip, &pc->sin_addr) < 0) return -5;
    return sock;
}

static void send_control(int sock, const SceNetSockaddrIn *pc, uint8_t seq, int8_t roll, int8_t pitch,
                         int8_t throttle, int8_t yaw, uint16_t buttons, uint8_t speed) {
    uint8_t msg[VD_MESSAGE_LEN];
    vd_pack_control(msg, seq, roll, pitch, throttle, yaw, buttons, speed);
    sceNetSendto(sock, msg, sizeof msg, 0, (const SceNetSockaddr *)pc, sizeof *pc);
}

static const char *stage_name(uint8_t stage) {
    static const char *names[] = {"none", "neutral sticks", "LANDING", "MOTOR CUT"};
    return stage < 4 ? names[stage] : "?";
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    psvDebugScreenInit();
    printf("\e[2J\e[1;1H");

    char pc_ip[32];
    load_config(pc_ip, sizeof pc_ip);
    sceCtrlSetSamplingMode(SCE_CTRL_MODE_ANALOG);

    SceNetSockaddrIn pc;
    int sock = net_setup(pc_ip, &pc);
    if (sock < 0) {
        printf("Network setup failed (error %d). Is Wi-Fi on?\n", sock);
        sceKernelDelayThread(5 * 1000 * 1000);
        sceKernelExitProcess(0);
        return 0;
    }

    HoldButton hold_takeoff = {0, 0}, hold_calibrate = {0, 0}, hold_reset = {0, 0};
    VdTelemetry tel;
    memset(&tel, 0, sizeof tel);
    uint64_t last_tel = 0;
    int have_tel = 0;
    uint8_t seq = 0, speed = 1;
    uint32_t prev_pad = 0;
    unsigned long sent = 0;
    int8_t roll = 0, pitch = 0, throttle = 0, yaw = 0;
    uint64_t next_send = 0, next_draw = 0;
    const uint32_t quit_combo = SCE_CTRL_START | SCE_CTRL_SELECT | SCE_CTRL_LTRIGGER | SCE_CTRL_RTRIGGER;

    for (;;) {
        SceCtrlData pad;
        sceCtrlPeekBufferPositive(0, &pad, 1);
        uint64_t now = sceKernelGetProcessTimeWide();

        if ((pad.buttons & quit_combo) == quit_combo) break;
        if ((pad.buttons & SCE_CTRL_START) && !(prev_pad & SCE_CTRL_START)) speed = (uint8_t)(speed % 3 + 1);
        prev_pad = pad.buttons;

        if (now >= next_send) {
            next_send = now + SEND_PERIOD_US;
            roll = vd_axis(pad.rx, 0);
            pitch = vd_axis(pad.ry, 1);     /* stick up = forward */
            throttle = vd_axis(pad.ly, 1);  /* stick up = climb */
            yaw = vd_axis(pad.lx, 0);

            uint16_t buttons = 0;
            if (held_for(&hold_takeoff, (pad.buttons & SCE_CTRL_TRIANGLE) != 0, now, 600000)) buttons |= VD_BTN_TAKEOFF;
            if (pad.buttons & SCE_CTRL_CROSS) buttons |= VD_BTN_LAND;
            if (pad.buttons & SCE_CTRL_CIRCLE) buttons |= VD_BTN_EMERGENCY;
            if (held_for(&hold_calibrate, (pad.buttons & SCE_CTRL_SQUARE) != 0, now, 1000000)) buttons |= VD_BTN_CALIBRATE;
            if (pad.buttons & SCE_CTRL_UP) buttons |= VD_BTN_TRIM_PITCH_UP;
            if (pad.buttons & SCE_CTRL_DOWN) buttons |= VD_BTN_TRIM_PITCH_DOWN;
            if (pad.buttons & SCE_CTRL_LEFT) buttons |= VD_BTN_TRIM_ROLL_LEFT;
            if (pad.buttons & SCE_CTRL_RIGHT) buttons |= VD_BTN_TRIM_ROLL_RIGHT;
            if (pad.buttons & SCE_CTRL_LTRIGGER) buttons |= VD_BTN_TRIM_YAW_LEFT;
            if (pad.buttons & SCE_CTRL_RTRIGGER) buttons |= VD_BTN_TRIM_YAW_RIGHT;
            if (held_for(&hold_reset, (pad.buttons & SCE_CTRL_SELECT) != 0, now, 800000)) buttons |= VD_BTN_TRIM_RESET;

            send_control(sock, &pc, seq++, roll, pitch, throttle, yaw, buttons, speed);
            sent++;
        }

        /* Telemetry from the bridge (non-blocking socket: returns <= 0 when nothing is waiting). */
        for (;;) {
            uint8_t buf[64];
            SceNetSockaddrIn from;
            unsigned int fromlen = sizeof from;
            int n = sceNetRecvfrom(sock, buf, sizeof buf, 0, (SceNetSockaddr *)&from, &fromlen);
            if (n <= 0) break;
            VdTelemetry t;
            if (vd_parse_telemetry(buf, n, &t)) {
                tel = t;
                have_tel = 1;
                last_tel = now;
            }
        }

        if (now >= next_draw) {
            next_draw = now + DRAW_PERIOD_US;
            int link_ok = have_tel && (now - last_tel) < LINK_TIMEOUT_US;
            printf("\e[1;1HVITA DRONE CONTROLLER\e[K\n");
            printf("PC %s:%d   %s\e[K\n", pc_ip, PC_PORT,
                   link_ok ? "BRIDGE LINK OK" : "\e[91mNO ANSWER FROM BRIDGE\e[m");
            if (link_ok) {
                printf("mode %s   failsafe: %s   emergency: %s\e[K\n", tel.mode ? "control" : "idle",
                       stage_name(tel.stage), tel.emergency ? "\e[91mYES\e[m" : "no");
                printf("video %u fps   last command %.1f s ago\e[K\n", (unsigned)tel.fps, tel.age_tenths / 10.0);
                printf("trim  R %+d  P %+d  Y %+d\e[K\n", tel.trim_roll, tel.trim_pitch, tel.trim_yaw);
            } else {
                printf("\e[K\n\e[K\n\e[K\n");
            }
            printf("sticks  R %+4d  P %+4d  T %+4d  Y %+4d   range %u\e[K\n", roll, pitch, throttle, yaw,
                   (unsigned)speed);
            printf("packets sent: %lu\e[K\n\e[K\n", sent);
            printf("TRIANGLE hold = takeoff    CROSS = land    CIRCLE = EMERGENCY STOP\e[K\n");
            printf("SQUARE hold = calibrate (on the ground)    START = range 1/2/3\e[K\n");
            printf("D-pad = pitch/roll trim    L / R = yaw trim    SELECT hold = reset trim\e[K\n");
            printf("L + R + START + SELECT = quit\e[K\n");
        }

        sceKernelDelayThread(5000);
    }

    /* Leave the drone neutral; the bridge failsafe handles anything after that. */
    for (int i = 0; i < 10; i++) {
        send_control(sock, &pc, seq++, 0, 0, 0, 0, 0, speed);
        sceKernelDelayThread(SEND_PERIOD_US);
    }
    sceNetSocketClose(sock);
    sceNetTerm();
    sceKernelExitProcess(0);
    return 0;
}