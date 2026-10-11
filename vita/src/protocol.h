/* protocol.h - message format between the Vita app and the PC bridge (bridge.py).
 * Pure C, no Vita APIs, so it can be tested on a normal computer (see test/protocol_test.c).
 * MUST stay in sync with vitalink.py. */
#ifndef VD_PROTOCOL_H
#define VD_PROTOCOL_H

#include <stdint.h>

#define VD_MESSAGE_LEN 12
#define VD_VERSION 1

enum {
    VD_BTN_TAKEOFF = 1u << 0,
    VD_BTN_LAND = 1u << 1,
    VD_BTN_EMERGENCY = 1u << 2,
    VD_BTN_CALIBRATE = 1u << 3,
    VD_BTN_TRIM_PITCH_UP = 1u << 4,
    VD_BTN_TRIM_PITCH_DOWN = 1u << 5,
    VD_BTN_TRIM_ROLL_LEFT = 1u << 6,
    VD_BTN_TRIM_ROLL_RIGHT = 1u << 7,
    VD_BTN_TRIM_YAW_LEFT = 1u << 8,
    VD_BTN_TRIM_YAW_RIGHT = 1u << 9,
    VD_BTN_TRIM_RESET = 1u << 10
};

typedef struct {
    uint8_t mode;        /* 0 idle, 1 control */
    uint8_t stage;       /* failsafe: 0 none, 1 neutral, 2 landing, 3 motor cut */
    uint8_t emergency;   /* 1 while an emergency stop is active */
    uint8_t fps;         /* video frames per second */
    int8_t trim_roll, trim_pitch, trim_yaw;
    uint8_t age_tenths;  /* time since the bridge last heard from the Vita, in 0.1 s */
} VdTelemetry;

static inline uint8_t vd_xor(const uint8_t *data, int n) {
    uint8_t c = 0;
    for (int i = 0; i < n; i++) c ^= data[i];
    return c;
}

/* Control message: Vita -> PC. Layout: 'V' 'D' version seq roll pitch throttle yaw buttons_lo buttons_hi speed checksum */
static inline void vd_pack_control(uint8_t out[VD_MESSAGE_LEN], uint8_t seq, int8_t roll, int8_t pitch,
                                   int8_t throttle, int8_t yaw, uint16_t buttons, uint8_t speed) {
    out[0] = 'V';
    out[1] = 'D';
    out[2] = VD_VERSION;
    out[3] = seq;
    out[4] = (uint8_t)roll;
    out[5] = (uint8_t)pitch;
    out[6] = (uint8_t)throttle;
    out[7] = (uint8_t)yaw;
    out[8] = (uint8_t)(buttons & 0xFF);
    out[9] = (uint8_t)(buttons >> 8);
    out[10] = speed;
    out[11] = vd_xor(out + 2, 9);
}

/* Telemetry message: PC -> Vita. Returns 1 if valid (and fills t), 0 otherwise. */
static inline int vd_parse_telemetry(const uint8_t *in, int len, VdTelemetry *t) {
    if (len != VD_MESSAGE_LEN || in[0] != 'D' || in[1] != 'V' || in[2] != VD_VERSION) return 0;
    if (vd_xor(in + 2, 9) != in[11]) return 0;
    t->mode = in[3];
    t->stage = in[4];
    t->emergency = in[5];
    t->fps = in[6];
    t->trim_roll = (int8_t)in[7];
    t->trim_pitch = (int8_t)in[8];
    t->trim_yaw = (int8_t)in[9];
    t->age_tenths = in[10];
    return 1;
}

/* Raw analog stick value (0..255, centre ~128) -> -127..127 with a dead zone of about 12 %. */
static inline int8_t vd_axis(uint8_t raw, int invert) {
    const int dz = 15;
    int v = (int)raw - 128;
    if (invert) v = -v;
    if (v > -dz && v < dz) return 0;
    int sign = v < 0 ? -1 : 1;
    int mag = (v < 0 ? -v : v) - dz;
    int out = mag * 127 / (127 - dz);
    if (out > 127) out = 127;
    return (int8_t)(sign * out);
}

#endif /* VD_PROTOCOL_H */