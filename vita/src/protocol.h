/* protocol.h - message format between the Vita app and the PC bridge (bridge.py).
 * Pure C, no Vita APIs, so it can be tested on a normal computer (see test/protocol_test.c).
 * MUST stay in sync with vitalink.py. */
#ifndef VD_PROTOCOL_H
#define VD_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define VD_MESSAGE_LEN 12
#define VD_VIDEO_HEADER_LEN 12
#define VD_VIDEO_FLAG_LAST 0x01
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

/* Video message: PC -> Vita. A few rows of the camera picture as RGB565 (little endian), see vitalink.py. */
typedef struct {
    uint8_t frame_id;
    uint16_t width, height, row_start;
    uint8_t rows, flags;
} VdVideoHeader;

/* Returns 1 if the packet is valid (fills h and points *pixels at the pixel data), 0 otherwise. */
static inline int vd_parse_video(const uint8_t *in, int len, VdVideoHeader *h, const uint8_t **pixels) {
    if (len < VD_VIDEO_HEADER_LEN || in[0] != 'D' || in[1] != 'F' || in[2] != VD_VERSION) return 0;
    h->frame_id = in[3];
    h->width = (uint16_t)(in[4] | (in[5] << 8));
    h->height = (uint16_t)(in[6] | (in[7] << 8));
    h->row_start = (uint16_t)(in[8] | (in[9] << 8));
    h->rows = in[10];
    h->flags = in[11];
    if (h->rows < 1 || h->width < 1 || h->height < 1) return 0;
    if ((int)h->row_start + h->rows > h->height) return 0;
    if (len - VD_VIDEO_HEADER_LEN != (int)h->rows * h->width * 2) return 0;
    *pixels = in + VD_VIDEO_HEADER_LEN;
    return 1;
}

/* Largest whole-number zoom that fits a width x height picture into max_w x max_h; 0 if it does not fit. */
static inline int vd_video_scale(int width, int height, int max_w, int max_h) {
    if (width < 1 || height < 1) return 0;
    int sx = max_w / width, sy = max_h / height;
    return sx < sy ? sx : sy;
}

/* One RGB565 pixel -> the Vita framebuffer's 32-bit format (memory order R, G, B, A). */
static inline uint32_t vd_rgb565_to_abgr(uint16_t v) {
    uint32_t r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
    return 0xFF000000u | (b << 16) | (g << 8) | r;
}

/* Draw `rows` picture rows (first one is picture row row0) into a 32-bit framebuffer with `pitch` pixels per
 * line, enlarged by the whole number `scale`. The framebuffer is only written, never read (reading video
 * memory is slow). Returns 0 on success, -1 if anything would fall outside the picture or the screen. */
static inline int vd_blit_rows(uint32_t *fb, int pitch, int fb_w, int fb_h, const uint8_t *src, int width,
                               int height, int row0, int rows, int scale) {
    uint32_t line[1024];
    if (!fb || !src || scale < 1 || width < 1 || rows < 1 || row0 < 0) return -1;
    if (row0 + rows > height || width * scale > fb_w || width * scale > pitch || width * scale > 1024) return -1;
    if ((row0 + rows) * scale > fb_h) return -1;
    for (int r = 0; r < rows; r++) {
        const uint8_t *s = src + (size_t)r * width * 2;
        for (int x = 0; x < width; x++) {
            uint32_t px = vd_rgb565_to_abgr((uint16_t)(s[2 * x] | (s[2 * x + 1] << 8)));
            for (int dx = 0; dx < scale; dx++) line[x * scale + dx] = px;
        }
        uint32_t *dst = fb + (size_t)(row0 + r) * scale * pitch;
        for (int dy = 0; dy < scale; dy++) memcpy(dst + (size_t)dy * pitch, line, (size_t)width * scale * sizeof(uint32_t));
    }
    return 0;
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