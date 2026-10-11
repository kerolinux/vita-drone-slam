/* Host-side check of protocol.h (compile with a normal gcc; see ../../test_vita_protocol.py).
 * Usage: protocol_test pack    -> prints the control message for a fixed example as hex
 *        protocol_test tel HEX -> parses a telemetry message given as 24 hex digits
 *        protocol_test axis    -> prints vd_axis() for every raw value 0..255 (normal and inverted) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/protocol.h"

int main(int argc, char **argv) {
    if (argc >= 2 && strcmp(argv[1], "pack") == 0) {
        uint8_t out[VD_MESSAGE_LEN];
        vd_pack_control(out, 7, 10, -10, 20, -20, VD_BTN_TAKEOFF | VD_BTN_CALIBRATE, 2);
        for (int i = 0; i < VD_MESSAGE_LEN; i++) printf("%02x", out[i]);
        printf("\n");
        return 0;
    }
    if (argc >= 3 && strcmp(argv[1], "tel") == 0) {
        uint8_t in[VD_MESSAGE_LEN];
        if (strlen(argv[2]) != 2 * VD_MESSAGE_LEN) return 2;
        for (int i = 0; i < VD_MESSAGE_LEN; i++) {
            unsigned v;
            sscanf(argv[2] + 2 * i, "%2x", &v);
            in[i] = (uint8_t)v;
        }
        VdTelemetry t;
        if (!vd_parse_telemetry(in, VD_MESSAGE_LEN, &t)) {
            printf("invalid\n");
            return 1;
        }
        printf("%u %u %u %u %d %d %d %u\n", t.mode, t.stage, t.emergency, t.fps, t.trim_roll, t.trim_pitch,
               t.trim_yaw, t.age_tenths);
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "axis") == 0) {
        for (int raw = 0; raw < 256; raw++) printf("%d %d %d\n", raw, vd_axis((uint8_t)raw, 0), vd_axis((uint8_t)raw, 1));
        return 0;
    }
    return 2;
}