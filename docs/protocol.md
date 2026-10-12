# Drone protocol notes: E4 Drone (WIFI_CAM app)

Last updated: 12 Oct 2026 (after flying with the PS Vita). Everything here concerns my own drone. Each item is marked with how well it is confirmed (see [Verification status](#verification-status)).

## Summary

| Item | Value |
|---|---|
| Drone | "E4 Drone" (Temu), front + bottom camera, 3.7 V 800 mAh (2.96 Wh) battery |
| Official app | WIFI_CAM (package `com.tzh.wifi.wificam`) |
| Wi-Fi | The drone is the access point. The PC gets an address like `192.168.4.101`; the drone is the gateway at `192.168.4.153` |
| Transport | UDP only (the app also contains a TCP variant for drones at `192.168.4.151`; not mine) |
| Video | MJPEG, 640x480, UDP |
| Control | Short UDP packets with a header, a checksum and a trailer |
| My controller | PS Vita app talking to a PC bridge that drives the drone (see [Vita link](#vita-link)) |

## How this was worked out

1. **Video handshake:** taken from the open-source *opendrone* app by EinarTheSad (a small Android viewer for these drones), then re-implemented in Python and tested against my drone.
2. **Packet layouts:** read from the official app's decompiled Java code (jadx), written up here in my own words. No app code is stored in this repo.
3. **Ports and real traffic:** captured the official app running in BlueStacks on a PC joined to the drone's Wi-Fi (Wireshark, about 206 s: connect, lock button, several one-key pulses, stick movements, and a video stop/start that may have been a camera switch).
4. **Check:** ten packets built by `drone.py` (idle, 20-byte, 8-byte, flags, checksums) are compared byte for byte with captured packets in `drone.self_check()` and in the unit tests.

## Network and ports

| Purpose | Direction | UDP port |
|---|---|---|
| Video start / stop commands | PC to drone | 8080 |
| Video data and short replies | drone to PC | from 8080, to the port the commands came from |
| Idle and control packets | PC to drone | **8090** |

The PC uses local port 40238 (the opendrone default) for everything. My first control tests sent packets to 8080 by mistake and the drone ignored them; the capture showed 8090.

## Video protocol

- **Start:** send bytes `42 76` ("Bv") to `192.168.4.153:8080`. **Stop:** `42 77` ("Bw").
- **Replies:** right after the start command the drone sends a burst of 8-byte packets `55 00 02 00 00 00 02 99` (30 and 20 in two runs, all within 0.3 s). Their meaning is unknown. They stop after the burst.
- **Packet layout** (video data), 8-byte header then payload:

| Byte | Meaning |
|---|---|
| 0 | frame ID |
| 1 | `1` on the last packet of a frame |
| 2 | number of packets in the frame |
| 3 | unknown (unused by opendrone) |
| 4-6 | `54 5A 48` = "TZH" marker |
| 7 | unknown |

- **Frames:** payloads of one frame are concatenated; the JPEG sits between `FF D8` and `FF D9`.
- **Completeness rule used:** a frame is kept only if the packet count equals byte 2 and the JPEG decodes (opendrone also rejects JPEGs under 3000 or over 500000 bytes).
- **Stuck stream:** after one failed run the video stopped, and a power cycle of the drone fixed it. The official app stops and restarts the video (`Bw` then `Bv`, about 5 s apart) when frames stall; `drone.py` does the same.
- **Source of the handshake:** opendrone by EinarTheSad.

### Stream measurements (12 windows of 5 s, drone on the desk)

| Measure | Result |
|---|---|
| Resolution | 640x480 |
| Complete frame rate | about 8-10 fps (11.4 in the first window, then 8.1-9.8) |
| Complete frames | 73-84 % per window (the rest incomplete or abandoned) |
| Frame size | 14.7 KB in the first window, then about 20 KB (JPEG) |

Earlier debug run: 3854 video packets and 237 displayed JPEGs; packet loss is the reason about one frame in five is dropped. Note: an earlier draft said 77-84 %; the last window was 73 %, so the range is corrected here. Timestamps in recordings are PC arrival times, not camera capture times, so latency is unknown.

## Control protocol

### Modes (what the official app does)

1. **Idle:** until the user taps the lock button, the app sends the 8-byte idle packet about every 0.78 s.
2. **Control:** after the lock button it sends control packets continuously, about 50 per second (20-byte format) or about 25 per second (8-byte format).

### Common rules

- **Framing:** control packets start with `66`, end with `99`. The idle packet starts with `AA`, ends with `55`.
- **Stick bytes:** centre is `0x80`. In the app's "stay high" mode (on by default after unlock) the **neutral throttle is also `0x80`** (hover); above is assumed to be up, below down. My first tests used throttle 0 and the drone did not react.
- **Checksum:** XOR of the data bytes (see each format). If the result is `0x66` or `0x99` it is increased by one. The app applies the same rule to every stick byte.

### 20-byte format (used by the app when it first took control; what works on my drone)

| Byte | Meaning |
|---|---|
| 0 | `66` |
| 1 | `14` (20, the packet length) |
| 2 | roll |
| 3 | pitch |
| 4 | throttle |
| 5 | yaw |
| 6 | flags: bit 0 = one-key fly **and** land (same bit); bit 1 = emergency stop (from app code) |
| 7 | flags 2: bit 0 headless mode, bit 1 stay-high (from app code) |
| 8-17 | zero |
| 18 | checksum = XOR of bytes 2-17 |
| 19 | `99` |

Neutral packet: `66 14 80 80 80 80 00 00 00 00 00 00 00 00 00 00 00 00 00 99`.

### 8-byte format (seen later in the same capture, about 25 packets/s)

| Byte | Meaning |
|---|---|
| 0 | `66` |
| 1-4 | roll, pitch, throttle, yaw |
| 5 | flags: bit 0 fly, bit 1 land, bit 2 emergency stop, bit 3 rotate, bit 4 headless, bit 7 "checkout" (probably calibration or trim reset; unknown) |
| 6 | checksum = XOR of bytes 1-5 |
| 7 | `99` |

Neutral packet: `66 80 80 80 80 00 00 99`.

### Idle packet

`AA 80 80 00 80 00 80 55` (checksum = XOR of bytes 1-5).

### Timing

- One-key flags are held for about 1 s: 51 packets at 20 ms (20-byte format) or about 21 packets at 40 ms (8-byte format). The captured pulses lasted about 1.04 s.
### Stick ranges

The official app's speed button changes only the maximum deflection of its right stick (roll and pitch): the code suggests 40, 60 and 128 (default 40), probably offsets from the centre byte. It does not limit its left stick (throttle and yaw) the same way; in the capture the throttle byte went far above centre.

My first Vita build used the same small range on all four axes (20 at range 1). The right stick (roll, pitch) worked, but **throttle and yaw did nothing at 20**. Giving throttle and yaw larger ranges fixed it. The ranges my software uses are my own choices, not protocol values (largest offset from the centre byte `0x80`):

| Speed setting | Roll / pitch | Throttle / yaw |
|---|---|---|
| 1 | 20 | 40 |
| 2 | 40 | 60 |
| 3 | 70 | 90 |

Setting 2 felt "just right" in flight. Trim offsets (see below) are added on top and are limited to +-40.

## Verification status

| Item | Status |
|---|---|
| Control and idle on UDP 8090 | Confirmed in the capture; motors reacted when sent from my script |
| 20-byte layout, checksum, idle packet | Confirmed in the capture (byte-for-byte match); motors spun on my drone |
| Fly flag (20-byte) | Confirmed: the drone takes off (propellers fitted, in a safe space) |
| Motors stopped when my script finished | Confirmed, but **which command stopped them is not known** (land flag in the test, or the emergency stop sent at the end) |
| Land flag (20-byte, same bit as fly) | Confirmed: the drone lands. Same bit as fly; whether it acts as a toggle has not been tested |
| Emergency stop | Confirmed: the teleop's emergency stop cuts the motors. It sends both formats' emergency packets, so which flag does the work is unknown. The 20-byte flag (bit 1 of byte 6) comes from app code only |
| 8-byte layout, flags 1 / 2 / 4 | Seen in the capture; not tested on the drone |
| Stick directions (roll right, pitch forward, throttle up, yaw clockwise) | All four axes behave as expected in flight on the keyboard teleop and the Vita (roll and pitch from the start, throttle and yaw once their range was raised); not recorded axis by axis |
| Stick ranges | Throttle and yaw need more than +-20 to react; the presets in the table above work (setting 2 "just right"). The values are my own, not from the protocol |
| Why the app switches between the two formats | Unknown (it happened around the video stop/start and again later) |
| Trim (three offsets added to the roll, pitch and yaw bytes; the app has three trim sliders) | Implemented in `drone.py` (`set_trim`), the teleop keys and the Vita D-pad / L / R buttons, saved in `trim.json`; the effect on the drift is not recorded yet |
| Calibrate flag (the app's "checkout": bit 2 of byte 6 in the 20-byte format, bit 7 of byte 5 in the 8-byte format) | From app code only; probably a gyro calibration or trim reset; untested |
| Vita link (control, telemetry, camera picture) | Control and telemetry confirmed in flight; the camera picture appears on the Vita but at a low frame rate (not yet measured) |
| Bridge failsafe (no Vita packets: neutral at 0.3 s, land at 1.5 s, motor cut at 5 s) | Implemented and unit-tested; **not yet tested on the real drone** (props-off test: quit the Vita app while the motors spin) |
| Camera switch (front / bottom) | Not decoded; handled inside the app's native library |
| Meaning of the 8-byte replies `55 00 02 ...` | Unknown (burst after video start only) |

## Flight observations

- **10 Oct 2026, first flight with propellers fitted (safe space, keyboard teleop):** takeoff, landing, climbing and descending, and the emergency stop all work, and the controls feel right.
- **Drift:** the drone does not stay in place. It drifts around horizontally while hovering, so it has no position hold. Attitude is stabilised by the drone itself, but nothing holds its position, and no trim was being sent (all roll/pitch/yaw bytes were exactly centred).
- **Delay:** a slight delay was felt. Not measured. Likely contributors: video latency (JPEG over Wi-Fi) and the teleop's key-hold window (`--hold`, default 0.35 s). The control packets themselves are sent every 20 ms.
- **11-12 Oct 2026, flying with the Vita (controlled environment):** the right stick (roll, pitch) moved the drone; takeoff, land and emergency stop worked. The left stick (throttle, yaw) did nothing at the starting range of 20. After throttle and yaw got their own larger ranges, speed setting 2 was "just right". The camera picture shows on the Vita at a lower frame rate than the drone's own 8-10 fps.
- **Next:** gyro calibration on a level surface, then trim until it hovers still, then try a vision-based position hold.

## Vita link

The PS Vita app (`vita/`) talks to `bridge.py` on the PC, which drives the drone through `drone.py`. The Vita joins the home router and the PC is on the same router by Ethernet, while the PC's Wi-Fi dongle is joined to the drone. The exact byte layouts are in `vitalink.py` (Python) and `vita/src/protocol.h` (C), checked against each other by `test_vita_protocol.py`.

| Message | Direction | UDP port | Size | Rate |
|---|---|---|---|---|
| Control: sticks (signed bytes), buttons, speed setting, sequence number, checksum | Vita to PC | 9000 | 12 bytes | about 50 per second |
| Telemetry: mode, failsafe stage, emergency flag, video fps, trim, time since last command | PC to Vita | 9001 | 12 bytes | about 5 per second |
| Camera picture: rows of RGB565 pixels (the PC decodes the drone's JPEG and shrinks it, default 320x240 at up to 8 fps) | PC to Vita | 9001 | up to 1412 bytes | about 120 packets per frame |

- Buttons act on their rising edge only, and buttons already held when a session starts are ignored (except the emergency stop). Takeoff needs a 0.6 s hold and calibrate a 1 s hold on the Vita.
- The picture needs roughly 10 Mbit/s at the defaults; `--vita-size` and `--vita-fps` lower it.
- For SLAM the PC keeps the full-quality video: `bridge.py --record` saves every frame as `<seconds>.jpg`. See `docs/slam_walkthrough.md`.

## Safety findings

- **The drone does not stop by itself when packets stop.** After a test script ended without an explicit stop, I had to press the drone's power button. Control software must send a stop itself.
- `drone.py` runs a staged failsafe: no `set_sticks()`/`heartbeat()` call for 0.3 s gives neutral sticks, 1.5 s sends a land pulse, 5 s sends the emergency stop. `close()` always sends the emergency stop.
- The Vita bridge feeds the same failsafe: the Vita's control packets are the heartbeat, so a dead Vita, Wi-Fi drop or crashed app ends in neutral, land, then motor cut. This still needs a props-off test on the real drone.
- Props stay off for new tests until the stop is confirmed to work every time, and new flight tests happen in a controlled space at low range. Check CAA rules before flying outdoors.

## Open questions

1. Which command actually stops the motors on a 20-byte drone (land vs emergency)?
2. The best stick ranges per axis (current presets are my own choices) and whether yaw needs the same range as throttle.
3. How the camera switch works, and whether both cameras can stream at once.
4. What the drone's 8-byte replies encode (battery? state?). Next step: extract the drone-to-PC packets from the capture around the takeoff taps.
5. Why the app changes packet format, and whether the drone needs it.
6. Does the calibrate flag fix the drift? How much trim does the drone need, and does it change with battery level?
7. How large is the real delay (video versus control)? Test idea: film a running stopwatch with the drone's camera and compare it with the stopwatch itself on screen.
8. Why the picture on the Vita runs below the drone's frame rate: the drone's own rate and losses, the bridge's frame cap, or the Wi-Fi? (Compare the two numbers on the Vita screen: `drone video` and `picture`.)
9. Whether the drone accepts a second Wi-Fi client, which would allow a standalone mode with the Vita flying the drone directly (the failsafe above would then have to live on the Vita).

## Reproducing the capture

PC on the drone's Wi-Fi (Ethernet for internet), WIFI_CAM in BlueStacks, Wireshark on the Wi-Fi adapter with filter `ip.addr == 192.168.4.153 && udp`. Tap the lock button, then one-key fly, then land. Export with `tshark -T fields -e frame.time_relative -e udp.dstport -e udp.length -e udp.payload` (write the output as ASCII or UTF-8 so it can be read later). Capture files are not committed to this repo.