# vita-drone-slam

Reverse-engineering a cheap Wi-Fi camera drone (E4 Drone, app: WIFI_CAM), flying it from a PS Vita, and running monocular visual SLAM on its video stream.

> **Status:** the drone's control and video protocol is decoded, and it flies from a Python keyboard teleop and from a PS Vita app that also shows the camera picture. Camera calibration and SLAM are next (the tools are written and tested on synthetic data, not yet on real flights).

https://github.com/user-attachments/assets/efbb8a70-ea3d-4095-aed1-51404e11730f

## What it does

- **Protocol:** control and video packets worked out from my own captures of the official app, plus the open-source *opendrone* viewer for the video handshake. Written up in [`docs/protocol.md`](docs/protocol.md).
- **Control library** (`python/drone.py`): sends the control packets at 50 Hz, receives the video, and runs a staged failsafe (neutral sticks, then land, then motor cut) because the drone does **not** stop by itself when packets stop.
- **Keyboard teleop** (`python/teleop.py`): live video, trim, recording.
- **PS Vita controller** (`vita/`, `python/bridge.py`): a homebrew app with the Vita's sticks and buttons, telemetry, and the drone's camera picture on the Vita screen.
- **SLAM tools** (`python/calibrate.py`, `capture_calib.py`, `recorder.py`, `make_tum_sequence.py`): camera calibration, flight recording and an ORB-SLAM3 input converter. Walkthrough in [`docs/slam_walkthrough.md`](docs/slam_walkthrough.md).

## System overview

```
PS Vita --Wi-Fi--> router --Ethernet--> PC (bridge.py) --USB Wi-Fi dongle--> drone
                                          |
                                          +-- video: back to the Vita screen, saved flights for calibration and SLAM
```

The drone's Wi-Fi is joined by the PC's USB dongle, while the PC stays on the router by Ethernet. The Vita joins the same router, so no second Wi-Fi radio is needed.

## Hardware

| Item | Notes |
|---|---|
| E4 Drone (app: WIFI_CAM) | Front and bottom camera, 3.7 V 800 mAh (2.96 Wh) battery. Weight: _TBD_ |
| PS Vita | Running homebrew (HENkaku), app built with VitaSDK |
| PC | Ethernet to the router, USB Wi-Fi dongle for the drone |
| Printed checkerboard and AprilTag | For camera calibration and tag experiments |

## Quick start

```
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt

python python\test_drone.py           # tests that need no drone
python python\teleop.py               # keyboard teleop with live video (props off first!)
python python\bridge.py               # Vita bridge; then start the app on the Vita
```

The Vita app is built by GitHub Actions (`.github/workflows/build-vita.yml`); see [`vita/README.md`](vita/README.md) for installing it.

## Repository layout

```
docs/           protocol notes and the SLAM walkthrough
python/         control library, teleop, Vita bridge, calibration and SLAM tools, tests
vita/           PS Vita app (C, VitaSDK), icon and LiveArea files
tools/          early test scripts used while decoding the protocol
.github/        build workflow for the Vita app
```

## Roadmap

- [ ] Weigh drone and note specs
- [x] Capture app traffic (Wireshark + BlueStacks)
- [x] Document control packet format
- [x] Document video stream format
- [x] Control the drone from Python (keyboard)
- [x] Add failsafes (deadman timeout, throttle limit)
- [x] Display live video in OpenCV
- [x] Vita controller app (sticks, buttons, trim, telemetry)
- [x] Camera picture on the Vita
- [x] SLAM tools: calibration, flight recording, ORB-SLAM3 sequence converter
- [ ] Test the bridge failsafe on the real drone (props off)
- [ ] Calibrate camera (checkerboard)
- [ ] Run ORB-SLAM3 offline on recorded flights
- [ ] Live SLAM while flying with the Vita
- [ ] Evaluate against a known floor path
- [ ] Bottom camera: AprilTag detection / landing
- [ ] Demo video of the Vita flying the drone
- [ ] Stretch: ROS 2 wrapper (/cmd_vel, /image_raw)
- [ ] Stretch: IMU logger and monocular-inertial SLAM

## Protocol notes

Short version; the full byte-level notes, how each item was checked, and what is still unknown are in [`docs/protocol.md`](docs/protocol.md).

| | |
|---|---|
| Drone address | `192.168.4.153` (the drone is the Wi-Fi access point) |
| Video | Start with the bytes `42 76` to UDP port 8080. The drone sends MJPEG as UDP packets with an 8-byte header; frames are the JPEGs between `FF D8` and `FF D9` |
| Control | UDP port 8090. 20-byte packets about 50 times a second: `66 14`, roll, pitch, throttle, yaw, flags, padding, checksum, `99` |
| Checksum | XOR of the data bytes, nudged by one if it would equal a framing byte (`66` or `99`) |
| Idle packet | `AA 80 80 00 80 00 80 55` about every 0.78 s before control starts |
| Centre values | `0x80` on every stick (including throttle in the app's "stay high" mode) |
| Takeoff / land / emergency | Flag bits held for about one second |
| Not decoded | Camera switch (front / bottom), the meaning of the drone's 8-byte replies |

## Results

Measured so far (more to come as calibration and SLAM are run):

| Measurement | Result |
|---|---|
| Video stream | 640x480 JPEG; about 8-10 complete frames per second; 73-84 % of frames arrive complete (12 windows of 5 s, drone on a desk) |
| Control rate | about 50 packets per second (20-byte format) |
| Flight | Takeoff, landing, climb, descent and emergency stop confirmed with propellers fitted, from the keyboard and from the Vita |
| Stick ranges | Throttle and yaw needed larger ranges than roll and pitch to react (see `docs/protocol.md`) |
| Drift | The drone drifts around while hovering: it has no position hold. Effect of trim and calibration: _not yet recorded_ |
| Automated tests | 63 tests, none needing the drone (21 control library, 15 Vita bridge, 9 video, 8 C-versus-Python messages, 10 SLAM tools) |
| Camera calibration | _TBD_ (reprojection error, focal length) |
| SLAM | _TBD_ (tracking-loss rate, trajectory against the taped floor path) |
| Latency (video, control) | _not measured yet_ |
| Bridge failsafe on the real drone | _props-off test not done yet_ |

What failed or surprised me is written down in `docs/protocol.md`: control packets sent to the wrong port were ignored, throttle 0 meant "stick down", and a script that just stopped sending left the drone running.

## Safety

- Test with the propellers removed until a new feature is confirmed.
- The drone does not stop by itself when packets stop. The software therefore sends the stop itself: sticks go neutral after 0.3 s without commands, a land command after 1.5 s, and an emergency motor cut after 5 s. `close()` always sends the emergency stop.
- Fly in a clear, controlled space, indoors, with a net if you can.
- Check the current UK CAA rules before flying outdoors.

## Acknowledgements

- **opendrone** by EinarTheSad: the small open-source viewer for these drones that gave me the video start command and packet layout.
- **VitaSDK** and the **vitasdk samples** (the `debugScreen` helper), **VitaShell** and the PS Vita homebrew community.
- **ORB-SLAM3** (UZ-SLAMLab), **OpenCV** and **COLMAP** for the SLAM side.
- **Wireshark** and **jadx**: used to study the official WIFI_CAM app's traffic and structure so I could talk to my own drone. No code from the official app is included in this repository.

## License

MIT