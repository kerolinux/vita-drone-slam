# vita-drone-slam

Reverse-engineering a low-cost E4 Wi-Fi camera drone (official app: `WIFI_CAM`), controlling it from a PlayStation Vita, and experimenting with monocular visual SLAM using ORB-SLAM3.

The project combines drone protocol analysis, Python control, PS Vita homebrew, live camera streaming, and robotics perception. The longer-term goal is to build toward vision-assisted landing on a rotating platform.

[Watch the project demo](https://github.com/user-attachments/assets/ae998638-6149-4a7e-8dea-fff653482aef)

## Project status

- **Drone control:** decoded control packets and working Python keyboard and PS Vita control.
- **Live video:** drone camera stream displayed on the PC and PS Vita.
- **ORB-SLAM3:** live camera frames forwarded from Windows to an ORB-SLAM3 worker in WSL2. SLAM initialization and visualisation have been demonstrated under suitable lighting.
- **Camera calibration:** calibration parameters are configured; their accuracy still needs independent validation.
- **Autonomous landing:** planned. Downward-camera switching, AprilTag-based relative pose estimation, and rotating-platform landing have not yet been implemented.

## Features

- **Protocol reverse engineering:** control and video packet formats investigated using captures from the official app and the open-source `opendrone` viewer. See [`docs/protocol.md`](docs/protocol.md).
- **Python control library:** `python/drone.py` sends control packets at approximately 50 Hz and implements a staged failsafe.
- **Keyboard teleoperation:** `python/teleop.py` provides keyboard control and live video.
- **PS Vita controller:** `vita/` and `python/bridge.py` connect a Vita homebrew controller to the drone, with sticks, buttons, trim, telemetry, and camera video.
- **Camera and SLAM tools:** calibration, recording, dataset conversion, and live frame forwarding to ORB-SLAM3.
- **Live monocular SLAM:** C++ worker based on ORB-SLAM3, with trajectory output and an optional Pangolin visualiser.

## System overview

```text
                 Wi-Fi
PS Vita <----------------------> Router
                                    |
                                  Ethernet
                                    |
                                    v
                               Windows PC
                                    |
                          USB Wi-Fi adapter
                                    |
                                    v
                              E4 Wi-Fi drone
                                    |
                               Camera video
                                    |
                                    v
                          Python bridge (Windows)
                                    |
                             TCP frame stream
                                    |
                                    v
                               WSL2 Ubuntu
                                    |
                                    v
                               ORB-SLAM3
                                    |
                           Pose / trajectory output
```

The PC connects to the drone through a USB Wi-Fi adapter while remaining connected to the local network over Ethernet. The Vita joins the same local network. The bridge forwards camera frames to the SLAM worker running inside WSL2.

## Hardware

| Component | Details |
|---|---|
| Drone | E4 Wi-Fi camera drone, official app `WIFI_CAM` |
| Cameras | Front-facing and downward-facing cameras; switching protocol not yet decoded |
| Battery | Labelled 3.7 V, 800 mAh (2.96 Wh) |
| Controller | PS Vita running homebrew |
| PC | Windows 11 with WSL2 Ubuntu |
| Wireless connection | USB Wi-Fi adapter for the drone; Ethernet to the router |
| Calibration target | Printed checkerboard |
| Landing experiment | AprilTag and a proposed rotating platform |

## Software stack

- Python for drone communication, control, recording, and the Windows bridge.
- C and VitaSDK for the PS Vita homebrew application.
- C++ and ORB-SLAM3 for live monocular visual SLAM.
- OpenCV for image processing and camera calibration.
- WSL2 Ubuntu for the SLAM worker and its dependencies.
- Pangolin for optional SLAM visualisation.
- GitHub Actions for building the Vita application.

## Quick start

Create a Python virtual environment and install the project dependencies:

**Windows PowerShell**

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Run the available non-flight tests:

```powershell
python python\test_drone.py
```

Start keyboard teleoperation:

```powershell
python python\teleop.py
```

Start the PS Vita bridge:

```powershell
python python\bridge.py
```

Then launch the corresponding homebrew application on the Vita.

**Live SLAM setup:** the live SLAM pipeline additionally requires ORB-SLAM3, its vocabulary and compatible settings, a built C++ worker, and the WSL2 receiver. Consult [`docs/slam_walkthrough.md`](docs/slam_walkthrough.md) for the project's SLAM setup. Build and runtime commands may depend on your local paths and installed dependencies.

The Vita application is built by [GitHub Actions](.github/workflows/build-vita.yml). See [`vita/README.md`](vita/README.md) for installation instructions.

## Repository layout

```text
docs/           Protocol documentation and SLAM walkthrough
python/         Drone control, keyboard teleop, Vita bridge, tests and SLAM tools
cpp/            C++ live ORB-SLAM3 worker
vita/           PS Vita application source and build assets
tools/          Early protocol investigation scripts
.github/        GitHub Actions workflows
```

Recorded frames, trajectories, local build products, and machine-specific configuration belong in local development storage, not in version control.

## Development roadmap

### Control and integration

- [x] Capture and investigate official-app traffic
- [x] Document control packet format
- [x] Document video stream format
- [x] Control the drone from Python
- [x] Implement staged control failsafes
- [x] Display live video on the PC
- [x] Build the PS Vita controller app
- [x] Display camera video on the Vita
- [x] Validate the bridge failsafe with propellers removed

### Visual SLAM

- [x] Implement calibration and recording tools
- [x] Implement an ORB-SLAM3 sequence converter
- [x] Forward live camera frames from Windows to WSL2
- [x] Build the C++ live SLAM worker
- [x] Demonstrate live monocular SLAM initialization under suitable lighting
- [x] Validate calibration against a checkerboard dataset
- [ ] Evaluate tracking loss and recovery over repeated runs
- [ ] Compare trajectories against a measured reference path
- [ ] Measure latency and end-to-end frame rate
- [ ] Evaluate trajectory scale and accuracy

### Landing and autonomy

- [ ] Decode or otherwise establish a reliable front/bottom camera switching method
- [ ] Capture and validate downward-camera footage
- [ ] Implement AprilTag detection and relative pose estimation
- [ ] Estimate rotating-platform motion
- [ ] Test a bounded landing controller in a safe, staged setup
- [ ] Record a reproducible project demonstration

### Stretch goals

- [ ] ROS 2 integration (`/cmd_vel`, `/image_raw`)
- [ ] IMU logging and monocular-inertial SLAM investigation

## Protocol notes

The full packet descriptions, observations, and unresolved questions are documented in [`docs/protocol.md`](docs/protocol.md).

| Item | Current understanding |
|---|---|
| Drone address | `192.168.4.153` when connected to the drone's Wi-Fi network |
| Video start | Bytes `42 76` sent to UDP port `8080` |
| Video stream | MJPEG over UDP, with an 8-byte packet header |
| Control | UDP port `8090`; 20-byte packets at approximately 50 Hz |
| Control framing | Starts with `66 14`; ends with `99` |
| Checksum | XOR-based checksum with special handling for framing-byte values |
| Camera switching | Front/bottom switching protocol remains unresolved |
| Drone replies | Meaning of the 8-byte replies remains unresolved |

These are reverse-engineering findings for this particular drone and firmware, not a claim that all E4-style drones use the same protocol.

## Results so far

| Measurement | Result |
|---|---|
| Camera image size | 640 × 480 JPEG |
| Observed video rate | Approximately 8–10 complete frames per second in desk tests |
| Complete-frame rate | Approximately 73–84% across 12 five-second observation windows |
| Control packet rate | Approximately 50 packets per second |
| Flight controls | Takeoff, landing, climb, descent, and emergency-stop commands tested |
| SLAM input | Live frames successfully forwarded to the WSL2 worker |
| SLAM initialisation | Achieved under suitable lighting and visual-texture conditions |
| SLAM accuracy | Not yet quantitatively validated against a reference trajectory |
| Metric scale | Not yet independently validated |
| Landing performance | Not implemented or evaluated |
| End-to-end latency | Not yet measured |

SLAM performance is sensitive to illumination, image texture, motion, and camera calibration. Monocular SLAM also has an inherent scale ambiguity unless additional information constrains scale.

## Safety

- Remove the propellers while developing and testing new features.
- Do not assume the drone stops when communication is interrupted. The control software implements staged neutral-stick, landing, and emergency-stop behaviour, but the complete bridge failsafe still requires validation on the actual drone.
- Test flight behaviour only in a suitable, clear and controlled area after the relevant failsafes have been verified.
- Check the current UK CAA requirements before outdoor operation.
- Treat SLAM output as experimental until position accuracy, tracking-loss behaviour, and recovery conditions have been measured.

## Acknowledgements

- **opendrone** by EinarTheSad — useful reference material for investigating the drone video handshake and packet layout.
- **VitaSDK**, its samples, **VitaShell**, and the PS Vita homebrew community.
- **ORB-SLAM3** by UZ-SLAMLab, **OpenCV**, **Pangolin**, and **COLMAP**.
- **Wireshark** and **jadx** for investigating the official application's traffic and structure.

No code from the official `WIFI_CAM` application is included in this repository.

## License

MIT
