
# vita-drone-slam

Reverse-engineering and developing a low-cost E4 Wi-Fi camera drone for PS Vita teleoperation, monocular visual SLAM, AprilTag-based landing, and autonomous flight experiments.

The long-term goal is to enable the drone to locate a visual landing target and land on a rotating platform using onboard camera feedback and a vision-guided control system.

## Demo

[Watch the project demonstration](https://github.com/user-attachments/assets/ae998638-6149-4a7e-8dea-fff653482aef)

## Project Status

| Component | Status |
|---|---|
| Drone communication and control | Implemented |
| PS Vita teleoperation | Implemented |
| Wi-Fi video reception | Implemented |
| ORB-SLAM3 integration | Working experimentally |
| Camera calibration | Available; further validation required |
| AprilTag detection and pose estimation | Implemented experimentally |
| Autonomous landing controller | Initial implementation; requires flight validation |
| Stable vision-guided landing | Not yet validated |
| Rotating-platform landing | Future validation target |

**Current limitation:** The drone can drift significantly during landing attempts. Reliable target-relative positioning, controller tuning, and controlled flight testing are still required before autonomous landing can be considered successful.

## Features

- Reverse-engineered the drone's Wi-Fi communication protocol.
- Python-based drone control and telemetry.
- PS Vita-based manual teleoperation.
- Live video reception from the drone's Wi-Fi camera.
- ORB-SLAM3 integration for monocular visual SLAM.
- Camera calibration and image-processing tools.
- AprilTag detection and pose-estimation experiments.
- Initial autonomous landing controller.
- Experimental separation between manual teleoperation and autonomous control.
- Documentation and tests for landing-related functionality.

## Hardware

- **Drone:** E4 Wi-Fi camera drone.
- **Cameras:** Front-facing camera and downward-facing camera.
- **Controller:** Sony PS Vita.
- **Computer:** Windows 11 with WSL2 and Ubuntu.
- **Networking:** Wi-Fi adapter and Ethernet connection where required.
- **Calibration:** Checkerboard calibration target.
- **Landing target:** Printed AprilTag.
- **Future test environment:** Controlled landing area and, eventually landing while rotating on platform.

The drone uses a 3.7 V, 800 mAh battery. Camera-switching behaviour remains under investigation.

## Software Stack

- Python
- C and VitaSDK
- C++ and ORB-SLAM3
- OpenCV
- AprilTag detection and pose estimation
- Ubuntu on WSL2
- Pangolin
- Wireshark
- Git and GitHub Actions

## Repository Structure

```text
vita-drone-slam/
├── docs/
│   ├── autonomous_landing.md
│   └── slam_walkthrough.md
├── python/
│   ├── drone.py
│   ├── teleop.py
│   ├── bridge.py
│   ├── apriltag_detect.py
│   ├── apriltag_detect_bottom.py
│   ├── autonomous_land.py
│   └── test_autonomous_land.py
├── cpp/
├── vita/
├── tools/
├── requirements.txt
└── README.md
```

Additional calibration scripts, datasets, logs, and local development files may be present in the working directory but are not necessarily tracked in Git.

## Getting Started

### 1. Set up Python

Run the following commands in PowerShell from the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

If PowerShell blocks virtual-environment activation, use an appropriate execution policy for your environment or run the environment's Python executable directly.

### 2. Test drone communication

```powershell
python python\test_drone.py
```

### 3. Start manual teleoperation

```powershell
python python\teleop.py
```

### 4. Start the bridge

```powershell
python python\bridge.py
```

Follow the relevant documentation for the PS Vita connection and network configuration.

## Visual SLAM

The project integrates ORB-SLAM3 with the drone's live camera feed. Frames are forwarded from the Windows-side video receiver to the SLAM environment running under WSL2.

See [`docs/slam_walkthrough.md`](docs/slam_walkthrough.md) for setup and execution instructions.

The current SLAM implementation is experimental. Tracking quality depends on lighting, scene texture, camera motion, calibration, and initialization. Monocular SLAM does not inherently provide absolute metric scale.

## AprilTag Detection and Autonomous Landing

The landing experiments use AprilTags as visual references for estimating the target's relative pose.

Relevant files:

- `python/apriltag_detect.py` — AprilTag detection using the available camera feed.
- `python/apriltag_detect_bottom.py` — detection experiments for the downward-facing camera.
- `python/autonomous_land.py` — initial autonomous landing controller.
- `python/test_autonomous_land.py` — tests for landing-controller functionality.
- `docs/autonomous_landing.md` — landing experiment documentation.

The intended workflow is:

1. Receive camera frames from the drone.
2. Detect an AprilTag in the image.
3. Estimate the tag's relative pose using camera calibration.
4. Use visual feedback to guide the drone toward the target.
5. Reduce motion and descend only when the target-relative estimate is sufficiently reliable.

This is an experimental workflow, not a claim of reliable autonomous flight. Detection alone does not guarantee accurate distance estimation or stable control. Camera calibration, tag visibility, coordinate-frame conventions, control latency, and the drone's flight dynamics all affect performance.

**Known issue:** The drone has shown significant drift during landing attempts. The controller must be validated in a controlled environment before further autonomous flight testing.

### AprilTag Detection Demo

This video demonstrates real-time AprilTag detection using the drone's camera feed, as an intermediate milestone toward vision-guided autonomous landing.

**Demonstrated**
- Real-time visual marker detection.
- AprilTag identification in the camera feed.

**In development**
- Reliable target-relative pose estimation.
- Drift compensation and stable flight control.
- Autonomous landing and rotating-platform landing experiments.


https://github.com/user-attachments/assets/0a1ff7c5-a228-4d12-a979-7ccd8fa6eccf




For additional details, see [`docs/autonomous_landing.md`](docs/autonomous_landing.md).

## Development and Testing

Run the landing-controller tests from the repository root:

```powershell
python -m pytest python\test_autonomous_land.py -v
```

These tests are intended to check controller behaviour in software. Passing software tests does not establish that the drone can land safely in real flight.

Before committing changes, inspect the staged and unstaged files:

```powershell
git status
git diff
git diff --cached
```

Avoid committing virtual environments, generated logs, raw datasets, or local calibration outputs unless they are deliberately part of the project.

## Reverse-Engineered Protocol Notes

The following observations were recorded during development. They describe the current findings, not a complete specification of the drone's protocol.

| Item | Observation |
|---|---|
| Drone address | `192.168.4.153` |
| Video port | UDP `8080` |
| Video start command | `42 76` |
| Video stop command | `42 77` |
| Video format | MJPEG over UDP, with an 8-byte packet header |
| Control port | UDP `8090` |
| Control packet framing | `66 14` prefix and `99` terminator |
| Control checksum | XOR-based checksum observed |
| Camera switching | Not fully decoded |
| Additional 8-byte replies | Not fully understood |

The control stream has been observed at approximately 50 packets per second. Further protocol analysis may be required to understand all packet types and camera-switching behaviour.

## Experimental Results

Observed results from development testing include:

- Camera frames at 640 × 480 resolution.
- Approximately 8–10 complete video frames per second in earlier capture tests.
- Complete-frame rates of approximately 73–84% across twelve five-second measurement windows.
- Control packets observed at approximately 50 packets per second.
- Manual flight controls tested.
- Live camera frames forwarded to the ORB-SLAM3 pipeline.
- SLAM initialization achieved under suitable lighting and scene conditions.

The following have **not** been established through controlled measurement:

- Absolute SLAM accuracy or metric scale.
- End-to-end vision-to-control latency.
- Reliable AprilTag-based distance estimation throughout a landing approach.
- Stable autonomous landing performance.
- Successful landing while rotating on platform.

These figures are development observations, not guaranteed performance specifications.

## Roadmap

### Drone Control and Networking

- [x] Reverse-engineer core video and control communication.
- [x] Implement Python-based drone control.
- [x] Establish manual teleoperation.
- [x] Receive and process the live camera feed.
- [x] Fully decode camera switching.
- [ ] Improve protocol documentation and reliability testing.

### Visual SLAM

- [x] Integrate ORB-SLAM3.
- [x] Forward live camera frames to the SLAM pipeline.
- [ ] Improve calibration and validate tracking accuracy.
- [ ] Measure tracking stability under different lighting and motion conditions.
- [ ] Evaluate end-to-end latency.

### AprilTag Landing

- [x] Implement initial AprilTag detection.
- [x] Implement initial pose-estimation and landing-controller functionality.
- [x] Validate downward-camera detection during flight.
- [ ] Validate pose estimates at different heights and viewing angles.
- [ ] Reduce drift and improve target-relative control.
- [ ] Implement and validate conservative approach and descent behaviour.
- [ ] Measure landing accuracy and repeatability.
- [x] Test landing while rotating on a platform in a controlled environment.
- [ ] Record a repeatable end-to-end demonstration.

## Safety

This project involves an experimental aircraft and unvalidated autonomous control.

- Remove the propellers during bench testing whenever possible.
- Test flight-control changes in a clear, controlled area.
- Keep manual control and a reliable means of stopping the experiment available.
- Do not assume software tests prove that flight is safe.
- Validate emergency-stop, failsafe, and landing behaviour before autonomous flight.
- Check applicable UK Civil Aviation Authority rules before flying.
- Do not test autonomous landing near people, animals, roads, or fragile objects.

The landing controller is experimental and must not be treated as a safety-certified system.

## Acknowledgements

This project builds on open-source tools and community work, including:

- [opendrone](https://github.com/EinarTheSad/opendrone) by EinarTheSad.
- [VitaSDK](https://vitasdk.org/) and [VitaShell](https://github.com/TheOfficialFloW/VitaShell).
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3).
- [OpenCV](https://opencv.org/).
- [Pangolin](https://github.com/stevenlovegrove/Pangolin).
- [Wireshark](https://www.wireshark.org/) and other protocol-analysis tools.
- [JADX](https://github.com/skylot/jadx) for Android application decompilation and inspection.

No official `WIFI_CAM` application source code is included in this repository.

## License

See the repository's `LICENSE` file for licensing terms.
