# vita-drone-slam

Reverse-engineering a cheap Wi-Fi camera drone (E4 Drone, app: WIFI_CAM), flying it from a PS Vita, and running monocular visual SLAM on its video stream.


https://github.com/user-attachments/assets/efbb8a70-ea3d-4095-aed1-51404e11730f


> **Status:** early stage. See the roadmap below for what works so far.

<!-- Add a demo GIF or video link here once you have one -->

## Goals

1. Decode the drone's control and video protocol from packet captures.
2. Control it from Python, then from a PS Vita homebrew app via a ROS 2 bridge.
3. Run monocular SLAM (ORB-SLAM3) on the front-camera stream and evaluate it against a known path.
4. Extension: AprilTag detection and position hold or landing with the bottom camera.
5. Stretch: add an IMU logger for monocular-inertial SLAM.

## System overview

```
Vita --(UDP, Wi-Fi A)--> Laptop/Pi bridge --(UDP, Wi-Fi B)--> Drone
                              |
                              +-- video -> OpenCV -> ORB-SLAM3 -> map + pose
```

The drone's Wi-Fi accepts one client, so a bridge machine with two Wi-Fi interfaces joins the drone on one and hosts a hotspot for the Vita on the other.

## Hardware

| Item | Notes | Cost |
|---|---|---|
| E4 Drone (app: WIFI_CAM) | Front and bottom camera, 3.7 V 800 mAh battery | already owned |
| PS Vita | Homebrew via VitaSDK | already owned |
| Laptop | Runs the bridge, SLAM and tools | already owned |
| USB Wi-Fi dongle | Second interface for the bridge | ~£5 |

Measured drone weight: _TBD_

## Repository layout (planned)

```
docs/          protocol notes, packet captures, calibration results
captures/      .pcap files and annotated notes
python/        drone control and video scripts
bridge/        ROS 2 bridge (/cmd_vel, /image_raw)
vita/          VitaSDK controller app
slam/          calibration and ORB-SLAM3 configs
eval/          trajectory evaluation scripts and results
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

_Fill in as you decode it: destination IP and port, control packet length and byte layout, how video starts, packet format, camera switching._

## Results

_Add calibration error, trajectory error vs ground truth, tracking-loss rate, and what failed._

## Safety

- Test with props removed until control is confirmed.
- Control code must send a neutral or land command if input stops.
- Fly indoors with a net or in a clear space.
- Check current UK CAA rules before flying outdoors.

## Acknowledgements

_Link any projects or write-ups you used as reference._

## License

MIT
