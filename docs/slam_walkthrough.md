# SLAM walkthrough (monocular, E4 drone)

What is tested and what is not: the Python tools in this walkthrough (`capture_calib.py`, `calibrate.py`, `make_tum_sequence.py`, `bridge.py --record`) are tested on synthetic data. The ORB-SLAM3 build and run steps (step 7) are **not tested**; they are the usual recipe and may need small fixes.

## What to expect

One camera, 640x480 JPEG at about 8-10 fps with some lost frames, rolling shutter, and no depth. So:

- The map is **sparse** (feature points, not a surface) and the scale is **unknown**: the trajectory has the right shape but not the right size.
- Tracking may be lost during fast turns or in front of blank walls. That is normal; it is a result worth measuring, not a failure of the project.
- A good outcome for this hardware is a recognisable map and path from a slow, smooth flight in a room with plenty of texture.

## 1. Print the checkerboard

Make a checkerboard with **10 x 7 squares** (that is 9 x 6 inner corners) and **25 mm squares**: calib.io's pattern generator, "Checkerboard", A4 landscape, rows 7, columns 10, square size 25 mm. Print at **100 % scale** ("actual size", not "fit to page"), then measure one square with a ruler: it must be 25 mm. Glue it to something flat and rigid (a clipboard or a piece of card). A bent board ruins the calibration.

## 2. Take the calibration photos

Drone on the ground, props off, PC on the drone's Wi-Fi:

```
python python\capture_calib.py --cols 9 --rows 6
```

Hold the board in front of the camera. Corners are drawn on the picture when the board is found. Press **SPACE** to save (it only saves when the board is found) and **Q** to finish. Aim for 25 photos: near and far, tilted every way, and with the board in every corner and along every edge of the picture.

## 3. Calibrate

```
python python\calibrate.py --images calib --cols 9 --rows 6 --square 0.025
```

It prints the focal lengths, distortion and **reprojection error** (below 0.5 px is good, above 1.0 is poor), warns about problems, and writes `calibration.json`, `drone_orbslam3.yaml` (the ORB-SLAM3 settings file) and `undistort_preview.jpg` (original next to corrected: straight lines should look straight on the right). Re-take photos if there are warnings.

## 4. Record flights

Start the bridge with recording on and fly with the Vita:

```
python python\bridge.py --record
```

Frames are saved as `recordings\<date_time>\<seconds>.jpg`. Tips for SLAM:

- Fly **slowly and smoothly**; avoid fast yaw turns.
- Choose a room with texture (posters, shelves, books); avoid blank walls and shiny floors.
- Start with a gentle sideways move, not a straight climb, so the system can initialise.
- Keep each flight 30-60 seconds. Coming back to the start point helps loop closure.
- Mark a path on the floor with tape (for example a 2 m x 1 m rectangle) so you can judge the result.

## 5. Make an ORB-SLAM3 sequence

```
python python\make_tum_sequence.py recordings\<folder> sequences\flight1
```

This creates `sequences\flight1\rgb\` and `rgb.txt` in the format ORB-SLAM3's stock `mono_tum` example reads.

## 6. Quick check with COLMAP (optional)

COLMAP (colmap.github.io) has a Windows version. "Automatic reconstruction" on the frames in `sequences\flight1\rgb` gives camera positions and a sparse point cloud without any building. It is offline, not SLAM, but it tells you quickly whether your video has enough texture and motion.

## 7. ORB-SLAM3 in WSL2 (untested recipe)

ORB-SLAM3 needs **OpenCV 4.4 or newer**, Eigen, Pangolin and a Linux build. Ubuntu 22.04 has OpenCV 4.5, so use that.

1. PowerShell as administrator: `wsl --install -d Ubuntu-22.04`, then restart and open Ubuntu.
2. Packages:
   ```
   sudo apt update
   sudo apt install -y build-essential cmake git pkg-config libeigen3-dev libopencv-dev \
       libgl1-mesa-dev libglew-dev libepoxy-dev libwayland-dev libxkbcommon-dev wayland-protocols python3-dev
   ```
3. Pangolin:
   ```
   git clone --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git
   cd Pangolin && mkdir build && cd build && cmake .. && make -j4 && sudo make install && sudo ldconfig && cd ../..
   ```
4. ORB-SLAM3:
   ```
   git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
   cd ORB_SLAM3
   sed -i 's/++11/++14/g' CMakeLists.txt
   chmod +x build.sh && ./build.sh
   ```
   Common fixes: if `usleep` is undeclared, add `#include <unistd.h>` at the top of `Examples/Monocular/mono_tum.cc`. If the viewer window gives trouble in WSL2, change the last argument of `ORB_SLAM3::System SLAM(...)` in `mono_tum.cc` from `true` to `false` and rebuild.
5. Run (the Windows folders appear under `/mnt/c/`):
   ```
   ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt /mnt/c/.../drone_orbslam3.yaml /mnt/c/.../sequences/flight1
   ```
   It writes `KeyFrameTrajectory.txt` (keyframe positions, TUM format) in the folder you ran it from.

## 8. Judge the result

Because the scale is unknown, compare the **shape** of the trajectory with the taped path (after scaling it to fit), and report how often tracking was lost and how many keyframes were created. Later we can add a script that aligns the trajectory to the known path (a similarity transform) and reports the error in centimetres.