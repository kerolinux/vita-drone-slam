# Vita Drone Controller

A PS Vita homebrew app that sends the Vita's sticks and buttons to `bridge.py` on your PC over Wi-Fi, shows the bridge's telemetry, and shows the drone's camera picture on the Vita screen.

```
Vita --Wi-Fi--> home router --Ethernet--> PC (bridge.py) --Wi-Fi dongle--> drone
```

The PC keeps its Ethernet connection to the router and uses the Wi-Fi dongle for the drone, so no second Wi-Fi radio is needed.

> **Status:** the controls and telemetry run on a real Vita. The camera picture (version 2) and the app icon are new: the PC side and the message and drawing code in `src/protocol.h` are tested on a PC, and `src/main.c` compiles cleanly against the real Vita headers, but the picture and icon have **not been run on a Vita yet**.

## Controls

| Input | Action |
|---|---|
| Left stick | up/down = throttle, left/right = yaw |
| Right stick | up/down = pitch, left/right = roll |
| TRIANGLE (hold 0.6 s) | takeoff |
| CROSS | land |
| CIRCLE | **emergency stop** (motors cut) |
| SQUARE (hold 1 s) | calibrate (drone on a level surface, not flying; untested) |
| D-pad up/down, left/right | pitch trim, roll trim |
| L / R | yaw trim left / right |
| SELECT (hold 0.8 s) | reset trim |
| START | stick range 1 / 2 / 3 (starts at 1, the smallest) |
| L + R + START + SELECT | quit |

## Camera picture

The PC decodes the drone's JPEG video, shrinks it (default 320x240), converts it to 16-bit colour and sends it to the Vita in small UDP packets (a few rows each). The Vita draws it, doubled to 640x480, at the top left of its screen; the text is in the column to the right and under the picture. A lost packet only leaves a few old rows on screen.

Bandwidth is about 10 Mbit/s at the defaults (320x240, 8 frames per second). If the picture stutters, lower it:

```
python python\bridge.py --vita-size 240x180 --vita-fps 6
python python\bridge.py --no-vita-video          # controls only
```

## Icon

`sce_sys/icon0.png` is the bubble icon and `sce_sys/livearea/contents/` has the LiveArea pictures. They are drawn by `tools/make_assets.py` (needs `pip install pillow`; run it from the `vita` folder) and added to the VPK by `CMakeLists.txt`. Edit the colours in the script and re-run it to restyle them. If the icon doesn't change after installing, delete the old "Vita Drone" bubble first and install again (the Vita can cache the old icon), and rebuild the database if needed.

## 1. Build the app

**Option A: GitHub Actions (no local toolchain).** Push the repo. The workflow in `.github/workflows/build-vita.yml` installs VitaSDK and builds the VPK. Download `vita-drone-vpk` from the run's *Artifacts*. This workflow has not been run yet; if it fails, send me the log.

**Option B: local build.** Install VitaSDK (WSL2 with Ubuntu is easiest on Windows; follow vitasdk.org), then:

```
export VITASDK=/usr/local/vitasdk      # wherever you installed it
cd vita
cmake -S . -B build && cmake --build build
```

This produces `build/vita_drone.vpk`. CMake downloads the official vitasdk samples to get `debugScreen`; pass `-DDEBUGSCREEN_DIR=...` if you already have them.

## 2. Install it on the Vita

1. Open VitaShell on the Vita and press SELECT to start its FTP server (it shows an address like `ftp://192.168.0.xx:1337`).
2. Upload the `.vpk` to `ux0:/` with any FTP client (FileZilla works).
3. In VitaShell, open the `.vpk` and press X to install it. "Vita Drone" appears on the LiveArea.

## 3. Tell the Vita your PC's address

On the PC, run `ipconfig` and find the IPv4 address of the **Ethernet** adapter (it was `192.168.0.53` earlier; routers can change it, so check). On the Vita, create `ux0:/data/vitadrone/config.txt` containing just that address on one line (upload it with FTP). Without the file, the app uses `192.168.0.53`.

The Vita must be on the **same router** as the PC (not a guest network; some routers isolate Wi-Fi clients from wired ones).

## 4. Run it

1. Drone on, PC's Wi-Fi dongle connected to it, **propellers off** for the first run.
2. `python bridge.py` (type `YES` at the prompt). If Windows Firewall asks, allow Python on private networks (UDP 9000).
3. Start the app on the Vita. Within a second it should show **BRIDGE LINK OK** and the drone's state. The bridge prints `Vita connected`.
4. Test in this order: sticks move the numbers on screen; hold TRIANGLE (motors spin); CROSS (they stop); CIRCLE (emergency stop); tap the D-pad and watch the trim values change.
5. **Failsafe test:** while the motors are spinning (props off), quit the Vita app or turn its Wi-Fi off. The bridge stops getting packets: the drone should go neutral at 0.3 s, get a land command at 1.5 s, and cut the motors at 5 s.

## Test without a Vita

`python fake_vita.py --host 127.0.0.1 --press TAKEOFF` sends the same messages as the Vita. `python test_bridge.py` and `python test_vita_protocol.py` run the automatic tests (no hardware needed).

## Message format

See `vitalink.py` (Python) and `src/protocol.h` (C). Control and telemetry are 12-byte UDP messages with a magic number, a version, a checksum and (control only) a sequence number. Control goes from the Vita to UDP port 9000; telemetry and the camera picture go back to the Vita on UDP port 9001.