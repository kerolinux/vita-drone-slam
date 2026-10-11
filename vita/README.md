# Vita Drone Controller

A PS Vita homebrew app that sends the Vita's sticks and buttons to `bridge.py` on your PC over Wi-Fi, and shows the bridge's telemetry on the Vita screen.

```
Vita --Wi-Fi--> home router --Ethernet--> PC (bridge.py) --Wi-Fi dongle--> drone
```

The PC keeps its Ethernet connection to the router and uses the Wi-Fi dongle for the drone, so no second Wi-Fi radio is needed.

> **Status:** the PC side (`bridge.py`, `vitalink.py`) and the message code in `src/protocol.h` are tested. `src/main.c` has been checked against the real Vita headers but **has not been built or run on a Vita yet**. Expect to fix small things on the first build.

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

See `vitalink.py` (Python) and `src/protocol.h` (C). Both are 12-byte UDP messages with a magic number, a version, a checksum and, for control messages, a sequence number. Control goes from the Vita to UDP port 9000; telemetry goes back to the Vita on UDP port 9001.