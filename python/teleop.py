"""teleop.py - keyboard teleop with live video for the WIFI_CAM / E4 drone.

Keys (click the video window first; keys only work while it is focused):
  w / s   pitch  (forward / back)        r / f   throttle (up / down)
  a / d   roll   (left / right)          q / e   yaw (left / right)
  t       takeoff pulse                  g       land pulse
  SPACE   EMERGENCY STOP (motors cut)    ESC     emergency stop and quit
  1 2 3   stick range small / medium / large (throttle and yaw get larger ranges than roll and pitch)
  i / k   pitch trim + / -   j / l   roll trim - / +   u / o   yaw trim - / +   0   reset trim
  x       calibrate (the app's "checkout" flag; drone on a LEVEL SURFACE, not flying; untested)
  v       start/stop recording frames to recordings/<timestamp>/ (for SLAM)
  c       save one frame to calib/ (for camera calibration)

Stick directions are ASSUMED (roll + = right, pitch + = forward, throttle + = up, yaw + = clockwise).
Verify them with the props OFF first, and use --invert if one is backwards.

Windows only repeats the last key you hold, so only one movement key repeats at a time. A tap gives a short
nudge (--hold seconds); holding gives continuous movement after a brief pause. A gamepad will be better later.
"""
import argparse
import json
import os
import time

AXES = {  # key -> (axis, direction)
    "w": ("pitch", +1), "s": ("pitch", -1),
    "a": ("roll", -1), "d": ("roll", +1),
    "r": ("throttle", +1), "f": ("throttle", -1),
    "q": ("yaw", -1), "e": ("yaw", +1),
}
TRIM_KEYS = {"i": ("pitch", +1), "k": ("pitch", -1),
             "j": ("roll", -1), "l": ("roll", +1),
             "u": ("yaw", -1), "o": ("yaw", +1)}
SPEED_KEYS = ("1", "2", "3")             # stick range presets, see drone.SPEED_PRESETS
TRIM_FILE = "trim.json"


def load_trim(path=TRIM_FILE):
    """Saved trim as {"roll": n, "pitch": n, "yaw": n}; {} if the file is missing or unreadable."""
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return {k: int(data[k]) for k in ("roll", "pitch", "yaw") if k in data}
    except (OSError, ValueError, TypeError, AttributeError):
        return {}


def save_trim(trim, path=TRIM_FILE):
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump({k: int(v) for k, v in trim.items()}, f)
    except OSError:
        pass


class KeyHold:
    """Turns key presses into stick values that expire shortly after the last key event."""

    def __init__(self, hold=0.35):
        self.hold = hold
        self._expiry = {}

    def press(self, ch, now):
        if ch in AXES:
            self._expiry[ch] = now + self.hold

    def clear(self):
        self._expiry.clear()

    def sticks(self, now):
        out = {"roll": 0.0, "pitch": 0.0, "throttle": 0.0, "yaw": 0.0}
        for ch, until in list(self._expiry.items()):
            if now >= until:
                del self._expiry[ch]
                continue
            axis, direction = AXES[ch]
            out[axis] = max(-1.0, min(1.0, out[axis] + direction))
        return out


def main():
    import cv2
    import numpy as np
    from drone import DroneLink, SPEED_PRESETS

    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.4.153")
    ap.add_argument("--fmt", type=int, choices=[8, 20], default=20, help="packet format (20 worked on our drone)")
    ap.add_argument("--max-offset", type=int, default=40, help="stick range: 1..127 (speeds 1/2/3 = 20/40/70)")
    ap.add_argument("--hold", type=float, default=0.35, help="seconds a key press stays active")
    ap.add_argument("--trim-step", type=int, default=1, help="trim change per key press, in byte units")
    ap.add_argument("--invert", default="", help="comma list of axes to flip, e.g. pitch,yaw")
    ap.add_argument("--land-after", type=float, default=1.5, help="failsafe: seconds without input before landing")
    ap.add_argument("--cut-after", type=float, default=5.0, help="failsafe: seconds without input before motor cut")
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()

    ans = input("Are the PROPS REMOVED (or the drone tethered / in a cage)? Type YES: ")
    if ans.strip() != "YES":
        print("Aborted.")
        return

    signs = {a.strip(): -1 for a in args.invert.split(",") if a.strip()}
    link = DroneLink(ip=args.ip, fmt=args.fmt, max_offset=args.max_offset, signs=signs, video=not args.no_video,
                     land_after=args.land_after, cut_after=args.cut_after)
    link.set_trim(**load_trim())
    keys = KeyHold(args.hold)
    win = "drone teleop (ESC = emergency stop + quit)"
    blank = np.zeros((480, 640, 3), np.uint8)
    img, last_seq = blank.copy(), 0
    rec_dir, rec_count, calib_count = None, 0, 0
    fps_t, fps_seq, fps = time.time(), 0, 0.0

    link.start()
    cv2.namedWindow(win)
    try:
        while True:
            now = time.monotonic()
            code = cv2.waitKey(10) & 0xFF
            if code == 27:                                    # ESC
                break
            if code == 32:                                    # SPACE
                link.emergency_stop()
                keys.clear()
            elif code != 255:
                ch = chr(code).lower()
                if ch in AXES:
                    keys.press(ch, now)
                elif ch == "t":
                    link.takeoff()
                elif ch == "g":
                    link.land()
                elif ch in SPEED_KEYS:
                    link.max_offset = SPEED_PRESETS[int(ch)]
                elif ch in TRIM_KEYS:
                    axis, direction = TRIM_KEYS[ch]
                    link.adjust_trim(axis, direction * args.trim_step)
                    save_trim(link.get_trim())
                elif ch == "0":
                    link.set_trim(roll=0, pitch=0, yaw=0)
                    save_trim(link.get_trim())
                elif ch == "x":
                    link.calibrate()
                elif ch == "v":
                    if rec_dir is None:
                        rec_dir = os.path.join("recordings", time.strftime("%Y%m%d_%H%M%S"))
                        os.makedirs(rec_dir, exist_ok=True)
                        rec_count = 0
                    else:
                        rec_dir = None
                elif ch == "c" and last_seq:
                    frame = link.get_frame()
                    if frame:
                        os.makedirs("calib", exist_ok=True)
                        with open(os.path.join("calib", f"calib_{calib_count:03d}.jpg"), "wb") as f:
                            f.write(frame.jpeg)
                        calib_count += 1

            sticks = keys.sticks(now)
            link.set_sticks(**sticks)                          # also the failsafe heartbeat

            frame = link.get_frame()
            if frame and frame.seq != last_seq:
                last_seq = frame.seq
                decoded = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
                if decoded is not None:
                    img = decoded
                if rec_dir is not None:
                    with open(os.path.join(rec_dir, f"{frame.ts:.3f}.jpg"), "wb") as f:
                        f.write(frame.jpeg)
                    rec_count += 1
            if time.time() - fps_t >= 2.0:
                fps, fps_t, fps_seq = (last_seq - fps_seq) / (time.time() - fps_t), time.time(), last_seq

            st = link.status()
            disp = img.copy()
            lines = [
                f"mode {st['mode']}  failsafe {st['failsafe_stage']}  {'EMERGENCY STOP' if st['emergency'] else ''}",
                f"R {sticks['roll']:+.0f} P {sticks['pitch']:+.0f} T {sticks['throttle']:+.0f} Y {sticks['yaw']:+.0f}"
                f"   range {link.max_offset}   video {fps:.1f} fps   {'REC ' + str(rec_count) if rec_dir else ''}",
                f"trim R {st['trim']['roll']:+d} P {st['trim']['pitch']:+d} Y {st['trim']['yaw']:+d}",
            ]
            for i, text in enumerate(lines):
                cv2.putText(disp, text, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(disp, "wasd=move r/f=throttle q/e=yaw t=takeoff g=land SPACE=STOP ESC=quit",
                        (8, disp.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.imshow(win, disp)
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping motors and closing...")
        save_trim(link.get_trim())
        link.close()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()