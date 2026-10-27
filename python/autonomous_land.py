"""Bottom-camera AprilTag landing for the E4 drone.

Default: camera preview + computed commands; video start/stop/toggle packets only.
--live: enables flight controls; with --vita-takeover, take off manually then A engages landing.
Q/Esc: quit preview, or request landing in live mode. X: emergency motor stop in live mode.
A successful land() return means the request was queued, NOT that touchdown was confirmed.
"""
import argparse
import csv
import math
import time
from pathlib import Path

import cv2
import numpy as np

import apriltag_detect_bottom as vision
from drone import DroneLink, SPEED_PRESETS
from teleop import load_trim
from vita_takeover import VitaTakeover

KP, KD = 0.35, 0.08
MAX_XY = 0.25
DEADBAND = 0.04
ALIGN_TOL = 0.12
ALIGN_HOLD_S = 0.6
MAX_ALIGN_RATE = 0.25  # normalised image error per second, not measured vehicle velocity
DESCEND_THROTTLE = -0.30
LAND_Z_M = 0.25
TAG_LOST_LAND_S = 1.5
MAX_FRAME_AGE_S = 0.5
TAKEOFF_WAIT_S = 4.0
LAND_WAIT_S = 3.0
SIGN_ROLL, SIGN_PITCH = 1, -1  # still require physical verification
AIM_OFFSET_PX = (0, 0)
NEUTRAL = dict(roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0)
ROOT = Path(__file__).resolve().parent.parent


def clamp(value, limit):
    return max(-limit, min(limit, value))


def lateral_command(error, rate, sign, offset):
    value = KP * (0.0 if abs(error) < DEADBAND else error) + KD * rate
    value = clamp(value, MAX_XY)
    # Preserve a nonzero correction after byte quantisation; this is not physical deadband compensation.
    if abs(error) >= DEADBAND and 0 < abs(value) * offset < 1:
        value = math.copysign(1.0 / offset, value)
    return sign * value


def frame_age(frame, wall_now):
    if frame is None or not math.isfinite(frame.ts):
        return float("inf")
    age = wall_now - frame.ts
    return age if age >= 0 else float("inf")


class PreviewLink(DroneLink):
    """Enforce preview isolation at the transport, including inherited failsafe/emergency paths."""
    def _next_packets(self, now):
        return []

    def _send(self, data, port):
        if port != self.video_port or data not in (b"Bv", b"Bw", b"By"):
            raise RuntimeError("Preview blocked a non-video packet")
        super()._send(data, port)


def find_tag(image, detector, K, D):
    """Use distortion-corrected tag corners for centre and optical-axis distance."""
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return None
    for corner, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) != vision.TARGET_ID:
            continue
        raw = corner.reshape(4, 2).astype(np.float64)
        points = cv2.undistortPoints(raw.reshape(-1, 1, 2), K, D).reshape(4, 2)
        edge = np.mean(np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1))
        if not np.isfinite(points).all() or not np.isfinite(edge) or edge <= 1e-6:
            return None
        # shortcut: apparent-size height assumes a near-frontoparallel tag; verify at known heights before flight.
        z = float(vision.TAG_SIZE_M / edge)
        centre = points.mean(axis=0)
        h, w = image.shape[:2]
        ex = float((centre[0] * K[0, 0] - AIM_OFFSET_PX[0]) / (w / 2))
        ey = float((centre[1] * K[1, 1] - AIM_OFFSET_PX[1]) / (h / 2))
        return (ex, ey, z), raw.mean(axis=0)
    return None


class Lander:
    def __init__(self, now, roll_sign=SIGN_ROLL, pitch_sign=SIGN_PITCH):
        self.state = "ALIGN"
        self.signs = roll_sign, pitch_sign
        self.prev = None
        self.aligned_since = None
        self.last_seen = now

    def update(self, obs, now):
        cmd = NEUTRAL.copy()
        if self.state == "LAND":
            return self.state, cmd
        if obs is None or not all(math.isfinite(v) for v in obs) or obs[2] <= 0:
            self.prev = self.aligned_since = None
            self.state = "LAND" if now - self.last_seen >= TAG_LOST_LAND_S else "ALIGN"
            return self.state, cmd
        ex, ey, z = obs
        self.last_seen = now
        dx = dy = 0.0
        continuous = self.prev is not None and 0 < now - self.prev[2] <= MAX_FRAME_AGE_S
        if continuous:
            dt = now - self.prev[2]
            dx, dy = (ex - self.prev[0]) / dt, (ey - self.prev[1]) / dt
        else:
            self.aligned_since = None
        self.prev = ex, ey, now
        cmd["roll"] = lateral_command(ex, dx, self.signs[0], SPEED_PRESETS[1]["roll"])
        cmd["pitch"] = lateral_command(ey, dy, self.signs[1], SPEED_PRESETS[1]["pitch"])
        stable = continuous and max(abs(ex), abs(ey)) < ALIGN_TOL and max(abs(dx), abs(dy)) < MAX_ALIGN_RATE
        if not stable:
            self.aligned_since = None
            self.state = "ALIGN"
        else:
            if self.aligned_since is None:
                self.aligned_since = now
            if now - self.aligned_since >= ALIGN_HOLD_S:
                self.state = "DESCEND"
        if self.state == "DESCEND":
            if z < LAND_Z_M:
                self.state = "LAND"
                return self.state, NEUTRAL.copy()
            cmd["throttle"] = DESCEND_THROTTLE
        return self.state, cmd


class LandingSequence:
    def __init__(self):
        self.accepted_at = None

    def step(self, link, now):
        # Firmware controls the one-key landing; do not add unverified lateral overrides to that mode.
        link.set_sticks()
        if self.accepted_at is None:
            if link.land():
                self.accepted_at = now
                print("\nLanding request queued; touchdown is NOT confirmed by telemetry.")
            return False
        return now - self.accepted_at >= LAND_WAIT_S


def read_key():
    return cv2.waitKey(1) & 0xFF


def handle_key(key, link, live):
    if key == ord("x"):
        if live:
            link.emergency_stop()
        return "STOP"
    if key in (ord("q"), 27):
        if live and isinstance(link, VitaTakeover):
            link.request_land()
        return "LAND" if live else "STOP"
    return None


def ensure_bottom_camera(link, live=False, timeout=12.0):
    deadline, last_toggle = time.monotonic() + timeout, -float("inf")
    while time.monotonic() < deadline:
        action = handle_key(read_key(), link, live)
        if action:
            if live and action == "STOP":
                time.sleep(1.7)
            return False
        link.heartbeat()
        frame = link.get_frame()
        if frame_age(frame, time.time()) <= MAX_FRAME_AGE_S:
            image = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                cv2.imshow("Landing", image)
                if image.shape[1::-1] == vision.BOTTOM_SIZE:
                    return True
                if time.monotonic() - last_toggle > 2:
                    link._send(b"By", link.video_port)
                    last_toggle = time.monotonic()
        time.sleep(0.03)
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--vita-takeover", action="store_true", help="require Vita link; stick movement latches manual control")
    parser.add_argument("--vita-port", type=int, default=9000, help="Vita input UDP port; do not also run bridge.py")
    parser.add_argument("--roll-sign", type=int, choices=(-1, 1), default=SIGN_ROLL)
    parser.add_argument("--pitch-sign", type=int, choices=(-1, 1), default=SIGN_PITCH)
    parser.add_argument("--trim-file", type=Path, default=ROOT / "trim.json")
    parser.add_argument("--log", type=Path, help="CSV diagnostic output (default: logs/landing_<time>.csv)")
    args = parser.parse_args(argv)
    if not 1 <= args.vita_port <= 65535:
        parser.error("--vita-port must be between 1 and 65535")
    K, D, expected = vision.load_calibration()
    if expected != vision.BOTTOM_SIZE:
        raise SystemExit("Calibration resolution does not match the bottom camera; refusing to run.")
    detector = vision.make_detector()
    cls = DroneLink if args.live else PreviewLink
    link = cls(video=True, local_port=vision.LOCAL_PORT, max_offset=SPEED_PRESETS[1], log=print)
    link.set_trim(**load_trim(args.trim_file))
    takeover = VitaTakeover(link, port=args.vita_port, preview=not args.live) if args.vita_takeover else None
    if takeover is not None:
        link = takeover
    print(f"Trim: {link.get_trim()}; roll sign={args.roll_sign}, pitch sign={args.pitch_sign}")
    print("LIVE: T = takeoff (Vita: manual, otherwise requires tag); A = engage Vita-assisted landing; Q/Esc = land; X = motor cut."
          if args.live else "PREVIEW: camera packets only; all flight packets blocked. Q/X = exit.")
    path = args.log or ROOT / "logs" / f"landing_{time.time_ns()}.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    phase, last_seq, cmd = "READY" if args.live else "PREVIEW", -1, NEUTRAL.copy()
    lander = Lander(time.monotonic(), args.roll_sign, args.pitch_sign)
    landing = LandingSequence()
    took_off = False
    takeoff_at = None
    last_good = time.monotonic()
    obs = None
    print(f"Diagnostic log: {path}")
    try:
        cv2.namedWindow("Landing")
        link.start()  # preserve the existing idle handshake; no takeoff until explicit T
        if not ensure_bottom_camera(link, args.live):
            print("No fresh bottom-camera stream (or cancelled); no takeoff.")
            return
        if takeover is not None:
            takeover.takeoff_enabled = args.live
            print("Vita: Triangle or PC T = manual takeoff (no tag needed); centre sticks and PC A = engage landing.")
        with path.open("w", newline="") as f:
            log = csv.writer(f)
            log.writerow(["monotonic_s", "phase", "seq", "arrival_age_s", "ex", "ey", "z_m",
                          "roll", "pitch", "throttle", "yaw", "roll_byte", "pitch_byte", "throttle_byte", "yaw_byte"])
            while True:
                now = time.monotonic()
                key = read_key()
                if takeover is not None and takeover.started:
                    took_off = True
                action = handle_key(key, link, args.live)
                if action == "STOP":
                    if args.live:
                        time.sleep(1.7)  # allow the library's emergency pulse train to finish
                    break
                if action == "LAND":
                    if not took_off:
                        break
                    phase = "LAND"
                if takeover is not None:
                    if takeover.mode == "EMERGENCY":
                        if args.live:
                            time.sleep(1.7)
                        break
                    if takeover.mode in ("MANUAL", "LAND"):
                        phase = takeover.mode
                        if phase == "LAND" and args.live and not took_off:
                            break
                frame = link.get_frame()
                age = frame_age(frame, time.time())
                new = frame is not None and frame.seq != last_seq
                if new:
                    last_seq = frame.seq
                    image = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
                    obs = None
                    if image is not None and image.shape[1::-1] == expected and age <= MAX_FRAME_AGE_S:
                        hit = find_tag(image, detector, K, D)
                        if hit is not None:
                            obs, centre = hit
                            cv2.circle(image, tuple(np.round(centre).astype(int)), 4, (0, 255, 0), -1)
                    # Include decode/detection time in freshness, not just time before processing.
                    now = time.monotonic()
                    age = frame_age(frame, time.time())
                    if age > MAX_FRAME_AGE_S:
                        obs = None
                    if obs is not None:
                        last_good = now
                    if phase not in ("READY", "TAKEOFF", "LAND", "MANUAL"):
                        if not args.live and lander.state == "LAND" and obs is not None:
                            lander = Lander(now, args.roll_sign, args.pitch_sign)
                        state, cmd = lander.update(obs, now)
                        phase = state if args.live else "PREVIEW"
                    if image is not None:
                        cv2.putText(image, f"{phase}/{lander.state} age={age:.2f}s", (5, 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                        if obs is not None:
                            cv2.putText(image, f"ex={obs[0]:+.2f} ey={obs[1]:+.2f} z~{obs[2]:.2f}m", (5, 40),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                        cv2.imshow("Landing", image)
                if args.live and phase == "READY" and key == ord("t"):
                    if takeover is None and (obs is None or age > MAX_FRAME_AGE_S):
                        print("Takeoff refused: no fresh target.")
                    elif link.takeoff():
                        took_off, takeoff_at, last_good, phase = True, now, now, "MANUAL" if takeover is not None else "TAKEOFF"
                if args.live and takeover is not None and key == ord("a"):
                    if obs is None or age > MAX_FRAME_AGE_S:
                        print("Engagement refused: no fresh target; remain in manual control.")
                    elif takeover.engage():
                        lander = Lander(now, args.roll_sign, args.pitch_sign)
                        phase, cmd, last_good = "ALIGN", NEUTRAL.copy(), now
                        print("Autonomous landing engaged; move a Vita stick to take control back.")
                    else:
                        print("Engagement refused: take off first and centre the connected Vita sticks.")
                if phase == "TAKEOFF" and now - takeoff_at >= TAKEOFF_WAIT_S:
                    lander = Lander(now, args.roll_sign, args.pitch_sign)
                    phase, cmd = "ALIGN", NEUTRAL.copy()
                if took_off and phase not in ("LAND", "MANUAL") and now - last_good >= TAG_LOST_LAND_S:
                    print("\nTarget/video unavailable: requesting landing in place.")
                    phase = "LAND"
                # Re-check after vision work: the receiver may have taken control during detection.
                if takeover is not None and takeover.mode in ("MANUAL", "LAND"):
                    phase = takeover.mode
                if takeover is not None and takeover.mode == "MANUAL":
                    send = takeover.manual_command
                elif args.live:
                    if phase == "LAND":
                        if landing.step(link, now):
                            break
                    else:
                        send = cmd if phase not in ("READY", "TAKEOFF") and age <= MAX_FRAME_AGE_S else NEUTRAL
                        link.set_sticks(**send)
                else:
                    link.heartbeat()
                    send = cmd if age <= MAX_FRAME_AGE_S else NEUTRAL
                if args.live and phase == "LAND":
                    send = NEUTRAL
                values = obs or ("", "", "")
                wire = [link._stick_byte(axis, send[axis]) for axis in NEUTRAL]
                log.writerow([now, phase, last_seq, age, *values, *[send[k] for k in NEUTRAL], *wire])
                if new:
                    print(f"\r{phase:8} age={age:.2f}s tag={obs is not None} "
                          f"R/P={send['roll']:+.2f}/{send['pitch']:+.2f} bytes={wire[:2]}   ", end="", flush=True)
                    f.flush()
                time.sleep(0.02)
    except (KeyboardInterrupt, Exception) as exc:
        print(f"\nStopped: {exc!r}")
        if takeover is not None and takeover.started:
            took_off = True
        if args.live and took_off:
            if takeover is not None and takeover.mode == "MANUAL":
                print("Vision loop stopped; Vita still owns manual control. Cross=land, Circle=motor cut.")
                while takeover.mode == "MANUAL":
                    time.sleep(0.05)
            if takeover is not None and takeover.mode == "EMERGENCY":
                time.sleep(1.7)
                return
            end = time.monotonic() + 5
            while time.monotonic() < end:
                if handle_key(read_key(), link, True) == "STOP":
                    time.sleep(1.7)
                    break
                if landing.step(link, time.monotonic()):
                    break
                time.sleep(0.03)
            print("Landing requested after interruption; verify the drone has actually landed.")
    finally:
        link.close(stop_motors=False)
        cv2.destroyAllWindows()
        print("\nStopped sending. No telemetry confirms touchdown." if took_off else "\nDone.")


if __name__ == "__main__":
    main()

