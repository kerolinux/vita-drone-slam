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
from collections import deque
from pathlib import Path

import cv2
import numpy as np

import apriltag_detect_bottom as vision
from drone import DroneLink, SPEED_PRESETS
from teleop import load_trim
from vita_takeover import VitaTakeover

# PID Gains - tuned for E4 drone with bottom camera
# KP: proportional gain for position error (normalized image coords)
# KI: integral gain to eliminate steady-state error
# KD: derivative gain on error rate (damping)
KP, KI, KD = 0.45, 0.08, 0.12

# Control limits
MAX_XY = 0.30              # max stick deflection (normalized -1..1)
MAX_INTEGRAL = 0.15        # anti-windup clamp for integral term
DEADBAND = 0.03            # ignore tiny errors (normalized)
ALIGN_TOL = 0.08           # alignment tolerance for state transition (normalized)
ALIGN_HOLD_S = 1.0         # must hold alignment for this long before descend
MAX_ALIGN_RATE = 0.20      # max error rate for "stable" (normalized/s)
DESCEND_THROTTLE = -0.25   # descent throttle (negative = down)
LAND_Z_M = 0.20            # trigger land when tag distance < this (meters)
TAG_LOST_LAND_S = 1.5      # request land if tag lost for this long
MAX_FRAME_AGE_S = 0.5      # reject stale frames
TAKEOFF_WAIT_S = 4.0
LAND_WAIT_S = 3.0

# Signs for roll/pitch axes (require physical verification)
SIGN_ROLL, SIGN_PITCH = 1, -1
AIM_OFFSET_PX = (0, 0)
NEUTRAL = dict(roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0)
ROOT = Path(__file__).resolve().parent.parent


def clamp(value, limit):
    return max(-limit, min(limit, value))


class PIDController:
    """PID controller with anti-windup and deadband."""
    def __init__(self, kp, ki, kd, max_output, max_integral, deadband, sign=1, offset=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral
        self.deadband = deadband
        self.sign = sign
        self.offset = offset  # byte quantization offset for non-zero compensation

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error, now):
        # Deadband
        if abs(error) < self.deadband:
            error = 0.0

        dt = 0.0
        if self.prev_time is not None:
            dt = now - self.prev_time
        self.prev_time = now

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        if dt > 0:
            self.integral += error * dt
            self.integral = clamp(self.integral, self.max_integral)
        i = self.ki * self.integral

        # Derivative on error (not measurement, to avoid kick on setpoint change)
        d = 0.0
        if dt > 0:
            d = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        output = p + i + d
        output = clamp(output, self.max_output)

        # Quantization compensation: ensure non-zero output after byte rounding
        if abs(error) >= self.deadband and 0 < abs(output) * self.offset < 1:
            output = math.copysign(1.0 / self.offset, output)

        return self.sign * output


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
        # shortcut: apparent-size height assumes a near-frontoparallel tag
        z = float(vision.TAG_SIZE_M / edge)
        centre = points.mean(axis=0)
        h, w = image.shape[:2]
        ex = float((centre[0] * K[0, 0] - AIM_OFFSET_PX[0]) / (w / 2))
        ey = float((centre[1] * K[1, 1] - AIM_OFFSET_PX[1]) / (h / 2))
        return (ex, ey, z), raw.mean(axis=0)
    return None


class ObservationSmoother:
    """Median filter for tag observations (ex, ey, z)."""
    def __init__(self, window=5):
        self.window = window
        self.history = deque(maxlen=window)

    def update(self, obs):
        if obs is None:
            return None
        self.history.append(obs)
        if len(self.history) < 2:
            return obs
        # Median per component
        ex = np.median([o[0] for o in self.history])
        ey = np.median([o[1] for o in self.history])
        z  = np.median([o[2] for o in self.history])
        return (float(ex), float(ey), float(z))

    def reset(self):
        self.history.clear()


class Lander:
    def __init__(self, now, roll_sign=SIGN_ROLL, pitch_sign=SIGN_PITCH,
                 kp=KP, ki=KI, kd=KD, max_xy=MAX_XY, max_integral=MAX_INTEGRAL,
                 deadband=DEADBAND, speed_preset=SPEED_PRESETS[1]):
        self.state = "ALIGN"
        self.roll_sign = roll_sign
        self.pitch_sign = pitch_sign

        # PID controllers for roll (ex) and pitch (ey)
        roll_offset = speed_preset["roll"]
        pitch_offset = speed_preset["pitch"]
        self.pid_x = PIDController(kp, ki, kd, max_xy, max_integral, deadband, roll_sign, roll_offset)
        self.pid_y = PIDController(kp, ki, kd, max_xy, max_integral, deadband, pitch_sign, pitch_offset)

        self.smoother = ObservationSmoother(window=5)
        self.aligned_since = None
        self.last_seen = now
        self.prev_obs = None
        self.prev_time = None

    def update(self, obs, now):
        cmd = NEUTRAL.copy()

        if self.state == "LAND":
            return self.state, cmd

        if obs is None or not all(math.isfinite(v) for v in obs) or obs[2] <= 0:
            self.pid_x.reset()
            self.pid_y.reset()
            self.smoother.reset()
            self.aligned_since = None
            self.state = "LAND" if now - self.last_seen >= TAG_LOST_LAND_S else "ALIGN"
            return self.state, cmd

        self.last_seen = now
        smoothed = self.smoother.update(obs)
        ex, ey, z = smoothed

        # Compute PID outputs
        cmd["roll"] = self.pid_x.update(ex, now)
        cmd["pitch"] = self.pid_y.update(ey, now)

        # State machine
        if self.state == "ALIGN":
            # Check if stably aligned
            stable = (max(abs(ex), abs(ey)) < ALIGN_TOL)
            if stable:
                if self.aligned_since is None:
                    self.aligned_since = now
                elif now - self.aligned_since >= ALIGN_HOLD_S:
                    self.state = "DESCEND"
                    self.aligned_since = None  # reuse for descend timing if needed
            else:
                self.aligned_since = None

        elif self.state == "DESCEND":
            # Re-check alignment during descent; go back to ALIGN if lost
            stable = (max(abs(ex), abs(ey)) < ALIGN_TOL)
            if not stable:
                self.state = "ALIGN"
                self.aligned_since = None
                # PID continues correcting (integral preserved)
            elif z < LAND_Z_M:
                self.state = "LAND"
                return self.state, NEUTRAL.copy()
            else:
                cmd["throttle"] = DESCEND_THROTTLE

        return self.state, cmd


class LandingSequence:
    def __init__(self):
        self.accepted_at = None

    def step(self, link, now):
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
    # PID tuning arguments
    parser.add_argument("--kp", type=float, default=KP, help="Proportional gain")
    parser.add_argument("--ki", type=float, default=KI, help="Integral gain")
    parser.add_argument("--kd", type=float, default=KD, help="Derivative gain")
    parser.add_argument("--max-xy", type=float, default=MAX_XY, help="Max stick deflection")
    parser.add_argument("--max-integral", type=float, default=MAX_INTEGRAL, help="Integral anti-windup limit")
    parser.add_argument("--deadband", type=float, default=DEADBAND, help="Deadband threshold")
    parser.add_argument("--align-tol", type=float, default=ALIGN_TOL, help="Alignment tolerance")
    parser.add_argument("--align-hold", type=float, default=ALIGN_HOLD_S, help="Hold time before descend (s)")
    parser.add_argument("--descend-throttle", type=float, default=DESCEND_THROTTLE, help="Descent throttle")
    parser.add_argument("--land-z", type=float, default=LAND_Z_M, help="Land trigger distance (m)")
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
    print(f"PID: KP={args.kp} KI={args.ki} KD={args.kd} MAX_XY={args.max_xy} MAX_I={args.max_integral} DEADBAND={args.deadband}")
    print(f"Align: tol={args.align_tol} hold={args.align_hold}s  Descend: throttle={args.descend_throttle} land_z={args.land_z}m")
    print("LIVE: T = takeoff (Vita: manual, otherwise requires tag); A = engage Vita-assisted landing; Q/Esc = land; X = motor cut."
          if args.live else "PREVIEW: camera packets only; all flight packets blocked. Q/X = exit.")

    path = args.log or ROOT / "logs" / f"landing_{time.time_ns()}.csv"
    path.parent.mkdir(parents=True, exist_ok=True)

    phase, last_seq, cmd = "READY" if args.live else "PREVIEW", -1, NEUTRAL.copy()
    lander = Lander(time.monotonic(), args.roll_sign, args.pitch_sign,
                    kp=args.kp, ki=args.ki, kd=args.kd,
                    max_xy=args.max_xy, max_integral=args.max_integral, deadband=args.deadband)
    landing = LandingSequence()
    took_off = False
    takeoff_at = None
    last_good = time.monotonic()
    obs = None

    print(f"Diagnostic log: {path}")
    try:
        cv2.namedWindow("Landing")
        link.start()
        if not ensure_bottom_camera(link, args.live):
            print("No fresh bottom-camera stream (or cancelled); no takeoff.")
            return

        if takeover is not None:
            takeover.takeoff_enabled = args.live
            print("Vita: Triangle or PC T = manual takeoff (no tag needed); centre sticks and PC A = engage landing.")

        with path.open("w", newline="") as f:
            log = csv.writer(f)
            log.writerow(["monotonic_s", "phase", "seq", "arrival_age_s", "ex", "ey", "z_m",
                          "roll", "pitch", "throttle", "yaw", "roll_byte", "pitch_byte", "throttle_byte", "yaw_byte",
                          "pid_x_p", "pid_x_i", "pid_x_d", "pid_y_p", "pid_y_i", "pid_y_d"])
            while True:
                now = time.monotonic()
                key = read_key()
                if takeover is not None and takeover.started:
                    took_off = True
                action = handle_key(key, link, args.live)
                if action == "STOP":
                    if args.live:
                        time.sleep(1.7)
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
                    now = time.monotonic()
                    age = frame_age(frame, time.time())
                    if age > MAX_FRAME_AGE_S:
                        obs = None
                    if obs is not None:
                        last_good = now
                    if phase not in ("READY", "TAKEOFF", "LAND", "MANUAL"):
                        if not args.live and lander.state == "LAND" and obs is not None:
                            lander = Lander(now, args.roll_sign, args.pitch_sign,
                                            kp=args.kp, ki=args.ki, kd=args.kd,
                                            max_xy=args.max_xy, max_integral=args.max_integral, deadband=args.deadband)
                        state, cmd = lander.update(obs, now)
                        phase = state if args.live else "PREVIEW"
                    if image is not None:
                        cv2.putText(image, f"{phase}/{lander.state} age={age:.2f}s", (5, 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                        if obs is not None:
                            cv2.putText(image, f"ex={obs[0]:+.2f} ey={obs[1]:+.2f} z~{obs[2]:.2f}m", (5, 40),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                            # Show PID internals
                            cv2.putText(image, f"IX={lander.pid_x.integral:+.3f} IY={lander.pid_y.integral:+.3f}", (5, 60),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
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
                        lander = Lander(now, args.roll_sign, args.pitch_sign,
                                        kp=args.kp, ki=args.ki, kd=args.kd,
                                        max_xy=args.max_xy, max_integral=args.max_integral, deadband=args.deadband)
                        phase, cmd, last_good = "ALIGN", NEUTRAL.copy(), now
                        print("Autonomous landing engaged; move a Vita stick to take control back.")
                    else:
                        print("Engagement refused: take off first and centre the connected Vita sticks.")
                if phase == "TAKEOFF" and now - takeoff_at >= TAKEOFF_WAIT_S:
                    lander = Lander(now, args.roll_sign, args.pitch_sign,
                                    kp=args.kp, ki=args.ki, kd=args.kd,
                                    max_xy=args.max_xy, max_integral=args.max_integral, deadband=args.deadband)
                    phase, cmd = "ALIGN", NEUTRAL.copy()
                if took_off and phase not in ("LAND", "MANUAL") and now - last_good >= TAG_LOST_LAND_S:
                    print("\nTarget/video unavailable: requesting landing in place.")
                    phase = "LAND"
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
                # Log PID components
                pid_log = [lander.pid_x.kp * (obs[0] if obs else 0),
                           lander.pid_x.integral * lander.pid_x.ki,
                           lander.pid_x.kd * ((obs[0] - lander.pid_x.prev_error) / 0.02) if obs else 0,
                           lander.pid_y.kp * (obs[1] if obs else 0),
                           lander.pid_y.integral * lander.pid_y.ki,
                           lander.pid_y.kd * ((obs[1] - lander.pid_y.prev_error) / 0.02) if obs else 0]
                log.writerow([now, phase, last_seq, age, *values, *[send[k] for k in NEUTRAL], *wire, *pid_log])
                if new:
                    print(f"\r{phase:8} age={age:.2f}s tag={obs is not None} "
                          f"R/P={send['roll']:+.2f}/{send['pitch']:+.2f} bytes={wire[:2]} I={lander.pid_x.integral:+.3f}/{lander.pid_y.integral:+.3f}   ", end="", flush=True)
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