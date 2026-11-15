"""Main-camera AprilTag landing for the E4 drone (camera facing down).

Default: camera preview + computed commands; video start/stop/toggle packets only.
--live: enables flight controls; with --vita-takeover, take off manually then A engages landing.
Q/Esc: quit preview, or request landing in live mode. X: emergency motor stop in live mode.
A successful land() return means the request was queued, NOT that touchdown was confirmed.

Blind landing: if tag lost below --blind-land-z during DESCEND, continues descending
  blindly for --blind-land-max-s seconds then auto-lands.
"""
import argparse
import csv
import math
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from drone import DroneLink, SPEED_PRESETS
from teleop import load_trim
from vita_takeover import VitaTakeover

# Camera calibration for the 640x480 drone main camera (from apriltag_detect.py)
CAMERA_MATRIX = np.array([
    [931.432295, 0.0, 311.308958],
    [0.0, 929.633035, 236.214237],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

DIST_COEFFS = np.array([
    0.147505, -0.613989, 0.008993, -0.009062, 0.395905
], dtype=np.float64)

TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11
TARGET_ID = 0
TAG_SIZE_M = 0.100  # Physical side length: 100 mm

MAIN_SIZE = (640, 480)
BOTTOM_SIZE = (240, 320)

# Bottom camera calibration (approximate - adjust for your drone)
BOTTOM_CAMERA_MATRIX = np.array([
    [300.0, 0.0, 120.0],
    [0.0, 300.0, 160.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

BOTTOM_DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

# Camera switching
CAMERA_SWITCH_Z = 0.60       # Switch to bottom camera below this altitude (m)
CAMERA_SWITCH_HYSTERESIS = 0.10  # Don't switch back until above this + hysteresis

# PID Gains - strong drift rejection, edge-damped for safety
KP, KI, KD = 0.80, 0.20, 0.10

# Edge-avoidance: minimal gains at edges, rely on KD damping only
EDGE_KP_MULT = 1.0      # No KP boost at edges - prevents overshoot
EDGE_KI_MULT = 1.0      # No KI boost - prevents windup
EDGE_KD_MULT = 2.5      # Strong KD damping near edges

# Control limits
MAX_XY = 0.25              # Enough authority to counter wind/drift
MAX_INTEGRAL = 0.15        # Integral builds to cancel steady drift
DEADBAND = 0.005           # Respond to tiny errors
ALIGN_TOL = 0.03           # Must be very centered
ALIGN_HOLD_S = 0.30        # Prove stability longer
DESCEND_ALIGN_TOL = 0.04   # Strict during descent
DESCEND_HOLD_S = 0.25
MAX_ALIGN_RATE = 0.10
DESCEND_THROTTLE = -0.08
LAND_Z_M = 0.80
TAG_LOST_LAND_S = 2.0
MAX_FRAME_AGE_S = 0.3
TAKEOFF_WAIT_S = 4.0
LAND_WAIT_S = 3.0

# Blind landing commit
BLIND_LAND_Z_M = 0.95
BLIND_LAND_MAX_S = 1.2

# Edge thresholds - wider safe zone
EDGE_WARN_THRESHOLD = 0.60
EDGE_CRITICAL_THRESHOLD = 0.80
EDGE_LOST_THRESHOLD = 0.95

# Signs for roll/pitch axes (require physical verification)
SIGN_ROLL, SIGN_PITCH = 1, -1
NEUTRAL = dict(roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0)
ROOT = Path(__file__).resolve().parent.parent


def clamp(value, limit):
    return max(-limit, min(limit, value))


class PIDController:
    """PID controller with anti-windup, deadband, and dynamic gain scheduling for edge avoidance."""
    def __init__(self, kp, ki, kd, max_output, max_integral, deadband, sign=1, offset=1.0,
                 edge_kp_mult=1.0, edge_ki_mult=1.0, edge_kd_mult=1.0):
        # Base gains
        self.base_kp = kp
        self.base_ki = ki
        self.base_kd = kd
        # Edge multipliers
        self.edge_kp_mult = edge_kp_mult
        self.edge_ki_mult = edge_ki_mult
        self.edge_kd_mult = edge_kd_mult
        # Current effective gains (updated per frame)
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.max_output = max_output
        self.max_integral = max_integral
        self.deadband = deadband
        self.sign = sign
        self.offset = offset

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        # Reset to base gains
        self.kp = self.base_kp
        self.ki = self.base_ki
        self.kd = self.base_kd

    def set_edge_multiplier(self, edge_factor):
        """Adjust gains based on edge proximity (0.0 = center, 1.0 = critical edge)."""
        self.kp = self.base_kp * (1.0 + (self.edge_kp_mult - 1.0) * edge_factor)
        self.ki = self.base_ki * (1.0 + (self.edge_ki_mult - 1.0) * edge_factor)
        self.kd = self.base_kd * (1.0 + (self.edge_kd_mult - 1.0) * edge_factor)

    def update(self, error, now):
        if abs(error) < self.deadband:
            error = 0.0

        dt = 0.0
        if self.prev_time is not None:
            dt = now - self.prev_time
        self.prev_time = now

        p = self.kp * error

        if dt > 0:
            self.integral += error * dt
            self.integral = clamp(self.integral, self.max_integral)
        i = self.ki * self.integral

        d = 0.0
        if dt > 0:
            d = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        output = p + i + d
        output = clamp(output, self.max_output)

        if abs(error) >= self.deadband and 0 < abs(output) * self.offset < 1:
            output = math.copysign(1.0 / self.offset, output)

        return self.sign * output


def frame_age(frame, wall_now):
    if frame is None or not math.isfinite(frame.ts):
        return float("inf")
    age = wall_now - frame.ts
    return age if age >= 0 else float("inf")


class PreviewLink(DroneLink):
    """Enforce preview isolation at the transport."""
    def _next_packets(self, now):
        return []

    def _send(self, data, port):
        if port != self.video_port or data not in (b"Bv", b"Bw", b"By"):
            raise RuntimeError("Preview blocked a non-video packet")
        super()._send(data, port)


def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
    parameters = cv2.aruco.DetectorParameters()
    # Aggressive detection for edge-of-frame and extreme angles
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.adaptiveThreshConstant = 7
    parameters.minMarkerPerimeterRate = 0.01  # Smaller tags at distance
    parameters.maxMarkerPerimeterRate = 4.0   # Larger tags up close
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.minCornerDistanceRate = 0.01
    parameters.minDistanceToBorder = 0        # Allow tags at image border
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 5
    parameters.cornerRefinementMaxIterations = 30
    parameters.cornerRefinementMinAccuracy = 0.1
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def find_tag_main(image, detector):
    """Use solvePnP (IPPE_SQUARE) for pose estimation with main camera.
    Falls back to corner-based center if solvePnP fails at extreme angles."""
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return None

    half = TAG_SIZE_M / 2.0
    object_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)

    h, w = image.shape[:2]

    for corner, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) != TARGET_ID:
            continue
        points = corner.reshape(4, 2).astype(np.float64)

        # Try solvePnP first for accurate pose
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            points,
            CAMERA_MATRIX,
            DIST_COEFFS,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if ok:
            x, y, z = tvec.flatten()
            # Project tag center to image plane
            center_3d = np.array([[0., 0., 0.]], dtype=np.float64)
            center_2d, _ = cv2.projectPoints(center_3d, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS)
            cx, cy = center_2d[0, 0]
        else:
            # Fallback: use raw corner center (works at extreme angles where solvePnP fails)
            cx, cy = points.mean(axis=0)
            # Estimate Z from apparent tag size
            edge = np.mean(np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1))
            z = float(TAG_SIZE_M * CAMERA_MATRIX[0, 0] / edge) if edge > 0 else 1.0

        # Normalized error: -1 to 1 across image
        ex = float((cx - w/2) / (w/2))
        ey = float((cy - h/2) / (h/2))

        return (ex, ey, float(z)), points.mean(axis=0)
    return None


def find_tag_bottom(image, detector):
    """Use solvePnP (IPPE_SQUARE) for pose estimation with bottom camera (240x320)."""
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return None

    half = TAG_SIZE_M / 2.0
    object_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)

    h, w = image.shape[:2]

    for corner, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) != TARGET_ID:
            continue
        points = corner.reshape(4, 2).astype(np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            points,
            BOTTOM_CAMERA_MATRIX,
            BOTTOM_DIST_COEFFS,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if ok:
            x, y, z = tvec.flatten()
            center_3d = np.array([[0., 0., 0.]], dtype=np.float64)
            center_2d, _ = cv2.projectPoints(center_3d, rvec, tvec, BOTTOM_CAMERA_MATRIX, BOTTOM_DIST_COEFFS)
            cx, cy = center_2d[0, 0]
        else:
            cx, cy = points.mean(axis=0)
            edge = np.mean(np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1))
            z = float(TAG_SIZE_M * BOTTOM_CAMERA_MATRIX[0, 0] / edge) if edge > 0 else 1.0

        ex = float((cx - w/2) / (w/2))
        ey = float((cy - h/2) / (h/2))

        return (ex, ey, float(z)), points.mean(axis=0)
    return None


# ===== Landing Platform Detector (color + shape) =====
# Detects a colored landing pad (e.g., red/orange circle) as backup when AprilTag lost

PLATFORM_HSV_LOWER = np.array([0, 100, 100])      # Red/orange hue range
PLATFORM_HSV_UPPER = np.array([25, 255, 255])
PLATFORM_MIN_AREA = 500                           # Minimum contour area (pixels)
PLATFORM_CIRCULARITY = 0.6                        # How circular (1.0 = perfect circle)
PLATFORM_SIZE_M = 0.50                            # Physical diameter of platform (meters)

class PlatformDetector:
    """Detects a colored circular landing platform using HSV + contour analysis."""
    def __init__(self, hsv_lower=PLATFORM_HSV_LOWER, hsv_upper=PLATFORM_HSV_UPPER,
                 min_area=PLATFORM_MIN_AREA, circularity=PLATFORM_CIRCULARITY):
        self.hsv_lower = hsv_lower
        self.hsv_upper = hsv_upper
        self.min_area = min_area
        self.circularity = circularity

    def detect(self, image):
        """Returns (ex, ey, z_estimate, center_px) or None."""
        h, w = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_score = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circ = 4 * np.pi * area / (perimeter * perimeter)
            if circ < self.circularity:
                continue
            # Score by area * circularity
            score = area * circ
            if score > best_score:
                best_score = score
                best = cnt

        if best is None:
            return None

        # Get center and equivalent diameter
        M = cv2.moments(best)
        if M["m00"] == 0:
            return None
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        eq_diameter = 2 * np.sqrt(cv2.contourArea(best) / np.pi)

        # Estimate Z from apparent size (pinhole camera model)
        # z = focal_length * real_size / pixel_size
        fx = CAMERA_MATRIX[0, 0]
        z = float(fx * PLATFORM_SIZE_M / eq_diameter) if eq_diameter > 0 else 1.0

        # Normalized error
        ex = float((cx - w/2) / (w/2))
        ey = float((cy - h/2) / (h/2))

        return (ex, ey, z), (cx, cy)


def find_platform(image, detector):
    """Wrapper for platform detection."""
    return detector.detect(image)


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
        ex = np.median([o[0] for o in self.history])
        ey = np.median([o[1] for o in self.history])
        z  = np.median([o[2] for o in self.history])
        return (float(ex), float(ey), float(z))

    def reset(self):
        self.history.clear()


class Lander:
    def __init__(self, now, roll_sign=SIGN_ROLL, pitch_sign=SIGN_PITCH,
                 kp=KP, ki=KI, kd=KD, max_xy=MAX_XY, max_integral=MAX_INTEGRAL,
                 deadband=DEADBAND, speed_preset=SPEED_PRESETS[1],
                 edge_kp_mult=EDGE_KP_MULT, edge_ki_mult=EDGE_KI_MULT, edge_kd_mult=EDGE_KD_MULT,
                 edge_warn=EDGE_WARN_THRESHOLD, edge_critical=EDGE_CRITICAL_THRESHOLD,
                 blind_land_z=BLIND_LAND_Z_M, blind_land_max_s=BLIND_LAND_MAX_S,
                 align_tol=ALIGN_TOL, align_hold=ALIGN_HOLD_S,
                 descend_align_tol=DESCEND_ALIGN_TOL, descend_hold=DESCEND_HOLD_S):
        self.state = "ALIGN"
        self.roll_sign = roll_sign
        self.pitch_sign = pitch_sign
        self.edge_warn = edge_warn
        self.edge_critical = edge_critical
        self.blind_land_z = blind_land_z
        self.blind_land_max_s = blind_land_max_s
        self.align_tol = align_tol
        self.align_hold = align_hold
        self.descend_align_tol = descend_align_tol
        self.descend_hold = descend_hold

        roll_offset = speed_preset["roll"]
        pitch_offset = speed_preset["pitch"]
        self.pid_x = PIDController(
            kp, ki, kd, max_xy, max_integral, deadband, roll_sign, roll_offset,
            edge_kp_mult=edge_kp_mult, edge_ki_mult=edge_ki_mult, edge_kd_mult=edge_kd_mult
        )
        self.pid_y = PIDController(
            kp, ki, kd, max_xy, max_integral, deadband, pitch_sign, pitch_offset,
            edge_kp_mult=edge_kp_mult, edge_ki_mult=edge_ki_mult, edge_kd_mult=edge_kd_mult
        )

        self.smoother = ObservationSmoother(window=5)
        self.platform_smoother = ObservationSmoother(window=5)
        self.aligned_since = None
        self.last_seen = now
        self.prev_obs = None
        self.prev_time = None
        # Edge tracking
        self.edge_factor_x = 0.0
        self.edge_factor_y = 0.0
        self.tag_in_view = True
        self.platform_in_view = False
        # Blind landing commit
        self.last_known_z = None
        self.blind_land_start = None
        # Fusion weights: prefer AprilTag (precise), fall back to platform (coarse)
        self.tag_weight = 0.8
        self.platform_weight = 0.2
        # Camera state
        self.current_camera = "main"  # "main" or "bottom"
        self.camera_switch_z = CAMERA_SWITCH_Z
        self.camera_hysteresis = CAMERA_SWITCH_HYSTERESIS

    def _compute_edge_factor(self, ex, ey):
        """Compute edge proximity factor (0.0 = center, 1.0 = critical edge)."""
        max_abs = max(abs(ex), abs(ey))
        if max_abs <= self.edge_warn:
            return 0.0
        elif max_abs >= self.edge_critical:
            return 1.0
        else:
            # Linear interpolation between warn and critical
            return (max_abs - self.edge_warn) / (self.edge_critical - self.edge_warn)

    def update(self, tag_obs, platform_obs, now):
        """Fuse AprilTag (precise) + Platform (coarse) observations."""
        cmd = NEUTRAL.copy()

        if self.state == "LAND":
            return self.state, cmd

        # Fuse observations: prefer tag, fall back to platform
        fused_obs = None
        if tag_obs is not None and all(math.isfinite(v) for v in tag_obs) and tag_obs[2] > 0:
            self.tag_in_view = True
            self.last_seen = now
            smoothed_tag = self.smoother.update(tag_obs)
            fused_obs = smoothed_tag
            self.last_known_z = smoothed_tag[2]
        else:
            self.tag_in_view = False

        if platform_obs is not None and all(math.isfinite(v) for v in platform_obs) and platform_obs[2] > 0:
            self.platform_in_view = True
            smoothed_platform = self.platform_smoother.update(platform_obs)
            if fused_obs is None:
                fused_obs = smoothed_platform
                self.last_known_z = smoothed_platform[2]
            else:
                # Weighted fusion
                ex = self.tag_weight * fused_obs[0] + self.platform_weight * smoothed_platform[0]
                ey = self.tag_weight * fused_obs[1] + self.platform_weight * smoothed_platform[1]
                z  = self.tag_weight * fused_obs[2] + self.platform_weight * smoothed_platform[2]
                fused_obs = (ex, ey, z)
        else:
            self.platform_in_view = False

        # BLIND LANDING COMMIT: if descending and both lost low, continue blind
        if fused_obs is None:
            if self.state == "DESCEND" and self.last_known_z is not None and self.last_known_z < self.blind_land_z:
                if self.blind_land_start is None:
                    self.blind_land_start = now
                    print(f"\n[BLIND LAND] All targets lost at z={self.last_known_z:.2f}m - committing to land")
                elif now - self.blind_land_start < self.blind_land_max_s:
                    cmd["throttle"] = DESCEND_THROTTLE
                    return self.state, cmd
                else:
                    self.state = "LAND"
                    return self.state, NEUTRAL.copy()

            # Normal loss handling
            self.pid_x.reset()
            self.pid_y.reset()
            self.smoother.reset()
            self.platform_smoother.reset()
            self.aligned_since = None
            self.edge_factor_x = 0.0
            self.edge_factor_y = 0.0
            self.blind_land_start = None
            self.state = "LAND" if now - self.last_seen >= TAG_LOST_LAND_S else "ALIGN"
            return self.state, cmd

        # Valid fused observation
        self.blind_land_start = None
        ex, ey, z = fused_obs

        # Camera auto-switch logic
        self._update_camera(z)

        # Edge proximity
        edge_factor = self._compute_edge_factor(ex, ey)
        self.edge_factor_x = edge_factor
        self.edge_factor_y = edge_factor
        self.pid_x.set_edge_multiplier(edge_factor)
        self.pid_y.set_edge_multiplier(edge_factor)

        # PID control
        cmd["roll"] = self.pid_x.update(ex, now)
        cmd["pitch"] = self.pid_y.update(ey, now)

        # State machine
        if self.state == "ALIGN":
            stable = (max(abs(ex), abs(ey)) < self.align_tol)
            if stable:
                if self.aligned_since is None:
                    self.aligned_since = now
                elif now - self.aligned_since >= self.align_hold:
                    self.state = "DESCEND"
                    self.aligned_since = None
            else:
                self.aligned_since = None

        elif self.state == "DESCEND":
            stable = (max(abs(ex), abs(ey)) < self.descend_align_tol)
            if not stable:
                self.state = "ALIGN"
                self.aligned_since = None
            elif z < LAND_Z_M:
                if self.aligned_since is None:
                    self.aligned_since = now
                elif now - self.aligned_since >= self.descend_hold:
                    self.state = "LAND"
                    return self.state, NEUTRAL.copy()
                cmd["throttle"] = DESCEND_THROTTLE
            else:
                self.aligned_since = None
                cmd["throttle"] = DESCEND_THROTTLE

        return self.state, cmd

    def flip_roll_sign(self):
        self.roll_sign *= -1
        self.pid_x.sign = self.roll_sign
        print(f"\n[RUNTIME] Roll sign flipped to {self.roll_sign}")

    def flip_pitch_sign(self):
        self.pitch_sign *= -1
        self.pid_y.sign = self.pitch_sign
        print(f"\n[RUNTIME] Pitch sign flipped to {self.pitch_sign}")

    def adjust_roll_trim(self, delta):
        self.pid_x.offset += delta
        print(f"\n[RUNTIME] Roll offset: {self.pid_x.offset:.2f}")

    def adjust_pitch_trim(self, delta):
        self.pid_y.offset += delta
        print(f"\n[RUNTIME] Pitch offset: {self.pid_y.offset:.2f}")

    def _update_camera(self, z):
        """Auto-switch between main and bottom camera based on altitude."""
        if self.current_camera == "main" and z < self.camera_switch_z:
            self.current_camera = "bottom"
            print(f"\n[CAMERA] Switching to BOTTOM camera at z={z:.2f}m")
        elif self.current_camera == "bottom" and z > self.camera_switch_z + self.camera_hysteresis:
            self.current_camera = "main"
            print(f"\n[CAMERA] Switching to MAIN camera at z={z:.2f}m")

    def get_camera(self):
        return self.current_camera


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


def handle_key(key, link, live, lander=None):
    if key == ord("x"):
        if live:
            link.emergency_stop()
        return "STOP"
    if key in (ord("q"), 27):
        if live and isinstance(link, VitaTakeover):
            link.request_land()
        return "LAND" if live else "STOP"
    # Runtime adjustments
    if lander is not None:
        if key == ord("r"):          # Flip roll sign
            lander.flip_roll_sign()
        elif key == ord("p"):        # Flip pitch sign
            lander.flip_pitch_sign()
        elif key == ord("["):        # Roll trim left
            lander.adjust_roll_trim(-0.05)
        elif key == ord("]"):        # Roll trim right
            lander.adjust_roll_trim(0.05)
        elif key == ord(";"):        # Pitch trim forward
            lander.adjust_pitch_trim(-0.05)
        elif key == ord("'"):        # Pitch trim back
            lander.adjust_pitch_trim(0.05)
    return None


def ensure_main_camera(link, live=False, timeout=12.0):
    deadline, last_toggle = time.monotonic() + timeout, -float("inf")
    while time.monotonic() < deadline:
        action = handle_key(read_key(), link, live, None)
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
                if image.shape[1::-1] == MAIN_SIZE:
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
    parser.add_argument("--kp", type=float, default=KP, help="Proportional gain")
    parser.add_argument("--ki", type=float, default=KI, help="Integral gain")
    parser.add_argument("--kd", type=float, default=KD, help="Derivative gain")
    parser.add_argument("--max-xy", type=float, default=MAX_XY, help="Max stick deflection")
    parser.add_argument("--max-integral", type=float, default=MAX_INTEGRAL, help="Integral anti-windup limit")
    parser.add_argument("--deadband", type=float, default=DEADBAND, help="Deadband threshold")
    parser.add_argument("--align-tol", type=float, default=ALIGN_TOL, help="Alignment tolerance (ALIGN state)")
    parser.add_argument("--align-hold", type=float, default=ALIGN_HOLD_S, help="Hold time before descend (s)")
    parser.add_argument("--descend-align-tol", type=float, default=DESCEND_ALIGN_TOL, help="Alignment tolerance during descent")
    parser.add_argument("--descend-hold", type=float, default=DESCEND_HOLD_S, help="Hold time at land_z before land (s)")
    parser.add_argument("--descend-throttle", type=float, default=DESCEND_THROTTLE, help="Descent throttle")
    parser.add_argument("--land-z", type=float, default=LAND_Z_M, help="Land trigger distance (m)")
    parser.add_argument("--blind-land-z", type=float, default=BLIND_LAND_Z_M, help="Blind land commit altitude (m)")
    parser.add_argument("--blind-land-max-s", type=float, default=BLIND_LAND_MAX_S, help="Max blind descent time (s)")
    parser.add_argument("--trim-file", type=Path, default=ROOT / "trim.json")
    parser.add_argument("--log", type=Path, help="CSV diagnostic output (default: logs/landing_<time>.csv)")
    args = parser.parse_args(argv)
    if not 1 <= args.vita_port <= 65535:
        parser.error("--vita-port must be between 1 and 65535")

    detector = make_detector()
    platform_detector = PlatformDetector()

    cls = DroneLink if args.live else PreviewLink
    link = cls(video=True, local_port=64356, max_offset=SPEED_PRESETS[1], log=print)
    link.set_trim(**load_trim(args.trim_file))

    takeover = VitaTakeover(link, port=args.vita_port, preview=not args.live) if args.vita_takeover else None
    if takeover is not None:
        link = takeover

    print(f"Trim: {link.get_trim()}; roll sign={args.roll_sign}, pitch sign={args.pitch_sign}")
    print(f"PID: KP={args.kp} KI={args.ki} KD={args.kd} MAX_XY={args.max_xy} MAX_I={args.max_integral} DEADBAND={args.deadband}")
    print(f"Align: tol={args.align_tol} hold={args.align_hold}s  Descend: throttle={args.descend_throttle} land_z={args.land_z}m")
    print(f"Blind land: commit_z={args.blind_land_z}m max_s={args.blind_land_max_s}s")
    print(f"Camera: MAIN ({MAIN_SIZE[0]}x{MAIN_SIZE[1]})")
    print("LIVE: T = takeoff (Vita: manual, otherwise requires tag); A = engage Vita-assisted landing; Q/Esc = land; X = motor cut."
          if args.live else "PREVIEW: camera packets only; all flight packets blocked. Q/X = exit.")

    path = args.log or ROOT / "logs" / f"landing_main_{time.time_ns()}.csv"
    path.parent.mkdir(parents=True, exist_ok=True)

    phase, last_seq, cmd = "READY" if args.live else "PREVIEW", -1, NEUTRAL.copy()
    lander = Lander(time.monotonic(), args.roll_sign, args.pitch_sign,
                    kp=args.kp, ki=args.ki, kd=args.kd,
                    max_xy=args.max_xy, max_integral=args.max_integral, deadband=args.deadband,
                    blind_land_z=args.blind_land_z, blind_land_max_s=args.blind_land_max_s,
                    align_tol=args.align_tol, align_hold=args.align_hold,
                    descend_align_tol=args.descend_align_tol, descend_hold=args.descend_hold)
    landing = LandingSequence()
    took_off = False
    takeoff_at = None
    last_good = time.monotonic()
    obs = None

    print(f"Diagnostic log: {path}")
    try:
        cv2.namedWindow("Landing")
        link.start()
        if not ensure_main_camera(link, args.live):
            print("No fresh main-camera stream (or cancelled); no takeoff.")
            return

        if takeover is not None:
            takeover.takeoff_enabled = args.live
            print("Vita: Triangle or PC T = manual takeoff (no tag needed); centre sticks and PC A = engage landing.")

        with path.open("w", newline="") as f:
            log = csv.writer(f)
            log.writerow(["monotonic_s", "phase", "seq", "arrival_age_s",
                          "tag_ex", "tag_ey", "tag_z", "plat_ex", "plat_ey", "plat_z",
                          "roll", "pitch", "throttle", "yaw", "roll_byte", "pitch_byte", "throttle_byte", "yaw_byte",
                          "pid_x_p", "pid_x_i", "pid_x_d", "pid_y_p", "pid_y_i", "pid_y_d"])
            while True:
                now = time.monotonic()
                key = read_key()
                if takeover is not None and takeover.started:
                    took_off = True
                action = handle_key(key, link, args.live, lander)
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
                    tag_obs = None
                    platform_obs = None
                    if image is not None and image.shape[1::-1] == MAIN_SIZE and age <= MAX_FRAME_AGE_S:
                        hit = find_tag_main(image, detector)
                        if hit is not None:
                            tag_obs, centre = hit
                            cv2.circle(image, tuple(np.round(centre).astype(int)), 4, (0, 255, 0), -1)
                        # Platform detection (runs in parallel)
                        plat_hit = find_platform(image, platform_detector)
                        if plat_hit is not None:
                            platform_obs, plat_centre = plat_hit
                            cv2.circle(image, tuple(np.round(plat_centre).astype(int)), 4, (0, 0, 255), -1)
                    now = time.monotonic()
                    age = frame_age(frame, time.time())
                    if age > MAX_FRAME_AGE_S:
                        tag_obs = None
                        platform_obs = None
                    if tag_obs is not None or platform_obs is not None:
                        last_good = now
                    if phase not in ("READY", "TAKEOFF", "LAND", "MANUAL"):
                        if not args.live and lander.state == "LAND" and (tag_obs is not None or platform_obs is not None):
                            lander = Lander(now, args.roll_sign, args.pitch_sign,
                                            kp=args.kp, ki=args.ki, kd=args.kd,
                                            max_xy=args.max_xy, max_integral=args.max_integral, deadband=args.deadband,
                                            blind_land_z=args.blind_land_z, blind_land_max_s=args.blind_land_max_s,
                                            align_tol=args.align_tol, align_hold=args.align_hold,
                                            descend_align_tol=args.descend_align_tol, descend_hold=args.descend_hold)
                        state, cmd = lander.update(tag_obs, platform_obs, now)
                        phase = state if args.live else "PREVIEW"
                    if image is not None:
                        h, w = image.shape[:2]
                        # Draw frame center crosshair
                        cv2.line(image, (w//2 - 15, h//2), (w//2 + 15, h//2), (100, 100, 100), 1)
                        cv2.line(image, (w//2, h//2 - 15), (w//2, h//2 + 15), (100, 100, 100), 1)

                        cv2.putText(image, f"{phase}/{lander.state} age={age:.2f}s", (5, 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                        cv2.putText(image, f"R_sign={lander.roll_sign} P_sign={lander.pitch_sign} R_off={lander.pid_x.offset:.2f} P_off={lander.pid_y.offset:.2f}", (5, 35),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
                        y_pos = 50
                        # Show tag observation
                        if tag_obs is not None:
                            ex, ey, z = tag_obs
                            cv2.putText(image, f"TAG ex={ex:+.2f} ey={ey:+.2f} z~{z:.2f}m", (5, y_pos),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                            tag_x = int(w//2 + ex * w/2)
                            tag_y = int(h//2 + ey * h/2)
                            cv2.circle(image, (tag_x, tag_y), 8, (0, 255, 0), 2)
                            cv2.line(image, (w//2, h//2), (tag_x, tag_y), (0, 255, 0), 1)
                            y_pos += 20
                        # Show platform observation
                        if platform_obs is not None:
                            ex, ey, z = platform_obs
                            cv2.putText(image, f"PLAT ex={ex:+.2f} ey={ey:+.2f} z~{z:.2f}m", (5, y_pos),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                            plat_x = int(w//2 + ex * w/2)
                            plat_y = int(h//2 + ey * h/2)
                            cv2.circle(image, (plat_x, plat_y), 8, (0, 0, 255), 2)
                            cv2.line(image, (w//2, h//2), (plat_x, plat_y), (0, 0, 255), 1)
                            y_pos += 20
                        # Show PID integrals
                        cv2.putText(image, f"IX={lander.pid_x.integral:+.3f} IY={lander.pid_y.integral:+.3f}", (5, y_pos),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                        y_pos += 20
                        # Blind landing indicator
                        if lander.state == "DESCEND" and lander.blind_land_start is not None:
                            cv2.putText(image, "BLIND LAND", (5, y_pos),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                        cv2.imshow("Landing", image)
                if args.live and phase == "READY" and key == ord("t"):
                    if takeover is None and (tag_obs is None and platform_obs is None or age > MAX_FRAME_AGE_S):
                        print("Takeoff refused: no fresh target.")
                    elif link.takeoff():
                        took_off, takeoff_at, last_good, phase = True, now, now, "MANUAL" if takeover is not None else "TAKEOFF"
                if args.live and takeover is not None and key == ord("a"):
                    if tag_obs is None and platform_obs is None or age > MAX_FRAME_AGE_S:
                        print("Engagement refused: no fresh target; remain in manual control.")
                    elif takeover.engage():
                        lander = Lander(now, args.roll_sign, args.pitch_sign,
                                        kp=args.kp, ki=args.ki, kd=args.kd,
                                        max_xy=args.max_xy, max_integral=args.max_integral, deadband=args.deadband,
                                        blind_land_z=args.blind_land_z, blind_land_max_s=args.blind_land_max_s,
                                        align_tol=args.align_tol, align_hold=args.align_hold,
                                        descend_align_tol=args.descend_align_tol, descend_hold=args.descend_hold)
                        phase, cmd, last_good = "ALIGN", NEUTRAL.copy(), now
                        print("Autonomous landing engaged; move a Vita stick to take control back.")
                    else:
                        print("Engagement refused: take off first and centre the connected Vita sticks.")
                if phase == "TAKEOFF" and now - takeoff_at >= TAKEOFF_WAIT_S:
                    lander = Lander(now, args.roll_sign, args.pitch_sign,
                                    kp=args.kp, ki=args.ki, kd=args.kd,
                                    max_xy=args.max_xy, max_integral=args.max_integral, deadband=args.deadband,
                                    blind_land_z=args.blind_land_z, blind_land_max_s=args.blind_land_max_s,
                                    align_tol=args.align_tol, align_hold=args.align_hold,
                                    descend_align_tol=args.descend_align_tol, descend_hold=args.descend_hold)
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
                tag_vals = tag_obs if tag_obs else ("", "", "")
                plat_vals = platform_obs if platform_obs else ("", "", "")
                wire = [link._stick_byte(axis, send[axis]) for axis in NEUTRAL]
                # Use fused observation for PID logging (approximate)
                fused_ex = (tag_obs[0] if tag_obs else 0) * lander.tag_weight + (platform_obs[0] if platform_obs else 0) * lander.platform_weight
                fused_ey = (tag_obs[1] if tag_obs else 0) * lander.tag_weight + (platform_obs[1] if platform_obs else 0) * lander.platform_weight
                pid_log = [lander.pid_x.kp * fused_ex,
                           lander.pid_x.integral * lander.pid_x.ki,
                           lander.pid_x.kd * ((fused_ex - lander.pid_x.prev_error) / 0.02),
                           lander.pid_y.kp * fused_ey,
                           lander.pid_y.integral * lander.pid_y.ki,
                           lander.pid_y.kd * ((fused_ey - lander.pid_y.prev_error) / 0.02)]
                log.writerow([now, phase, last_seq, age, *tag_vals, *plat_vals, *[send[k] for k in NEUTRAL], *wire, *pid_log])
                if new:
                    blind_str = " BLIND" if lander.state == "DESCEND" and lander.blind_land_start is not None else ""
                    print(f"\r{phase:8} age={age:.2f}s tag={tag_obs is not None} plat={platform_obs is not None}{blind_str} "
                          f"R/P={send['roll']:+.2f}/{send['pitch']:+.2f} bytes={wire[:2]} I={lander.pid_x.integral:+.3f}/{lander.pid_y.integral:+.3f}   ", end="", flush=True)
                    f.flush()
                time.sleep(0.01)
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
                if handle_key(read_key(), link, True, lander) == "STOP":
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