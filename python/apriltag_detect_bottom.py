
import argparse
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11
TARGET_ID = 0
TAG_SIZE_M = 0.100  # Assumed black-square side length: 100 mm

CALIBRATION_FILE = (
    Path(__file__).resolve().parent.parent
    / "calibration"
    / "bottom_camera"
    / "calibration_25mm.npz"
)

WINDOW_NAME = "Drone AprilTag Detection"
LOCAL_PORT = 64356

# Number of valid detections used for median smoothing.
SMOOTHING_WINDOW = 5

# Prevent accidental rapid camera toggles.
SWITCH_COOLDOWN_S = 1.2

# Camera resolutions confirmed during testing.
BOTTOM_SIZE = (240, 320)
MAIN_SIZE = (640, 480)


# ------------------------------------------------------------
# Calibration and detector
# ------------------------------------------------------------

def load_calibration():
    if not CALIBRATION_FILE.exists():
        raise SystemExit(
            f"Calibration file not found:\n{CALIBRATION_FILE}"
        )

    with np.load(CALIBRATION_FILE) as data:
        K = data["camera_matrix"].astype(np.float64)
        D = data["dist_coeffs"].astype(np.float64)
        expected = (
            int(data["image_width"]),
            int(data["image_height"]),
        )
        rms = float(data["rms"])

    print(f"Calibration file: {CALIBRATION_FILE}")
    print(f"Calibration resolution: {expected[0]}x{expected[1]}")
    print(f"Calibration RMS: {rms:.3f} pixels")
    print(f"Assumed tag size: {TAG_SIZE_M:.3f} m")
    print(f"Target AprilTag ID: {TARGET_ID}")

    if expected != BOTTOM_SIZE:
        print(
            "WARNING: Calibration resolution differs from the "
            f"expected bottom-camera resolution {BOTTOM_SIZE}."
        )

    if rms > 2.0:
        print(
            "WARNING: Calibration RMS is relatively high. "
            "Pose accuracy may be affected."
        )

    print(
        "WARNING: Tag size and calibration accuracy have not "
        "been independently verified."
    )

    return K, D, expected


def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
    parameters = cv2.aruco.DetectorParameters()

    # Retain OpenCV defaults for the initial working version.
    return cv2.aruco.ArucoDetector(dictionary, parameters)


# ------------------------------------------------------------
# AprilTag detection and pose estimation
# ------------------------------------------------------------

def detect(frame, detector, K, D):
    output = frame.copy()
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return output, []

    cv2.aruco.drawDetectedMarkers(output, corners, ids)

    found = []
    half = TAG_SIZE_M / 2.0

    # Required corner ordering for SOLVEPNP_IPPE_SQUARE:
    # top-left, top-right, bottom-right, bottom-left.
    object_points = np.array(
        [
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ],
        dtype=np.float64,
    )

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)

        if marker_id != TARGET_ID:
            continue

        points = marker_corners.reshape(4, 2).astype(np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            points,
            K,
            D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok:
            continue

        x, y, z = tvec.flatten()
        distance = float(np.linalg.norm(tvec))

        cv2.drawFrameAxes(
            output,
            K,
            D,
            rvec,
            tvec,
            TAG_SIZE_M * 0.5,
        )

        edge_lengths = [
            float(np.linalg.norm(points[(i + 1) % 4] - points[i]))
            for i in range(4)
        ]
        mean_edge = float(np.mean(edge_lengths))

        label = f"ID {marker_id} | Z={z:.3f}m | R={distance:.3f}m"
        label_y = max(65, int(points[0, 1]) - 8)

        cv2.putText(
            output,
            label,
            (5, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        found.append(
            {
                "id": marker_id,
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "distance": distance,
                "mean_edge_px": mean_edge,
            }
        )

    return output, found


# ------------------------------------------------------------
# Camera-switch controls
# ------------------------------------------------------------

def request_camera_switch(state):
    now = time.monotonic()

    if now - state["last_switch_time"] < SWITCH_COOLDOWN_S:
        print("Please wait before switching cameras again.")
        return

    print("Sending camera toggle 42 79...")

    try:
        state["link"]._send(
            bytes.fromhex("42 79"),
            state["link"].video_port,
        )
        state["last_switch_time"] = now

        # Do not mix measurements from before and after a switch.
        state["history"].clear()
        state["last_logged"] = None

    except Exception as exc:
        print(f"Camera switch command failed: {exc}")


def mouse_callback(event, x, y, flags, state):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Clickable button in the top-left corner.
        if 10 <= x <= 190 and 10 <= y <= 50:
            request_camera_switch(state)


def draw_switch_button(image):
    cv2.rectangle(
        image,
        (10, 10),
        (190, 50),
        (45, 45, 45),
        -1,
    )
    cv2.rectangle(
        image,
        (10, 10),
        (190, 50),
        (220, 220, 220),
        1,
    )
    cv2.putText(
        image,
        "SWITCH CAMERA",
        (19, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


# ------------------------------------------------------------
# Median smoothing
# ------------------------------------------------------------

def get_smoothed_pose(history):
    if not history:
        return None

    keys = ("x", "y", "z", "distance")

    return {
        key: float(np.median([item[key] for item in history]))
        for key in keys
    }


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Bottom-camera AprilTag detection and pose estimation."
    )
    parser.add_argument(
        "--image",
        type=Path,
        help="Test a saved bottom-camera image instead of live video.",
    )
    args = parser.parse_args()

    K, D, expected_size = load_calibration()
    detector = make_detector()

    # Saved-image testing does not require a drone connection.
    if args.image:
        image = cv2.imread(str(args.image))

        if image is None:
            raise SystemExit(f"Could not read image: {args.image}")

        h, w = image.shape[:2]

        if (w, h) != expected_size:
            raise SystemExit(
                f"Image is {w}x{h}; calibration expects "
                f"{expected_size[0]}x{expected_size[1]}."
            )

        output, found = detect(image, detector, K, D)

        if found:
            for item in found:
                print(item)
        else:
            print(f"No AprilTag ID {TARGET_ID} detected.")

        cv2.imshow(WINDOW_NAME, output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    from drone import DroneLink

    link = DroneLink(video=True, local_port=LOCAL_PORT)

    state = {
        "link": link,
        "last_switch_time": -1e9,
        "history": deque(maxlen=SMOOTHING_WINDOW),
        "last_logged": None,
    }

    last_seq = -1
    last_size = None
    last_status_time = 0.0

    try:
        print("Starting drone video. Keep the drone stationary.")
        link.start()

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback, state)

        print()
        print("Controls:")
        print("  Click SWITCH CAMERA or press C: toggle camera")
        print("  Q or Esc: quit")
        print()
        print("AprilTag detection runs with bottom-camera calibration only.")

        while True:
            link.heartbeat()

            frame = link.get_frame()

            if frame is not None and frame.seq != last_seq:
                last_seq = frame.seq

                image = cv2.imdecode(
                    np.frombuffer(frame.jpeg, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )

                if image is not None:
                    h, w = image.shape[:2]
                    size = (w, h)

                    if size != last_size:
                        print(f"Video resolution: {w}x{h}")
                        last_size = size

                    if size == expected_size:
                        output, found = detect(
                            image,
                            detector,
                            K,
                            D,
                        )

                        target = next(
                            (
                                item for item in found
                                if item["id"] == TARGET_ID
                            ),
                            None,
                        )

                        if target is not None:
                            state["history"].append(target)
                            smooth = get_smoothed_pose(state["history"])

                            # Raw estimate: current frame only.
                            cv2.putText(
                                output,
                                (
                                    f"RAW Z={target['z']:.3f}m "
                                    f"R={target['distance']:.3f}m"
                                ),
                                (5, 70),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.43,
                                (0, 255, 255),
                                1,
                                cv2.LINE_AA,
                            )

                            # Median estimate over the last 5 detections.
                            cv2.putText(
                                output,
                                (
                                    f"MEDIAN({len(state['history'])}) "
                                    f"Z={smooth['z']:.3f}m "
                                    f"R={smooth['distance']:.3f}m"
                                ),
                                (5, 90),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.43,
                                (0, 255, 0),
                                1,
                                cv2.LINE_AA,
                            )

                            cv2.putText(
                                output,
                                (
                                    f"X={smooth['x']:.3f}m "
                                    f"Y={smooth['y']:.3f}m"
                                ),
                                (5, 110),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.43,
                                (255, 255, 0),
                                1,
                                cv2.LINE_AA,
                            )

                            # Print only once per new video frame with a detection.
                            print(
                                f"RAW: X={target['x']:.3f} "
                                f"Y={target['y']:.3f} "
                                f"Z={target['z']:.3f}m "
                                f"Range={target['distance']:.3f}m | "
                                f"Median({len(state['history'])}): "
                                f"X={smooth['x']:.3f} "
                                f"Y={smooth['y']:.3f} "
                                f"Z={smooth['z']:.3f}m "
                                f"Range={smooth['distance']:.3f}m | "
                                f"edge={target['mean_edge_px']:.1f}px"
                            )

                            state["last_logged"] = time.monotonic()

                        else:
                            # Do not keep displaying a stale pose when the tag disappears.
                            state["history"].clear()

                            cv2.putText(
                                output,
                                f"Target ID {TARGET_ID} not detected",
                                (5, 70),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.45,
                                (0, 165, 255),
                                1,
                                cv2.LINE_AA,
                            )

                        cv2.putText(
                            output,
                            "BOTTOM CAMERA",
                            (5, h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            (0, 255, 0),
                            1,
                            cv2.LINE_AA,
                        )

                    else:
                        # Display other camera resolutions without using
                        # the bottom-camera calibration on them.
                        output = image.copy()
                        state["history"].clear()

                        camera_name = (
                            "MAIN CAMERA"
                            if size == MAIN_SIZE
                            else f"UNRECOGNISED CAMERA {w}x{h}"
                        )

                        cv2.putText(
                            output,
                            camera_name,
                            (5, 75),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 165, 255),
                            1,
                            cv2.LINE_AA,
                        )

                        cv2.putText(
                            output,
                            "Pose estimation paused for this resolution",
                            (5, 100),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.38,
                            (0, 165, 255),
                            1,
                            cv2.LINE_AA,
                        )

                    draw_switch_button(output)
                    cv2.imshow(WINDOW_NAME, output)

            # Process key presses even if a new video frame has not arrived.
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord("c"):
                request_camera_switch(state)

            # Periodic status message if the target is not being detected.
            now = time.monotonic()
            if now - last_status_time >= 5.0:
                if state["last_logged"] is None:
                    print("Waiting for target tag detection...")
                elif now - state["last_logged"] >= 5.0:
                    print("No recent target detection.")
                last_status_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        cv2.destroyAllWindows()
        link.close(stop_motors=False)
        print("Detector stopped.")


if __name__ == "__main__":
    main()
