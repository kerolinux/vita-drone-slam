
"""AprilTag detection and pose estimation for the E4 drone camera."""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

# Camera calibration for the 640x480 drone camera.
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

def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
    parameters = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, parameters)

def detect(frame, detector):
    corners, ids, _ = detector.detectMarkers(frame)
    output = frame.copy()

    if ids is None:
        return output, []

    cv2.aruco.drawDetectedMarkers(output, corners, ids)
    found = []

    # IPPE_SQUARE expects these four corners in this order.
    half = TAG_SIZE_M / 2.0
    object_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        points = marker_corners.reshape(4, 2)

        if marker_id != TARGET_ID:
            continue

        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            points.astype(np.float64),
            CAMERA_MATRIX,
            DIST_COEFFS,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok:
            print("Tag detected, but pose estimation failed.")
            continue

        x, y, z = tvec.flatten()
        distance = float(np.linalg.norm(tvec))

        cv2.drawFrameAxes(
            output, CAMERA_MATRIX, DIST_COEFFS,
            rvec, tvec, TAG_SIZE_M * 0.5
        )

        label = f"ID {marker_id} | Z={z:.2f}m"
        cv2.putText(
            output, label,
            tuple(points[0].astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (0, 255, 0), 2
        )

        found.append({
            "id": marker_id,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "distance": distance,
        })

    return output, found

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image", type=Path,
        help="Detect tags in a saved image instead of the live drone feed."
    )
    args = parser.parse_args()

    detector = make_detector()

    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise SystemExit(f"Could not read image: {args.image}")

        output, found = detect(frame, detector)
        print("Detections:", found or "No target tag found")

        window_name = "AprilTag Detection"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 800, 600)
        cv2.imshow(window_name, display)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    # Import from python/drone.py; no bridge.py changes required.
    from drone import DroneLink

    
    link = DroneLink(video=True, local_port=64356)
    last_seq = -1

    try:
        print("Starting video. Keep the drone on the ground.")
        link.start()

        # Observe the stream for 2 seconds before deciding whether to toggle.
        counts = {}
        deadline = time.monotonic() + 2.0

        while time.monotonic() < deadline:
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
                    counts[size] = counts.get(size, 0) + 1

            time.sleep(0.005)

        print(f"Startup camera resolutions: {counts}")

        if not counts:
            raise RuntimeError("No valid video frames received.")

        current_size = max(counts, key=counts.get)

        if current_size == (640, 480):
            print("Main camera detected. Toggling to bottom camera.")
            link._send(bytes.fromhex("42 79"), link.video_port)

            # Wait until bottom-camera frames are actually received.
            deadline = time.monotonic() + 6.0
            bottom_frames = 0

            while time.monotonic() < deadline:
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

                        if (w, h) == (240, 320):
                            bottom_frames += 1
                            if bottom_frames >= 3:
                                break
                        else:
                            bottom_frames = 0

                time.sleep(0.005)

            if bottom_frames < 3:
                raise RuntimeError(
                    "Bottom camera was not confirmed. "
                    "Restart after checking the camera state."
                )

            print("Bottom camera confirmed.")

        elif current_size == (240, 320):
            print("Bottom camera already active. Not toggling.")

        else:
            raise RuntimeError(
                f"Unknown starting resolution: {current_size}"
            )

        print("AprilTag detection running. Press Q to quit.")

        while True:
            link.heartbeat()
            frame = link.get_frame()

            if frame is None or frame.seq == last_seq:
                time.sleep(0.005)
                continue

            last_seq = frame.seq
            image = cv2.imdecode(
                np.frombuffer(frame.jpeg, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )

            if image is None or image.shape[:2] != (320, 240):
                continue

            output, found = detect(image, detector, K, D)

            for item in found:
                print(
                    f"id={item['id']} "
                    f"x={item['x']:.3f}m "
                    f"y={item['y']:.3f}m "
                    f"z={item['z']:.3f}m "
                    f"distance={item['distance']:.3f}m"
                )

            cv2.imshow("Bottom Camera AprilTag Detection", output)

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        link.close(stop_motors=False)


if __name__ == "__main__":
    main()
