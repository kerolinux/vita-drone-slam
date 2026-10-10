"""Drone video viewer with stream statistics and clean-frame recording.

Keys (click the video window first):
  r  start/stop recording complete frames to recordings/<timestamp>/ (one JPEG per frame, timestamped)
  c  save one frame to calib/ (for camera calibration with a checkerboard)
  q  quit

Only frames where every packet arrived and the JPEG decodes are shown or saved.
The timestamps are PC arrival times, not camera capture times, so there is an unknown latency.

Run with your PC on the drone's Wi-Fi, props removed. Power-cycle the drone first if the stream is stuck.
Install first:  pip install opencv-python numpy
"""
import os
import socket
import time

try:
    import cv2  # type: ignore[reportMissingImports]
except ImportError as exc:
    raise RuntimeError("Install required dependencies: pip install opencv-python numpy") from exc
try:
    import numpy as np  # type: ignore[reportMissingImports]
except ImportError as exc:
    raise RuntimeError("Install required dependencies: pip install opencv-python numpy") from exc

DRONE_IP = "192.168.4.153"
DRONE_PORT = 8080
LOCAL_PORT = 40238
START = bytes([0x42, 0x76])
STOP = bytes([0x42, 0x77])
STATS_INTERVAL = 5.0


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", LOCAL_PORT))
    sock.settimeout(1.0)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)

    for _ in range(3):
        sock.sendto(START, (DRONE_IP, DRONE_PORT))
        time.sleep(0.1)
    print("Start command sent, waiting for packets...")

    parts = []
    current_id = None

    # Counters for the current stats window
    complete = incomplete = abandoned = 0
    sizes = []
    resolution = None
    window_start = time.time()

    rec_dir = None
    saved = 0
    calib_dir = "calib"
    calib_count = 0

    def print_stats():
        nonlocal complete, incomplete, abandoned, sizes, window_start
        elapsed = time.time() - window_start
        total = complete + incomplete + abandoned
        if total == 0:
            print("stats: no frames in this window")
        else:
            avg_kb = (sum(sizes) / len(sizes) / 1024) if sizes else 0
            print(f"stats: {complete / elapsed:.1f} fps complete | "
                  f"{100 * complete / total:.0f}% of frames complete "
                  f"({complete} ok, {incomplete} incomplete, {abandoned} abandoned) | "
                  f"resolution {resolution} | avg {avg_kb:.1f} KB/frame")
        complete = incomplete = abandoned = 0
        sizes = []
        window_start = time.time()

    try:
        while True:
            if time.time() - window_start >= STATS_INTERVAL:
                print_stats()

            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                print("No packets in the last second - resending start")
                sock.sendto(START, (DRONE_IP, DRONE_PORT))
                continue

            if len(data) < 8 or data[4:7] != b"TZH":
                continue

            frame_id, last_flag, expected = data[0], data[1], data[2]

            if frame_id != current_id:
                if current_id is not None and parts:
                    abandoned += 1
                current_id = frame_id
                parts = []
            parts.append(data[8:])

            if last_flag != 1:
                continue

            ok = False
            jpg = None
            img = None
            if len(parts) == expected:
                raw = b"".join(parts)
                soi = raw.find(b"\xff\xd8")
                eoi = raw.find(b"\xff\xd9", soi) if soi != -1 else -1
                if soi != -1 and eoi != -1:
                    jpg = raw[soi:eoi + 2]
                    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    ok = img is not None

            current_id = None
            parts = []

            if not ok:
                incomplete += 1
                continue

            complete += 1
            sizes.append(len(jpg))
            resolution = f"{img.shape[1]}x{img.shape[0]}"
            now = time.time()

            if rec_dir is not None:
                with open(os.path.join(rec_dir, f"{now:.3f}.jpg"), "wb") as f:
                    f.write(jpg)
                saved += 1

            display = img.copy()
            if rec_dir is not None:
                cv2.putText(display, f"REC {saved}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("drone (r=record, c=calib image, q=quit)", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                if rec_dir is None:
                    rec_dir = os.path.join("recordings", time.strftime("%Y%m%d_%H%M%S"))
                    os.makedirs(rec_dir, exist_ok=True)
                    saved = 0
                    print(f"Recording to {rec_dir}")
                else:
                    print(f"Stopped recording, {saved} frames saved in {rec_dir}")
                    rec_dir = None
            elif key == ord("c"):
                os.makedirs(calib_dir, exist_ok=True)
                path = os.path.join(calib_dir, f"calib_{calib_count:03d}.jpg")
                with open(path, "wb") as f:
                    f.write(jpg)
                calib_count += 1
                print(f"Saved {path}")
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            sock.sendto(STOP, (DRONE_IP, DRONE_PORT))
            time.sleep(0.05)
        sock.close()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()