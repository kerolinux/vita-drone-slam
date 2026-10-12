"""capture_calib.py - collect checkerboard photos for calibrate.py from the drone's live video.

The drone stays on the ground: this script only watches the video (it sends neutral sticks and nothing else).
Hold the printed checkerboard in front of the camera. The corners are drawn on the picture when the board is
found (green = found). Press SPACE to save a photo (only works while the board is found), Q to finish.

Move the board between photos: near, far, tilted left/right/up/down, and into every corner and edge of the
picture. 20-30 different views make a good calibration.
"""
import argparse
import os
import time

import cv2
import numpy as np

from calibrate import find_corners
from drone import DroneLink


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9, help="inner corners across")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down")
    ap.add_argument("--out", default="calib")
    ap.add_argument("--ip", default="192.168.4.153")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    saved = len([f for f in os.listdir(args.out) if f.lower().endswith((".jpg", ".png"))])
    link = DroneLink(ip=args.ip, land_after=None, cut_after=None)
    win = "calibration photos (SPACE = save, Q = finish)"
    last_seq, img, found, last_save = 0, None, False, 0.0
    link.start()
    cv2.namedWindow(win)
    try:
        while True:
            link.heartbeat()                                    # neutral sticks only, nothing flies
            frame = link.get_frame()
            if frame and frame.seq != last_seq:
                last_seq = frame.seq
                img = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    corners = find_corners(gray, args.cols, args.rows, fast=True)
                    found = corners is not None
                    shown = img.copy()
                    if found:
                        cv2.drawChessboardCorners(shown, (args.cols, args.rows), corners, True)
                    cv2.putText(shown, f"saved {saved}   board {'FOUND' if found else 'not found'}", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if found else (0, 0, 255), 2)
                    cv2.imshow(win, shown)
            key = cv2.waitKey(10) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == 32 and frame and found and time.time() - last_save > 0.5:
                with open(os.path.join(args.out, f"calib_{saved:03d}.jpg"), "wb") as f:
                    f.write(frame.jpeg)                         # the original JPEG, without the drawn corners
                saved += 1
                last_save = time.time()
                print(f"saved photo {saved}")
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        cv2.destroyAllWindows()
        print(f"Done: {saved} photos in {args.out}. Next: python calibrate.py --images {args.out}")


if __name__ == "__main__":
    main()