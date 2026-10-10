"""Debug version of the drone video test.

Prints what happens to each frame and saves a few files for inspection:
  packets.txt     - header bytes of the first 60 video packets
  first_frame.jpg - the first JPEG found, even if the frame was incomplete

Run with your PC on the drone's Wi-Fi, props removed. Power-cycle the drone first.
Install first:  pip install opencv-python numpy
"""
import socket
import time

import cv2
import numpy as np

DRONE_IP = "192.168.4.153"
DRONE_PORT = 8080
LOCAL_PORT = 40238
START = bytes([0x42, 0x76])
STOP = bytes([0x42, 0x77])
MAX_FRAME_LOGS = 30


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", LOCAL_PORT))
    sock.settimeout(1.0)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)

    for _ in range(3):
        sock.sendto(START, (DRONE_IP, DRONE_PORT))
        time.sleep(0.1)
    print("Start command sent, waiting for packets...")

    log = open("packets.txt", "w")
    parts = []
    current_id = None
    video_pkts = 0
    other_pkts = 0
    frame_logs = 0
    jpeg_found = 0
    shown = 0

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                print(f"No packets in the last second (video: {video_pkts}, other: {other_pkts}) - resending start")
                sock.sendto(START, (DRONE_IP, DRONE_PORT))
                continue

            if len(data) < 8 or data[4:7] != b"TZH":
                other_pkts += 1
                continue

            video_pkts += 1
            frame_id, last_flag, expected = data[0], data[1], data[2]

            if video_pkts <= 60:
                log.write(f"len={len(data)} header={data[:16].hex(' ')}\n")
                log.flush()

            if frame_id != current_id:
                if current_id is not None and parts and frame_logs < MAX_FRAME_LOGS:
                    print(f"  frame {current_id} abandoned with {len(parts)} packets (no last-flag packet seen)")
                    frame_logs += 1
                current_id = frame_id
                parts = []
            parts.append(data[8:])

            if last_flag == 1:
                raw = b"".join(parts)
                soi = raw.find(b"\xff\xd8")
                eoi = raw.find(b"\xff\xd9", soi) if soi != -1 else -1
                if frame_logs < MAX_FRAME_LOGS:
                    print(f"frame {frame_id}: {len(parts)}/{expected} packets, {len(raw)} bytes, "
                          f"SOI at {soi}, EOI at {eoi}")
                    frame_logs += 1

                if soi != -1 and eoi != -1:
                    jpg = raw[soi:eoi + 2]
                    jpeg_found += 1
                    if jpeg_found == 1:
                        with open("first_frame.jpg", "wb") as f:
                            f.write(jpg)
                    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        shown += 1
                        cv2.imshow("drone", img)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break

                current_id = None
                parts = []
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            sock.sendto(STOP, (DRONE_IP, DRONE_PORT))
            time.sleep(0.05)
        sock.close()
        log.close()
        cv2.destroyAllWindows()
        print(f"Done. video packets: {video_pkts}, other packets: {other_pkts}, "
              f"JPEGs found: {jpeg_found}, frames displayed: {shown}")


if __name__ == "__main__":
    main()