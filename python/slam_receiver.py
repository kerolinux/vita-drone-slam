#!/usr/bin/env python3
"""Receive timestamped JPEG frames over TCP and feed live_slam via stdin."""

import argparse
import socket
import struct
import subprocess
import time
from pathlib import Path


def read_exact(conn, size):
    data = bytearray()
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            if not data:
                return None
            raise ConnectionError("Connection closed during frame")
        data.extend(chunk)
    return bytes(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5011)
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    worker = root / "build/slam/live_slam.new"
    vocabulary = Path.home() / "ORB_SLAM3/Vocabulary/ORBvoc.txt"
    settings = root / "drone_orbslam3.yaml"
    output = root / "trajectories" / f"relay_{time.strftime('%Y%m%d_%H%M%S')}"

    for path in (worker, vocabulary, settings):
        if not path.is_file():
            raise FileNotFoundError(path)

    output.parent.mkdir(parents=True, exist_ok=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)

    print(f"[RELAY] Listening on {args.host}:{args.port}", flush=True)
    print("[RELAY] Waiting for the Windows bridge...", flush=True)

    try:
        conn, address = server.accept()
        with conn:
            print(f"[RELAY] Connected from {address}", flush=True)
            process = subprocess.Popen(
                [
                    str(worker), str(vocabulary), str(settings),
                    str(output), "5010",
                ],
                stdin=subprocess.PIPE,
                bufsize=0,
            )

            frames = 0
            try:
                while True:
                    header = read_exact(conn, 12)
                    if header is None:
                        break

                    timestamp, length = struct.unpack("=dI", header)
                    if not 4 <= length <= 2_000_000:
                        raise ValueError(f"Invalid JPEG length: {length}")

                    jpeg = read_exact(conn, length)
                    if jpeg is None:
                        raise ConnectionError("Missing JPEG payload")
                    if not jpeg.startswith(b"\xff\xd8"):
                        raise ValueError("Frame does not start with JPEG SOI")

                    process.stdin.write(header)
                    process.stdin.write(jpeg)
                    frames += 1

                    if frames % 30 == 0:
                        print(f"[RELAY] Forwarded {frames} frames", flush=True)

            finally:
                if process.stdin and not process.stdin.closed:
                    process.stdin.close()
                try:
                    code = process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        code = process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        code = process.wait()
                print(
                    f"[RELAY] Worker exited {code}; frames received: {frames}",
                    flush=True,
                )
    finally:
        server.close()


if __name__ == "__main__":
    main()
