"""videoforward.py - sends the drone's video to the Vita as small RGB565 images (format: see vitalink.py).

The Vita is slow at decoding JPEG, so the PC does the work: decode, shrink, convert to RGB565, send in rows.
Bandwidth is about width * height * 2 * fps bytes per second (320x240 at 8 fps is about 1.2 MB/s, roughly
10 Mbit/s). If the picture stutters, lower --vita-fps or --vita-size.
"""
import socket
import threading
import time

import vitalink as vl

DEFAULT_SIZE = (320, 240)
BURST = 20          # packets sent back to back before a short pause, so the Vita's receive buffer keeps up


def to_rgb565(bgr):
    """numpy uint8 array (height, width, 3) in OpenCV's BGR order -> little-endian uint16 RGB565."""
    import numpy as np
    r = (bgr[:, :, 2] >> 3).astype(np.uint16)
    g = (bgr[:, :, 1] >> 2).astype(np.uint16)
    b = (bgr[:, :, 0] >> 3).astype(np.uint16)
    return ((r << 11) | (g << 5) | b).astype("<u2")


def encode_chunks(jpeg, frame_id, size=DEFAULT_SIZE):
    """JPEG bytes -> list of video packets for the Vita. Returns [] if the JPEG cannot be decoded."""
    import cv2
    import numpy as np
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return []
    width, height = size
    if img.shape[1] != width or img.shape[0] != height:
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    pixels = to_rgb565(img)
    rows_per_packet = max(1, vl.VIDEO_MAX_PAYLOAD // (width * 2))
    chunks = []
    for row in range(0, height, rows_per_packet):
        rows = min(rows_per_packet, height - row)
        chunks.append(vl.pack_video_chunk(frame_id, width, height, row, rows, row + rows >= height,
                                          pixels[row:row + rows].tobytes()))
    return chunks


class VideoForwarder:
    """Background thread: newest frame from link.get_frame() -> Vita, at most `fps` frames per second."""

    def __init__(self, link, get_dest, size=DEFAULT_SIZE, fps=8.0, log=print):
        self.link, self.get_dest, self.size, self.fps, self._log = link, get_dest, size, fps, log
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = False
        self._thread = None
        self.frames_sent = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass

    def _run(self):
        last_seq, last_sent, frame_id = 0, 0.0, 0
        while self._running:
            frame = self.link.get_frame()
            dest = self.get_dest()
            now = time.monotonic()
            if frame is None or frame.seq == last_seq or dest is None or now - last_sent < 1.0 / self.fps:
                time.sleep(0.005)
                continue
            last_seq, last_sent = frame.seq, now
            chunks = encode_chunks(frame.jpeg, frame_id, self.size)
            frame_id = (frame_id + 1) & 0xFF
            try:
                for i, chunk in enumerate(chunks):
                    self._sock.sendto(chunk, dest)
                    if (i + 1) % BURST == 0:
                        time.sleep(0.002)
            except OSError:
                continue
            if chunks:
                self.frames_sent += 1