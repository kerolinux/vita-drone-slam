"""recorder.py - saves every complete video frame of a DroneLink to disk, named by its arrival time.

The files are the original JPEGs from the drone (no re-compression), named <seconds since 1970>.jpg, so a
recording folder can be fed to the SLAM tools (see make_tum_sequence.py).
"""
import os
import threading
import time


class FrameRecorder:
    def __init__(self, link, directory, log=print):
        self.link, self.directory, self._log = link, directory, log
        self.count = 0
        self._running = False
        self._thread = None

    def start(self):
        os.makedirs(self.directory, exist_ok=True)
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._log(f"[recorder] saving frames to {self.directory}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self._log(f"[recorder] {self.count} frames saved in {self.directory}")

    def _run(self):
        last_seq = 0
        while self._running:
            frame = self.link.get_frame()
            if frame is None or frame.seq == last_seq:
                time.sleep(0.004)
                continue
            last_seq = frame.seq
            try:
                with open(os.path.join(self.directory, f"{frame.ts:.3f}.jpg"), "wb") as f:
                    f.write(frame.jpeg)
                self.count += 1
            except OSError as exc:
                self._log(f"[recorder] could not save a frame: {exc}")