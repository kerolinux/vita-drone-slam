"""drone.py - control and video library for the WIFI_CAM / E4 Wi-Fi drone.

Protocol facts (see docs/protocol.md): video start/stop on UDP 8080, idle and control packets on UDP 8090.
One UDP socket (local port 40238) is used for everything, because the drone sends its video and replies
back to the port the commands came from.

Typical use:
    with DroneLink() as drone:           # starts video + idle packets, then control mode
        drone.set_sticks(pitch=0.5)      # call this ~10-30x per second (it is also the heartbeat)
        drone.takeoff()
        frame = drone.get_frame()        # newest complete JPEG frame, or None

SAFETY: the drone keeps its motors running when packets stop. This library therefore runs a staged failsafe:
no set_sticks()/heartbeat() call for command_timeout s -> sticks go neutral;
for land_after s -> a land pulse is sent; for cut_after s -> emergency motor stop.
The land flag and the 20-byte emergency flag are NOT yet verified on the real drone.
"""
import socket
import threading
import time
from collections import Counter, namedtuple

DEFAULT_IP = "192.168.4.153"
VIDEO_PORT = 8080
CONTROL_PORT = 8090
LOCAL_PORT = 40238
CENTRE = 0x80
START_VIDEO = bytes([0x42, 0x76])
STOP_VIDEO = bytes([0x42, 0x77])

IDLE_PERIOD = 0.78            # seconds between idle packets (seen in the capture)
EMERGENCY_SECONDS = 1.5
PULSE_PACKETS = {20: 51, 8: 21}                       # a one-key flag is held ~1 s (matches the app)
FLAG_BITS = {20: {"fly": 0x01, "land": 0x01, "calibrate": 0x04},   # 20-byte: bit 0 for fly AND land
             8: {"fly": 0x01, "land": 0x02, "calibrate": 0x80}}    # 8-byte: bit 0 fly, bit 1 land
# "calibrate" is the app's "checkout" flag: from the app's code only, never tested on the drone.
EMERGENCY_FLAG_20 = 0x02      # from the app's code, NOT seen in the capture
EMERGENCY_FLAG_8 = 0x04       # seen in the capture
MAX_TRIM = 40                 # largest trim offset in byte units

AXES = ("roll", "pitch", "throttle", "yaw")
# Stick range per speed setting (largest offset from the centre byte). The official app's speed button only
# limited the right stick (roll/pitch); its left stick (throttle/yaw) was not limited the same way, and a
# range of 20 on throttle/yaw did nothing on my drone, so those two get larger ranges. These numbers are
# my own conservative choices, not values from the protocol.
SPEED_PRESETS = {
    1: {"roll": 20, "pitch": 20, "throttle": 40, "yaw": 40},
    2: {"roll": 40, "pitch": 40, "throttle": 60, "yaw": 60},
    3: {"roll": 70, "pitch": 70, "throttle": 90, "yaw": 90},
}

Frame = namedtuple("Frame", "jpeg ts seq")            # ts = wall-clock arrival time (time.time())


# ---------------------------------------------------------------- packet building
def fix(b: int) -> int:
    """The app never lets a stick byte or checksum equal the framing bytes 0x66 / 0x99."""
    return b + 1 if b in (0x66, 0x99) else b


def checksum(data) -> int:
    c = 0
    for b in data:
        c ^= b
    return fix(c)


def packet_8(roll=CENTRE, pitch=CENTRE, throttle=CENTRE, yaw=CENTRE, flags=0) -> bytes:
    body = [fix(roll), fix(pitch), fix(throttle), fix(yaw), flags]
    return bytes([0x66] + body + [checksum(body), 0x99])


def packet_20(roll=CENTRE, pitch=CENTRE, throttle=CENTRE, yaw=CENTRE, flags=0, flags2=0) -> bytes:
    body = [fix(roll), fix(pitch), fix(throttle), fix(yaw), flags, flags2] + [0] * 10
    return bytes([0x66, 0x14] + body + [checksum(body), 0x99])


def idle_packet() -> bytes:
    body = [CENTRE, CENTRE, 0, CENTRE, 0]
    return bytes([0xAA] + body + [checksum(body), 0x55])


def self_check():
    """Compare the builders with packets captured from the official app. Raises AssertionError on mismatch."""
    cases = {
        "idle":           (idle_packet(),                      "aa80800080008055"),
        "20B neutral":    (packet_20(),                        "6614808080800000000000000000000000000099"),
        "20B fly flag":   (packet_20(flags=1),                 "6614808080800100000000000000000000000199"),
        "20B throttle":   (packet_20(throttle=0x25),           "661480802580000000000000000000000000a599"),
        "20B roll/pitch": (packet_20(roll=0x92, pitch=0x8c),   "6614928c80800000000000000000000000001e99"),
        "8B neutral":     (packet_8(),                         "6680808080000099"),
        "8B fly flag":    (packet_8(flags=1),                  "6680808080010199"),
        "8B land flag":   (packet_8(flags=2),                  "6680808080020299"),
        "8B emergency":   (packet_8(flags=4),                  "6680808080040499"),
        "8B yaw 0x83":    (packet_8(yaw=0x83),                 "6680808083000399"),
    }
    for name, (got, want) in cases.items():
        assert got.hex() == want, f"{name}: got {got.hex()} want {want}"


# ---------------------------------------------------------------- the link
class DroneLink:
    def __init__(self, ip=DEFAULT_IP, control_port=CONTROL_PORT, video_port=VIDEO_PORT, local_port=LOCAL_PORT,
                 fmt=20, max_offset=40, signs=None, video=True,
                 command_timeout=0.3, land_after=1.5, cut_after=5.0, log=print):
        """fmt: 20 or 8 (packet format). max_offset: largest stick deflection from centre (1..127), one number
        for all axes or a dict per axis, e.g. SPEED_PRESETS[1].
        signs: e.g. {"pitch": -1} to flip an axis (stick directions are not verified yet)."""
        if fmt not in (8, 20):
            raise ValueError("fmt must be 8 or 20")
        self.ip, self.control_port, self.video_port, self.local_port = ip, control_port, video_port, local_port
        self.fmt, self.video = fmt, video
        self._lock = threading.Lock()                      # must exist before max_offset is set
        self._offsets = {axis: 40 for axis in AXES}
        self.max_offset = max_offset
        self.command_timeout, self.land_after, self.cut_after = command_timeout, land_after, cut_after
        self.signs = {"roll": 1, "pitch": 1, "throttle": 1, "yaw": 1}
        self.signs.update(signs or {})
        self._log = log

        self._sock = None
        self._threads = []
        self._running = False
        self._closed = False
        self._mode = "idle"
        self._last_idle = 0.0
        self._sticks = {"roll": 0.0, "pitch": 0.0, "throttle": 0.0, "yaw": 0.0}
        self._trim = {"roll": 0, "pitch": 0, "yaw": 0}     # added to the centre value, like the app's trim sliders
        self._pulses = {}
        self._emergency_until = 0.0
        self._last_heartbeat = time.monotonic()
        self._stage = 0

        self._frame = None
        self._seq = 0
        self._counts = Counter()
        self._replies = Counter()

    @property
    def max_offset(self):
        """Stick range of the roll axis (the other axes may differ; see axis_offsets)."""
        return self._offsets["roll"]

    @max_offset.setter
    def max_offset(self, value):
        """An int sets every axis; a dict (axis -> int) sets those axes."""
        new = dict.fromkeys(AXES, value) if not isinstance(value, dict) else value
        for axis, v in new.items():
            if axis not in AXES:
                raise ValueError(f"unknown axis {axis!r}")
            if not isinstance(v, int) or not 1 <= v <= 127:
                raise ValueError("max_offset must be an integer 1..127")
        with self._lock:
            self._offsets.update(new)

    @property
    def axis_offsets(self):
        with self._lock:
            return dict(self._offsets)

    # ------------------------------------------------------------ lifecycle
    def start(self, idle_seconds=2.0):
        """Open the socket, start video, send idle packets for idle_seconds (like the app before its lock
        button), then switch to control packets."""
        if self._running:
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("", self.local_port))
        self._sock.settimeout(0.2)
        self._running = True
        self._mode = "idle"
        self._last_idle = 0.0
        for target in (self._send_loop, self._recv_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        if self.video:
            for _ in range(3):
                self._send(START_VIDEO, self.video_port)
                time.sleep(0.1)
        time.sleep(idle_seconds)
        with self._lock:
            self._mode = "control"
            self._last_heartbeat = time.monotonic()
            self._stage = 0

    def close(self, stop_motors=True):
        """Stop the motors (emergency stop), stop the video, and release the socket."""
        if self._closed:
            return
        self._closed = True
        if self._sock is None:
            return
        if stop_motors:
            self.emergency_stop()
            time.sleep(EMERGENCY_SECONDS + 0.2)
        if self.video:
            for _ in range(3):
                self._send(STOP_VIDEO, self.video_port)
                time.sleep(0.03)
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------ commands
    def set_sticks(self, roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0):
        """Each axis -1..1 (0 = centre). Also acts as the heartbeat for the failsafe."""
        with self._lock:
            self._sticks = {"roll": roll, "pitch": pitch, "throttle": throttle, "yaw": yaw}
            self._last_heartbeat = time.monotonic()
            self._stage = 0

    def heartbeat(self):
        """Tell the failsafe the controller is alive, without changing the sticks."""
        with self._lock:
            self._last_heartbeat = time.monotonic()
            self._stage = 0

    def takeoff(self):
        with self._lock:
            self._pulses["fly"] = PULSE_PACKETS[self.fmt]

    def land(self):
        with self._lock:
            self._pulses["land"] = PULSE_PACKETS[self.fmt]

    def calibrate(self):
        """Send the app's "checkout" flag (probably gyro calibration / trim reset). Drone on a level surface,
        NOT flying. Untested on the real drone."""
        with self._lock:
            self._pulses["calibrate"] = PULSE_PACKETS[self.fmt]

    def set_trim(self, roll=None, pitch=None, yaw=None):
        """Trim in byte units (same direction as the stick), clamped to +-MAX_TRIM. Applies to neutral too."""
        with self._lock:
            for axis, value in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
                if value is not None:
                    self._trim[axis] = max(-MAX_TRIM, min(MAX_TRIM, int(value)))

    def adjust_trim(self, axis, delta):
        if axis not in self._trim:
            raise ValueError("axis must be roll, pitch or yaw")
        with self._lock:
            self._trim[axis] = max(-MAX_TRIM, min(MAX_TRIM, self._trim[axis] + int(delta)))

    def get_trim(self):
        with self._lock:
            return dict(self._trim)

    def emergency_stop(self):
        with self._lock:
            self._pulses.clear()
            self._emergency_until = time.monotonic() + EMERGENCY_SECONDS

    # ------------------------------------------------------------ video / status
    def get_frame(self):
        """Newest complete frame as Frame(jpeg, ts, seq), or None if none has arrived."""
        with self._lock:
            return self._frame

    def status(self):
        with self._lock:
            return {"mode": self._mode, "failsafe_stage": self._stage,
                    "emergency": time.monotonic() < self._emergency_until,
                    "trim": dict(self._trim),
                    "heartbeat_age": time.monotonic() - self._last_heartbeat,
                    "frames_ok": self._counts["ok"], "frames_incomplete": self._counts["incomplete"],
                    "frames_abandoned": self._counts["abandoned"],
                    "replies": dict(self._replies)}

    # ------------------------------------------------------------ internals
    def _send(self, data, port):
        try:
            self._sock.sendto(data, (self.ip, port))
        except OSError:
            pass

    def _stick_byte(self, name, value):
        offset = int(round(max(-1.0, min(1.0, value)) * self._offsets[name])) + self._trim.get(name, 0)
        return fix(max(0, min(255, CENTRE + self.signs[name] * offset)))

    def _next_packets(self, now):
        with self._lock:
            if now < self._emergency_until:
                return [packet_20(flags=EMERGENCY_FLAG_20), packet_8(flags=EMERGENCY_FLAG_8)]
            if self._mode == "idle":
                if now - self._last_idle >= IDLE_PERIOD:
                    self._last_idle = now
                    return [idle_packet()]
                return []

            age = now - self._last_heartbeat
            stage = 0
            if age > self.command_timeout:
                stage = 1
            if self.land_after is not None and age > self.land_after:
                stage = 2
            if self.cut_after is not None and age > self.cut_after:
                stage = 3
            if stage > self._stage:
                self._stage = stage
                self._log(f"[drone] no commands for {age:.1f}s -> failsafe stage {stage} "
                          f"({['', 'neutral sticks', 'land', 'emergency stop'][stage]})")
                if stage == 2:
                    self._pulses["land"] = PULSE_PACKETS[self.fmt]
                elif stage == 3:
                    self._pulses.clear()
                    self._emergency_until = now + EMERGENCY_SECONDS
                    return [packet_20(flags=EMERGENCY_FLAG_20), packet_8(flags=EMERGENCY_FLAG_8)]

            sticks = {k: 0.0 for k in self._sticks} if self._stage >= 1 else self._sticks
            flags = 0
            for name in list(self._pulses):
                flags |= FLAG_BITS[self.fmt][name]
                self._pulses[name] -= 1
                if self._pulses[name] <= 0:
                    del self._pulses[name]
            b = {k: self._stick_byte(k, v) for k, v in sticks.items()}
            if self.fmt == 20:
                return [packet_20(b["roll"], b["pitch"], b["throttle"], b["yaw"], flags)]
            return [packet_8(b["roll"], b["pitch"], b["throttle"], b["yaw"], flags)]

    def _send_loop(self):
        period = 0.020 if self.fmt == 20 else 0.040        # ~50 Hz / ~25 Hz, as the app does
        next_t = time.perf_counter()
        while self._running:
            for pkt in self._next_packets(time.monotonic()):
                self._send(pkt, self.control_port)
            next_t += period
            while self._running:                           # accurate timing (Windows sleep is coarse)
                left = next_t - time.perf_counter()
                if left <= 0:
                    break
                if left > 0.003:
                    time.sleep(left - 0.002)
            if time.perf_counter() - next_t > 0.1:         # fell far behind: resync
                next_t = time.perf_counter()

    def _recv_loop(self):
        parts, current = [], None
        last_video = last_restart = time.monotonic()
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                data = None
            except ConnectionResetError:
                continue
            except OSError:
                break
            now = time.monotonic()
            if data is not None:
                if len(data) >= 8 and data[4:7] == b"TZH":
                    last_video = now
                    frame_id, last_flag, expected = data[0], data[1], data[2]
                    if frame_id != current:
                        if current is not None and parts:
                            self._counts["abandoned"] += 1
                        current, parts = frame_id, []
                    parts.append(data[8:])
                    if last_flag == 1:
                        self._finish_frame(parts, expected)
                        current, parts = None, []
                else:
                    with self._lock:
                        self._replies[data.hex()] += 1
            # the official app stops and restarts the video when frames stall; do the same
            if self.video and now - last_video > 5.0 and now - last_restart > 5.0:
                last_restart = now
                self._send(STOP_VIDEO, self.video_port)
                time.sleep(0.1)
                self._send(START_VIDEO, self.video_port)

    def _finish_frame(self, parts, expected):
        raw = b"".join(parts)
        soi = raw.find(b"\xff\xd8")
        eoi = raw.find(b"\xff\xd9", soi) if soi != -1 else -1
        if len(parts) == expected and soi != -1 and eoi != -1 and 3000 <= eoi + 2 - soi <= 500000:
            with self._lock:
                self._seq += 1
                self._frame = Frame(raw[soi:eoi + 2], time.time(), self._seq)
                self._counts["ok"] += 1
        else:
            with self._lock:
                self._counts["incomplete"] += 1


if __name__ == "__main__":
    self_check()
    print("drone.py: packet builders match the captured packets.")