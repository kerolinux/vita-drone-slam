"""vitalink.py - UDP message format between the PS Vita controller app and the PC bridge (bridge.py).

Vita -> PC (control, 12 bytes, ~50 per second, to UDP port 9000):
    0-1  'V' 'D'          magic
    2    version (1)
    3    sequence number (wraps at 256)
    4-7  roll, pitch, throttle, yaw as signed bytes (-127..127; 0 = centre)
    8-9  button bits, little endian (see BUTTONS)
    10   speed setting 1..3
    11   checksum = XOR of bytes 2..10

PC -> Vita (telemetry, 12 bytes, ~5 per second, to UDP port 9001):
    0-1  'D' 'V'          magic
    2    version (1)
    3    mode (0 idle, 1 control)
    4    failsafe stage (0 none, 1 neutral, 2 landing, 3 motor cut)
    5    emergency stop active (0/1)
    6    video frames per second (0..255)
    7-9  trim roll, pitch, yaw (signed bytes)
    10   seconds since the last command, in 0.1 s units (capped at 255)
    11   checksum = XOR of bytes 2..10

The C code in vita/src/main.c must stay in sync with this file.
"""
import struct

CONTROL_MAGIC = b"VD"
TELEMETRY_MAGIC = b"DV"
VERSION = 1
MESSAGE_LEN = 12
DEFAULT_PORT = 9000        # the PC bridge listens here
VITA_PORT = 9001           # the Vita listens here for telemetry

BUTTONS = {
    "TAKEOFF": 1 << 0,
    "LAND": 1 << 1,
    "EMERGENCY": 1 << 2,
    "CALIBRATE": 1 << 3,
    "TRIM_PITCH_UP": 1 << 4,
    "TRIM_PITCH_DOWN": 1 << 5,
    "TRIM_ROLL_LEFT": 1 << 6,
    "TRIM_ROLL_RIGHT": 1 << 7,
    "TRIM_YAW_LEFT": 1 << 8,
    "TRIM_YAW_RIGHT": 1 << 9,
    "TRIM_RESET": 1 << 10,
}


def xor(data) -> int:
    c = 0
    for b in data:
        c ^= b
    return c


def _clamp(v, lo, hi):
    return max(lo, min(hi, int(v)))


def pack_control(seq, roll=0, pitch=0, throttle=0, yaw=0, buttons=0, speed=1) -> bytes:
    body = struct.pack("<BBbbbbHB", VERSION, seq & 0xFF,
                       _clamp(roll, -127, 127), _clamp(pitch, -127, 127),
                       _clamp(throttle, -127, 127), _clamp(yaw, -127, 127),
                       buttons & 0xFFFF, _clamp(speed, 0, 255))
    return CONTROL_MAGIC + body + bytes([xor(body)])


def parse_control(data):
    """Returns a dict, or None if the message is invalid (wrong size, magic, version or checksum)."""
    if len(data) != MESSAGE_LEN or data[:2] != CONTROL_MAGIC or data[2] != VERSION:
        return None
    if xor(data[2:11]) != data[11]:
        return None
    _, seq, roll, pitch, throttle, yaw, buttons, speed = struct.unpack("<BBbbbbHB", data[2:11])
    return {"seq": seq, "roll": roll, "pitch": pitch, "throttle": throttle, "yaw": yaw,
            "buttons": buttons, "speed": speed}


def pack_telemetry(mode, stage, emergency, fps, trim, age_s) -> bytes:
    body = struct.pack("<BBBBBbbbB", VERSION, _clamp(mode, 0, 255), _clamp(stage, 0, 255),
                       1 if emergency else 0, _clamp(fps, 0, 255),
                       _clamp(trim.get("roll", 0), -127, 127), _clamp(trim.get("pitch", 0), -127, 127),
                       _clamp(trim.get("yaw", 0), -127, 127), _clamp(age_s * 10, 0, 255))
    return TELEMETRY_MAGIC + body + bytes([xor(body)])


def parse_telemetry(data):
    if len(data) != MESSAGE_LEN or data[:2] != TELEMETRY_MAGIC or data[2] != VERSION:
        return None
    if xor(data[2:11]) != data[11]:
        return None
    _, mode, stage, emergency, fps, tr, tp, ty, age = struct.unpack("<BBBBBbbbB", data[2:11])
    return {"mode": mode, "stage": stage, "emergency": bool(emergency), "fps": fps,
            "trim": {"roll": tr, "pitch": tp, "yaw": ty}, "age_s": age / 10.0}