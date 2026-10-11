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

PC -> Vita (video, one packet per group of image rows, also to UDP port 9001):
    0-1   'D' 'F'         magic
    2     version (1)
    3     frame id (wraps at 256)
    4-5   image width in pixels, little endian
    6-7   image height in pixels, little endian
    8-9   index of the first image row in this packet, little endian
    10    number of rows in this packet
    11    flags (bit 0 = last packet of the frame)
    12..  rows * width * 2 bytes of RGB565 pixels, little endian, row after row
The PC decodes the drone's JPEG, shrinks it (default 320x240), converts it to RGB565 and sends it in rows, so
the Vita only has to copy pixels. A lost packet leaves a few old rows on screen instead of losing the frame.

The C code in vita/src/protocol.h must stay in sync with this file.
"""
import struct

CONTROL_MAGIC = b"VD"
TELEMETRY_MAGIC = b"DV"
VERSION = 1
MESSAGE_LEN = 12
VIDEO_MAGIC = b"DF"
VIDEO_HEADER_LEN = 12
VIDEO_MAX_PAYLOAD = 1400   # keeps every UDP packet under the usual 1500-byte Ethernet / Wi-Fi limit
VIDEO_FLAG_LAST = 0x01
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


def pack_video_chunk(frame_id, width, height, row_start, rows, last, pixels) -> bytes:
    """pixels must be rows * width * 2 bytes (RGB565, little endian)."""
    if len(pixels) != rows * width * 2:
        raise ValueError("pixel data does not match rows * width * 2")
    header = struct.pack("<2sBBHHHBB", VIDEO_MAGIC, VERSION, frame_id & 0xFF, width, height, row_start, rows,
                         VIDEO_FLAG_LAST if last else 0)
    return header + bytes(pixels)


def parse_video_chunk(data):
    """Returns a dict (with the pixel bytes under "pixels"), or None if the packet is invalid."""
    if len(data) < VIDEO_HEADER_LEN or data[:2] != VIDEO_MAGIC or data[2] != VERSION:
        return None
    _, _, frame_id, width, height, row_start, rows, flags = struct.unpack("<2sBBHHHBB", data[:VIDEO_HEADER_LEN])
    pixels = data[VIDEO_HEADER_LEN:]
    if rows < 1 or width < 1 or height < 1 or row_start + rows > height or len(pixels) != rows * width * 2:
        return None
    return {"frame_id": frame_id, "width": width, "height": height, "row_start": row_start, "rows": rows,
            "last": bool(flags & VIDEO_FLAG_LAST), "pixels": pixels}