"""Control test for the WIFI_CAM / E4 drone, based on a capture of the official app.

What the capture showed (app running in BlueStacks, PC on the drone's Wi-Fi):
  - video start/stop commands ("Bv" / "Bw") go to UDP port 8080
  - idle and control packets go to UDP port 8090   <-- our first tests used 8080 by mistake
  - idle packet before the lock button is tapped: aa 80 80 00 80 00 80 55, every ~0.78 s
  - control, 20-byte format, ~50 packets/s:  66 14 roll pitch throttle yaw flags flags2 0*10 checksum 99
  - control, 8-byte format, ~25 packets/s:   66 roll pitch throttle yaw flags checksum 99
  - the app's neutral throttle is 0x80 (not 0); flags bit0 = fly, bit1 = land, bit2 = emergency (8-byte format)
  - a one-key flag is held for ~1 s (51 packets at 20 ms; ~21 packets at 40 ms)

Usage (PC on the drone's Wi-Fi, PROPS REMOVED, no other drone script running):
  python control_test.py --start-video             # idle + neutral packets, 20-byte format
  python control_test.py --start-video --takeoff   # also pulses the fly flag, then the land flag
  add --type 0 to use the 8-byte format instead
"""
import argparse
import socket
import time

DRONE_IP = "192.168.4.153"
VIDEO_PORT = 8080
CONTROL_PORT = 8090
LOCAL_PORT = 40238
CENTRE = 0x80
START_VIDEO = bytes([0x42, 0x76])
STOP_VIDEO = bytes([0x42, 0x77])
RESETS = [0]


def fix(b: int) -> int:
    """The app never lets a checksum or stick byte equal the framing bytes 0x66 / 0x99."""
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
    """Compare our packet builders with packets seen in the real app's capture."""
    cases = {
        "idle":           (idle_packet(),                 "aa80800080008055"),
        "20B neutral":    (packet_20(),                   "6614808080800000000000000000000000000099"),
        "20B fly flag":   (packet_20(flags=1),            "6614808080800100000000000000000000000199"),
        "20B throttle":   (packet_20(throttle=0x25),      "661480802580000000000000000000000000a599"),
        "20B roll/pitch": (packet_20(roll=0x92, pitch=0x8c), "6614928c80800000000000000000000000001e99"),
        "8B neutral":     (packet_8(),                    "6680808080000099"),
        "8B fly flag":    (packet_8(flags=1),             "6680808080010199"),
        "8B land flag":   (packet_8(flags=2),             "6680808080020299"),
        "8B emergency":   (packet_8(flags=4),             "6680808080040499"),
        "8B yaw 0x83":    (packet_8(yaw=0x83),            "6680808083000399"),
    }
    ok = True
    for name, (got, want) in cases.items():
        match = got.hex() == want
        ok &= match
        print(f"  {name:15s} {'OK' if match else 'MISMATCH ' + got.hex()}")
    assert ok, "Packet builders do not match the capture"


def emergency_stop(sock, seconds=1.5):
    """Hard motor cut.
    8-byte format: flags bit 2 (0x04) - this exact packet was seen in the app capture.
    20-byte format: flags bit 1 (0x02) - taken from the app's code, NOT seen in the capture.
    Both are sent, alternating, because we don't know for sure which format the drone is in."""
    print("Sending emergency stop...")
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        sock.sendto(packet_20(flags=0x02), (DRONE_IP, CONTROL_PORT))
        sock.sendto(packet_8(flags=0x04), (DRONE_IP, CONTROL_PORT))
        time.sleep(0.02)


def drain(sock, seen, t0):
    while True:
        try:
            data, addr = sock.recvfrom(4096)
        except BlockingIOError:
            return
        except ConnectionResetError:
            RESETS[0] += 1
            return
        except OSError:
            return
        if len(data) >= 8 and data[4:7] == b"TZH":
            continue                                  # video data
        key = data.hex()
        if key not in seen:
            seen[key] = 1
            print(f"  [{time.time() - t0:5.1f}s] reply from {addr[0]}:{addr[1]} ({len(data)} B): {data.hex(' ')}")
        else:
            seen[key] += 1


def run_phase(sock, name, seconds, period, make_packet, seen, t0):
    print(f"{name} ({seconds:.1f}s, one packet every {period * 1000:.0f} ms)")
    before = sum(seen.values())
    end = time.perf_counter() + seconds
    nxt = time.perf_counter()
    while time.perf_counter() < end:
        sock.sendto(make_packet(), (DRONE_IP, CONTROL_PORT))
        drain(sock, seen, t0)
        nxt += period
        while True:                                   # accurate timing (Windows sleep is coarse)
            left = nxt - time.perf_counter()
            if left <= 0:
                break
            if left > 0.003:
                time.sleep(left - 0.002)
    print(f"  replies during this phase: {sum(seen.values()) - before}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--type", type=int, choices=[0, 2], default=2, help="0 = 8-byte format, 2 = 20-byte format")
    ap.add_argument("--takeoff", action="store_true", help="also pulse the fly flag, then the land flag")
    ap.add_argument("--start-video", action="store_true", help="send the video start command to port 8080 first")
    ap.add_argument("--stop", action="store_true", help="only send the emergency stop, then exit")
    args = ap.parse_args()

    if args.stop:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("", LOCAL_PORT))
        emergency_stop(s)
        s.close()
        print("Done.")
        return

    print("Self-check against the captured packets:")
    self_check()

    if args.takeoff:
        ans = input("Motors may spin. Are the PROPS REMOVED and the drone held down? Type YES: ")
        if ans.strip() != "YES":
            print("Aborted.")
            return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", LOCAL_PORT))
    sock.setblocking(False)

    if args.type == 2:
        make = lambda flags=0: packet_20(flags=flags)
        period, pulse_packets = 0.020, 51
        fly_flag, land_flag = 0x01, 0x01          # the app uses bit 0 for both in this format
    else:
        make = lambda flags=0: packet_8(flags=flags)
        period, pulse_packets = 0.040, 21
        fly_flag, land_flag = 0x01, 0x02

    seen = {}
    t0 = time.time()
    try:
        if args.start_video:
            print("Sending video start command to port", VIDEO_PORT)
            for _ in range(3):
                sock.sendto(START_VIDEO, (DRONE_IP, VIDEO_PORT))
                time.sleep(0.1)
            drain(sock, seen, t0)
        run_phase(sock, "Phase A: idle packets", 3, 0.78, idle_packet, seen, t0)
        run_phase(sock, "Phase B: neutral control packets", 5, period, make, seen, t0)
        if args.takeoff:
            pulse = pulse_packets * period
            run_phase(sock, "Phase C: fly flag", pulse, period, lambda: make(fly_flag), seen, t0)
            run_phase(sock, "Phase D: neutral", 4, period, make, seen, t0)
            run_phase(sock, "Phase E: land flag", pulse, period, lambda: make(land_flag), seen, t0)
            run_phase(sock, "Phase F: neutral", 2, period, make, seen, t0)
    except KeyboardInterrupt:
        pass
    finally:
        if args.takeoff:
            emergency_stop(sock)          # never leave the motors running after a flag test
        for _ in range(10):
            sock.sendto(make(), (DRONE_IP, CONTROL_PORT))
            time.sleep(period)
        if args.start_video:
            for _ in range(3):
                sock.sendto(STOP_VIDEO, (DRONE_IP, VIDEO_PORT))
                time.sleep(0.05)
        sock.close()
        print(f"Connection-reset errors (ICMP port unreachable): {RESETS[0]}")
        print("Reply counts:")
        for k, v in seen.items():
            print(f"  {v:4d} x {bytes.fromhex(k).hex(' ')}")
        print("Done.")


if __name__ == "__main__":
    main()