"""fake_vita.py - pretends to be the Vita app, to test bridge.py without the console.

Examples:
  python fake_vita.py --seconds 5                       # neutral sticks for 5 s, prints telemetry
  python fake_vita.py --roll 0.5 --speed 1              # half roll for the whole run
  python fake_vita.py --press TAKEOFF                   # presses TAKEOFF for 0.3 s, one second in
"""
import argparse
import socket
import time

import vitalink as vl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1", help="PC running bridge.py")
    ap.add_argument("--port", type=int, default=vl.DEFAULT_PORT)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--speed", type=int, default=1)
    for axis in ("roll", "pitch", "throttle", "yaw"):
        ap.add_argument(f"--{axis}", type=float, default=0.0, help="-1..1")
    ap.add_argument("--press", default="", help="button name from vitalink.BUTTONS, pressed 1 s in, for 0.3 s")
    args = ap.parse_args()

    press = vl.BUTTONS[args.press.upper()] if args.press else 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", vl.VITA_PORT))
    sock.setblocking(False)
    start = time.time()
    seq = 0
    last_print = 0.0
    while time.time() - start < args.seconds:
        t = time.time() - start
        buttons = press if press and 1.0 <= t < 1.3 else 0
        sock.sendto(vl.pack_control(seq, round(args.roll * 127), round(args.pitch * 127),
                                    round(args.throttle * 127), round(args.yaw * 127), buttons, args.speed),
                    (args.host, args.port))
        seq += 1
        try:
            tel = vl.parse_telemetry(sock.recvfrom(256)[0])
            if tel and time.time() - last_print > 1.0:
                print(f"[{t:4.1f}s] telemetry: {tel}")
                last_print = time.time()
        except (BlockingIOError, ConnectionResetError):
            pass
        time.sleep(0.02)
    print("Done.")


if __name__ == "__main__":
    main()