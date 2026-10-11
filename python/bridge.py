"""bridge.py - receives Vita controller packets over UDP and drives the drone through DroneLink.

Network: the Vita joins your home Wi-Fi router; the PC is on the router by Ethernet; the PC's Wi-Fi dongle is
joined to the drone. The bridge listens on 0.0.0.0:9000 (allow it through Windows Firewall, private network).

Safety: every valid packet is a heartbeat for DroneLink's failsafe. If the Vita stops sending (battery,
Wi-Fi, crash) the drone goes neutral after 0.3 s, lands after 1.5 s, and cuts the motors after 5 s.
Buttons act on their rising edge only, and buttons already held when a session starts are ignored,
except EMERGENCY, which always acts (and repeats once per second while held).
"""
import argparse
import socket
import time

import vitalink as vl

SPEED_OFFSETS = {1: 20, 2: 40, 3: 70}      # same stick ranges as the keyboard teleop (1 / 2 / 3)
B = vl.BUTTONS
TRIM_BUTTONS = {B["TRIM_PITCH_UP"]: ("pitch", +1), B["TRIM_PITCH_DOWN"]: ("pitch", -1),
                B["TRIM_ROLL_LEFT"]: ("roll", -1), B["TRIM_ROLL_RIGHT"]: ("roll", +1),
                B["TRIM_YAW_LEFT"]: ("yaw", -1), B["TRIM_YAW_RIGHT"]: ("yaw", +1)}


class VitaBridge:
    def __init__(self, link, port=vl.DEFAULT_PORT, bind="0.0.0.0", telemetry_port=vl.VITA_PORT,
                 telemetry_hz=5.0, trim_step=1, on_trim_change=None, log=print):
        self.link = link
        self.telemetry_port = telemetry_port
        self.telemetry_period = 1.0 / telemetry_hz
        self.trim_step = trim_step
        self.on_trim_change = on_trim_change
        self._log = log
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self._running = False
        self._vita_ip = None
        self._last_rx = None
        self._last_seq = None
        self._prev_buttons = None
        self._last_emergency = 0.0
        self._next_telemetry = 0.0
        self._fps_time = time.monotonic()
        self._fps_frames = 0
        self._fps = 0
        self.packets_ok = self.packets_bad = self.packets_stale = 0

    # ------------------------------------------------------------ incoming
    def handle_packet(self, data, addr, now=None):
        """Process one control packet. Returns True if it was applied."""
        now = time.monotonic() if now is None else now
        msg = vl.parse_control(data)
        if msg is None:
            self.packets_bad += 1
            return False

        if self._last_rx is None or now - self._last_rx > 1.0:      # new session
            self._last_seq = None
            self._prev_buttons = None
            self._log(f"[bridge] Vita connected from {addr[0]}")
        if self._last_seq is not None:
            diff = (msg["seq"] - self._last_seq) & 0xFF
            if diff == 0 or diff > 128:                              # duplicate or out of order
                self.packets_stale += 1
                return False
        self._last_seq, self._last_rx, self._vita_ip = msg["seq"], now, addr[0]
        self.packets_ok += 1

        self.link.set_sticks(roll=msg["roll"] / 127.0, pitch=msg["pitch"] / 127.0,
                             throttle=msg["throttle"] / 127.0, yaw=msg["yaw"] / 127.0)
        if msg["speed"] in SPEED_OFFSETS:
            self.link.max_offset = SPEED_OFFSETS[msg["speed"]]

        buttons = msg["buttons"]
        prev = self._prev_buttons
        if prev is None:
            prev = buttons & ~B["EMERGENCY"]                         # ignore buttons already held at connect
        edges = buttons & ~prev
        self._prev_buttons = buttons

        if buttons & B["EMERGENCY"] and (edges & B["EMERGENCY"] or now - self._last_emergency > 1.0):
            self._last_emergency = now
            self.link.emergency_stop()
            self._log("[bridge] EMERGENCY STOP")
        if edges & B["TAKEOFF"]:
            self.link.takeoff()
            self._log("[bridge] takeoff")
        if edges & B["LAND"]:
            self.link.land()
            self._log("[bridge] land")
        if edges & B["CALIBRATE"]:
            self.link.calibrate()
            self._log("[bridge] calibrate")
        changed = False
        for bit, (axis, direction) in TRIM_BUTTONS.items():
            if edges & bit:
                self.link.adjust_trim(axis, direction * self.trim_step)
                changed = True
        if edges & B["TRIM_RESET"]:
            self.link.set_trim(roll=0, pitch=0, yaw=0)
            changed = True
        if changed and self.on_trim_change:
            self.on_trim_change(self.link.get_trim())
        return True

    # ------------------------------------------------------------ outgoing
    def send_telemetry(self, now=None):
        now = time.monotonic() if now is None else now
        if self._vita_ip is None or now < self._next_telemetry:
            return
        self._next_telemetry = now + self.telemetry_period
        st = self.link.status()
        if now - self._fps_time >= 1.0:
            self._fps = int(round((st["frames_ok"] - self._fps_frames) / (now - self._fps_time)))
            self._fps_time, self._fps_frames = now, st["frames_ok"]
        msg = vl.pack_telemetry(1 if st["mode"] == "control" else 0, st["failsafe_stage"], st["emergency"],
                                self._fps, st["trim"], st["heartbeat_age"])
        try:
            self.sock.sendto(msg, (self._vita_ip, self.telemetry_port))
        except OSError:
            pass

    # ------------------------------------------------------------ loop
    def run(self):
        self._running = True
        while self._running:
            try:
                data, addr = self.sock.recvfrom(256)
                self.handle_packet(data, addr)
            except socket.timeout:
                pass
            except ConnectionResetError:
                pass                                                  # Windows: ICMP from a closed Vita port
            except OSError:
                break
            self.send_telemetry()

    def stop(self):
        self._running = False

    def close(self):
        self.stop()
        try:
            self.sock.close()
        except OSError:
            pass


def main():
    from drone import DroneLink
    from teleop import TRIM_FILE, load_trim, save_trim

    ap = argparse.ArgumentParser(description="Vita controller bridge for the WIFI_CAM drone")
    ap.add_argument("--ip", default="192.168.4.153", help="drone address")
    ap.add_argument("--port", type=int, default=vl.DEFAULT_PORT, help="UDP port the Vita sends to")
    ap.add_argument("--fmt", type=int, choices=[8, 20], default=20)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--trim-file", default=TRIM_FILE)
    ap.add_argument("--land-after", type=float, default=1.5)
    ap.add_argument("--cut-after", type=float, default=5.0)
    args = ap.parse_args()

    ans = input("Are the PROPS REMOVED (or the drone tethered / in a cage)? Type YES: ")
    if ans.strip() != "YES":
        print("Aborted.")
        return

    link = DroneLink(ip=args.ip, fmt=args.fmt, video=not args.no_video,
                     land_after=args.land_after, cut_after=args.cut_after)
    link.set_trim(**load_trim(args.trim_file))
    bridge = VitaBridge(link, port=args.port, on_trim_change=lambda t: save_trim(t, args.trim_file))
    print(f"Bridge listening on UDP {args.port}. Start the Vita app and point it at this PC's address.")
    link.start()
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping motors and closing...")
        save_trim(link.get_trim(), args.trim_file)
        bridge.close()
        link.close()
        print("Done.")


if __name__ == "__main__":
    main()