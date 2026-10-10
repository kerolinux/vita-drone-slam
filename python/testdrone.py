"""Hardware-free tests for drone.py and teleop.KeyHold, using a fake drone on localhost.

Run:  python test_drone.py
"""
import os
import socket
import tempfile
import threading
import time
import unittest

import drone
from drone import DroneLink, checksum
from teleop import KeyHold, load_trim, save_trim


class FakeDrone:
    """Listens on two UDP ports like the real drone (control + video) and records what it receives."""

    def __init__(self):
        self.control_sock = self._bind()
        self.video_sock = self._bind()
        self.control_port = self.control_sock.getsockname()[1]
        self.video_port = self.video_sock.getsockname()[1]
        self.control_packets, self.video_packets = [], []
        self.client = None
        self._run = True
        for sock, store in ((self.control_sock, self.control_packets), (self.video_sock, self.video_packets)):
            threading.Thread(target=self._loop, args=(sock, store), daemon=True).start()

    @staticmethod
    def _bind():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        s.settimeout(0.1)
        return s

    def _loop(self, sock, store):
        while self._run:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            self.client = addr
            store.append(data)

    def send_frame(self, jpeg, frame_id, drop_index=None, chunk=1100):
        chunks = [jpeg[i:i + chunk] for i in range(0, len(jpeg), chunk)]
        for i, c in enumerate(chunks):
            if i == drop_index:
                continue
            header = bytes([frame_id, 1 if i == len(chunks) - 1 else 0, len(chunks), 0]) + b"TZH" + b"\x00"
            self.video_sock.sendto(header + c, self.client)

    def close(self):
        self._run = False
        self.control_sock.close()
        self.video_sock.close()


def quiet(*_a, **_k):
    pass


class DroneTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeDrone()

    def tearDown(self):
        self.fake.close()

    def link(self, **kw):
        kw.setdefault("log", quiet)
        return DroneLink(ip="127.0.0.1", control_port=self.fake.control_port, video_port=self.fake.video_port,
                         local_port=0, **kw)

    def pkts(self, length):
        return [p for p in list(self.fake.control_packets) if len(p) == length and p[0] == 0x66]

    def hold_heartbeat(self, link, seconds, **sticks):
        end = time.time() + seconds
        while time.time() < end:
            link.set_sticks(**sticks)
            time.sleep(0.03)

    def test_packets_match_capture(self):
        drone.self_check()

    def test_checksums_are_consistent(self):
        link = self.link()
        link.start(idle_seconds=0.1)
        self.hold_heartbeat(link, 0.3, roll=0.7, pitch=-0.4, throttle=0.2, yaw=0.9)
        link.close(stop_motors=False)
        for p in self.pkts(20):
            self.assertEqual(p[18], checksum(p[2:18]))
            self.assertEqual(p[19], 0x99)

    def test_idle_then_control(self):
        link = self.link()
        link.start(idle_seconds=0.5)
        self.hold_heartbeat(link, 0.3)
        link.close(stop_motors=False)
        packets = list(self.fake.control_packets)
        self.assertEqual(packets[0].hex(), "aa80800080008055")
        self.assertTrue(any(len(p) == 20 for p in packets))
        first_ctrl = next(i for i, p in enumerate(packets) if len(p) == 20)
        self.assertTrue(all(p[0] == 0xAA for p in packets[:first_ctrl]))

    def test_stick_mapping(self):
        link = self.link(max_offset=40)
        link.start(idle_seconds=0.1)
        self.hold_heartbeat(link, 0.3, roll=1.0, pitch=-1.0, throttle=0.5, yaw=-0.5)
        link.close(stop_motors=False)
        want = (128 + 40, 128 - 40, 128 + 20, 128 - 20)
        self.assertTrue(any(tuple(p[2:6]) == want for p in self.pkts(20)), "no packet with the expected stick bytes")

    def test_stick_bytes_avoid_framing_values(self):
        link = self.link(max_offset=40)
        self.assertEqual(link._stick_byte("roll", -26 / 40), 0x67)    # would be 0x66, bumped by one
        self.assertEqual(link._stick_byte("roll", 25 / 40), 0x9A)     # would be 0x99, bumped by one

    def test_takeoff_pulse_length_20(self):
        link = self.link(fmt=20)
        link.start(idle_seconds=0.1)
        link.takeoff()
        self.hold_heartbeat(link, 1.5)
        link.close(stop_motors=False)
        self.assertEqual(sum(1 for p in self.pkts(20) if p[6] == 1), 51)

    def test_takeoff_and_land_8_byte(self):
        link = self.link(fmt=8)
        link.start(idle_seconds=0.1)
        link.takeoff()
        self.hold_heartbeat(link, 1.2)
        link.land()
        self.hold_heartbeat(link, 1.2)
        link.close(stop_motors=False)
        self.assertEqual(sum(1 for p in self.pkts(8) if p[5] == 1), 21)
        self.assertEqual(sum(1 for p in self.pkts(8) if p[5] == 2), 21)

    def test_deadman_returns_to_neutral(self):
        link = self.link(command_timeout=0.2, land_after=None, cut_after=None)
        link.start(idle_seconds=0.1)
        self.hold_heartbeat(link, 0.3, roll=1.0)
        time.sleep(0.7)
        link.close(stop_motors=False)
        self.assertEqual(self.pkts(20)[-1][2], 0x80)

    def test_failsafe_lands_then_cuts(self):
        link = self.link(command_timeout=0.1, land_after=0.3, cut_after=0.8)
        link.start(idle_seconds=0.1)
        link.set_sticks(roll=1.0)
        time.sleep(2.2)
        link.close(stop_motors=False)
        self.assertTrue(any(p[6] == 1 for p in self.pkts(20)), "no land pulse")
        self.assertTrue(any(p[6] == 2 for p in self.pkts(20)), "no 20-byte emergency packet")
        self.assertTrue(any(p[5] == 4 for p in self.pkts(8)), "no 8-byte emergency packet")

    def test_close_stops_motors_and_video(self):
        link = self.link()
        link.start(idle_seconds=0.1)
        link.close()
        self.assertTrue(any(p[6] == 2 for p in self.pkts(20)))
        self.assertTrue(any(p[5] == 4 for p in self.pkts(8)))
        self.assertIn(drone.START_VIDEO, self.fake.video_packets)
        self.assertIn(drone.STOP_VIDEO, self.fake.video_packets)

    def test_video_frames(self):
        link = self.link()
        link.start(idle_seconds=0.1)
        jpeg = b"\xff\xd8" + bytes(range(256)) * 18 + b"\xff\xd9"        # ~4.6 KB fake JPEG
        self.fake.send_frame(jpeg, frame_id=7)
        time.sleep(0.3)
        frame = link.get_frame()
        self.assertIsNotNone(frame)
        self.assertEqual(frame.jpeg, jpeg)
        seq = frame.seq
        self.fake.send_frame(jpeg, frame_id=8, drop_index=2)               # one packet lost: must be rejected
        time.sleep(0.3)
        self.assertEqual(link.get_frame().seq, seq)
        self.assertGreaterEqual(link.status()["frames_incomplete"], 1)
        link.close(stop_motors=False)

    def test_trim_shifts_neutral_bytes(self):
        link = self.link()
        link.set_trim(roll=3, pitch=-2, yaw=1)
        link.start(idle_seconds=0.1)
        self.hold_heartbeat(link, 0.3)
        link.close(stop_motors=False)
        self.assertTrue(any(tuple(p[2:6]) == (0x83, 0x7E, 0x80, 0x81) for p in self.pkts(20)))

    def test_trim_is_clamped_and_validated(self):
        link = self.link()
        link.adjust_trim("roll", 1000)
        link.adjust_trim("pitch", -1000)
        self.assertEqual(link.get_trim(), {"roll": drone.MAX_TRIM, "pitch": -drone.MAX_TRIM, "yaw": 0})
        with self.assertRaises(ValueError):
            link.adjust_trim("throttle", 1)

    def test_trim_follows_axis_sign(self):
        link = self.link(signs={"roll": -1})
        link.set_trim(roll=3)
        self.assertEqual(link._stick_byte("roll", 0.0), 128 - 3)

    def test_calibrate_pulse_lengths(self):
        link = self.link(fmt=20)
        link.start(idle_seconds=0.1)
        link.calibrate()
        self.hold_heartbeat(link, 1.5)
        link.close(stop_motors=False)
        self.assertEqual(sum(1 for p in self.pkts(20) if p[6] == 0x04), 51)

        link = self.link(fmt=8)
        link.start(idle_seconds=0.1)
        link.calibrate()
        self.hold_heartbeat(link, 1.2)
        link.close(stop_motors=False)
        self.assertEqual(sum(1 for p in self.pkts(8) if p[5] == 0x80), 21)

    def test_status_and_validation(self):
        with self.assertRaises(ValueError):
            DroneLink(fmt=7)
        with self.assertRaises(ValueError):
            DroneLink(max_offset=200)
        link = self.link()
        link.start(idle_seconds=0.1)
        self.assertEqual(link.status()["mode"], "control")
        link.close(stop_motors=False)


class TrimFileTests(unittest.TestCase):
    def test_round_trip_missing_and_corrupt(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "trim.json")
            self.assertEqual(load_trim(path), {})
            save_trim({"roll": 2, "pitch": -1, "yaw": 0}, path)
            self.assertEqual(load_trim(path), {"roll": 2, "pitch": -1, "yaw": 0})
            with open(path, "w") as f:
                f.write("not json")
            self.assertEqual(load_trim(path), {})


class KeyHoldTests(unittest.TestCase):
    def test_press_expires(self):
        k = KeyHold(hold=0.35)
        k.press("w", 10.0)
        self.assertEqual(k.sticks(10.1)["pitch"], 1.0)
        self.assertEqual(k.sticks(10.4)["pitch"], 0.0)

    def test_opposite_keys_cancel_and_axes_independent(self):
        k = KeyHold(hold=1.0)
        k.press("a", 0.0)
        k.press("d", 0.0)
        k.press("r", 0.0)
        s = k.sticks(0.1)
        self.assertEqual(s["roll"], 0.0)
        self.assertEqual(s["throttle"], 1.0)

    def test_unknown_key_ignored_and_clear(self):
        k = KeyHold()
        k.press("z", 0.0)
        k.press("q", 0.0)
        k.clear()
        self.assertEqual(sum(abs(v) for v in k.sticks(0.0).values()), 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)