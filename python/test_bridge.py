"""Hardware-free tests for vitalink.py and bridge.py (fake drone + fake Vita on localhost).

Run:  python test_bridge.py
"""
import socket
import threading
import time
import unittest

import vitalink as vl
from bridge import VitaBridge
from drone import DroneLink
from test_drone import FakeDrone, quiet

B = vl.BUTTONS


class MessageTests(unittest.TestCase):
    def test_control_round_trip(self):
        data = vl.pack_control(300, 127, -127, 5, -5, B["TAKEOFF"] | B["TRIM_RESET"], 3)
        self.assertEqual(len(data), 12)
        msg = vl.parse_control(data)
        self.assertEqual(msg, {"seq": 300 & 0xFF, "roll": 127, "pitch": -127, "throttle": 5, "yaw": -5,
                               "buttons": B["TAKEOFF"] | B["TRIM_RESET"], "speed": 3})

    def test_values_are_clamped(self):
        msg = vl.parse_control(vl.pack_control(0, 1000, -1000, 0, 0))
        self.assertEqual((msg["roll"], msg["pitch"]), (127, -127))

    def test_invalid_messages_rejected(self):
        good = vl.pack_control(1, 1, 2, 3, 4)
        self.assertIsNone(vl.parse_control(good[:-1]))                       # short
        self.assertIsNone(vl.parse_control(b"XX" + good[2:]))                # bad magic
        self.assertIsNone(vl.parse_control(good[:2] + b"\x02" + good[3:]))   # bad version
        self.assertIsNone(vl.parse_control(good[:-1] + bytes([good[-1] ^ 1])))  # bad checksum
        self.assertIsNone(vl.parse_control(vl.pack_telemetry(1, 0, False, 8, {}, 0)))   # wrong direction

    def test_telemetry_round_trip(self):
        data = vl.pack_telemetry(1, 2, True, 9, {"roll": -3, "pitch": 2, "yaw": 0}, 0.34)
        self.assertEqual(len(data), 12)
        self.assertEqual(vl.parse_telemetry(data), {"mode": 1, "stage": 2, "emergency": True, "fps": 9,
                                                    "trim": {"roll": -3, "pitch": 2, "yaw": 0}, "age_s": 0.3})
        self.assertIsNone(vl.parse_telemetry(data[:-1] + bytes([data[-1] ^ 1])))

    def test_known_bytes(self):
        """Fixed example, also useful for checking the C implementation on the Vita."""
        data = vl.pack_control(7, 10, -10, 20, -20, B["TAKEOFF"] | B["CALIBRATE"], 2)
        self.assertEqual(data.hex(), "564401" + "07" + "0af614ec" + "0900" + "02" + "%02x" % vl.xor(data[2:11]))


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeDrone()
        self.link = DroneLink(ip="127.0.0.1", control_port=self.fake.control_port, video_port=self.fake.video_port,
                              local_port=0, log=quiet, command_timeout=0.3, land_after=None, cut_after=None)
        self.link.start(idle_seconds=0.1)
        self.tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tel.bind(("127.0.0.1", 0))
        self.tel.settimeout(1.0)
        self.trim_saved = []
        self.bridge = VitaBridge(self.link, port=0, bind="127.0.0.1", telemetry_port=self.tel.getsockname()[1],
                                 telemetry_hz=20, on_trim_change=self.trim_saved.append, log=quiet)
        self.thread = threading.Thread(target=self.bridge.run, daemon=True)
        self.thread.start()
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def tearDown(self):
        self.bridge.close()
        self.thread.join(timeout=1.0)
        self.link.close(stop_motors=False)
        self.fake.close()
        self.sender.close()
        self.tel.close()

    def stream(self, seconds, buttons=0, speed=2, **sticks):
        end = time.time() + seconds
        while time.time() < end:
            self.sender.sendto(vl.pack_control(self.seq, buttons=buttons, speed=speed, **sticks),
                               ("127.0.0.1", self.bridge.port))
            self.seq += 1
            time.sleep(0.02)

    def pkts(self, length=20):
        return [p for p in list(self.fake.control_packets) if len(p) == length and p[0] == 0x66]

    def test_sticks_and_speed_reach_the_drone(self):
        for speed, (rp, ty) in ((1, (20, 40)), (2, (40, 60)), (3, (70, 90))):
            self.stream(0.4, speed=speed, roll=127, pitch=-127, throttle=127, yaw=-127)
            want = (128 + rp, 128 - rp, 128 + ty, 128 - ty)
            self.assertTrue(any(tuple(p[2:6]) == want for p in self.pkts()), f"speed {speed}")

    def test_takeoff_acts_once_per_press(self):
        self.stream(0.3)                                  # session starts with nothing pressed
        self.stream(0.5, buttons=B["TAKEOFF"])            # held for a long time
        self.stream(1.5)                                  # released, wait for the pulse to finish
        self.assertEqual(sum(1 for p in self.pkts() if p[6] == 1), 51)

    def test_button_held_at_connect_is_ignored(self):
        self.stream(0.6, buttons=B["TAKEOFF"])
        self.stream(1.2)
        self.assertEqual(sum(1 for p in self.pkts() if p[6] == 1), 0)

    def test_emergency_always_acts(self):
        self.stream(0.4, buttons=B["EMERGENCY"])          # even when held from the first packet
        self.stream(0.2)
        self.assertTrue(any(p[6] == 2 for p in self.pkts()))
        self.assertTrue(any(p[5] == 4 for p in self.pkts(8)))

    def test_trim_buttons(self):
        self.stream(0.2)
        for _ in range(3):
            self.stream(0.1, buttons=B["TRIM_ROLL_RIGHT"])
            self.stream(0.1)
        self.stream(0.1, buttons=B["TRIM_PITCH_DOWN"])
        self.stream(0.1)
        self.assertEqual(self.link.get_trim(), {"roll": 3, "pitch": -1, "yaw": 0})
        self.stream(0.1, buttons=B["TRIM_RESET"])
        self.assertEqual(self.link.get_trim(), {"roll": 0, "pitch": 0, "yaw": 0})
        self.assertGreaterEqual(len(self.trim_saved), 4)

    def test_calibrate_button(self):
        self.stream(0.2)
        self.stream(0.2, buttons=B["CALIBRATE"])
        self.stream(1.5)
        self.assertEqual(sum(1 for p in self.pkts() if p[6] == 4), 51)

    def test_link_loss_returns_to_neutral(self):
        self.stream(0.4, roll=127)
        time.sleep(0.8)                                   # Vita stops sending
        self.assertEqual(self.pkts()[-1][2], 0x80)

    def test_stale_and_bad_packets_are_dropped(self):
        b = VitaBridge(self.link, port=0, bind="127.0.0.1", log=quiet)
        addr = ("127.0.0.1", 1234)
        self.assertTrue(b.handle_packet(vl.pack_control(10), addr, now=100.0))
        self.assertFalse(b.handle_packet(vl.pack_control(10), addr, now=100.02))     # duplicate
        self.assertFalse(b.handle_packet(vl.pack_control(9), addr, now=100.04))      # older
        self.assertTrue(b.handle_packet(vl.pack_control(11), addr, now=100.06))
        self.assertFalse(b.handle_packet(b"garbage", addr, now=100.08))
        self.assertTrue(b.handle_packet(vl.pack_control(3), addr, now=105.0))        # new session after a gap
        self.assertEqual((b.packets_ok, b.packets_stale, b.packets_bad), (3, 2, 1))
        b.close()

    def test_wraparound_sequence_is_accepted(self):
        b = VitaBridge(self.link, port=0, bind="127.0.0.1", log=quiet)
        addr = ("127.0.0.1", 1234)
        self.assertTrue(b.handle_packet(vl.pack_control(254), addr, now=1.0))
        self.assertTrue(b.handle_packet(vl.pack_control(255), addr, now=1.02))
        self.assertTrue(b.handle_packet(vl.pack_control(0), addr, now=1.04))
        b.close()

    def test_telemetry_reaches_the_vita(self):
        self.stream(0.6)
        tel = None
        end = time.time() + 1.0
        while time.time() < end and tel is None:
            try:
                tel = vl.parse_telemetry(self.tel.recvfrom(256)[0])
            except socket.timeout:
                break
        self.assertIsNotNone(tel)
        self.assertEqual(tel["mode"], 1)
        self.assertEqual(tel["stage"], 0)
        self.assertFalse(tel["emergency"])


if __name__ == "__main__":
    unittest.main(verbosity=2)