"""Checks that the C message code for the Vita (vita/src/protocol.h) agrees with vitalink.py.

Needs a C compiler (gcc or cc); the test is skipped if there is none.
Run:  python test_vita_protocol.py
"""
import os
import shutil
import subprocess
import tempfile
import unittest

import vitalink as vl

HERE = os.path.dirname(os.path.abspath(__file__))
# The vita/ folder sits next to this file, or one level up when the Python files live in a python/ folder.
SOURCE = next((p for p in (os.path.join(HERE, "vita", "test", "protocol_test.c"),
                           os.path.join(HERE, "..", "vita", "test", "protocol_test.c")) if os.path.exists(p)),
              os.path.join(HERE, "vita", "test", "protocol_test.c"))


@unittest.skipUnless(shutil.which("gcc") or shutil.which("cc"), "no C compiler found")
class CProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.exe = os.path.join(cls.tmp, "protocol_test")
        compiler = shutil.which("gcc") or shutil.which("cc")
        subprocess.run([compiler, "-std=gnu11", "-Wall", "-Wextra", "-Werror", SOURCE, "-o", cls.exe], check=True)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def run_c(self, *args):
        return subprocess.run([self.exe, *args], capture_output=True, text=True, check=False)

    def test_control_message_matches_python(self):
        c_hex = self.run_c("pack").stdout.strip()
        py = vl.pack_control(7, 10, -10, 20, -20, vl.BUTTONS["TAKEOFF"] | vl.BUTTONS["CALIBRATE"], 2)
        self.assertEqual(c_hex, py.hex())

    def test_telemetry_parsing_matches_python(self):
        data = vl.pack_telemetry(1, 2, True, 9, {"roll": -3, "pitch": 2, "yaw": 1}, 0.7)
        self.assertEqual(self.run_c("tel", data.hex()).stdout.split(), ["1", "2", "1", "9", "-3", "2", "1", "7"])

    def test_corrupt_telemetry_is_rejected(self):
        data = bytearray(vl.pack_telemetry(1, 0, False, 8, {}, 0))
        data[5] ^= 1
        self.assertEqual(self.run_c("tel", bytes(data).hex()).stdout.strip(), "invalid")

    def test_video_header_parsing(self):
        pixels = bytes(range(64))                               # 2 rows of 16 pixels
        data = vl.pack_video_chunk(3, 16, 8, 4, 2, True, pixels)
        self.assertEqual(self.run_c("video", data.hex()).stdout.split(), ["16", "8", "4", "2", "1"])
        for bad in (data[:-1], data + b"x", b"XX" + data[2:], data[:2] + b"\x07" + data[3:]):
            self.assertEqual(self.run_c("video", bad.hex()).stdout.strip(), "invalid")
        past_end = bytearray(data)
        past_end[8] = 7                                           # row_start 7 + 2 rows > height 8
        self.assertEqual(self.run_c("video", bytes(past_end).hex()).stdout.strip(), "invalid")

    def test_video_scale(self):
        for args, want in (((320, 240), 2), ((640, 480), 1), ((240, 180), 2), ((100, 75), 6), ((700, 100), 0),
                           ((320, 241), 1)):
            self.assertEqual(self.run_c("scale", *map(str, args), "640", "480").stdout.strip(), str(want), args)

    def test_blit_matches_python(self):
        width, height, scale, row0, rows = 5, 6, 3, 2, 3
        values = [(i * 7919 + 13) & 0xFFFF for i in range(width * rows)]
        pixels = b"".join(v.to_bytes(2, "little") for v in values)

        def abgr(v):
            r, g, b = (v >> 11) & 31, (v >> 5) & 63, v & 31
            r, g, b = (r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)
            return 0xFF000000 | (b << 16) | (g << 8) | r

        out = self.run_c("blit", str(width), str(height), str(scale), str(row0), str(rows), pixels.hex()).stdout
        lines = out.splitlines()
        self.assertEqual(int(lines[0]), width * scale * rows * scale)             # nothing written elsewhere
        got = [[int(x, 16) for x in line.split()] for line in lines[1:]]
        want = []
        for r in range(rows):
            row = [abgr(values[r * width + x]) for x in range(width) for _ in range(scale)]
            want.extend([row] * scale)
        self.assertEqual(got, want)

    def test_blit_rejects_out_of_range(self):
        cases = (
            ("4", "4", "1", "3", "2", "00" * 16),       # rows run past the end of the picture
            ("4", "4", "300", "0", "1", "00" * 8),      # enlarged picture wider than the 960 pixel screen
            ("2", "300", "2", "290", "10", "00" * 40),  # enlarged picture taller than the 544 pixel screen
            ("4", "4", "0", "0", "1", "00" * 8),        # zoom of 0
        )
        for case in cases:
            self.assertEqual(self.run_c("blit", *case).stdout.strip(), "error", case)

    def test_stick_conversion(self):
        rows = [tuple(int(v) for v in line.split()) for line in self.run_c("axis").stdout.splitlines()]
        table = {raw: (normal, inverted) for raw, normal, inverted in rows}
        for raw in range(113, 144):                        # around the centre: dead zone
            self.assertEqual(table[raw], (0, 0), raw)
        self.assertEqual(table[128], (0, 0))
        self.assertEqual(table[255][0], 127)               # full right/up
        self.assertEqual(table[0], (-127, 127))            # full left/down; inverted flips the sign
        for raw in range(255):                             # monotonic, never out of range
            self.assertLessEqual(table[raw][0], table[raw + 1][0])
            self.assertTrue(-127 <= table[raw][0] <= 127)
            self.assertTrue(-127 <= table[raw][1] <= 127)


if __name__ == "__main__":
    unittest.main(verbosity=2)