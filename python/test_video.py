"""Hardware-free tests for the Vita video path (vitalink video messages and videoforward.py).

Needs opencv-python and numpy (already in requirements.txt). Run:  python test_video.py
"""
import socket
import threading
import time
import unittest

import cv2
import numpy as np

import vitalink as vl
import videoforward as vf
from drone import Frame


def make_jpeg(width=640, height=480):
    """A colourful test picture as JPEG bytes (similar size to the drone's frames)."""
    y, x = np.mgrid[0:height, 0:width]
    img = np.dstack([(x * 255 // width), (y * 255 // height), ((x + y) * 255 // (width + height))]).astype(np.uint8)
    cv2.circle(img, (width // 2, height // 2), height // 4, (0, 0, 255), -1)
    ok, data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    assert ok
    return data.tobytes()


def reassemble(chunks):
    """Rebuild the RGB565 image from video packets, exactly as the Vita does (rows written where they belong)."""
    first = vl.parse_video_chunk(chunks[0])
    w, h = first["width"], first["height"]
    image = np.zeros((h, w), dtype="<u2")
    for raw in chunks:
        c = vl.parse_video_chunk(raw)
        rows = np.frombuffer(c["pixels"], dtype="<u2").reshape(c["rows"], w)
        image[c["row_start"]:c["row_start"] + c["rows"]] = rows
    return image


class MessageTests(unittest.TestCase):
    def test_round_trip(self):
        pixels = bytes(range(256)) * 5                       # 1280 bytes = 2 rows of 320 pixels... use width 640
        data = vl.pack_video_chunk(300, 640, 480, 10, 1, True, pixels[:1280])
        c = vl.parse_video_chunk(data)
        self.assertEqual((c["frame_id"], c["width"], c["height"], c["row_start"], c["rows"], c["last"]),
                         (300 & 0xFF, 640, 480, 10, 1, True))
        self.assertEqual(c["pixels"], pixels[:1280])

    def test_invalid_packets_rejected(self):
        good = vl.pack_video_chunk(1, 4, 4, 0, 2, False, bytes(16))
        self.assertIsNotNone(vl.parse_video_chunk(good))
        self.assertIsNone(vl.parse_video_chunk(good[:-1]))                 # pixel data too short
        self.assertIsNone(vl.parse_video_chunk(good + b"x"))               # too long
        self.assertIsNone(vl.parse_video_chunk(b"XX" + good[2:]))          # bad magic
        self.assertIsNone(vl.parse_video_chunk(good[:2] + b"\x09" + good[3:]))   # bad version
        self.assertIsNone(vl.parse_video_chunk(vl.pack_video_chunk(1, 4, 4, 3, 1, False, bytes(8))[:12] +
                                               bytes(16)))                 # wrong pixel length for 1 row
        with self.assertRaises(ValueError):
            vl.pack_video_chunk(1, 4, 4, 0, 2, False, bytes(15))
        past_end = bytearray(good)
        past_end[8] = 3                                                    # row_start 3 + 2 rows > height 4
        self.assertIsNone(vl.parse_video_chunk(bytes(past_end)))


class ConversionTests(unittest.TestCase):
    def test_rgb565_values(self):
        bgr = np.array([[[0, 0, 255], [0, 255, 0], [255, 0, 0], [255, 255, 255], [0, 0, 0]]], dtype=np.uint8)
        self.assertEqual([hex(v) for v in vf.to_rgb565(bgr)[0]], ["0xf800", "0x7e0", "0x1f", "0xffff", "0x0"])

    def test_chunking_covers_the_image(self):
        jpeg = make_jpeg()
        chunks = vf.encode_chunks(jpeg, 5, (320, 240))
        self.assertEqual(len(chunks), 120)                                 # 2 rows of 640 bytes per packet
        self.assertTrue(all(len(c) <= vl.VIDEO_HEADER_LEN + vl.VIDEO_MAX_PAYLOAD for c in chunks))
        parsed = [vl.parse_video_chunk(c) for c in chunks]
        self.assertEqual([p["row_start"] for p in parsed], list(range(0, 240, 2)))
        self.assertEqual([p["last"] for p in parsed], [False] * 119 + [True])
        self.assertTrue(all(p["frame_id"] == 5 for p in parsed))

        img = cv2.resize(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), (320, 240),
                         interpolation=cv2.INTER_AREA)
        np.testing.assert_array_equal(reassemble(chunks), vf.to_rgb565(img))

    def test_other_sizes(self):
        for size in ((240, 180), (640, 480), (100, 75)):
            chunks = vf.encode_chunks(make_jpeg(), 0, size)
            image = reassemble(chunks)
            self.assertEqual(image.shape, (size[1], size[0]))
            self.assertTrue(vl.parse_video_chunk(chunks[-1])["last"])

    def test_bad_jpeg_gives_no_packets(self):
        self.assertEqual(vf.encode_chunks(b"not a jpeg", 0), [])


class FakeLink:
    def __init__(self, jpeg):
        self.jpeg, self.seq = jpeg, 0

    def new_frame(self):
        self.seq += 1

    def get_frame(self):
        return Frame(self.jpeg, time.time(), self.seq) if self.seq else None


class ForwarderTests(unittest.TestCase):
    def setUp(self):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", 0))
        self.rx.settimeout(1.0)
        self.dest = ("127.0.0.1", self.rx.getsockname()[1])
        self.link = FakeLink(make_jpeg())

    def tearDown(self):
        self.rx.close()

    def receive_frame(self):
        chunks = []
        while True:
            data, _ = self.rx.recvfrom(2048)
            chunks.append(data)
            if vl.parse_video_chunk(data)["last"]:
                return chunks

    def test_frame_arrives_complete(self):
        fwd = vf.VideoForwarder(self.link, lambda: self.dest, size=(320, 240), fps=20, log=lambda *a: None)
        fwd.start()
        self.link.new_frame()
        chunks = self.receive_frame()
        fwd.stop()
        self.assertEqual(len(chunks), 120)
        self.assertEqual(reassemble(chunks).shape, (240, 320))

    def test_nothing_sent_until_the_vita_is_known(self):
        dest = [None]
        fwd = vf.VideoForwarder(self.link, lambda: dest[0], size=(160, 120), fps=20, log=lambda *a: None)
        fwd.start()
        self.link.new_frame()
        self.rx.settimeout(0.3)
        with self.assertRaises(socket.timeout):
            self.rx.recvfrom(2048)
        dest[0] = self.dest
        self.rx.settimeout(1.0)
        self.link.new_frame()
        self.assertTrue(self.receive_frame())
        fwd.stop()

    def test_frame_rate_is_capped(self):
        fwd = vf.VideoForwarder(self.link, lambda: self.dest, size=(160, 120), fps=5, log=lambda *a: None)
        fwd.start()
        end = time.time() + 1.5
        while time.time() < end:
            self.link.new_frame()                                    # a new frame every 10 ms
            time.sleep(0.01)
        fwd.stop()
        self.assertTrue(4 <= fwd.frames_sent <= 9, fwd.frames_sent)


if __name__ == "__main__":
    unittest.main(verbosity=2)