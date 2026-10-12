"""Hardware-free tests for the SLAM helper tools: recorder.py, calibrate.py and make_tum_sequence.py.

Calibration is tested on synthetic checkerboard pictures made with a known camera, so the result can be
compared with the truth. Needs opencv-python and numpy. Run:  python test_slam_tools.py
"""
import glob
import os
import tempfile
import time
import unittest

import cv2
import numpy as np

import calibrate
import make_tum_sequence as tum
from drone import Frame
from recorder import FrameRecorder

COLS, ROWS, SQUARE = 9, 6, 0.025
TRUE_K = np.array([[520.0, 0, 320.0], [0, 520.0, 240.0], [0, 0, 1]])


def render_board(square_px=60, margin=1):
    sx, sy = COLS + 1, ROWS + 1
    img = np.full(((sy + 2 * margin) * square_px, (sx + 2 * margin) * square_px), 255, np.uint8)
    for j in range(sy):
        for i in range(sx):
            if (i + j) % 2 == 0:
                y0, x0 = (margin + j) * square_px, (margin + i) * square_px
                img[y0:y0 + square_px, x0:x0 + square_px] = 0
    return img


def render_view(board, rvec, tvec, margin=1):
    """The board as a 640x480 camera with intrinsics TRUE_K (no lens distortion) would see it."""
    sx, sy = COLS + 1, ROWS + 1
    x0, y0, x1, y1 = -margin * SQUARE, -margin * SQUARE, (sx + margin) * SQUARE, (sy + margin) * SQUARE
    world = np.array([[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]], np.float64)
    dst, _ = cv2.projectPoints(world, rvec, tvec, TRUE_K, None)
    h, w = board.shape
    H = cv2.getPerspectiveTransform(np.array([[0, 0], [w, 0], [w, h], [0, h]], np.float32),
                                    dst.reshape(-1, 2).astype(np.float32))
    out = cv2.warpPerspective(board, H, (640, 480), flags=cv2.INTER_LINEAR, borderValue=128)
    out = cv2.GaussianBlur(out, (3, 3), 0).astype(np.float32)
    out += np.random.default_rng(1).normal(0, 2, out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def make_views(directory, count=24, seed=7):
    rng = np.random.default_rng(seed)
    board = render_board()
    centre = np.array([(COLS + 1) * SQUARE / 2, (ROWS + 1) * SQUARE / 2, 0.0])
    spots = [(-0.14, -0.09), (0.14, -0.09), (-0.14, 0.09), (0.14, 0.09)]
    for n in range(count):
        rot, _ = cv2.Rodrigues(np.array([rng.uniform(-0.55, 0.55), rng.uniform(-0.55, 0.55), rng.uniform(-0.35, 0.35)]))
        tx, ty = spots[n] if n < len(spots) else (rng.uniform(-0.12, 0.12), rng.uniform(-0.08, 0.08))
        z = rng.uniform(0.42, 0.7)
        tvec = np.array([tx, ty, z]) - rot @ centre
        rvec, _ = cv2.Rodrigues(rot)
        cv2.imwrite(os.path.join(directory, f"view_{n:02d}.png"), render_view(board, rvec, tvec))


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        make_views(cls.tmp.name)
        cls.paths = sorted(glob.glob(os.path.join(cls.tmp.name, "*.png")))
        cls.result = calibrate.calibrate_images(cls.paths, COLS, ROWS, SQUARE)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_recovers_the_known_camera(self):
        r = self.result
        self.assertGreaterEqual(len(r["images_used"]), 15)
        self.assertAlmostEqual(r["fx"], 520.0, delta=520 * 0.04)
        self.assertAlmostEqual(r["fy"], 520.0, delta=520 * 0.04)
        self.assertAlmostEqual(r["cx"], 320.0, delta=12)
        self.assertAlmostEqual(r["cy"], 240.0, delta=12)
        self.assertLess(r["rms"], 1.0)
        self.assertEqual(r["image_size"], [640, 480])

    def test_yaml_is_readable_by_opencv_like_orbslam3(self):
        path = os.path.join(self.tmp.name, "settings.yaml")
        with open(path, "w") as f:
            f.write(calibrate.orbslam_yaml(self.result, fps=8))
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        self.assertTrue(fs.isOpened())
        self.assertAlmostEqual(fs.getNode("Camera1.fx").real(), self.result["fx"], places=3)
        self.assertEqual(fs.getNode("Camera.type").string(), "PinHole")
        self.assertEqual(int(fs.getNode("Camera.width").real()), 640)
        self.assertEqual(int(fs.getNode("Camera.fps").real()), 8)
        self.assertEqual(int(fs.getNode("Camera.RGB").real()), 0)
        for key in ("Camera1.k1", "Camera1.k2", "Camera1.p1", "Camera1.p2", "Camera1.k3", "ORBextractor.nFeatures",
                    "ORBextractor.scaleFactor", "ORBextractor.nLevels", "ORBextractor.iniThFAST",
                    "ORBextractor.minThFAST", "Viewer.ViewpointF"):
            self.assertFalse(fs.getNode(key).empty(), key)
        fs.release()

    def test_blank_picture_has_no_board(self):
        self.assertIsNone(calibrate.find_corners(np.full((480, 640), 128, np.uint8), COLS, ROWS))

    def test_too_few_photos_are_refused(self):
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.calibrate_images(self.paths[:5], COLS, ROWS, SQUARE)

    def test_bad_photos_are_reported_not_fatal(self):
        blank = os.path.join(self.tmp.name, "blank.jpg")
        cv2.imwrite(blank, np.full((480, 640), 128, np.uint8))
        result = calibrate.calibrate_images(self.paths + [blank], COLS, ROWS, SQUARE)
        self.assertEqual([os.path.basename(p) for p, _ in result["images_rejected"]], ["blank.jpg"])

    def test_warnings_for_a_poor_set(self):
        result = calibrate.calibrate_images(self.paths[:9], COLS, ROWS, SQUARE)
        self.assertTrue(any("photos used" in w for w in result["warnings"]))


class FakeLink:
    def __init__(self):
        self.frame = None

    def push(self, seq, ts, jpeg=b"\xff\xd8data\xff\xd9"):
        self.frame = Frame(jpeg, ts, seq)

    def get_frame(self):
        return self.frame


class RecorderTests(unittest.TestCase):
    def test_each_frame_is_saved_once_under_its_timestamp(self):
        with tempfile.TemporaryDirectory() as d:
            link = FakeLink()
            rec = FrameRecorder(link, os.path.join(d, "rec"), log=lambda *a: None)
            rec.start()
            for seq, ts in ((1, 1000.125), (2, 1000.25), (3, 1000.375)):
                link.push(seq, ts, jpeg=b"\xff\xd8" + bytes([seq]) * 10 + b"\xff\xd9")
                time.sleep(0.05)
                time.sleep(0.05)                                       # the same frame is seen many times
            rec.stop()
            names = sorted(os.listdir(os.path.join(d, "rec")))
            self.assertEqual(names, ["1000.125.jpg", "1000.250.jpg", "1000.375.jpg"])
            self.assertEqual(rec.count, 3)
            with open(os.path.join(d, "rec", "1000.250.jpg"), "rb") as f:
                self.assertEqual(f.read(), b"\xff\xd8" + bytes([2]) * 10 + b"\xff\xd9")


def load_like_mono_tum(path):
    """The same parsing as ORB-SLAM3's mono_tum.cc LoadImages: skip three lines, then 'time name' per line."""
    with open(path) as f:
        lines = f.read().split("\n")[3:]
    return [(float(l.split()[0]), l.split()[1]) for l in lines if l.strip()]


class TumSequenceTests(unittest.TestCase):
    def make_recording(self, d, n=20):
        rec = os.path.join(d, "rec")
        os.makedirs(rec)
        for i in range(n):
            with open(os.path.join(rec, f"{1700000000 + i * 0.125:.3f}.jpg"), "wb") as f:
                f.write(b"jpeg%d" % i)
        with open(os.path.join(rec, "notes.jpg"), "wb") as f:           # not a timestamped frame: ignored
            f.write(b"x")
        return rec

    def test_format_matches_what_orbslam3_reads(self):
        with tempfile.TemporaryDirectory() as d:
            n, first, last = tum.make_sequence(self.make_recording(d), os.path.join(d, "seq"))
            self.assertEqual((n, first, last), (20, 1700000000.0, 1700000002.375))
            entries = load_like_mono_tum(os.path.join(d, "seq", "rgb.txt"))
            self.assertEqual(len(entries), 20)
            self.assertEqual(entries[0], (1700000000.0, "rgb/1700000000.000.jpg"))
            self.assertEqual(sorted(t for t, _ in entries), [t for t, _ in entries])
            for _, name in entries:
                self.assertTrue(os.path.exists(os.path.join(d, "seq", name)))
            with open(os.path.join(d, "seq", "rgb.txt")) as f:
                self.assertTrue(all(line.startswith("#") for line in f.read().split("\n")[:3]))

    def test_start_and_end_trim(self):
        with tempfile.TemporaryDirectory() as d:
            n, first, last = tum.make_sequence(self.make_recording(d), os.path.join(d, "seq"), start=0.5, end=1.5)
            self.assertAlmostEqual(first, 1700000000.5)
            self.assertAlmostEqual(last, 1700000001.5)
            self.assertEqual(n, 9)

    def test_empty_inputs_are_errors(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                tum.make_sequence(d, os.path.join(d, "seq"))
            with self.assertRaises(ValueError):
                tum.make_sequence(self.make_recording(d), os.path.join(d, "seq2"), start=100)


if __name__ == "__main__":
    unittest.main(verbosity=2)