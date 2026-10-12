"""calibrate.py - camera calibration from photos of a printed checkerboard.

1. Print a checkerboard (for example 9x6 inner corners, 25 mm squares) at 100 % scale on a flat, rigid surface.
2. Collect 20-30 photos with capture_calib.py (it only saves a photo when the board is found).
3. Run:  python calibrate.py --images calib --cols 9 --rows 6 --square 0.025

Writes calibration.json (camera matrix, distortion, error) and drone_orbslam3.yaml (settings file for
ORB-SLAM3). "cols" and "rows" count INNER corners (where four squares meet), not squares: a board with
10 x 7 squares has 9 x 6 inner corners.
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

MIN_IMAGES = 8
GOOD_IMAGES = 15


class CalibrationError(Exception):
    pass


def object_points(cols, rows, square):
    pts = np.zeros((cols * rows, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return pts


def find_corners(gray, cols, rows, fast=False):
    """Inner corners of the checkerboard as an (N, 1, 2) float32 array, or None if the board is not found."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast:
        flags |= cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))


def calibrate_images(paths, cols, rows, square):
    objp = object_points(cols, rows, square)
    obj, pts, used, rejected = [], [], [], []
    size = None
    for path in sorted(paths):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            rejected.append([path, "could not be read"])
            continue
        this_size = (img.shape[1], img.shape[0])
        if size is None:
            size = this_size
        elif this_size != size:
            rejected.append([path, "different image size"])
            continue
        corners = find_corners(img, cols, rows)
        if corners is None:
            rejected.append([path, "checkerboard not found"])
            continue
        obj.append(objp)
        pts.append(corners)
        used.append(path)
    if len(used) < MIN_IMAGES:
        raise CalibrationError(f"only {len(used)} usable photos (need at least {MIN_IMAGES}, 20-30 is better)")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj, pts, size, None, None)
    errors = {}
    for i, path in enumerate(used):
        proj, _ = cv2.projectPoints(obj[i], rvecs[i], tvecs[i], K, dist)
        diff = proj.reshape(-1, 2) - pts[i].reshape(-1, 2)
        errors[path] = float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))
    allc = np.concatenate([p.reshape(-1, 2) for p in pts])
    coverage = {"x": float((allc[:, 0].max() - allc[:, 0].min()) / size[0]),
                "y": float((allc[:, 1].max() - allc[:, 1].min()) / size[1])}
    d = dist.reshape(-1)
    result = {
        "image_size": [size[0], size[1]],
        "fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
        "dist": {"k1": float(d[0]), "k2": float(d[1]), "p1": float(d[2]), "p2": float(d[3]),
                 "k3": float(d[4]) if len(d) > 4 else 0.0},
        "rms": float(rms),
        "pattern": {"cols": cols, "rows": rows, "square": square},
        "images_used": used, "images_rejected": rejected,
        "per_image_error": errors, "coverage": coverage,
    }
    result["warnings"] = quality_warnings(result)
    return result


def quality_warnings(result):
    w = []
    n = len(result["images_used"])
    if n < GOOD_IMAGES:
        w.append(f"only {n} photos used; 20-30 gives a more reliable result")
    if result["rms"] > 1.0:
        w.append(f"reprojection error is {result['rms']:.2f} px (below 0.5 is good, above 1.0 is poor): "
                 "check the board is flat and the photos are sharp")
    if min(result["coverage"].values()) < 0.8:
        w.append("the board never reached the edges of the picture in one direction; take photos with the "
                 "board near the edges and corners (that is where lens distortion is strongest)")
    errs = result["per_image_error"]
    if errs:
        median = float(np.median(list(errs.values())))
        bad = [os.path.basename(p) for p, e in errs.items() if e > max(2 * median, 1.0)]
        if bad:
            w.append("worst photos (consider deleting them): " + ", ".join(bad))
    return w


def orbslam_yaml(result, fps=8, n_features=1000, ini_th_fast=20, min_th_fast=7):
    """Settings file for ORB-SLAM3 (same layout as its Examples/Monocular/TUM1.yaml). Images are read with
    OpenCV, which gives BGR order, hence Camera.RGB: 0."""
    d = result["dist"]
    w, h = result["image_size"]
    return f"""%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera parameters (made by calibrate.py; reprojection error {result['rms']:.3f} px)
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

Camera1.fx: {result['fx']:.6f}
Camera1.fy: {result['fy']:.6f}
Camera1.cx: {result['cx']:.6f}
Camera1.cy: {result['cy']:.6f}

Camera1.k1: {d['k1']:.6f}
Camera1.k2: {d['k2']:.6f}
Camera1.p1: {d['p1']:.6f}
Camera1.p2: {d['p2']:.6f}
Camera1.k3: {d['k3']:.6f}

Camera.fps: {fps}

# Color order of the images (0: BGR, 1: RGB). OpenCV reads BGR.
Camera.RGB: 0

Camera.width: {w}
Camera.height: {h}

#--------------------------------------------------------------------------------------------
# ORB parameters
#--------------------------------------------------------------------------------------------
ORBextractor.nFeatures: {n_features}
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: {ini_th_fast}
ORBextractor.minThFAST: {min_th_fast}

#--------------------------------------------------------------------------------------------
# Viewer parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
"""


def undistort_preview(result, image_path, out_path):
    """Side-by-side picture: original on the left, undistorted on the right."""
    img = cv2.imread(image_path)
    K = np.array([[result["fx"], 0, result["cx"]], [0, result["fy"], result["cy"]], [0, 0, 1]])
    d = result["dist"]
    dist = np.array([d["k1"], d["k2"], d["p1"], d["p2"], d["k3"]])
    cv2.imwrite(out_path, np.hstack([img, cv2.undistort(img, K, dist)]))


def main():
    ap = argparse.ArgumentParser(description="Camera calibration from checkerboard photos")
    ap.add_argument("--images", default="calib", help="folder with the photos (.jpg / .png)")
    ap.add_argument("--cols", type=int, default=9, help="inner corners across (squares across minus 1)")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down (squares down minus 1)")
    ap.add_argument("--square", type=float, default=0.025, help="side of one printed square in metres")
    ap.add_argument("--out", default="calibration.json")
    ap.add_argument("--yaml", default="drone_orbslam3.yaml", help="ORB-SLAM3 settings file to write")
    ap.add_argument("--fps", type=int, default=8, help="camera frame rate to put in the settings file")
    ap.add_argument("--preview", default="undistort_preview.jpg", help="side-by-side before/after picture")
    args = ap.parse_args()

    paths = [p for ext in ("*.jpg", "*.jpeg", "*.png") for p in glob.glob(os.path.join(args.images, ext))]
    if not paths:
        sys.exit(f"No photos found in {args.images!r}")
    try:
        result = calibrate_images(paths, args.cols, args.rows, args.square)
    except CalibrationError as exc:
        sys.exit(f"Calibration failed: {exc}")

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    with open(args.yaml, "w", encoding="utf-8") as f:
        f.write(orbslam_yaml(result, fps=args.fps))
    if args.preview:
        undistort_preview(result, result["images_used"][0], args.preview)

    d = result["dist"]
    print(f"Used {len(result['images_used'])} of {len(paths)} photos, image size {result['image_size']}")
    print(f"fx {result['fx']:.1f}  fy {result['fy']:.1f}  cx {result['cx']:.1f}  cy {result['cy']:.1f}")
    print(f"distortion k1 {d['k1']:.4f}  k2 {d['k2']:.4f}  p1 {d['p1']:.4f}  p2 {d['p2']:.4f}  k3 {d['k3']:.4f}")
    print(f"reprojection error: {result['rms']:.3f} px   (picture coverage x {result['coverage']['x']:.0%}, "
          f"y {result['coverage']['y']:.0%})")
    for path, why in result["images_rejected"]:
        print(f"  skipped {os.path.basename(path)}: {why}")
    for warning in result["warnings"]:
        print("WARNING:", warning)
    print(f"Wrote {args.out}, {args.yaml} and {args.preview}")


if __name__ == "__main__":
    main()