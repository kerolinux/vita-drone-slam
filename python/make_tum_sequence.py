"""make_tum_sequence.py - turns a recording folder (<time>.jpg files) into a TUM-style sequence that
ORB-SLAM3's stock mono_tum example can read.

    python make_tum_sequence.py recordings/20261012_153000 sequences/flight1

Creates sequences/flight1/rgb/ (copies of the JPEGs) and sequences/flight1/rgb.txt (three comment lines, then
"timestamp rgb/filename" per frame). Use --start / --end (seconds from the first frame) to cut a part out.
"""
import argparse
import glob
import os
import shutil
import sys


def make_sequence(src, dst, start=None, end=None):
    """Returns (frames written, first timestamp, last timestamp)."""
    frames = []
    for path in glob.glob(os.path.join(src, "*.jpg")):
        try:
            frames.append((float(os.path.splitext(os.path.basename(path))[0]), path))
        except ValueError:
            continue                                            # not a timestamped frame
    frames.sort()
    if not frames:
        raise ValueError(f"no <timestamp>.jpg files in {src!r}")
    t0 = frames[0][0]
    frames = [(t, p) for t, p in frames
              if (start is None or t - t0 >= start) and (end is None or t - t0 <= end)]
    if not frames:
        raise ValueError("no frames left after --start / --end")
    os.makedirs(os.path.join(dst, "rgb"), exist_ok=True)
    with open(os.path.join(dst, "rgb.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("# color images\n# file: '%s'\n# timestamp filename\n" % os.path.basename(os.path.normpath(src)))
        for t, path in frames:
            name = f"{t:.3f}.jpg"
            shutil.copyfile(path, os.path.join(dst, "rgb", name))
            f.write(f"{t:.3f} rgb/{name}\n")
    return len(frames), frames[0][0], frames[-1][0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording", help="folder with <timestamp>.jpg files")
    ap.add_argument("sequence", help="output folder to create")
    ap.add_argument("--start", type=float, help="skip the first N seconds")
    ap.add_argument("--end", type=float, help="stop N seconds after the first frame")
    args = ap.parse_args()
    try:
        n, first, last = make_sequence(args.recording, args.sequence, args.start, args.end)
    except ValueError as exc:
        sys.exit(f"Error: {exc}")
    secs = last - first
    print(f"{n} frames, {secs:.1f} s ({n / secs:.1f} fps average)" if secs > 0 else f"{n} frame(s)")
    print(f"Wrote {args.sequence}\\rgb.txt")


if __name__ == "__main__":
    main()