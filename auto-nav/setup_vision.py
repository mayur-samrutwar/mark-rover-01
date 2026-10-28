"""Add OpenCV locally without changing IsaacProtocol's Python environment.

Run using the same interpreter as the simulator:
    ../isaacprotocol/simulation/venv/bin/python auto-nav/setup_vision.py

The .deps directory must be on PYTHONPATH when running navigation. No package
is installed into the sibling simulator checkout or global Python environment.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
TARGET = ROOT / ".deps"
VERSION = "5.0.0.93"


def check() -> dict:
    """Exercise the native operations required by visual navigation."""
    sys.path.insert(0, str(TARGET))
    import cv2
    import numpy as np

    package_version = importlib.metadata.version("opencv-python-headless")
    if package_version != VERSION or not Path(cv2.__file__).is_relative_to(TARGET):
        raise RuntimeError("The pinned project-local OpenCV package is not active")
    rng = np.random.default_rng(31)
    gray = rng.integers(0, 256, (240, 320), dtype=np.uint8)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    moved = cv2.warpAffine(gray, np.float32([[1, 0, 3], [0, 1, 1]]), (320, 240))
    points = cv2.goodFeaturesToTrack(gray, 120, .01, 8)
    tracked, valid, _ = cv2.calcOpticalFlowPyrLK(gray, moved, points, None)
    valid = valid.reshape(-1).astype(bool)
    delta = np.median((tracked - points).reshape(-1, 2)[valid], axis=0)
    keypoints, descriptors = cv2.ORB_create().detectAndCompute(gray, None)
    if sum(valid) < 80 or np.linalg.norm(delta - [3, 1]) > .1:
        raise RuntimeError("OpenCV optical flow verification failed")
    if descriptors is None or len(keypoints) < 100:
        raise RuntimeError("OpenCV feature extraction verification failed")
    return {
        "python": sys.version.split()[0],
        "opencv": cv2.__version__,
        "opencv_package": package_version,
        "numpy": np.__version__,
        "tracked_features": int(sum(valid)),
        "median_pixel_translation": delta.tolist(),
        "orb_features": len(keypoints),
        "module_path": cv2.__file__,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify without downloading")
    args = parser.parse_args()
    if not args.check:
        installed = list(TARGET.glob(f"opencv_python_headless-{VERSION}.dist-info"))
        if not installed:
            subprocess.run([
                sys.executable, "-m", "pip", "install", "--disable-pip-version-check",
                "--no-deps", "--only-binary=:all:", "--no-cache-dir",
                "--target", str(TARGET), "-r", str(ROOT / "vision-requirements.txt"),
            ], check=True)
    print(json.dumps(check(), indent=2))


if __name__ == "__main__":
    main()
