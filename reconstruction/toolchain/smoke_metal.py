#!/usr/bin/env python3
"""Exercise actual Metal depth estimation on OpenMVS's bundled four-image scene.

This uses upstream sample data, never rover captures. It verifies GPU activation
and nonempty output, not room-scale geometric accuracy or COLMAP camera recovery.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / ".reconstruction-tools"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-resolution", type=int, default=960)
    args = parser.parse_args()
    if not 320 <= args.max_resolution <= 1600:
        parser.error("Smoke-test resolution must be between 320 and 1600")
    source = TOOLS / "src" / "openMVS" / "apps" / "Tests" / "data"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    target = TOOLS / "tests" / f"metal-smoke-{stamp}"
    shutil.copytree(source, target)
    # The JPEG-focused toolchain omits optional HEIF/JXL native codecs. Decode
    # this upstream fixture's two HEIC files with macOS, then change only the
    # equally sized image-name suffixes in the uncompressed MVSI interface.
    heic_files = sorted((target / "images").glob("*.heic"))
    scene_path = target / "scene.mvs"
    scene_bytes = scene_path.read_bytes()
    if not scene_bytes.startswith(b"MVSI") or scene_bytes.count(b".heic") != len(heic_files):
        raise RuntimeError("Unexpected upstream fixture format; refusing binary path rewrite")
    for image in heic_files:
        subprocess.run(["sips", "-s", "format", "jpeg", str(image), "--out", str(image.with_suffix(".jpeg"))],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    scene_path.write_bytes(scene_bytes.replace(b".heic", b".jpeg"))
    output = target / "scene_dense.mvs"
    log = target / "densify.log"
    argv = [str(TOOLS / "bin" / "DensifyPointCloud"), "--input-file", str(target / "scene.mvs"),
            "--output-file", str(output), "--archive-type", "2", "--gpu-device", "-1", "--max-threads", "2",
            "--resolution-level", "2", "--max-resolution", str(args.max_resolution),
            "--min-resolution", "320", "--number-views", "3", "--geometric-iters", "1",
            "--verbosity", "2"]
    start = time.monotonic()
    with log.open("w") as stream:
        result = subprocess.run(argv, cwd=target, stdout=stream, stderr=subprocess.STDOUT, timeout=300)
    text = log.read_text(errors="replace")
    active = "Using Metal compute backend for depth-map estimation" in text
    # An explicit backend message alone is insufficient: require successful
    # estimation, fusion and a genuine nonempty resulting point-cloud file.
    pointcloud = target / "scene_dense.ply"
    points = 0
    if pointcloud.exists():
        header = pointcloud.read_bytes()[:4096].decode("ascii", errors="ignore")
        match = re.search(r"element vertex (\d+)", header)
        points = int(match.group(1)) if match else 0
    passed = result.returncode == 0 and active and output.exists() and points > 0
    report = {"test": "openmvs-bundled-scene-metal-densification", "passed": passed,
              "metal_active": active, "returncode": result.returncode,
              "points": points, "elapsed_seconds": round(time.monotonic() - start, 3),
              "log": str(log), "output": str(output), "command": argv,
              "scope": "Upstream four-image scene with known cameras; no apartment quality claim."}
    (target / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    manifest_path = TOOLS / "toolchain.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        manifest["validation"] = {**manifest.get("validation", {}),
                                  "metal_active": active and passed, "metal_smoke": report,
                                  "note": "Actual Metal depth estimation and fusion passed on the bundled small scene." if passed else "Metal smoke failed; inspect its log."}
        temporary = manifest_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(manifest, indent=2) + "\n")
        temporary.replace(manifest_path)
    print(json.dumps(report, indent=2))
    if not passed:
        print(text[-10000:], file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
