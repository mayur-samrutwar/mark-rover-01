#!/usr/bin/env python3
"""Build and explicitly probe the local macOS Object Capture helper.

Uses installed Apple Command Line Tools/SDK and RealityKit; no package downloads
or global installation. Runtime support still requires native GPU access.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(value, indent=2) + "\n")
    temp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, default=Path(__file__).resolve().parents[2] / ".reconstruction-tools")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--json", action="store_true", help="Output is JSON by default")
    args = parser.parse_args()
    if sys.platform != "darwin":
        raise SystemExit("Apple Object Capture requires macOS")
    source = Path(__file__).with_name("photogrammetry.swift").resolve()
    prefix = args.prefix.expanduser().resolve()
    binary = prefix / "bin" / "PhotogrammetryReconstruct"
    logs = prefix / "logs"
    cache = prefix / "cache" / "swift"
    for folder in (binary.parent, logs, cache):
        folder.mkdir(parents=True, exist_ok=True)
    manifest_path = prefix / "photogrammetry-toolchain.json"
    swiftc = shutil.which("swiftc")
    if not swiftc:
        raise SystemExit("Apple swiftc is unavailable; install Apple's Command Line Tools")
    compiler = subprocess.check_output([swiftc, "--version"], text=True, timeout=15).strip()
    sdk = subprocess.check_output(["/usr/bin/xcrun", "--show-sdk-path"], text=True, timeout=15).strip()
    desired = {"source_sha256": sha(source), "compiler": compiler, "sdk_path": str(Path(sdk).resolve()),
               "platform": platform.platform(), "machine": platform.machine()}
    try:
        previous = json.loads(manifest_path.read_text())
    except (OSError, ValueError):
        previous = {}
    reusable = binary.is_file() and previous.get("build") == desired and previous.get("binary_sha256") == sha(binary)
    rebuilt = False
    if not args.probe_only and (args.force or not reusable):
        temporary = binary.with_name(binary.name + f".{os.getpid()}.tmp")
        argv = [swiftc, "-parse-as-library", "-O", "-module-cache-path", str(cache), str(source), "-o", str(temporary)]
        log = logs / "photogrammetry-build.log"
        started = time.monotonic()
        with log.open("w") as stream:
            stream.write("$ " + json.dumps(argv) + "\n")
            stream.flush()
            process = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=180)
            except BaseException:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
                temporary.unlink(missing_ok=True)
                raise
        if code:
            temporary.unlink(missing_ok=True)
            raise SystemExit(f"Native helper build failed; inspect {log}")
        temporary.chmod(0o755)
        temporary.replace(binary)
        rebuilt = True
        desired["build_seconds"] = time.monotonic() - started
    if not binary.is_file():
        raise SystemExit("Native helper is missing; run setup without --probe-only")
    result = subprocess.run([str(binary), "--probe"], capture_output=True, text=True, timeout=30)
    (logs / "photogrammetry-probe.log").write_text(result.stdout + result.stderr)
    capability = None
    for line in result.stdout.splitlines():
        if line.startswith("{"):
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if value.get("event") == "capability":
                capability = value
    if result.returncode or capability is None:
        raise SystemExit(f"Native capability probe failed; inspect {logs / 'photogrammetry-probe.log'}")
    # Build duration is descriptive; exclude it from future rebuild identity.
    build_seconds = desired.pop("build_seconds", previous.get("build_seconds"))
    manifest = {"version": 1, "engine": "Apple PhotogrammetrySession", "path": str(binary),
                "source": str(source), "build": desired, "binary_sha256": sha(binary),
                "build_seconds": build_seconds, "capability": capability,
                "available": bool(capability.get("supported")), "rebuilt": rebuilt,
                "ignoreBoundingBox_applied": False, "requires_external_packages": False}
    write_json(manifest_path, manifest)
    print(json.dumps(manifest, indent=2))
    return 0 if manifest["available"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
