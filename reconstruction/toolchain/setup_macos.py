#!/usr/bin/env python3
"""Install a pinned COLMAP release and build a headless OpenMVS Metal toolchain.

Only project-local tool files are written, except explicitly requested Homebrew
formula installation. Existing Homebrew Python files are never overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
PINS = json.loads((HERE / "pins.json").read_text())
TOOLS = ROOT / ".reconstruction-tools"
COMMANDS = ("InterfaceCOLMAP", "DensifyPointCloud", "ReconstructMesh", "RefineMesh", "TextureMesh", "TransformScene")


def run(argv, *, cwd=None, env=None, capture=False):
    argv = [str(x) for x in argv]
    print("+ " + shlex.join(argv), flush=True)
    result = subprocess.run(argv, cwd=cwd, env=env, check=True,
                            stdout=subprocess.PIPE if capture else None,
                            stderr=subprocess.STDOUT if capture else None,
                            text=True)
    return result.stdout.strip() if capture else ""


def sha256(path):
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def ensure_source(name, pin):
    path = TOOLS / "src" / name
    if not (path / ".git").is_dir():
        run(["git", "clone", "--filter=blob:none", "--no-checkout", pin["url"], path])
        run(["git", "fetch", "--depth", "1", "origin", pin["revision"]], cwd=path)
        run(["git", "checkout", "--detach", pin["revision"]], cwd=path)
    actual = run(["git", "rev-parse", "HEAD"], cwd=path, capture=True)
    if actual != pin["revision"]:
        # Do not discard edits in an existing checkout to make installation pass.
        if run(["git", "status", "--porcelain"], cwd=path, capture=True):
            raise RuntimeError(f"{path} contains changes; refusing to switch revisions")
        run(["git", "fetch", "--depth", "1", "origin", pin["revision"]], cwd=path)
        run(["git", "checkout", "--detach", pin["revision"]], cwd=path)
    actual = run(["git", "rev-parse", "HEAD"], cwd=path, capture=True)
    if actual != pin["revision"]:
        raise RuntimeError(f"Unexpected {name} revision: {actual}")
    return path


def apply_cmake_patch(source, filename):
    """Omit unused native SfM and GUI apps; dense algorithm sources are untouched."""
    patch = HERE / filename
    if subprocess.run(["git", "apply", "--unidiff-zero", "--reverse", "--check", str(patch)], cwd=source,
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        return
    run(["git", "apply", "--unidiff-zero", "--check", patch], cwd=source)
    run(["git", "apply", "--unidiff-zero", patch], cwd=source)


def verify_source_changes(source, patch_filename=None):
    diff = run(["git", "diff", "--no-ext-diff", "--binary", "--unified=0"], cwd=source, capture=True)
    expected = (HERE / patch_filename).read_text().strip() if patch_filename else ""
    untracked = run(["git", "ls-files", "--others", "--exclude-standard"], cwd=source, capture=True)
    if diff != expected or untracked:
        raise RuntimeError(f"{source} differs from its pin and documented CMake patch; refusing to build")


def link_command(name, target):
    link = TOOLS / "bin" / name
    if link.is_symlink():
        if link.resolve() == target.resolve():
            return
        link.unlink()
    elif link.exists():
        raise RuntimeError(f"Refusing to replace existing non-symlink: {link}")
    link.symlink_to(os.path.relpath(target, link.parent))


def cmake_build(source, build, prefix, jobs, extra=()):
    run(["cmake", "-S", source, "-B", build, "-G", "Ninja",
         "-DCMAKE_BUILD_TYPE=Release", f"-DCMAKE_INSTALL_PREFIX={prefix}",
         "-DCMAKE_POLICY_VERSION_MINIMUM=3.5", "-DCMAKE_CXX_STANDARD=23",
         "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
         "-DCMAKE_EXPORT_PACKAGE_REGISTRY=OFF", *extra])
    run(["cmake", "--build", build, "--parallel", str(jobs)])
    run(["cmake", "--install", build])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jobs", type=int, default=2, help="Concurrent C++ compile jobs (default: 2)")
    parser.add_argument("--skip-brew", action="store_true", help="Require Homebrew dependencies to exist")
    args = parser.parse_args()
    if platform.system() != "Darwin" or platform.machine() != "arm64":
        parser.error("This pinned setup targets macOS on Apple Silicon")
    if not 1 <= args.jobs <= 4:
        parser.error("Use 1–4 build jobs to limit memory pressure")
    for subdir in ("downloads", "src", "build", "bin", "prefix", "logs"):
        (TOOLS / subdir).mkdir(parents=True, exist_ok=True)
    brew = shutil.which("brew")
    if not brew:
        raise RuntimeError("Homebrew is required; install it separately first")
    brew_env = dict(os.environ, HOMEBREW_NO_AUTO_UPDATE="1", HOMEBREW_NO_INSTALL_CLEANUP="1")
    # Probe each formula. Avoid re-installing installed formulae or forcing links
    # over pip-managed cv2 files; use keg prefixes explicitly for CMake instead.
    missing = []
    for dependency in PINS["homebrew_dependencies"]:
        result = subprocess.run([brew, "list", "--versions", dependency], env=brew_env,
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        if result.returncode or not result.stdout.strip():
            missing.append(dependency)
    if missing and args.skip_brew:
        raise RuntimeError("Missing Homebrew dependencies: " + ", ".join(missing))
    if missing:
        result = subprocess.run([brew, "install", *missing], env=brew_env)
        # A keg may be installed while its global linking conflicts with existing
        # Python package files. Never force-overwrite those; explicit keg paths work.
        if result.returncode:
            for dependency in missing:
                run([brew, "list", "--versions", dependency], env=brew_env, capture=True)
    brew_prefix = Path(run([brew, "--prefix"], env=brew_env, capture=True))
    opencv_prefix = Path(run([brew, "--prefix", "opencv@4"], env=brew_env, capture=True))
    opencv_configs = sorted((opencv_prefix / "lib" / "cmake").glob("opencv*"))
    if len(opencv_configs) != 1:
        raise RuntimeError("Cannot identify OpenCV CMake configuration")
    prefix = TOOLS / "prefix"
    colmap = PINS["colmap"]
    archive = TOOLS / "downloads" / f"colmap-{colmap['version']}-macos-arm64.zip"
    if not archive.exists() or sha256(archive) != colmap["sha256"]:
        partial = archive.with_suffix(".partial")
        run(["curl", "-fL", "--retry", "3", "-o", partial, colmap["url"]])
        if sha256(partial) != colmap["sha256"]:
            raise RuntimeError("COLMAP archive checksum mismatch")
        partial.replace(archive)
    colmap_dir = TOOLS / f"colmap-{colmap['version']}"
    colmap_dir.mkdir(exist_ok=True)
    # ditto preserves app-bundle framework symlinks and executable permissions.
    run(["ditto", "-x", "-k", archive, colmap_dir])
    inner_archive = colmap_dir / "COLMAP-mac.zip"
    if inner_archive.exists():
        run(["ditto", "-x", "-k", inner_archive, colmap_dir])
    colmap_bin = colmap_dir / "COLMAP.app" / "Contents" / "MacOS" / "colmap"
    colmap_help = run([colmap_bin, "-h"], capture=True)
    if f"COLMAP {colmap['version']}" not in colmap_help:
        raise RuntimeError("Installed COLMAP version does not match pin")
    link_command("colmap", colmap_bin)
    sources = {name: ensure_source(name, pin) for name, pin in PINS["sources"].items()}
    include = prefix / "include"
    include.mkdir(exist_ok=True)
    for path in (sources["thread-pool"] / "include").glob("*.hpp"):
        shutil.copy2(path, include / path.name)
    shutil.copytree(sources["fast_float"] / "include" / "fast_float", include / "fast_float", dirs_exist_ok=True)
    for filename in ("tiny_gltf.h", "json.hpp"):
        shutil.copy2(sources["tinygltf"] / filename, include / filename)
    cmake_common = [f"-DCMAKE_PREFIX_PATH={prefix};{brew_prefix}", f"-DOpenCV_DIR={opencv_configs[0]}"]
    cmake_build(sources["tinyply"], TOOLS / "build" / "tinyply", prefix, args.jobs)
    for name in ("tinyply", "tinygltf", "thread-pool", "fast_float"):
        verify_source_changes(sources[name])
    apply_cmake_patch(sources["halfmesh"], "halfmesh-headless.patch")
    verify_source_changes(sources["halfmesh"], "halfmesh-headless.patch")
    cmake_build(sources["halfmesh"], TOOLS / "build" / "halfmesh", prefix, args.jobs,
                [*cmake_common, "-DHALFMESH_BUILD_TESTS=OFF", "-DHALFMESH_BUILD_TOOLS=OFF",
                 "-DHALFMESH_BUILD_PYTHON=OFF", "-DHALFMESH_LTO=OFF"])
    apply_cmake_patch(sources["openMVS"], "openmvs-headless.patch")
    verify_source_changes(sources["openMVS"], "openmvs-headless.patch")
    cmake_build(sources["openMVS"], TOOLS / "build" / "openMVS-jpeg", prefix, args.jobs,
                [*cmake_common, f"-DCMAKE_CXX_FLAGS=-I{include}", f"-DCMAKE_OBJCXX_FLAGS=-I{include}", "-DBUILD_SHARED_LIBS=OFF",
                 "-DCMAKE_DISABLE_FIND_PACKAGE_PkgConfig=ON",
                 "-DOpenMVS_BUILD_VIEWER=OFF", "-DOpenMVS_ENABLE_TESTS=OFF",
                 "-DOpenMVS_USE_PYTHON=OFF", "-DOpenMVS_USE_CUDA=OFF",
                 "-DOpenMVS_USE_SIFTGPU=OFF", "-DOpenMVS_USE_BREAKPAD=OFF",
                 "-DOpenMVS_ENABLE_IPO=OFF", "-DOpenMVS_HEADLESS_DEBUG=ON"])
    bins = {}
    for command in COMMANDS:
        candidates = [p for p in prefix.rglob(command) if p.is_file() and os.access(p, os.X_OK)]
        if len(candidates) != 1:
            raise RuntimeError(f"Cannot uniquely find installed {command}: {candidates}")
        bins[command] = candidates[0]
        link_command(command, candidates[0])
    config = (TOOLS / "build" / "openMVS-jpeg" / "ConfigLocal.h").read_text()
    metal_compiled = "#define _USE_METAL" in config
    if not metal_compiled:
        raise RuntimeError("OpenMVS was built without the required Metal backend")
    help_results = {}
    for name in COMMANDS:
        probe = subprocess.run([str(TOOLS / "bin" / name), "--help"], cwd=TOOLS / "logs",
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
        if probe.returncode not in (0, 1) or "Available options" not in probe.stdout:
            raise RuntimeError(f"{name} help probe failed: {probe.stdout[-2000:]}")
        help_results[name] = probe.stdout
    for name, output in help_results.items():
        (TOOLS / "logs" / f"{name}-help.txt").write_text(output + "\n")
    manifest = {
        "schema_version": 1, "platform": platform.platform(),
        "installed_at_unix": time.time(),
        "colmap": {"version": colmap["version"], "path": str(TOOLS / "bin" / "colmap"),
                   "distribution": "official-macos-arm64", "archive_sha256": colmap["sha256"]},
        "openmvs": {"version": "2.4.0-development", "revision": PINS["sources"]["openMVS"]["revision"],
                    "path": str(TOOLS / "bin"), "metal_compiled": metal_compiled,
                    "cmake_patch_sha256": sha256(HERE / "openmvs-headless.patch"),
                    "halfmesh_cmake_patch_sha256": sha256(HERE / "halfmesh-headless.patch"),
                    "binary_sha256": {name: sha256(path) for name, path in bins.items()},
                    "commands": {name: str(TOOLS / "bin" / name) for name in COMMANDS}},
        "validation": {"cli_help": True, "metal_active": False,
                       "note": "Metal compilation is confirmed; a real GPU reconstruction must verify activation."},
        "homebrew_versions": run([brew, "list", "--versions", *PINS["homebrew_dependencies"]], env=brew_env, capture=True).splitlines(),
        "compiler": run(["clang", "--version"], capture=True),
        "sdk_version": run(["xcrun", "--show-sdk-version"], capture=True),
        "build": {"parallel_jobs": args.jobs, "cxx_standard": 23, "optional_heif_jxl": False},
        "sources": PINS["sources"]
    }
    temporary = TOOLS / "toolchain.json.tmp"
    temporary.write_text(json.dumps(manifest, indent=2) + "\n")
    temporary.replace(TOOLS / "toolchain.json")
    print(f"Toolchain ready. Manifest: {TOOLS / 'toolchain.json'}")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        print(f"Setup failed: {error}", file=sys.stderr)
        sys.exit(1)
