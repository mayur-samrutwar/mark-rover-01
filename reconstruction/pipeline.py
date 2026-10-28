"""Resumable COLMAP/OpenMVS reconstruction, isolated from rover control.

The worker receives an immutable images/ snapshot in its job directory. Engine
processes run without a shell, in their own process groups, with bounded thread
counts, stage timeouts and cancellation. Completed markers are reusable only
when both their input fingerprint and stage-specific output checks match.
"""
from __future__ import annotations

import hashlib
from contextlib import closing
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import subprocess
import threading
import time
import zipfile

try:
    from . import geometry
except ImportError:
    import geometry


STAGES = [{"id": name, "label": label} for name, label in (
    ("prepare", "Prepare images"), ("features", "Extract features"),
    ("matching", "Match images"), ("alignment", "Align cameras"),
    ("undistort", "Undistort images"), ("import", "Import calibrated scene"),
    ("dense", "Estimate dense geometry"), ("mesh", "Reconstruct mesh"),
    ("refine", "Refine surfaces"), ("texture", "Texture mesh"),
    ("export", "Export scene"))]
_ENGINES = ("colmap", "InterfaceCOLMAP", "DensifyPointCloud", "ReconstructMesh", "RefineMesh", "TextureMesh")
_EXTENSIONS = {".jpg", ".jpeg", ".png", ".tif", ".tiff"}


class PipelineError(RuntimeError):
    def __init__(self, message, stage=None):
        super().__init__(message)
        self.stage = stage


class PipelineCancelled(PipelineError):
    pass


def _write_json(path, value):
    geometry._json(Path(path), value)


def _nonempty(path):
    return path.is_file() and path.stat().st_size > 0


def _read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def _hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _kill(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=3)


def _run(argv, *, cwd, log, cancel, timeout, threads):
    if cancel.is_set():
        raise PipelineCancelled("Reconstruction cancelled")
    env = os.environ.copy()
    env.update({key: str(threads) for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "MKL_NUM_THREADS")})
    log.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with log.open("ab") as output:
        output.write(("\n$ " + json.dumps([str(x) for x in argv]) + "\n").encode())
        output.flush()
        try:
            process = subprocess.Popen([str(x) for x in argv], cwd=str(cwd), env=env,
                                       stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        except OSError as exc:
            raise PipelineError(f"Could not start {Path(str(argv[0])).name}: {exc}") from exc
        try:
            while process.poll() is None:
                if cancel.wait(0.15):
                    raise PipelineCancelled("Reconstruction cancelled")
                if time.monotonic() - started > timeout:
                    raise PipelineError(f"{Path(str(argv[0])).name} exceeded its {timeout:g}-second stage timeout")
            if process.returncode:
                raise PipelineError(f"{Path(str(argv[0])).name} exited with code {process.returncode}; see {log.name}")
        finally:
            _kill(process)
    return time.monotonic() - started


def _probe(argv, cwd, log, cancel, threads):
    # Some engines return nonzero for help; the option text is still useful.
    log.unlink(missing_ok=True)
    try:
        _run(argv, cwd=cwd, log=log, cancel=cancel, timeout=30, threads=threads)
    except PipelineCancelled:
        raise
    except PipelineError:
        if not _nonempty(log):
            raise
    text = log.read_text(errors="replace")[-256000:]
    if not (_options(text) - {"--help"}):
        raise PipelineError(f"Cannot inspect CLI options for {Path(str(argv[0])).name}; see {log.name}")
    return text


def _options(help_text):
    return set(re.findall(r"--[A-Za-z][A-Za-z0-9_.-]*", help_text))


def _flag(argv, help_text, names, value, required=False):
    available = _options(help_text)
    for name in names:
        if name in available:
            argv.extend((name, str(value)))
            return name
    if required:
        raise PipelineError(f"Engine does not expose required option {' / '.join(names)}")
    return None


def _resolve_mvs_profile(name, helps):
    """Require the calibrated path's documented semantics, not optional flags."""
    if name not in ("standard", "calibrated", "calibrated-uniform"):
        raise PipelineError("mvs_profile must be standard, calibrated or calibrated-uniform")
    if name == "standard":
        # Older tools remain usable without adaptive-confidence support.
        return {"name": name}
    dense_help = helps["DensifyPointCloud"]
    mesh_help = helps["ReconstructMesh"]
    dense_text = " ".join(dense_help.split()).lower()
    mesh_text = " ".join(mesh_help.split()).lower()
    supported = ("--postprocess-dmaps" in _options(dense_help)
                 and re.search(r"\b8\s*-\s*adjust-confidence\b", dense_text)
                 and "--constant-weight" in _options(mesh_help)
                 and "per-view point confidence" in mesh_text
                 and "disable it only" in mesh_text and "recalibrated" in mesh_text)
    if not supported:
        raise PipelineError("calibrated mvs_profile requires documented --postprocess-dmaps 8 "
                            "confidence adjustment and --constant-weight 0 calibrated confidence weighting")
    return {"name": name, "postprocess_dmaps": 8, "constant_weight": 0 if name == "calibrated" else 1,
            "confidence_confirmation_required": True}


def _require_confidence_adjustment(log, registered_images):
    """Only a successful confirmation from the latest densifier run is usable."""
    try:
        text = log.read_text(errors="replace")
    except OSError as exc:
        raise PipelineError("Calibrated depth confidence has no dense invocation log", "dense") from exc
    _, separator, latest = text.rpartition("\n$ ")
    output = latest.partition("\n")[2] if separator else ""
    failed = re.search(r"confidence[^\n]*(?:failed|failure|error)|(?:failed|failure|error)[^\n]*confidence", output, re.I)
    counts = re.findall(r"Confidence-maps adjusted:\s*(\d+)\s+depth-maps\b", output, re.I)
    if failed or not counts or any(int(count) != registered_images for count in counts):
        raise PipelineError(f"Calibrated depth confidence requires a successful latest-run confirmation "
                            f"for all {registered_images} registered images; inspect logs/dense.log", "dense")
    return registered_images


def _qrotation(q):
    w, x, y, z = q
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise PipelineError("COLMAP produced an invalid camera rotation")
    w, x, y, z = [v / norm for v in q]
    return [[1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]]


def _read_model(folder):
    """Read COLMAP's documented text format, including names containing spaces."""
    camera_models = {}
    for line in (folder / "cameras.txt").read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        values = line.split()
        camera_models[int(values[0])] = {"model": values[1], "width": int(values[2]), "height": int(values[3]),
                                         "parameters": [float(v) for v in values[4:]]}
    lines = (folder / "images.txt").read_text().splitlines()
    cameras, index = [], 0
    while index < len(lines):
        line = lines[index].strip()
        index += 1
        if not line or line.startswith("#"):
            continue
        values = line.split(maxsplit=9)
        if len(values) != 10:
            raise PipelineError("COLMAP images.txt contains an incomplete camera record")
        q = [float(v) for v in values[1:5]]
        translation = [float(v) for v in values[5:8]]
        rotation = _qrotation(q)
        center = [-sum(rotation[row][col] * translation[row] for row in range(3)) for col in range(3)]
        if not all(math.isfinite(v) for v in center + translation):
            raise PipelineError("COLMAP produced non-finite camera coordinates")
        cameras.append({"image_id": int(values[0]), "image": values[9], "camera_id": int(values[8]),
                        "center": center, "translation": translation, "rotation_world_to_camera": rotation,
                        "quaternion_wxyz": q})
        index += 1  # The following line contains 2D observations and may be empty.
    errors, points = [], 0
    for line in (folder / "points3D.txt").read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        values = line.split()
        if len(values) < 8:
            raise PipelineError("COLMAP points3D.txt contains an incomplete point record")
        xyz = [float(v) for v in values[1:4]]
        error = float(values[7])
        if not all(math.isfinite(v) for v in xyz + [error]) or error < 0:
            raise PipelineError("COLMAP produced invalid sparse geometry")
        points += 1
        errors.append(error)
    errors.sort()
    metrics = {"registered_images": len(cameras), "triangulated_points": points,
               "mean_reprojection_error_px": sum(errors) / len(errors) if errors else None,
               "median_reprojection_error_px": errors[len(errors)//2] if errors else None}
    return {"units": "unknown", "coordinate_frame": "COLMAP world frame; arbitrary origin and orientation",
            "camera_models": {str(k): v for k, v in camera_models.items()}, "cameras": cameras}, metrics


def _backend(log, requested):
    text = log.read_text(errors="replace")[-200000:].rsplit("\n$ ", 1)[-1]
    text = "\n".join(text.splitlines()[1:])
    # OpenMVS logs selected compute devices. Merely mentioning a compiled option
    # or the argv's 'gpu-device' is not evidence that GPU work actually ran.
    active = "cpu" if requested == "cpu" else "unconfirmed"
    for line in text.splitlines():
        if re.search(r"(?:using|selected|fallback|falling back|backend)[^\n]{0,100}cpu|cpu[^\n]{0,100}(?:using|selected|backend)", line, re.I):
            active = "cpu"
        elif re.search(r"using\s+metal\s+(?:compute\s+)?backend", line, re.I):
            active = "metal"
        elif re.search(r"(?:using|selected|device)[^\n]{0,100}cuda|cuda[^\n]{0,100}(?:using|selected|device)", line, re.I):
            active = "cuda"
    return active


def _camera_group(paths, plan):
    """Share intrinsics only when the capture evidence identifies one lens."""
    from PIL import Image
    signatures = []
    planned = {item.get("filename"): item for item in (plan or {}).get("images", [])}
    for path in paths:
        with Image.open(path) as image:
            exif = image.getexif()
            nested = exif.get_ifd(34665) if 34665 in exif else {}
            make, model = exif.get(271), exif.get(272)
            focal = nested.get(37386)
            settings = planned.get(path.name, {}).get("camera_settings") or {}
            if make and model and focal:
                signatures.append((image.size, str(make), str(model), str(nested.get(42036)), float(focal),
                                   nested.get(41989), nested.get(41988), settings.get("zoom")))
            elif settings.get("deviceId") and settings.get("width") and settings.get("height") and "zoom" in settings:
                signatures.append((image.size, settings["deviceId"], settings.get("zoom")))
            else:
                return False
    return bool(signatures and all(value == signatures[0] for value in signatures))


def _toolchain_provenance(manifest, engines):
    """Retain reproducible build facts, excluding timestamps and smoke outputs."""
    if not isinstance(manifest, dict):
        return {"managed": False}
    colmap = manifest.get("colmap", {})
    mvs = manifest.get("openmvs", {})
    expected = {"colmap": colmap.get("path"), **mvs.get("commands", {})}
    if not all(expected.get(name) and str(Path(expected[name]).resolve()) == engines[name] for name in _ENGINES):
        return {"managed": False}
    def subset(values, keys):
        return {key: values[key] for key in keys if key in values}
    return {"managed": True, "platform": manifest.get("platform"),
            "colmap": subset(colmap, ("version", "distribution", "archive_sha256")),
            "openmvs": subset(mvs, ("version", "revision", "metal_compiled", "cmake_patch_sha256")),
            "sources": {name: subset(value, ("url", "revision", "tag")) for name, value in manifest.get("sources", {}).items()},
            "homebrew_versions": sorted(manifest.get("homebrew_versions", []))}


def _database_valid(path, matched=False):
    if not _nonempty(path):
        return False
    try:
        with closing(sqlite3.connect(f"file:{path}?mode=ro", uri=True)) as db:
            if db.execute("SELECT COUNT(*) FROM images").fetchone()[0] < 3:
                return False
            if matched:
                return db.execute("SELECT COUNT(*) FROM two_view_geometries WHERE rows > 0").fetchone()[0] > 0
            return db.execute("SELECT COUNT(*) FROM keypoints WHERE rows > 0").fetchone()[0] >= 3
    except sqlite3.Error:
        return False


def _texture_assets(obj, root):
    """Check OBJ material dependencies before allowing a textured export."""
    if not _nonempty(obj):
        return []
    materials = [line.split(maxsplit=1)[1].strip() for line in obj.read_text(errors="replace").splitlines()
                 if line.startswith("mtllib ")]
    if not materials:
        return []
    assets = [obj]
    textures = 0
    for name in materials:
        mtl = (obj.parent / name).resolve()
        if not mtl.is_relative_to(obj.parent.resolve()) or not _nonempty(mtl):
            return []
        assets.append(mtl)
        for line in mtl.read_text(errors="replace").splitlines():
            if line.startswith("map_Kd "):
                image = (mtl.parent / line.split(maxsplit=1)[1].strip()).resolve()
                if not image.is_relative_to(obj.parent.resolve()) or not _nonempty(image):
                    return []
                from PIL import Image
                with Image.open(image) as texture:
                    texture.verify()
                assets.append(image)
                textures += 1
    return list(dict.fromkeys(assets)) if textures else []


def _bundle(workdir, assets, report, cancelled):
    target = workdir / "exports" / "scene.zip"
    temporary = target.with_suffix(".zip.tmp")
    files = [(path, "visual/" + path.relative_to(assets[0].parent).as_posix()) for path in assets]
    files.extend((workdir / "exports" / name, name) for name in ("surface.ply", "collision.obj", "cameras.json", "geometry.json"))
    with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
        for path, name in files:
            if cancelled.is_set():
                raise PipelineCancelled("Reconstruction cancelled", "export")
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.external_attr = 0o100644 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            with path.open("rb") as source, archive.open(info, "w", force_zip64=True) as destination:
                while data := source.read(1024 * 1024):
                    if cancelled.is_set():
                        raise PipelineCancelled("Reconstruction cancelled", "export")
                    destination.write(data)
        documents = {"manifest.json": json.dumps(report, indent=2, allow_nan=False),
                     "README.txt": "Unscaled reconstruction in the COLMAP world frame.\nvisual/ contains OBJ, MTL and all referenced textures.\ncollision.obj is an unvalidated candidate, not a physics-ready asset.\nNo hole filling, floor alignment or material-property estimation was performed.\n"}
        for name, content in documents.items():
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.external_attr = 0o100644 << 16
            archive.writestr(info, content)
    temporary.replace(target)
    return target


def run_pipeline(workdir: Path, config: dict, on_update, cancelled: threading.Event):
    workdir = Path(workdir).resolve()
    images = workdir / "images"
    inputs = sorted(path for path in images.iterdir() if path.is_file() and path.suffix.lower() in _EXTENSIONS) if images.is_dir() else []
    if len(inputs) < 3:
        raise PipelineError("At least three photographs from different viewpoints are required", "prepare")
    quality = config.get("quality", "draft")
    if quality not in ("draft", "full"):
        raise PipelineError("quality must be draft or full")
    requested_backend = config.get("backend", "metal")
    if requested_backend not in ("metal", "cpu", "cuda"):
        raise PipelineError("backend must be metal, cpu or cuda")
    profile_name = config.get("mvs_profile", "standard")
    if profile_name not in ("standard", "calibrated", "calibrated-uniform"):
        raise PipelineError("mvs_profile must be standard, calibrated or calibrated-uniform")
    threads = max(1, min(16, int(config.get("max_threads", 4))))
    timeout = max(1, min(86400, float(config.get("stage_timeout_seconds", 21600))))
    paths = config.get("engine_paths", {})
    default_bin = Path(__file__).resolve().parent.parent / ".reconstruction-tools" / "bin"
    engines = {name: str(Path(paths.get(name, default_bin / name)).expanduser().resolve()) for name in _ENGINES}
    for name, path in engines.items():
        if not Path(path).is_file() or not os.access(path, os.X_OK):
            raise PipelineError(f"Missing {name}; run the reconstruction toolchain setup")
    provenance = _toolchain_provenance(_read_json(default_bin.parent / "toolchain.json"), engines)
    logs = workdir / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    marker_dir = workdir / ".stage-markers"
    marker_dir.mkdir(exist_ok=True)
    if cancelled.is_set():
        raise PipelineCancelled("Reconstruction cancelled")
    helps = {}
    engine_versions = {}
    for command in ("feature_extractor", "exhaustive_matcher", "mapper", "model_converter", "image_undistorter"):
        helps[command] = _probe([engines["colmap"], command, "--help"], logs,
                                logs / f"help-colmap-{command}.log", cancelled, threads)
    for name in _ENGINES[1:]:
        helps[name] = _probe([engines[name], "--help"], logs, logs / f"help-{name}.log", cancelled, threads)
    mvs_profile = _resolve_mvs_profile(profile_name, helps)
    # Fingerprint executable bytes as well as CLI text; wrappers may stay stable
    # while the underlying build changes, so both are retained for diagnostics.
    for name in _ENGINES:
        text = helps.get(name, helps["feature_extractor"])
        text = "\n".join(line for line in text.splitlines() if not line.startswith("$ "))
        version = re.search(r"(?:COLMAP|OpenMVS)[^\d\n]{0,30}\d+\.\d+(?:\.\d+)?", text, re.I)
        engine_versions[name] = {"path": engines[name], "version": version.group(0).strip() if version else "see help log",
                                 "executable_sha256": _hash(Path(engines[name])),
                                 "options_sha256": hashlib.sha256(json.dumps(sorted(_options(text))).encode()).hexdigest()}
    input_hashes = {path.name: _hash(path) for path in inputs}
    fingerprint_data = {"version": 1, "images": input_hashes, "quality": quality, "backend": requested_backend,
                        "threads": threads, "engines": engine_versions, "mvs_profile": mvs_profile,
                        "implementation_sha256": [_hash(Path(__file__)), _hash(Path(geometry.__file__))],
                        "source_plan_sha256": _hash(workdir / "source-plan.json") if (workdir / "source-plan.json").is_file() else None,
                        "minimum_registered_fraction": config.get("minimum_registered_fraction", 0.8),
                        "minimum_points": config.get("minimum_points", 50)}
    fingerprint = hashlib.sha256(json.dumps(fingerprint_data, sort_keys=True).encode()).hexdigest()
    state_path = workdir / "pipeline-state.json"
    previous = _read_json(state_path)
    if previous and previous.get("fingerprint") != fingerprint:
        for folder in ("prepared", "sfm", "dense", "openmvs", "exports", ".stage-markers"):
            target = workdir / folder
            if target.is_symlink():
                raise PipelineError("A pipeline output directory must not be a symbolic link")
            if target.exists():
                shutil.rmtree(target)
        marker_dir.mkdir(exist_ok=True)
    _write_json(state_path, {"fingerprint": fingerprint, "inputs": input_hashes, "engines": engine_versions,
                             "quality": quality, "backend_requested": requested_backend, "toolchain": provenance,
                             "mvs_profile": mvs_profile})
    warnings = []
    prepared = workdir / "prepared" / "images"
    sfm = workdir / "sfm"
    database = sfm / "database.db"
    dense = workdir / "dense"
    mvs = workdir / "openmvs"
    for folder in (sfm, mvs):
        folder.mkdir(exist_ok=True)
    exports = workdir / "exports"
    exports.mkdir(exist_ok=True)
    sparse_preview = exports / "sparse.ply"
    dense_preview = exports / "dense.ply"
    metrics = {"input_images": len(inputs), "quality": quality, "threads": threads, "engines": engine_versions, "toolchain": provenance,
               "mvs_profile": profile_name, "mvs_profile_settings": mvs_profile,
               "metric_scale_known": False, "collision_validated": False, "stage_seconds": {}}
    artifacts = []
    active_backend = "unconfirmed"
    redo_downstream = False

    def emit(stage, status, **details):
        on_update({"stage": stage, "status": status, **details})

    def stage(name, execute, validate, clean=None):
        nonlocal redo_downstream
        if cancelled.is_set():
            raise PipelineCancelled("Reconstruction cancelled", name)
        marker = _read_json(marker_dir / f"{name}.json")
        valid = False
        if marker and marker.get("fingerprint") == fingerprint and not redo_downstream:
            try:
                valid = bool(validate())
            except Exception:
                valid = False
        if valid:
            metrics["stage_seconds"][name] = marker.get("seconds", 0)
            emit(name, "completed", message="Reused validated output", resumed=True)
            return
        redo_downstream = True
        (marker_dir / f"{name}.json").unlink(missing_ok=True)
        if clean:
            clean()
        emit(name, "running", progress=next(i for i, item in enumerate(STAGES) if item["id"] == name) / len(STAGES))
        started = time.monotonic()
        try:
            execute()
            if cancelled.is_set():
                raise PipelineCancelled("Reconstruction cancelled", name)
            if not validate():
                raise PipelineError(f"{name} did not produce valid output; inspect logs/{name}.log", name)
        except PipelineError as exc:
            if not exc.stage:
                exc.stage = name
            raise
        except Exception as exc:
            raise PipelineError(f"{name} failed: {exc}", name) from exc
        elapsed = time.monotonic() - started
        metrics["stage_seconds"][name] = elapsed
        _write_json(marker_dir / f"{name}.json", {"fingerprint": fingerprint, "seconds": elapsed})
        emit(name, "completed")

    def command(name, argv):
        return _run(argv, cwd=workdir, log=logs / f"{name}.log", cancel=cancelled, timeout=timeout, threads=threads)

    def colmap(subcommand, values):
        argv = [engines["colmap"], subcommand]
        for names, value, required in values:
            _flag(argv, helps[subcommand], names, value, required)
        return argv

    def openmvs(name, source, target, extras=()):
        argv = [engines[name]]
        help_text = helps[name]
        _flag(argv, help_text, ["--input-file"], source, True)
        _flag(argv, help_text, ["--output-file"], target, True)
        _flag(argv, help_text, ["--working-folder"], mvs)
        _flag(argv, help_text, ["--max-threads"], threads)
        _flag(argv, help_text, ["--archive-type"], 2)
        _flag(argv, help_text, ["--close-holes"], 0)
        for names, value, required in extras:
            _flag(argv, help_text, names, value, required)
        return argv

    def prepare():
        from PIL import Image, ImageOps
        prepared.mkdir(parents=True, exist_ok=True)
        for path in inputs:
            if cancelled.is_set():
                raise PipelineCancelled("Reconstruction cancelled", "prepare")
            target = prepared / path.name
            with Image.open(path) as original:
                original.load()
                if original.width < 64 or original.height < 64:
                    raise PipelineError(f"Image is too small: {path.name}")
                if original.getexif().get(274, 1) == 1:
                    shutil.copy2(path, target)
                else:
                    corrected = ImageOps.exif_transpose(original)
                    kwargs = {"exif": corrected.info.get("exif", b"")}
                    if path.suffix.lower() in (".jpg", ".jpeg"):
                        kwargs.update(quality=98, subsampling=0)
                        if corrected.mode not in ("L", "RGB"):
                            corrected = corrected.convert("RGB")
                    corrected.save(target, **kwargs)

    stage("prepare", prepare, lambda: all(_nonempty(prepared / path.name) for path in inputs))
    shared_camera = _camera_group([prepared / path.name for path in inputs], _read_json(workdir / "source-plan.json"))
    metrics["intrinsics_mode"] = "shared verified lens" if shared_camera else "COLMAP automatic camera grouping"
    if not shared_camera:
        warnings.append("A single shared lens could not be verified from capture metadata; COLMAP automatic camera grouping is used.")
    feature_argv = colmap("feature_extractor", [(["--database_path"], database, True), (["--image_path"], prepared, True),
        (["--ImageReader.single_camera"], int(shared_camera), False), (["--ImageReader.camera_model"], "SIMPLE_RADIAL", False),
        (["--FeatureExtraction.use_gpu", "--SiftExtraction.use_gpu"], 0, False),
        (["--FeatureExtraction.num_threads", "--SiftExtraction.num_threads"], threads, False),
        (["--FeatureExtraction.max_image_size", "--SiftExtraction.max_image_size"], 1600 if quality == "draft" else 2400, False),
        (["--SiftExtraction.max_num_features"], 8192 if quality == "draft" else 16384, False)])
    stage("features", lambda: command("features", feature_argv), lambda: _database_valid(database),
          clean=lambda: database.unlink(missing_ok=True))
    match_argv = colmap("exhaustive_matcher", [(["--database_path"], database, True),
        (["--FeatureMatching.use_gpu", "--SiftMatching.use_gpu"], 0, False),
        (["--FeatureMatching.num_threads", "--SiftMatching.num_threads"], threads, False)])
    stage("matching", lambda: command("matching", match_argv), lambda: _database_valid(database, matched=True))

    models = sfm / "models"
    selection_path = sfm / "selected-model.json"
    def align():
        if models.exists():
            shutil.rmtree(models)
        models.mkdir()
        command("alignment", colmap("mapper", [(["--database_path"], database, True),
            (["--image_path"], prepared, True), (["--output_path"], models, True),
            (["--Mapper.num_threads"], threads, False)]))
        results = []
        for model in sorted(models.iterdir()):
            if not model.is_dir() or not (_nonempty(model / "images.bin") or _nonempty(model / "images.txt")):
                continue
            text_dir = sfm / f"text-{model.name}"
            text_dir.mkdir(exist_ok=True)
            command("alignment", colmap("model_converter", [(["--input_path"], model, True),
                (["--output_path"], text_dir, True), (["--output_type"], "TXT", True)]))
            _, model_metrics = _read_model(text_dir)
            results.append({"model": model.relative_to(workdir).as_posix(), "text": text_dir.relative_to(workdir).as_posix(), **model_metrics})
        if not results:
            raise PipelineError("Camera alignment produced no connected reconstruction; inspect image overlap and alignment.log")
        results.sort(key=lambda item: (item["registered_images"], item["triangulated_points"]), reverse=True)
        _write_json(selection_path, {"selected": results[0], "components": results})
        command("alignment", colmap("model_converter", [(["--input_path"], workdir / results[0]["model"], True),
            (["--output_path"], sparse_preview, True), (["--output_type"], "PLY", True)]))

    def alignment_valid():
        selection = _read_json(selection_path)
        if not selection or not selection.get("selected"):
            return False
        selected = selection["selected"]
        _, actual = _read_model(workdir / selected["text"])
        fraction = actual["registered_images"] / len(inputs)
        if actual["registered_images"] < 3 or fraction < float(config.get("minimum_registered_fraction", 0.8)):
            raise PipelineError(f"Largest connected model registered only {actual['registered_images']}/{len(inputs)} images; refusing an incomplete reconstruction", "alignment")
        if actual["triangulated_points"] < int(config.get("minimum_points", 50)):
            raise PipelineError("Camera alignment has too few triangulated points for reliable reconstruction", "alignment")
        if actual["mean_reprojection_error_px"] is None or actual["mean_reprojection_error_px"] > 8:
            raise PipelineError("Camera alignment has excessive reprojection error", "alignment")
        return _nonempty(sparse_preview)

    stage("alignment", align, alignment_valid)
    selection = _read_json(selection_path)
    selected = selection["selected"]
    cameras, sfm_metrics = _read_model(workdir / selected["text"])
    metrics.update(sfm_metrics)
    metrics.update({"registered_fraction": sfm_metrics["registered_images"] / len(inputs), "connected_components": len(selection["components"])})
    _write_json(exports / "cameras.json", cameras)
    artifacts = [geometry.artifact(workdir, sparse_preview, "sparse_points", units="unknown"),
                 geometry.artifact(workdir, exports / "cameras.json", "cameras", units="unknown")]
    emit("alignment", "completed", metrics=sfm_metrics, artifacts=artifacts)
    if sfm_metrics["registered_images"] < len(inputs):
        warnings.append(f"The selected connected model contains {sfm_metrics['registered_images']}/{len(inputs)} images. Other images are not reconstructed.")
    if len(selection["components"]) > 1:
        warnings.append(f"Alignment produced {len(selection['components'])} disconnected components; only the largest is reconstructed.")
    if sfm_metrics["mean_reprojection_error_px"] > 2:
        warnings.append("Mean sparse reprojection error exceeds 2 pixels; inspect camera alignment before trusting fine geometry.")

    stage("undistort", lambda: command("undistort", colmap("image_undistorter", [
        (["--image_path"], prepared, True), (["--input_path"], workdir / selected["model"], True),
        (["--output_path"], dense, True), (["--output_type"], "COLMAP", True),
        (["--max_image_size"], 1600 if quality == "draft" else 3200, False),
        (["--num_threads"], threads, False)])),
        lambda: (dense / "images").is_dir() and any((dense / "images").iterdir()) and (dense / "sparse").is_dir())
    scene = mvs / "scene.mvs"
    scene_dense = mvs / "scene_dense.mvs"
    scene_mesh = mvs / "scene_dense_mesh.mvs"
    refined = mvs / "scene_dense_mesh_refine.mvs"
    textured = mvs / "scene_dense_mesh_refine_texture.mvs"
    textured_obj = textured.with_suffix(".obj")
    stage("import", lambda: command("import", openmvs("InterfaceCOLMAP", dense, scene,
        [(["--image-folder"], dense / "images", False)])), lambda: _nonempty(scene))
    gpu_value = -2 if requested_backend == "cpu" else -1
    density_extras = [(["--gpu-device", "--cuda-device"], gpu_value, requested_backend != "cpu"),
                      (["--resolution-level"], 1 if quality == "draft" else 0, False),
                      (["--max-resolution"], 1600 if quality == "draft" else 2400, False),
                      (["--min-resolution"], 640, False),
                      (["--number-views"], 5 if quality == "draft" else 8, False),
                      (["--number-views-fuse"], 3, False),
                      (["--patch-match-cuda-instances"], 1 if quality == "draft" else min(2, threads), False)]
    if profile_name != "standard":
        density_extras.append((["--postprocess-dmaps"], mvs_profile["postprocess_dmaps"], True))

    def dense_valid():
        if not (_nonempty(scene_dense) and _nonempty(scene_dense.with_suffix(".ply"))):
            return False
        if profile_name != "standard":
            _require_confidence_adjustment(logs / "dense.log", sfm_metrics["registered_images"])
        return True

    stage("dense", lambda: command("dense", openmvs("DensifyPointCloud", scene, scene_dense, density_extras)), dense_valid)
    active_backend = _backend(logs / "dense.log", requested_backend)
    shutil.copyfile(scene_dense.with_suffix(".ply"), dense_preview)
    artifacts.append(geometry.artifact(workdir, dense_preview, "dense_points", units="unknown"))
    metrics["backend_requested"] = requested_backend
    metrics["backend_active"] = active_backend
    if active_backend != requested_backend:
        warnings.append(f"Dense backend requested {requested_backend}, but logs confirm {active_backend}. Inspect dense.log for GPU fallback or availability.")
    emit("dense", "completed", backend={"requested": requested_backend, "active": active_backend}, artifacts=artifacts,
         metrics={"backend_requested": requested_backend, "backend_active": active_backend})
    mesh_extras = []
    if profile_name != "standard":
        # Recheck even when mesh output is resumable. Both calibrated profiles
        # must prove calibration; only the experimental weighted profile uses
        # those confidences as graph-cut visibility weights.
        metrics["confidence_adjusted_images"] = _require_confidence_adjustment(logs / "dense.log", sfm_metrics["registered_images"])
        mesh_extras.append((["--constant-weight"], mvs_profile["constant_weight"], True))
    stage("mesh", lambda: command("mesh", openmvs("ReconstructMesh", scene_dense, scene_mesh, mesh_extras)),
          lambda: _nonempty(scene_mesh) and _nonempty(scene_mesh.with_suffix(".ply")))
    stage("refine", lambda: command("refine", openmvs("RefineMesh", scene_mesh, refined,
        [(["--resolution-level"], 1 if quality == "draft" else 0, False),
         (["--scales"], 2 if quality == "draft" else 3, False), (["--reduce-memory"], 1, False)])),
          lambda: _nonempty(refined) and _nonempty(refined.with_suffix(".ply")))
    stage("texture", lambda: command("texture", openmvs("TextureMesh", refined, textured,
        [(["--export-type"], "obj", False), (["--resolution-level"], 0, False),
         (["--max-texture-size"], 4096 if quality == "draft" else 8192, False)])),
          lambda: bool(_texture_assets(textured_obj, workdir)))

    export_report = workdir / "exports" / "pipeline-result.json"
    def export():
        result = geometry.export_geometry(workdir, textured_obj, collision_faces=10000 if quality == "draft" else 50000)
        _write_json(workdir / "exports" / "cameras.json", cameras)
        result["artifacts"].extend(artifacts)
        source_assets = _texture_assets(textured_obj, workdir)
        for path in source_assets:
            result["artifacts"].append(geometry.artifact(workdir, path, "source_mesh" if path.suffix.lower() == ".obj" else "source_asset", units="unknown"))
        result["metrics"] = {**metrics, **result["metrics"]}
        result["warnings"] = warnings + result["warnings"]
        result["backend"] = {"requested": requested_backend, "active": active_backend}
        bundle = _bundle(workdir, source_assets, result, cancelled)
        result["artifacts"].append(geometry.artifact(workdir, bundle, "scene_bundle", units="unknown", validated=False))
        _write_json(export_report, result)

    def export_valid():
        result = _read_json(export_report)
        if not (result and result.get("artifacts") and all(_nonempty(workdir / item["path"]) for item in result["artifacts"])):
            return False
        try:
            for name in ("scene.glb", "surface.ply", "collision.obj"):
                geometry._load(exports / name)
            camera_export = _read_json(exports / "cameras.json")
            if not camera_export or len(camera_export.get("cameras", [])) != sfm_metrics["registered_images"]:
                return False
            for camera in camera_export["cameras"]:
                geometry._point(camera["center"], "camera center")
            with zipfile.ZipFile(exports / "scene.zip") as archive:
                if archive.testzip() is not None:
                    return False
        except (OSError, ValueError, geometry.GeometryError, zipfile.BadZipFile):
            return False
        return True

    stage("export", export, export_valid)
    result = _read_json(export_report)
    # Preserve time metrics for the export stage and resumed runs.
    result["metrics"]["stage_seconds"] = metrics["stage_seconds"]
    _write_json(export_report, result)
    result["artifacts"].append(geometry.artifact(workdir, export_report, "reconstruction_report"))
    emit("export", "completed", metrics=result["metrics"], artifacts=result["artifacts"], backend=result["backend"], progress=1.0)
    return result
