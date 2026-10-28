"""Native macOS photogrammetry jobs, independent of COLMAP/OpenMVS.

Original JPEG/EXIF bytes are preserved. All generated geometry and camera centres
retain Apple's native Y-up frame and unknown physical scale. Estimated camera K
is not exposed by this SDK; camera overlays use an explicit preview field of view.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import threading
import time
import zipfile

try:
    from . import geometry, pipeline
except ImportError:
    import geometry
    import pipeline

NATIVE_STAGES = [{"id": name, "label": label} for name, label in (
    ("prepare", "Prepare original images"), ("native", "Reconstruct scene"),
    ("export", "Export scene"))]
FRAME = "Apple RealityKit native Y-up frame; arbitrary origin, unverified physical scale"


def _json(path, value):
    pipeline._write_json(Path(path), value)


def _events(path):
    values = []
    if not path.is_file():
        return values
    for line in path.read_text(errors="replace").splitlines():
        if not line.startswith("{"):
            continue
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict) and "event" in value:
            values.append(value)
    return values


def probe_native(engine_path, workdir, cancelled=None, threads=1):
    """Explicit native capability probe; never compiles or starts reconstruction."""
    workdir = Path(workdir)
    cancelled = cancelled or threading.Event()
    engine = Path(engine_path).expanduser().resolve()
    if not engine.is_file() or not os.access(engine, os.X_OK):
        raise pipeline.PipelineError("Native helper is unavailable; run reconstruction/toolchain/setup_photogrammetry.py", "prepare")
    log = workdir / "logs" / "native-probe.log"
    offset = log.stat().st_size if log.exists() else 0
    pipeline._run([engine, "--probe"], cwd=workdir, log=log, cancel=cancelled, timeout=30, threads=threads)
    with log.open("r", errors="replace") as stream:
        stream.seek(offset)
        text = stream.read()
    for line in reversed(text.splitlines()):
        if line.startswith("{"):
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if value.get("event") == "capability":
                return value
    raise pipeline.PipelineError("Native capability probe returned no capability record; inspect native-probe.log", "prepare")


def _ordered_inputs(workdir):
    folder = workdir / "images"
    files = {p.name: p for p in folder.iterdir()
             if p.is_file() and p.suffix.lower() in pipeline._EXTENSIONS} if folder.is_dir() else {}
    plan = pipeline._read_json(workdir / "source-plan.json") or {}
    planned = [item.get("filename") for item in plan.get("images", [])]
    names = [name for name in planned if name in files]
    if len(set(names)) != len(names):
        raise pipeline.PipelineError("Capture plan contains duplicate image names", "prepare")
    names += sorted(set(files) - set(names))
    paths = [files[name] for name in names]
    if len(paths) < 3:
        raise pipeline.PipelineError("At least three photographs from different viewpoints are required", "prepare")
    if any(p.is_symlink() or not p.resolve().is_relative_to(folder.resolve()) for p in paths):
        raise pipeline.PipelineError("Native input snapshot must contain local regular image files", "prepare")
    return paths


def _prepare_inputs(workdir, paths, hashes, cancelled):
    from PIL import Image
    native = workdir / "native"
    target = native / "input"
    target.mkdir(parents=True, exist_ok=True)
    records = []
    for index, source in enumerate(paths, 1):
        if cancelled.is_set():
            raise pipeline.PipelineCancelled("Reconstruction cancelled", "prepare")
        name = f"{index:06d}{source.suffix.lower()}"
        destination = target / name
        if destination.is_symlink():
            raise pipeline.PipelineError("Native input cannot be a symbolic link", "prepare")
        if not destination.is_file() or pipeline._hash(destination) != hashes[source.name]:
            if destination.exists():
                destination.unlink()
            shutil.copy2(source, destination)
        if pipeline._hash(destination) != hashes[source.name]:
            raise pipeline.PipelineError(f"Original byte snapshot failed verification: {source.name}", "prepare")
        with Image.open(destination) as image:
            width, height = image.size
            orientation = int(image.getexif().get(274, 1))
        if orientation not in range(1, 9):
            orientation = 1
        records.append({"ordered_filename": name, "filename": source.name,
                        "sha256": hashes[source.name], "bytes": source.stat().st_size,
                        "width": width, "height": height, "exif_orientation": orientation})
    expected = {r["ordered_filename"] for r in records}
    for item in target.iterdir():
        if item.name not in expected:
            if item.is_symlink() or item.is_dir():
                raise pipeline.PipelineError("Unexpected entry in native input snapshot", "prepare")
            item.unlink()
    manifest = {"version": 1, "images": records, "image_count": len(records),
                "order": "capture plan order, then remaining filenames", "reencoded": False,
                "exif_preserved_byte_identical": True}
    _json(native / "ordered-inputs.json", manifest)
    return manifest


def _native_model(output, detail):
    folder = output / f"obj-{detail}"
    objects = list(folder.rglob("*.obj")) if folder.is_dir() else []
    if len(objects) != 1:
        raise pipeline.PipelineError("Native session did not produce one OBJ mesh", "native")
    source = objects[0]
    assets = pipeline._texture_assets(source, output)
    if not assets:
        raise pipeline.PipelineError("Native OBJ is missing valid material/texture dependencies", "native")
    return source, assets


def _axis_metadata(source):
    metadata = {"up_axis": None, "up_axis_verified": False, "creator": None,
                "meters_per_unit_metadata": None, "metric_scale_validated": False}
    for path in source.parent.glob("*.usda"):
        with path.open(errors="replace") as stream:
            header = stream.read(16384)
        up = re.search(r'upAxis\s*=\s*"([XYZ])"', header)
        creator = re.search(r'creator\s*=\s*"([^"]+)"', header)
        units = re.search(r'metersPerUnit\s*=\s*([0-9.eE+-]+)', header)
        metadata.update({"path": str(path), "up_axis": up.group(1) if up else None,
                         "up_axis_verified": bool(up), "creator": creator.group(1) if creator else None,
                         "meters_per_unit_metadata": float(units.group(1)) if units else None})
        break
    return metadata


def upright_camera_axes(orientation):
    """Map upright OpenCV axes into native RealityKit encoded-image camera axes.

    OpenCV is +Z forward/+Y down; native is -Z forward/+Y up. EXIF6's
    clockwise image rotation adds its 90 degree pixel-axis change. That mapping
    matches the19-photo native/SfM orientation audit to 0.30 degrees median.
    Mirrored EXIF images cannot be represented by a proper camera rotation;
    their centre is retained and their orientation preview is disabled.
    """
    import numpy as np
    raw_from_upright = {
        1: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        3: [[-1, 0, 0], [0, -1, 0], [0, 0, 1]],
        6: [[0, 1, 0], [-1, 0, 0], [0, 0, 1]],
        8: [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
    }
    if orientation not in raw_from_upright:
        return None
    return np.diag([1., -1., -1.]) @ np.asarray(raw_from_upright[orientation], dtype=float)


def export_cameras(poses_path, manifest, axis_verified=True):
    import numpy as np
    poses = pipeline._read_json(poses_path) or {}
    records = {item["ordered_filename"]: (index, item) for index, item in enumerate(manifest["images"], 1)}
    cameras = []
    seen = set()
    for pose in poses.get("poses", []):
        name = Path(pose.get("input_url", "")).name
        if name not in records or name in seen:
            raise pipeline.PipelineError("Native pose does not match the immutable ordered input", "native")
        seen.add(name)
        index, item = records[name]
        matrix = np.asarray(pose.get("native_transform_rows"), dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise pipeline.PipelineError("Native pose has an invalid transform", "native")
        Q, center = matrix[:3, :3], matrix[:3, 3]
        if not np.allclose(Q.T @ Q, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(Q), 1, atol=1e-4):
            raise pipeline.PipelineError("Native pose rotation is not rigid", "native")
        axes = upright_camera_axes(item["exif_orientation"])
        orientation_known = axes is not None and axis_verified
        camera = {"image_id": index, "image": item["filename"], "camera_id": index,
                  "native_sample_id": pose["sample_id"], "center": center.tolist(),
                  "native_transform_rows": matrix.tolist(), "exif_orientation": item["exif_orientation"],
                  "intrinsics_known": False, "preview_fov_degrees": 60,
                  "orientation_preview_verified": orientation_known,
                  "image_width": item["height"] if item["exif_orientation"] in (6, 8) else item["width"],
                  "image_height": item["width"] if item["exif_orientation"] in (6, 8) else item["height"]}
        if orientation_known:
            rotation = axes.T @ Q.T
            camera.update(rotation_world_to_camera=rotation.tolist(), translation=(-rotation @ center).tolist())
        cameras.append(camera)
    return {"units": "unknown", "coordinate_frame": FRAME, "up_axis": "Y" if axis_verified else None,
            "camera_models": {}, "intrinsics_known": False,
            "projection_note": "Native SDK does not expose estimated intrinsics; a 60 degree field of view is display-only",
            "camera_axis_convention": "OpenCV upright image axes from native pose and preserved EXIF orientation",
            "cameras": cameras}


def _watch(log, offset, stop, on_update):
    """Read complete JSON event lines; retain all raw native log text on disk."""
    position = offset
    pending = ""
    while not stop.wait(.5):
        try:
            if not log.exists():
                continue
            with log.open(errors="replace") as stream:
                stream.seek(position)
                pending += stream.read()
                position = stream.tell()
            lines = pending.split("\n")
            pending = lines.pop()
            for line in lines:
                if not line.startswith("{"):
                    continue
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if event.get("event") == "requestProgress":
                    fraction = max(0., min(1., float(event.get("fraction", 0))))
                    on_update({"stage": "native", "status": "running", "progress": .15 + .65 * fraction})
                elif event.get("event") == "requestProgressInfo" and event.get("stage"):
                    on_update({"stage": "native", "status": "running", "message": event["stage"],
                               "native_stage": event["stage"]})
        except Exception:
            # Progress is advisory; command failure/cancellation is handled by_run.
            continue


def _bundle(workdir, assets, report, cancelled):
    target = workdir / "exports" / "scene.zip"
    temporary = target.with_suffix(".zip.tmp")
    items = [(path, "visual/" + path.relative_to(assets[0].parent).as_posix()) for path in assets]
    for name in ("scene.glb", "surface.ply", "collision.obj", "cameras.json", "geometry.json", "native-provenance.json", "native-points.ply"):
        path = workdir / "exports" / name
        if path.is_file():
            items.append((path, name))
    items.extend((workdir / "native" / "output" / name, "native/" + name)
                 for name in ("poses-native.json", "bounds.json", "result.json")
                 if (workdir / "native" / "output" / name).is_file())
    with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
        for path, name in items:
            if cancelled.is_set():
                raise pipeline.PipelineCancelled("Reconstruction cancelled", "export")
            with path.open("rb") as source, archive.open(name, "w", force_zip64=True) as output:
                while block := source.read(1024 * 1024):
                    if cancelled.is_set():
                        raise pipeline.PipelineCancelled("Reconstruction cancelled", "export")
                    output.write(block)
        archive.writestr("manifest.json", json.dumps(report, indent=2, allow_nan=False))
        archive.writestr("README.txt", "Native Apple photogrammetry in its Y-up world frame. Physical scale is unverified.\n"
            "visual/ contains OBJ, MTL and referenced baked photographs.\nCamera centres/orientations are native; estimated intrinsics are unavailable.\n"
            "collision.obj is a decimated, unvalidated candidate. No floor alignment, hole closure or physical reflectance calibration was added by export.\n")
    temporary.replace(target)
    return target


def _export_worker(workdir, source_obj, collision_faces):
    result = geometry.export_geometry(workdir, source_obj, collision_faces=collision_faces, coordinate_frame=FRAME)
    result["metrics"].update(up_axis="Y", display_up_vector=[0, 1, 0], floor_aligned=False,
                             metric_scale_known=False, collision_validated=False)
    geometry_report = workdir / "exports" / "geometry.json"
    _json(geometry_report, result["metrics"])
    # export_geometry recorded its report before native frame fields were added.
    # Refresh that artifact after the final write so manifests retain exact sizes.
    for item in result["artifacts"]:
        if item["path"] == "exports/geometry.json":
            item["bytes"] = geometry_report.stat().st_size
    _json(workdir / "exports" / "native-export-result.json", result)


def run_native(workdir: Path, config: dict, on_update, cancelled: threading.Event):
    workdir = Path(workdir).resolve()
    quality = config.get("quality", "full")
    if quality not in ("draft", "full"):
        raise pipeline.PipelineError("quality must be draft or full", "prepare")
    detail = "raw" if quality == "full" else "full"
    ordering = config.get("native_ordering", "unordered")
    sensitivity = config.get("native_sensitivity", "normal")
    if ordering not in ("unordered", "sequential") or sensitivity not in ("normal", "high"):
        raise pipeline.PipelineError("Invalid native quality profile", "prepare")
    threads = max(1, min(16, int(config.get("max_threads", 4))))
    timeout = max(1, min(86400., float(config.get("stage_timeout_seconds", 21600))))
    default = Path(__file__).resolve().parent.parent / ".reconstruction-tools" / "bin" / "PhotogrammetryReconstruct"
    engine = Path(config.get("native_engine_path") or config.get("engine_paths", {}).get("PhotogrammetryReconstruct") or default).expanduser().resolve()
    if not engine.is_file() or not os.access(engine, os.X_OK):
        raise pipeline.PipelineError("Missing PhotogrammetryReconstruct; run reconstruction/toolchain/setup_photogrammetry.py", "prepare")
    logs = workdir / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    native = workdir / "native"
    native.mkdir(exist_ok=True)
    if native.is_symlink():
        raise pipeline.PipelineError("Native output directory must not be a symbolic link", "prepare")
    stage_seconds = {}
    def emit(stage, status, **extra):
        on_update({"stage": stage, "status": status, **extra})
    emit("prepare", "running", progress=0)
    started = time.monotonic()
    paths = _ordered_inputs(workdir)
    hashes = {p.name: pipeline._hash(p) for p in paths}
    capability = probe_native(engine, workdir, cancelled, threads)
    if not capability.get("supported"):
        raise pipeline.PipelineError("Apple Object Capture is unavailable in this process; native GPU access and a supported Mac are required", "prepare")
    if capability.get("quality_profile_arguments") is not True:
        raise pipeline.PipelineError("Native helper predates quality profile arguments; rebuild with reconstruction/toolchain/setup_photogrammetry.py", "prepare")
    if len(paths) > int(capability.get("maximumNumberOfInputImages", 1000)):
        raise pipeline.PipelineError("Image set exceeds native session input limit", "prepare")
    fingerprint_data = {"images": hashes, "ordered_names": [p.name for p in paths], "detail": detail,
                        "helper_sha256": pipeline._hash(engine), "os_release": os.uname().release,
                        "configuration": {"featureSensitivity": sensitivity, "sampleOrdering": ordering, "isObjectMaskingEnabled": False,
                                          "ignoreBoundingBox_applied": False}}
    fingerprint = hashlib.sha256(json.dumps(fingerprint_data, sort_keys=True).encode()).hexdigest()
    previous = pipeline._read_json(workdir / "native-state.json") or {}
    if previous.get("fingerprint") not in (None, fingerprint):
        for child in (native / "input", native / "output"):
            if child.is_symlink():
                raise pipeline.PipelineError("Native output cannot be a symbolic link", "prepare")
            if child.exists():
                shutil.rmtree(child)
    manifest = _prepare_inputs(workdir, paths, hashes, cancelled)
    if any(max(item["width"], item["height"]) > int(capability.get("maximumInputImageDimension", 16384)) for item in manifest["images"]):
        raise pipeline.PipelineError("An original image exceeds native maximum input dimensions", "prepare")
    stage_seconds["prepare"] = time.monotonic() - started
    _json(workdir / "native-state.json", {"fingerprint": fingerprint, "configuration": fingerprint_data, "capability": capability})
    emit("prepare", "completed", progress=.15)
    output = native / "output"
    summary = pipeline._read_json(output / "result.json") or {}
    reusable = summary.get("status") == "complete" and previous.get("fingerprint") == fingerprint
    if reusable:
        try:
            _native_model(output, detail)
            reusable = bool((pipeline._read_json(output / "poses-native.json") or {}).get("poses"))
        except Exception:
            reusable = False
    if not reusable:
        if output.is_symlink():
            raise pipeline.PipelineError("Native output cannot be a symbolic link", "native")
        if output.exists():
            shutil.rmtree(output)
        emit("native", "running", progress=.15)
        log = logs / "native.log"
        offset = log.stat().st_size if log.exists() else 0
        stop = threading.Event()
        watcher = threading.Thread(target=_watch, args=(log, offset, stop, on_update), daemon=True)
        watcher.start()
        try:
            stage_seconds["native"] = pipeline._run([engine, "--allow-process", "--run-id", workdir.name,
                "--input", native / "input", "--output", output, "--detail", detail, "--formats", "obj",
                "--ordering", ordering, "--sensitivity", sensitivity],
                cwd=workdir, log=log, cancel=cancelled, timeout=timeout, threads=threads)
        except pipeline.PipelineError as exc:
            if cancelled.is_set():
                raise pipeline.PipelineCancelled("Reconstruction cancelled", "native")
            if not exc.stage:
                exc.stage = "native"
            raise
        finally:
            stop.set()
            watcher.join(timeout=2)
        summary = pipeline._read_json(output / "result.json") or {}
    else:
        stage_seconds["native"] = float(summary.get("elapsed_seconds", 0))
    if cancelled.is_set():
        raise pipeline.PipelineCancelled("Reconstruction cancelled", "native")
    if summary.get("status") != "complete":
        raise pipeline.PipelineError("Native session did not complete successfully; inspect logs/native.log", "native")
    source_obj, assets = _native_model(output, detail)
    axis = _axis_metadata(source_obj)
    if axis.get("up_axis") != "Y":
        raise pipeline.PipelineError("Native mesh lacks verified Y-up metadata; refusing an assumed camera/mesh frame", "native")
    cameras = export_cameras(output / "poses-native.json", manifest, axis_verified=True)
    registered = len(cameras["cameras"])
    if registered < 3 or registered / len(paths) < float(config.get("minimum_registered_fraction", .8)):
        raise pipeline.PipelineError(f"Native alignment contains only {registered}/{len(paths)} input images", "native")
    warnings = []
    if registered < len(paths):
        warnings.append(f"Native alignment includes {registered}/{len(paths)} images; inspect omitted views.")
    if summary.get("automatic_downsampling"):
        warnings.append("The native framework automatically downsampled images because of resource constraints.")
    if summary.get("skipped_sample_ids") or summary.get("invalid_samples"):
        warnings.append("Native session skipped or rejected input samples; exact records are in native provenance.")
    if summary.get("stitching_incomplete"):
        warnings.append("Native framework reported incomplete stitching; inspect missing or warped surfaces.")
    warnings.append("Native camera intrinsics are unavailable; camera field of view is display-only. Scene completeness and dimensions require validation.")
    for item in manifest["images"]:
        if pipeline._hash(native / "input" / item["ordered_filename"]) != item["sha256"] or pipeline._hash(workdir / "images" / item["filename"]) != item["sha256"]:
            raise pipeline.PipelineError("Immutable input image bytes changed during native processing", "native")
    exports = workdir / "exports"
    exports.mkdir(exist_ok=True)
    _json(exports / "cameras.json", cameras)
    pointcloud = output / "pointcloud-native.ply"
    if pipeline._nonempty(pointcloud):
        shutil.copy2(pointcloud, exports / "native-points.ply")
    provenance = {"engine": "Apple PhotogrammetrySession", "backend": "apple", "detail": detail,
                  "quality": quality, "originals_preserved": True, "inputs": manifest,
                  "helper_path": str(engine), "helper_sha256": pipeline._hash(engine), "capability": capability,
                  "native_configuration": pipeline._read_json(output / "session-config.json"),
                  "native_result": summary, "axis_metadata": axis, "coordinate_frame": FRAME,
                  "native_scheduler_thread_limit": "environment only; framework exposes no thread-count control",
                  "metric_scale_known": False, "floor_aligned": False, "collision_validated": False,
                  "implementation_sha256": pipeline._hash(Path(__file__)),
                  "helper_source_sha256": pipeline._hash(Path(__file__).parent / "toolchain" / "photogrammetry.swift"),
                  "geometry_sha256": pipeline._hash(Path(geometry.__file__))}
    _json(exports / "native-provenance.json", provenance)
    emit("native", "completed", progress=.8, backend={"requested": "apple", "active": "apple"},
         metrics={"registered_images": registered, "registered_fraction": registered / len(paths)})
    emit("export", "running", progress=.8)
    stage_seconds["export"] = pipeline._run([sys.executable, Path(__file__), "--export", workdir, source_obj,
        str(10000 if quality == "draft" else 50000)], cwd=workdir, log=logs / "export.log",
        cancel=cancelled, timeout=timeout, threads=threads)
    result = pipeline._read_json(exports / "native-export-result.json")
    if not result:
        raise pipeline.PipelineError("Native mesh export did not produce a result", "export")
    result["metrics"].update(input_images=len(paths), registered_images=registered,
        registered_fraction=registered / len(paths), quality=quality, native_detail=detail,
        backend_requested="apple", backend_active="apple", threads=threads, stage_seconds=stage_seconds,
        native_automatic_downsampling=bool(summary.get("automatic_downsampling")), native_creator=axis.get("creator"),
        native_ordering=ordering, native_sensitivity=sensitivity)
    result["warnings"] = warnings + result.get("warnings", [])
    result["backend"] = {"requested": "apple", "active": "apple"}
    for path, kind in ((exports / "cameras.json", "cameras"), (exports / "native-provenance.json", "native_provenance"),
                       (exports / "native-points.ply", "native_points")):
        if pipeline._nonempty(path):
            result["artifacts"].append(geometry.artifact(workdir, path, kind, units="unknown"))
    for path in assets:
        result["artifacts"].append(geometry.artifact(workdir, path, "source_mesh" if path.suffix.lower() == ".obj" else "source_asset", units="unknown"))
    bundle = _bundle(workdir, assets, result, cancelled)
    result["artifacts"].append(geometry.artifact(workdir, bundle, "scene_bundle", units="unknown", validated=False))
    report = exports / "pipeline-result.json"
    # Keep the persisted report non-self-referential. Its final file artifact is
    # added only to the returned result for JobStore; writing it back would make
    # its own byte count recursively stale.
    _json(report, result)
    result["artifacts"].append(geometry.artifact(workdir, report, "reconstruction_report"))
    emit("export", "completed", metrics=result["metrics"], artifacts=result["artifacts"], backend=result["backend"], progress=1.)
    return result


if __name__ == "__main__":
    if len(sys.argv) == 5 and sys.argv[1] == "--export":
        job, obj = Path(sys.argv[2]).resolve(), Path(sys.argv[3]).resolve()
        if not obj.is_relative_to(job):
            raise SystemExit("Source OBJ must remain inside the native job")
        _export_worker(job, obj, int(sys.argv[4]))
    else:
        raise SystemExit("Use run_native() from the reconstruction worker")
