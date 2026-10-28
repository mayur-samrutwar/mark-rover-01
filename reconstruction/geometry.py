"""Mesh exports and explicit scale calibration for the reconstruction worker.

All uncalibrated exports retain their reconstruction engine's world frame.  A collision
export is a candidate surface, never a claim of physical validation.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import struct
from pathlib import Path


class GeometryError(RuntimeError):
    pass


def _dependencies():
    try:
        import numpy as np
        import trimesh
    except ImportError as exc:
        raise GeometryError("Install reconstruction/requirements.txt in the reconstruction environment") from exc
    return np, trimesh


def _atomic(path: Path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    if isinstance(content, str):
        content = content.encode("utf-8")
    with tmp.open("wb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    tmp.replace(path)


def _json(path: Path, value):
    _atomic(path, json.dumps(value, indent=2, allow_nan=False) + "\n")


def artifact(workdir: Path, path: Path, kind: str, **extra):
    path = path.resolve()
    root = workdir.resolve()
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise GeometryError("Artifact is outside its job directory") from exc
    if not path.is_file() or path.stat().st_size == 0:
        raise GeometryError(f"Missing or empty artifact: {relative}")
    return {"name": path.name, "path": relative.as_posix(), "kind": kind,
            "bytes": path.stat().st_size, **extra}


def _load(path: Path):
    np, trimesh = _dependencies()
    try:
        scene = trimesh.load(str(path), force="scene", process=False)
    except Exception as exc:
        raise GeometryError(f"Cannot read mesh {path.name}: {exc}") from exc
    meshes = [mesh for mesh in scene.dump() if isinstance(mesh, trimesh.Trimesh) and len(mesh.faces)]
    if not meshes:
        raise GeometryError(f"Mesh {path.name} has no triangles")
    combined = trimesh.util.concatenate(meshes)
    if not np.isfinite(combined.vertices).all() or not len(combined.faces):
        raise GeometryError(f"Mesh {path.name} contains invalid coordinates")
    if (combined.faces < 0).any() or (combined.faces >= len(combined.vertices)).any():
        raise GeometryError(f"Mesh {path.name} contains invalid triangle indices")
    if float(np.max(combined.extents)) <= 1e-12:
        raise GeometryError(f"Mesh {path.name} is degenerate")
    return scene, combined


def _stats(mesh):
    np, _ = _dependencies()
    return {"vertices": int(len(mesh.vertices)), "triangles": int(len(mesh.faces)),
            "bounds": np.asarray(mesh.bounds).tolist(), "extent": np.asarray(mesh.extents).tolist(),
            "watertight": bool(mesh.is_watertight), "units": "unknown"}



def surface_topology(mesh):
    """Describe connectivity without changing the exported geometry.

    Weld coincident positions for this diagnostic only: OBJ/GLB texture seams
    otherwise count as cracks. Connectivity is not a room-coverage estimate;
    a connected mesh can still be wrong or incomplete.
    """
    np, _ = _dependencies()
    span = max(float(np.max(mesh.extents)), 1e-12)
    _, vertex_ids = np.unique(np.round((mesh.vertices - mesh.bounds[0]) / span, 9),
                              axis=0, return_inverse=True)
    faces = vertex_ids[np.asarray(mesh.faces)]
    edges = np.sort(faces[:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2), axis=1)
    keys = edges[:, 0].astype(np.uint64) * (int(vertex_ids.max()) + 1) + edges[:, 1]
    order = np.argsort(keys)
    ordered_keys = keys[order]
    adjacent = np.flatnonzero(ordered_keys[1:] == ordered_keys[:-1])
    owners = order // 3
    parents = list(range(len(faces)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for offset in adjacent:
        a, b = find(int(owners[offset])), find(int(owners[offset + 1]))
        if a != b:
            parents[max(a, b)] = min(a, b)
    labels = np.array([find(index) for index in range(len(faces))])
    _, compact = np.unique(labels, return_inverse=True)
    areas = np.bincount(compact, weights=np.asarray(mesh.area_faces))
    fractions = np.sort(areas / max(float(areas.sum()), 1e-30))[::-1]
    _, edge_counts = np.unique(keys, return_counts=True)
    count = len(areas)
    largest = float(fractions[0]) if len(fractions) else 0.0
    return {"components": count, "largest_component_area_fraction": largest,
            "largest_component_area_fractions": fractions[:10].tolist(),
            "boundary_edges": int(np.count_nonzero(edge_counts == 1)),
            "nonmanifold_edges": int(np.count_nonzero(edge_counts > 2)),
            "assessment": "fragmented" if count >= 20 and largest < 0.75 else "unverified",
            "room_coverage": "not measured",
            "position_weld_relative_tolerance": 1e-9}


def baked_colour_glb(payload):
    """Use a diffuse display material for baked photographic RGB.

    glTF defaults omitted metallicFactor to one. OBJ photogrammetry materials
    generally do not supply that property, so a direct export otherwise makes
    room interiors dark and reflective. This modifies only JSON material
    declarations; the geometry and embedded image BIN chunks stay identical.
    These are display assumptions, not measured physical material properties.
    """
    magic, version, length = struct.unpack_from("<4sII", payload)
    if magic != b"glTF" or version != 2 or length != len(payload):
        raise GeometryError("Invalid generated GLB header")
    size, kind = struct.unpack_from("<II", payload, 12)
    if kind != 0x4E4F534A:
        raise GeometryError("Generated GLB has no JSON chunk")
    document = json.loads(payload[20:20 + size])
    materials = document.setdefault("materials", [])
    default = None
    for mesh in document.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            if "material" not in primitive:
                if default is None:
                    default = len(materials)
                    materials.append({})
                primitive["material"] = default
    for material in materials:
        pbr = material.setdefault("pbrMetallicRoughness", {})
        pbr.update(metallicFactor=0.0, roughnessFactor=1.0)
        if "baseColorTexture" in pbr:
            pbr["baseColorFactor"] = [1.0, 1.0, 1.0, 1.0]
    encoded = json.dumps(document, separators=(",", ":"), allow_nan=False).encode()
    encoded += b" " * (-len(encoded) % 4)
    remainder = payload[20 + size:]
    return (struct.pack("<4sII", b"glTF", 2, 20 + len(encoded) + len(remainder))
            + struct.pack("<II", len(encoded), kind) + encoded + remainder)


def export_geometry(workdir: Path, source_obj: Path, collision_faces: int = 50000, *,
                    coordinate_frame="COLMAP world frame; arbitrary origin and orientation"):
    """Export the visual scene and a conservatively decimated collision candidate."""
    workdir = Path(workdir).resolve()
    scene, mesh = _load(source_obj)
    exports = workdir / "exports"
    warnings = []
    visual = exports / "scene.glb"
    surface = exports / "surface.ply"
    collision = exports / "collision.obj"
    _atomic(visual, baked_colour_glb(scene.export(file_type="glb")))
    _atomic(surface, mesh.export(file_type="ply"))
    target = max(100, min(200000, int(collision_faces)))
    candidate = mesh.copy()
    if len(candidate.faces) > target:
        try:
            candidate = candidate.simplify_quadric_decimation(face_count=target)
        except Exception as exc:
            warnings.append(f"Collision simplification was unavailable; original surface retained ({type(exc).__name__}).")
    # Decimation only: no hole filling, convex hull, or component deletion.
    if not len(candidate.faces) or not _dependencies()[0].isfinite(candidate.vertices).all():
        raise GeometryError("Collision simplification produced an invalid mesh")
    _atomic(collision, candidate.export(file_type="obj", include_texture=False))
    metrics = {"visual_mesh": _stats(mesh), "collision_mesh": _stats(candidate),
               "surface_topology": surface_topology(mesh),
               "collision_validated": False, "metric_scale_known": False,
               "coordinate_frame": coordinate_frame,
               "display_material": "Baked RGB, diffuse nonmetallic preview; physical reflectance is not estimated"}
    topology = metrics["surface_topology"]
    if topology["assessment"] == "fragmented":
        warnings.append(f"Fragmented surface: {topology['components']:,} connected pieces; "
                        f"the largest contains {topology['largest_component_area_fraction']:.0%} of mesh area. "
                        "Processing finished, but this is not a validated room reconstruction.")
    report = exports / "geometry.json"
    _json(report, metrics)
    artifacts = [artifact(workdir, visual, "visual_mesh", units="unknown"),
                 artifact(workdir, surface, "surface_mesh", units="unknown"),
                 artifact(workdir, collision, "collision_candidate", units="unknown", validated=False),
                 artifact(workdir, report, "geometry_report")]
    warnings.append("Raw exports have no metric scale or floor alignment. Collision geometry is an unvalidated candidate.")
    return {"metrics": metrics, "artifacts": artifacts, "warnings": warnings}


def _point(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise GeometryError(f"{name} must contain three coordinates")
    if any(isinstance(v, bool) for v in value):
        raise GeometryError(f"{name} contains an invalid coordinate")
    try:
        point = [float(v) for v in value]
    except (TypeError, ValueError, OverflowError) as exc:
        raise GeometryError(f"{name} contains an invalid coordinate") from exc
    if not all(math.isfinite(v) for v in point):
        raise GeometryError(f"{name} must contain finite coordinates")
    return point


def apply_scale(workdir: Path, point_a, point_b, distance_m, *, source_space="raw"):
    """Apply one measured uniform scale to fresh copies of every exported mesh.

    Points normally refer to the raw GLB.  For a subsequent calibration from the
    scaled viewer, source_space='scaled' maps the selected points back to raw
    coordinates before calculating a new absolute scale. Camera centers use the
    same transform; their rotation matrices are unchanged.
    """
    np, _ = _dependencies()
    workdir = Path(workdir).resolve()
    a, b = _point(point_a, "point_a"), _point(point_b, "point_b")
    if isinstance(distance_m, bool):
        raise GeometryError("Measured distance must be a positive number in metres")
    try:
        distance = float(distance_m)
    except (TypeError, ValueError, OverflowError) as exc:
        raise GeometryError("Measured distance must be a positive number in metres") from exc
    if not math.isfinite(distance) or not 0 < distance <= 10000:
        raise GeometryError("Measured distance must be positive and no more than 10,000 metres")
    if source_space not in ("raw", "scaled"):
        raise GeometryError("source_space must be raw or scaled")
    exports = workdir / "exports"
    if source_space == "scaled":
        try:
            previous = json.loads((exports / "scaled" / "calibration.json").read_text())["meters_per_unit"]
            previous = float(previous)
            if not math.isfinite(previous) or previous <= 0:
                raise ValueError("invalid previous scale")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise GeometryError("No valid previous scale exists") from exc
        a, b = [v / previous for v in a], [v / previous for v in b]
    separation = math.dist(a, b)
    if not math.isfinite(separation) or separation <= 1e-10:
        raise GeometryError("Select two distinct points on the reconstruction")
    factor = distance / separation
    if not math.isfinite(factor) or not 1e-8 <= factor <= 1e8:
        raise GeometryError("Scale is outside the supported range; check the selected points and distance")
    scaled = exports / "scaled"
    # Read and validate everything before replacing any previous calibration.
    raw = {name: _load(exports / name) for name in ("scene.glb", "surface.ply", "collision.obj")}
    cameras_path = exports / "cameras.json"
    try:
        cameras = json.loads(cameras_path.read_text())
        for camera in cameras["cameras"]:
            camera["center"] = [v * factor for v in _point(camera["center"], "camera center")]
            if "translation" in camera:
                camera["translation"] = [v * factor for v in _point(camera["translation"], "camera translation")]
            if "native_transform_rows" in camera:
                matrix = np.array(camera["native_transform_rows"], dtype=float, copy=True)
                if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7):
                    raise ValueError("Invalid native camera transform")
                matrix[:3, 3] *= factor
                camera["native_transform_rows"] = matrix.tolist()
        cameras["units"] = "metres"
        cameras["meters_per_unit"] = factor
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise GeometryError("Camera geometry is missing or invalid") from exc
    transform = np.eye(4)
    transform[:3, :3] *= factor
    pending = []
    for name, (scene, mesh) in raw.items():
        if name.endswith(".glb"):
            scene.apply_transform(transform)
            data = scene.export(file_type="glb")
        else:
            mesh.apply_transform(transform)
            data = mesh.export(file_type=Path(name).suffix[1:], include_texture=False) if name.endswith(".obj") else mesh.export(file_type="ply")
        pending.append((scaled / name, data))
    for target, data in pending:
        _atomic(target, data)
    _json(scaled / "cameras.json", cameras)
    calibration = {"version": 1, "method": "one measured distance", "point_a_raw": a, "point_b_raw": b,
                   "raw_distance": separation, "measured_distance_m": distance, "meters_per_unit": factor,
                   "transform": transform.tolist(), "units": "metres", "collision_validated": False,
                   "floor_aligned": False, "source_sha256": hashlib.sha256((exports / "scene.glb").read_bytes()).hexdigest()}
    _json(scaled / "calibration.json", calibration)
    artifacts = [artifact(workdir, scaled / name, kind, units="metres", **({"validated": False} if "collision" in name else {}))
                 for name, kind in (("scene.glb", "visual_mesh"), ("surface.ply", "surface_mesh"),
                                    ("collision.obj", "collision_candidate"), ("cameras.json", "cameras"),
                                    ("calibration.json", "calibration"))]
    metrics = _stats(raw["surface.ply"][1])
    metrics.update({"units": "metres", "meters_per_unit": factor, "metric_scale_known": True,
                    "collision_validated": False, "floor_aligned": False})
    return {"calibration": calibration, "metrics": metrics, "artifacts": artifacts,
            "warnings": ["Scale is calibrated from one measurement; validate other dimensions independently. Floor orientation and collision geometry still require validation."]}
