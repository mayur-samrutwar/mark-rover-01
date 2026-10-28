"""Conservative horizontal boundary caps, exported separately as inferred surfaces.

This is mesh completion, not recovered photographic evidence or validated physics.
Original geometry/texture buffers are retained verbatim in the completed GLB.
"""
from __future__ import annotations
import copy
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import numpy as np
import trimesh
from PIL import Image
try:
    from . import geometry
except ImportError:
    import geometry

METHOD = 'horizontal-boundary-caps-v1'
RULES = {'maximum_tilt_degrees': 15, 'plane_rms_relative_loop_diameter': .012,
         'plane_max_relative_loop_diameter': .04, 'maximum_loop_vertices': 512,
         'maximum_existing_projected_area_fraction': .03}

class CompletionError(RuntimeError):
    pass

def _sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''): h.update(block)
    return h.hexdigest()

def _safe(root, relative):
    path = root / relative
    if not path.resolve().is_relative_to(root): raise CompletionError('Completion path leaves the reconstruction directory')
    current = path
    while current != root:
        if current.is_symlink(): raise CompletionError('Completion paths must not contain symbolic links')
        current = current.parent
    return path

def _up(document):
    axes = {'X': [1., 0., 0.], 'Y': [0., 1., 0.], 'Z': [0., 0., 1.]}
    declared = document.get('up_axis')
    if isinstance(declared, str) and declared in axes: return np.array(axes[declared]), 'declared-file-axis'
    poses = []
    for c in document.get('cameras', []):
        try:
            r = np.asarray(c['rotation_world_to_camera'], dtype=float)
            if r.shape == (3, 3) and np.isfinite(r).all() and np.allclose(r @ r.T, np.eye(3), atol=.02) and np.isclose(np.linalg.det(r), 1, atol=.03): poses.append(-r[1])
        except (KeyError, ValueError, TypeError): continue
    if len(poses) >= 3:
        poses = np.asarray(poses); seed = poses[np.argmax((poses @ poses.T).sum(1))]
        inliers = poses[poses @ seed > math.cos(math.pi / 4)]
        if len(inliers) >= 3 and len(inliers) / len(poses) >= .7:
            mean = inliers.mean(0)
            if np.linalg.norm(mean) > .75: return mean / np.linalg.norm(mean), 'camera-image-up-estimate'
    raise CompletionError('A declared up axis or consistent saved camera orientations are required for horizontal gap completion')

def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

def _simple(p, eps):
    def on(a, b, c): return abs(_cross(a, b, c)) <= eps and np.all(c >= np.minimum(a, b) - math.sqrt(eps)) and np.all(c <= np.maximum(a, b) + math.sqrt(eps))
    for i in range(len(p)):
        a, b = p[i], p[(i + 1) % len(p)]
        for j in range(i + 2, len(p)):
            if (j + 1) % len(p) == i: continue
            c, d = p[j], p[(j + 1) % len(p)]
            x, y, z, w = _cross(a, b, c), _cross(a, b, d), _cross(c, d, a), _cross(c, d, b)
            if (x * y < -eps * eps and z * w < -eps * eps) or on(a, b, c) or on(a, b, d) or on(c, d, a) or on(c, d, b): return False
    return True

def _clip(p):
    # Ear clipping preserves concavity and every boundary vertex; no convex hull.
    eps = max(float(np.ptp(p, axis=0).max()) ** 2 * 1e-12, 1e-24)
    signed = float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1)) / 2)
    if abs(signed) <= eps or not _simple(p, eps): raise CompletionError('Degenerate or self-intersecting projected boundary')
    sign = 1 if signed > 0 else -1
    ring = list(range(len(p))); faces = []
    while len(ring) > 3:
        for j, b in enumerate(ring):
            a, c = ring[j - 1], ring[(j + 1) % len(ring)]
            if sign * _cross(p[a], p[b], p[c]) <= eps: continue
            others = np.array([k for k in ring if k not in (a, b, c)])
            q = p[others]
            def edge(x, y): return sign * ((y[0] - x[0]) * (q[:, 1] - x[1]) - (y[1] - x[1]) * (q[:, 0] - x[0]))
            if np.any((edge(p[a], p[b]) >= -eps) & (edge(p[b], p[c]) >= -eps) & (edge(p[c], p[a]) >= -eps)): continue
            faces.append([a, b, c]); del ring[j]; break
        else: raise CompletionError('Boundary cannot be triangulated without crossing its edges')
    if sign * _cross(*p[ring]) <= eps: raise CompletionError('Boundary leaves a degenerate triangle')
    faces.append(ring)
    return np.asarray(faces, dtype=np.int64), abs(signed)

def _inside(points, polygon):
    inside = np.zeros(len(points), dtype=bool)
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        dy = b[1] - a[1]
        if abs(dy) < 1e-15: continue
        crosses = ((a[1] > points[:, 1]) != (b[1] > points[:, 1]))
        inside ^= crosses & (points[:, 0] < a[0] + (points[:, 1] - a[1]) * (b[0] - a[0]) / dy)
    return inside

def _boundaries(mesh):
    v0, f0 = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    span = float(np.max(np.ptp(v0, axis=0)))
    _, keep, inverse = np.unique(np.round((v0 - v0.min(0)) / span, 9), axis=0, return_index=True, return_inverse=True)
    vertices = v0[keep]; faces = inverse[f0]
    edges = faces[:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2)
    _, first, counts = np.unique(np.sort(edges, axis=1), axis=0, return_index=True, return_counts=True)
    if np.any(counts > 2): raise CompletionError('Nonmanifold source edges require manual repair before gap completion')
    boundary = edges[first[counts == 1]]; adjacency = {}; outgoing = {}
    for a, b in boundary:
        a, b = int(a), int(b); adjacency.setdefault(a, []).append(b); adjacency.setdefault(b, []).append(a); outgoing.setdefault(a, []).append(b)
    remaining = set(adjacency); loops = []
    while remaining:
        stack = [min(remaining)]; component = []
        while stack:
            a = stack.pop()
            if a not in remaining: continue
            remaining.remove(a); component.append(a); stack.extend(adjacency[a])
        if not all(len(adjacency[a]) == 2 and len(outgoing.get(a, [])) == 1 for a in component):
            loops.append((None, {'boundary_vertices': len(component), 'reason': 'Open, branching or inconsistently oriented boundary'})); continue
        start = min(component); ring = [start]; following = outgoing[start][0]
        while following != start and following not in ring:
            ring.append(following); following = outgoing[following][0]
        if following != start or len(ring) != len(component):
            loops.append((None, {'boundary_vertices': len(component), 'reason': 'Boundary does not form one consistently oriented cycle'})); continue
        loops.append((np.asarray(ring), {'boundary_vertices': len(ring)}))
    return vertices, loops, span

def _plan(mesh, up):
    vertices, loops, span = _boundaries(mesh); records = []; patches = []
    triangles = np.asarray(mesh.triangles); normals = np.asarray(mesh.face_normals); centers = triangles.mean(1); areas = np.asarray(mesh.area_faces)
    for index, (ring, record) in enumerate(loops):
        record.update(id=index, status='preserved'); records.append(record)
        if ring is None: continue
        if len(ring) < 3 or len(ring) > RULES['maximum_loop_vertices']: record['reason'] = 'Boundary size exceeds conservative triangulation limits'; continue
        p = vertices[ring]; center = p.mean(0); _, singular, basis = np.linalg.svd(p - center, full_matrices=False)
        normal = basis[-1]; errors = np.abs((p - center) @ normal); q = (p - center) @ basis[:2].T
        local_span = float(np.linalg.norm(np.ptp(q, axis=0)))
        record.update(local_diameter=local_span, center=center.tolist(), normal=normal.tolist(), height=float(center @ up),
                      plane_rms=float(np.sqrt(np.mean(errors ** 2))), plane_max=float(errors.max()), horizontal_alignment=float(abs(normal @ up)))
        if singular[1] < max(local_span, 1e-12) * 1e-7: record['reason'] = 'Nearly collinear boundary'; continue
        if abs(normal @ up) < math.cos(math.radians(RULES['maximum_tilt_degrees'])): record['reason'] = 'Vertical or tilted opening preserved'; continue
        if record['plane_rms'] > local_span * RULES['plane_rms_relative_loop_diameter'] or record['plane_max'] > local_span * RULES['plane_max_relative_loop_diameter']:
            record['reason'] = 'Boundary is too nonplanar for a reliable local cap'; continue
        try: faces, area = _clip(q)
        except CompletionError as exc: record['reason'] = str(exc); continue
        # Avoid duplicating existing sheets (including an outer annulus boundary).
        nearby = (np.abs((centers - center) @ normal) < max(local_span * RULES['plane_rms_relative_loop_diameter'], record['plane_max'])) & (np.abs(normals @ normal) > .85)
        projected = (centers[nearby] - center) @ basis[:2].T
        overlap = float((areas[nearby] * np.abs(normals[nearby] @ normal))[_inside(projected, q)].sum())
        record.update(projected_area=area, existing_projected_area=overlap)
        if overlap > area * RULES['maximum_existing_projected_area_fraction']: record['reason'] = 'Existing near-planar surfaces occupy this boundary footprint'; continue
        # Reverse the native boundary direction so each new seam opposes its neighbour.
        faces = faces[:, [0, 2, 1]]
        tri = p[faces]; tri_area = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1) / 2
        if np.any(tri_area < local_span * local_span * 1e-14): record['reason'] = 'Cap contains degenerate triangles'; continue
        record.update(status='filled', reason='Bounded near-horizontal hole; inferred from its original boundary', added_triangles=len(faces), added_area=float(tri_area.sum()), boundary_vertex_ids=ring.tolist())
        patches.append({'vertices': p, 'faces': faces, 'record': record})
    return patches, records, span

def _glb(payload):
    if len(payload) < 28 or struct.unpack_from('<4sII', payload) != (b'glTF', 2, len(payload)): raise CompletionError('Source must be a valid standalone GLB')
    length, kind = struct.unpack_from('<II', payload, 12)
    if kind != 0x4e4f534a: raise CompletionError('Source GLB has no JSON document')
    document = json.loads(payload[20:20 + length]); offset = 20 + length
    if offset + 8 > len(payload): raise CompletionError('Source GLB has no embedded geometry buffer')
    count, kind = struct.unpack_from('<II', payload, offset)
    if kind != 0x004e4942 or offset + 8 + count != len(payload): raise CompletionError('Unsupported GLB buffer layout')
    if len(document.get('buffers', [])) != 1 or document['buffers'][0].get('uri') or any('uri' in i for i in document.get('images', [])):
        raise CompletionError('Completion requires a self-contained GLB without external resources')
    return document, payload[offset + 8:]

def _access(document, binary, index):
    a = document['accessors'][index]; v = document['bufferViews'][a['bufferView']]
    types = {5120:'i1', 5121:'u1', 5122:'<i2', 5123:'<u2', 5125:'<u4', 5126:'<f4'}; dims = {'SCALAR':1, 'VEC2':2, 'VEC3':3, 'VEC4':4}
    dtype = np.dtype(types[a['componentType']]); dim = dims[a['type']]
    if v.get('buffer', 0) != 0 or 'sparse' in a: raise CompletionError('Unsupported source accessor')
    return np.ndarray((a['count'], dim), dtype=dtype, buffer=binary,
        offset=v.get('byteOffset', 0) + a.get('byteOffset', 0), strides=(v.get('byteStride', dim * dtype.itemsize), dtype.itemsize))

def _world_nodes(document):
    def visit(index, parent):
        node = document['nodes'][index]
        if 'matrix' in node: local = np.array(node['matrix'], dtype=float).reshape(4, 4).T
        else:
            x, y, z, w = node.get('rotation', [0, 0, 0, 1]); local = trimesh.transformations.quaternion_matrix([w, x, y, z])
            local[:3, :3] *= np.array(node.get('scale', [1, 1, 1]))[None, :]; local[:3, 3] = node.get('translation', [0, 0, 0])
        world = parent @ local
        if 'mesh' in node: yield document['meshes'][node['mesh']], world
        for child in node.get('children', []): yield from visit(child, world)
    scene = document['scenes'][document.get('scene', 0)]
    for root in scene.get('nodes', []): yield from visit(root, np.eye(4))

def _linear(rgb):
    rgb = np.asarray(rgb, dtype=float)
    return np.where(rgb <= .04045, rgb / 12.92, ((rgb + .055) / 1.055) ** 2.4)

def _patch_colours(document, binary, patches, origin, span):
    lookup = {}; samples = [[] for _ in patches]; textures = {}
    for i, patch in enumerate(patches):
        for key in np.rint((patch['vertices'] - origin) / span * 1e5).astype(np.int64): lookup.setdefault(tuple(key), set()).add(i)
    for mesh, transform in _world_nodes(document):
        for primitive in mesh['primitives']:
            if 'POSITION' not in primitive['attributes']: continue
            p = _access(document, binary, primitive['attributes']['POSITION']); world = p @ transform[:3, :3].T + transform[:3, 3]
            quantized = np.rint((world - origin) / span * 1e5).astype(np.int64)
            hits = [(i, lookup[tuple(q)]) for i, q in enumerate(quantized) if tuple(q) in lookup]
            if not hits: continue
            material = document.get('materials', [])[primitive['material']] if 'material' in primitive else {}
            pbr = material.get('pbrMetallicRoughness', {}); factor = np.array(pbr.get('baseColorFactor', [1, 1, 1, 1]))[:3]
            image = None; uv = None
            if 'baseColorTexture' in pbr and 'TEXCOORD_0' in primitive['attributes']:
                index = document['textures'][pbr['baseColorTexture']['index']]['source']
                if index not in textures:
                    view = document['bufferViews'][document['images'][index]['bufferView']]; start = view.get('byteOffset', 0)
                    with Image.open(io.BytesIO(binary[start:start + view['byteLength']])) as full:
                        full.thumbnail((1024, 1024)); textures[index] = full.convert('RGB')
                image = textures[index]; uv = _access(document, binary, primitive['attributes']['TEXCOORD_0'])
            colors = _access(document, binary, primitive['attributes']['COLOR_0']) if 'COLOR_0' in primitive['attributes'] else None
            for vertex, patch_ids in hits:
                if image is not None:
                    u, v = np.clip(uv[vertex], 0, 1); rgb = np.array(image.getpixel((round(u * (image.width - 1)), round(v * (image.height - 1))))) / 255; linear = _linear(rgb) * factor
                    if rgb.max() < .08: continue  # Atlas padding/very dark edge texels are poor paint estimates.
                elif colors is not None:
                    linear = colors[vertex, :3].astype(float)
                    if np.issubdtype(colors.dtype, np.integer): linear /= np.iinfo(colors.dtype).max
                    linear *= factor
                else: continue
                for patch_id in patch_ids: samples[patch_id].append(linear)
    for patch, values in zip(patches, samples):
        patch['colour'] = np.median(values, axis=0).clip(0, 1).tolist() if values else _linear([.78, .79, .78]).tolist()
        patch['record'].update(colour_method='median boundary texture/vertex colour' if values else 'neutral fallback; source boundary colours unavailable', colour_samples=len(values), inferred_base_colour_linear=patch['colour'])
    for image in textures.values(): image.close()

def _vertex_normals(vertices, faces):
    triangles = vertices[faces]
    weighted = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals = np.zeros_like(vertices, dtype=float)
    for column in range(3): np.add.at(normals, faces[:, column], weighted)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    return np.divide(normals, length, out=np.zeros_like(normals), where=length > 1e-15)

def _append_glb(document, original_binary, patches):
    doc = copy.deepcopy(document); binary = bytearray(original_binary)
    def accessor(array, component, kind, target):
        while len(binary) % 4: binary.append(0)
        array = np.ascontiguousarray(array); offset = len(binary); binary.extend(array.tobytes())
        view = len(doc.setdefault('bufferViews', [])); doc['bufferViews'].append({'buffer':0, 'byteOffset':offset, 'byteLength':array.nbytes, 'target':target})
        result = {'bufferView':view, 'componentType':component, 'count':len(array), 'type':kind}
        if kind == 'VEC3': result.update(min=array.min(0).tolist(), max=array.max(0).tolist())
        index = len(doc.setdefault('accessors', [])); doc['accessors'].append(result); return index
    for patch in patches:
        mesh = trimesh.Trimesh(vertices=patch['vertices'], faces=patch['faces'], process=False)
        position = accessor(np.asarray(mesh.vertices, dtype='<f4'), 5126, 'VEC3', 34962)
        normal = accessor(np.asarray(_vertex_normals(patch['vertices'], patch['faces']), dtype='<f4'), 5126, 'VEC3', 34962)
        indices = accessor(np.asarray(mesh.faces, dtype='<u4').reshape(-1, 1), 5125, 'SCALAR', 34963)
        material = len(doc.setdefault('materials', [])); doc['materials'].append({'name':'Inferred surface', 'doubleSided':True, 'pbrMetallicRoughness':{'baseColorFactor':patch['colour'] + [1], 'metallicFactor':0, 'roughnessFactor':1}, 'extras':{'inferred':True}})
        mesh_id = len(doc.setdefault('meshes', [])); doc['meshes'].append({'name':f"inferred_gap_{patch['record']['id']}", 'primitives':[{'attributes':{'POSITION':position, 'NORMAL':normal}, 'indices':indices, 'material':material, 'mode':4}], 'extras':{'inferred':True, 'method':METHOD, 'boundary_loop':patch['record']['id']}})
        node = len(doc.setdefault('nodes', [])); doc['nodes'].append({'mesh':mesh_id, 'name':f"inferred_gap_{patch['record']['id']}", 'extras':{'inferred':True}})
        doc['scenes'][doc.get('scene', 0)].setdefault('nodes', []).append(node)
    while len(binary) % 4: binary.append(0)
    doc['buffers'][0]['byteLength'] = len(binary)
    doc.setdefault('extras', {})['completion'] = {'method':METHOD, 'contains_inferred_surfaces':bool(patches), 'collision_validated':False}
    encoded = json.dumps(doc, separators=(',', ':'), allow_nan=False).encode(); encoded += b' ' * (-len(encoded) % 4)
    return struct.pack('<4sII', b'glTF', 2, 28 + len(encoded) + len(binary)) + struct.pack('<II', len(encoded), 0x4e4f534a) + encoded + struct.pack('<II', len(binary), 0x004e4942) + binary

def complete_scene(workdir: Path, source_space='raw'):
    root = Path(workdir).resolve()
    if source_space not in ('raw', 'scaled'): raise CompletionError('source_space must be raw or scaled')
    prefix = 'exports/scaled' if source_space == 'scaled' else 'exports'
    paths = {name:_safe(root, prefix + '/' + name) for name in ('scene.glb', 'surface.ply', 'cameras.json')}
    if any(not p.is_file() for p in paths.values()): raise CompletionError('Saved visual mesh, surface and matching camera metadata are required')
    if source_space == 'scaled':
        paths['calibration.json'] = _safe(root, prefix + '/calibration.json')
        try:
            calibration = json.loads(paths['calibration.json'].read_text())
            factor = calibration['meters_per_unit']
            cameras = json.loads(paths['cameras.json'].read_text())
            if isinstance(factor, bool) or not isinstance(factor, (int, float)) or not math.isfinite(factor) or factor <= 0 or cameras.get('units') not in ('metres', 'meters'):
                raise ValueError('Unverified metric frame')
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise CompletionError('Scaled completion requires valid scale calibration and metric camera metadata') from exc
    hashes = {name:_sha(p) for name, p in paths.items()}; payload = paths['scene.glb'].read_bytes()
    try:
        doc, binary = _glb(payload); camera_data = json.loads(paths['cameras.json'].read_text()); up, up_source = _up(camera_data)
        source = trimesh.load(str(paths['surface.ply']), process=False)
        if not isinstance(source, trimesh.Trimesh) or not len(source.faces) or len(source.faces) > 3000000 or len(source.vertices) > 3000000 or not np.isfinite(source.vertices).all() or np.max(source.extents) <= 1e-10:
            raise CompletionError('Source must contain finite, bounded triangle geometry')
        patches, loops, span = _plan(source, up)
        _patch_colours(doc, binary, patches, source.bounds[0], span)
    except CompletionError: raise
    except Exception as exc: raise CompletionError(f'Cannot analyse the saved scene: {type(exc).__name__}: {exc}') from exc
    fingerprint = hashlib.sha256(json.dumps({'sources':hashes, 'rules':RULES, 'code':_sha(Path(__file__))}, sort_keys=True).encode()).hexdigest()[:20]
    output = _safe(root, prefix + '/completed/' + fingerprint); output.mkdir(parents=True, exist_ok=True)
    meshes = []; face_offset = 0
    for patch in patches:
        record = patch['record']; record['inferred_face_start'] = face_offset; face_offset += len(patch['faces'])
        mesh = trimesh.Trimesh(vertices=patch['vertices'], faces=patch['faces'], process=False)
        linear = np.array(patch['colour']); rgb = np.where(linear <= .0031308, linear * 12.92, 1.055 * linear ** (1 / 2.4) - .055)
        mesh.visual.vertex_colors = np.r_[np.rint(rgb * 255).astype(np.uint8), 255]; meshes.append(mesh)
    inferred = trimesh.util.concatenate(meshes) if meshes else None
    completed = trimesh.util.concatenate([source, inferred]) if inferred is not None else source.copy()
    before, after = geometry.surface_topology(source), geometry.surface_topology(completed)
    closed_edges = sum(p['record']['boundary_vertices'] for p in patches)
    if after['nonmanifold_edges'] > before['nonmanifold_edges'] or before['boundary_edges'] - after['boundary_edges'] != closed_edges:
        raise CompletionError('Completed seams failed the boundary/topology consistency check')
    completed_payload = _append_glb(doc, binary, patches) if patches else payload
    _, completed_binary = _glb(completed_payload)
    if completed_binary[:len(binary)] != binary: raise CompletionError('Original model buffer was modified during completion')
    geometry._atomic(output / 'scene-completed.glb', completed_payload)
    geometry._atomic(output / 'surface-completed.ply', completed.export(file_type='ply'))
    files = [('scene-completed.glb', 'completed_visual_mesh'), ('surface-completed.ply', 'completed_surface_mesh')]
    if inferred is not None:
        geometry._atomic(output / 'inferred-surfaces.ply', inferred.export(file_type='ply'))
        inferred_scene = trimesh.Scene()
        for i, patch in enumerate(patches):
            material = trimesh.visual.material.PBRMaterial(baseColorFactor=np.rint(np.r_[patch['colour'], 1] * 255).astype(np.uint8), metallicFactor=0, roughnessFactor=1, doubleSided=True)
            mesh = trimesh.Trimesh(vertices=patch['vertices'], faces=patch['faces'], vertex_normals=_vertex_normals(patch['vertices'], patch['faces']), process=False); mesh.visual = trimesh.visual.TextureVisuals(material=material)
            inferred_scene.add_geometry(mesh, node_name=f'inferred_gap_{i}')
        geometry._atomic(output / 'inferred.glb', inferred_scene.export(file_type='glb'))
        files += [('inferred-surfaces.ply', 'inferred_surface'), ('inferred.glb', 'inferred_surface')]
    if hashes != {name:_sha(p) for name, p in paths.items()}: raise CompletionError('Source exports changed while completion was running')
    report = {'version':1, 'method':METHOD, 'created_at':datetime.now(timezone.utc).isoformat(), 'source_space':source_space,
        'source_sha256':hashes['scene.glb'], 'source_files_sha256':hashes, 'original_bin_sha256':hashlib.sha256(binary).hexdigest(),
        'original_buffer_preserved':True, 'original_vertices':len(source.vertices), 'original_triangles':len(source.faces),
        'filled_loops':len(patches), 'skipped_loops':len(loops) - len(patches), 'added_triangles':sum(len(p['faces']) for p in patches),
        'added_area':sum(p['record']['added_area'] for p in patches), 'area_units':'metres squared' if source_space == 'scaled' else 'arbitrary units squared',
        'up':up.tolist(), 'up_source':up_source, 'rules':RULES, 'loops':loops, 'topology_before':before, 'topology_after':after,
        'collision_validated':False, 'metric_scale_known':source_space == 'scaled', 'contains_inferred_surfaces':bool(patches),
        'derived_files_sha256':{name:_sha(output / name) for name, _ in files},
        'limitations':['Only bounded near-horizontal openings are capped. Vertical, nonplanar, branching and overlapping openings are preserved.',
            'New surfaces and their flat boundary-matched colours are estimates, not observed photographs. No image generation or texture synthesis was used.',
            'Completion does not validate dimensions, physical floor alignment, wall openings or collision geometry.']}
    geometry._json(output / 'completion.json', report); files.append(('completion.json', 'completion_report'))
    units = 'metres' if source_space == 'scaled' else camera_data.get('units', 'unknown')
    artifacts = [geometry.artifact(root, output / name, kind, units=units, contains_inferred=bool(patches), validated=False) for name, kind in files]
    return {'completion':report, 'artifacts':artifacts, 'warnings':[f"Gap completion added {len(patches)} inferred horizontal patches; {len(loops) - len(patches)} openings were preserved. Completed geometry is unvalidated for collision."]}
