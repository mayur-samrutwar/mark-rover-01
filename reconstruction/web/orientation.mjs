// COLMAP image axes provide a display-up estimate, not measured gravity.
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = (v) => { const n = Math.hypot(...v); return Number.isFinite(n) && n > 1e-10 ? v.map((x) => x / n) : null; };
const clamp = (x) => Math.max(-1, Math.min(1, x));
const finiteVector = (v) => Array.isArray(v) && v.length === 3 && v.every((x) => typeof x === 'number' && Number.isFinite(x));
const fallback = (cameraCount = 0, reason = 'missing-camera-poses') => ({ up: [0, 1, 0], forward: [0, 0, -1], source: 'default', cameraCount, inlierCount: 0, strength: 0, reason });
function validRotation(r) {
  if (!Array.isArray(r) || r.length !== 3 || !r.every(finiteVector)) return false;
  if (r.some((row) => Math.abs(dot(row, row) - 1) > .05)) return false;
  if (Math.abs(dot(r[0], r[1])) > .05 || Math.abs(dot(r[0], r[2])) > .05 || Math.abs(dot(r[1], r[2])) > .05) return false;
  return Math.abs(dot(r[0], cross(r[1], r[2])) - 1) <= .08;
}
export function estimateDisplayFrame(document) {
  const cameras = Array.isArray(document?.cameras) ? document.cameras : [];
  const poses = cameras.filter((camera) => validRotation(camera?.rotation_world_to_camera)).map((camera) => {
    const r = camera.rotation_world_to_camera;
    return { up: unit(r[1].map((x) => -x)), forward: unit(r[2]) };
  });
  // A declared file axis defines its display convention, even when every
  // capture camera was pitched. It does not certify measured gravity.
  const declaredUp = document?.up_axis === 'X' ? [1, 0, 0] : document?.up_axis === 'Y' ? [0, 1, 0] : document?.up_axis === 'Z' ? [0, 0, 1] : null;
  if (declaredUp) {
    const projected = (vector) => unit(vector.map((x, i) => x - declaredUp[i] * dot(vector, declaredUp)));
    const forward = poses.map((pose) => projected(pose.forward)).find(Boolean)
      || projected(Math.abs(declaredUp[2]) < .9 ? [0, 0, -1] : [1, 0, 0]);
    return { up: declaredUp, forward, source: 'declared-up-axis', cameraCount: poses.length,
      inlierCount: poses.length, strength: 1, reason: 'file-coordinate-convention' };
  }
  if (poses.length < 3) return fallback(poses.length, 'insufficient-camera-poses');
  // An angular medoid avoids choosing a rolled outlier as the reference. Sample
  // candidates for large scans, but score against every valid camera direction.
  let seed = null; let best = Infinity;
  const step = Math.max(1, Math.ceil(poses.length / 256));
  for (let i = 0; i < poses.length; i += step) {
    const candidate = poses[i].up;
    const cost = poses.reduce((sum, pose) => sum + Math.acos(clamp(dot(candidate, pose.up))), 0);
    if (cost < best) { best = cost; seed = candidate; }
  }
  const cutoff = Math.cos(Math.PI / 4);
  const inliers = poses.filter((pose) => dot(seed, pose.up) >= cutoff);
  if (inliers.length < 3 || inliers.length / poses.length < .7) return fallback(poses.length, 'inconsistent-camera-up');
  const mean = inliers.reduce((sum, pose) => sum.map((x, i) => x + pose.up[i] / inliers.length), [0, 0, 0]);
  const strength = Math.hypot(...mean); const up = unit(mean);
  if (!up || strength < .75) return fallback(poses.length, 'weak-camera-up');
  const reference = inliers.reduce((bestPose, pose) => dot(pose.up, up) > dot(bestPose.up, up) ? pose : bestPose, inliers[0]);
  let forward = unit(reference.forward.map((x, i) => x - up[i] * dot(reference.forward, up)));
  if (!forward) {
    const axis = Math.abs(up[2]) < .9 ? [0, 0, -1] : [1, 0, 0];
    forward = unit(axis.map((x, i) => x - up[i] * dot(axis, up)));
  }
  return { up, forward, source: 'camera-poses', cameraCount: poses.length, inlierCount: inliers.length, strength };
}

// Camera-Z is forward and image-Y is down in the exported OpenCV convention.
export function captureViewPose(camera) {
  if (!finiteVector(camera?.center) || !validRotation(camera?.rotation_world_to_camera)) return null;
  return { center: [...camera.center], forward: [...camera.rotation_world_to_camera[2]],
    up: camera.rotation_world_to_camera[1].map((x) => -x) };
}
