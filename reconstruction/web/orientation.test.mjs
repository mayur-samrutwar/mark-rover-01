import test from 'node:test';
import assert from 'node:assert/strict';
import { captureViewPose, estimateDisplayFrame } from './orientation.mjs';

const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = (v) => v.map((x) => x / Math.hypot(...v));
const transpose = (matrix) => matrix[0].map((_, i) => matrix.map((row) => row[i]));
const multiply = (a, b) => a.map((row) => transpose(b).map((column) => dot(row, column)));
const apply = (matrix, v) => matrix.map((row) => dot(row, v));
const closeVector = (actual, expected, tolerance = 1e-10) => {
  assert.equal(actual.length, expected.length);
  for (let i = 0; i < expected.length; i++) assert.ok(Math.abs(actual[i] - expected[i]) < tolerance, `${actual} != ${expected}`);
};

// Rows map world vectors into COLMAP's right/down/forward camera axes.
function camera(up = [0, 1, 0], forward = [0, 0, -1]) {
  const down = unit(up).map((x) => -x);
  const right = unit(cross(down, forward));
  const view = cross(right, down);
  return { rotation_world_to_camera: [right, down, view] };
}

function rotation(axis, angle) {
  const [x, y, z] = unit(axis); const c = Math.cos(angle); const s = Math.sin(angle); const t = 1 - c;
  return [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
    [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
    [t*x*z-s*y, t*y*z+s*x, t*z*z+c]];
}

test('proper Y-up camera rotations produce a normalized upright display frame', () => {
  const cameras = [-0.4, -0.1, 0.2, 0.5].map((yaw) => camera([0, 1, 0], [Math.sin(yaw), 0, -Math.cos(yaw)]));
  for (const pose of cameras) {
    const r = pose.rotation_world_to_camera;
    assert.ok(Math.abs(dot(r[0], cross(r[1], r[2])) - 1) < 1e-12);
  }
  const frame = estimateDisplayFrame({ cameras });
  assert.equal(frame.source, 'camera-poses');
  assert.equal(frame.cameraCount, 4);
  assert.equal(frame.inlierCount, 4);
  closeVector(frame.up, [0, 1, 0]);
  assert.ok(Math.abs(Math.hypot(...frame.forward) - 1) < 1e-12);
  assert.ok(Math.abs(dot(frame.up, frame.forward)) < 1e-12);
  assert.ok(Math.abs(frame.strength - 1) < 1e-12);
});

test('display-up and forward transform equivariantly when the reconstruction world rotates', () => {
  const cameras = [[0, 1, 0], [.04, 1, 0], [-.02, 1, .03], [-.03, 1, -.01]].map((up) => camera(up));
  const original = estimateDisplayFrame({ cameras });
  const q = rotation([1, 2, 3], 0.91);
  // For w' = Qw, the camera mapping becomes R' = RQ^T.
  const rotated = estimateDisplayFrame({ cameras: cameras.map((pose) => ({
    rotation_world_to_camera: multiply(pose.rotation_world_to_camera, transpose(q)),
  })) });
  assert.equal(rotated.source, 'camera-poses');
  assert.equal(rotated.inlierCount, original.inlierCount);
  closeVector(rotated.up, apply(q, original.up));
  closeVector(rotated.forward, apply(q, original.forward));
  assert.ok(Math.abs(rotated.strength - original.strength) < 1e-12);
});

test('one or two rolled cameras do not overturn a strong upright consensus', () => {
  for (const outliers of [[camera([0, -1, 0])], [camera([0, -1, 0]), camera([1, 0, 0])]]) {
    const good = Array.from({ length: 6 }, (_, i) => camera([0, 1, 0], [Math.sin(i / 10), 0, -Math.cos(i / 10)]));
    const frame = estimateDisplayFrame({ cameras: [outliers[0], ...good, ...outliers.slice(1)] });
    assert.equal(frame.source, 'camera-poses');
    assert.equal(frame.cameraCount, 6 + outliers.length);
    assert.equal(frame.inlierCount, 6);
    closeVector(frame.up, [0, 1, 0]);
    assert.ok(Math.abs(frame.strength - 1) < 1e-12);
  }
});

test('equally split camera-up directions use the default frame', () => {
  const cameras = [...Array.from({ length: 4 }, () => camera([0, 1, 0])), ...Array.from({ length: 4 }, () => camera([0, -1, 0]))];
  const frame = estimateDisplayFrame({ cameras });
  assert.equal(frame.source, 'default');
  assert.equal(frame.reason, 'inconsistent-camera-up');
  assert.equal(frame.cameraCount, 8);
  assert.equal(frame.inlierCount, 0);
  closeVector(frame.up, [0, 1, 0]);
});

test('malformed, reflected, nonorthogonal and nonfinite rotations are excluded', () => {
  const invalid = [null, [], [[1, 0], [0, 1], [0, 0]],
    [[1, 0, 0], [0, -1, 0], [0, 0, 1]], // determinant -1: reflection
    [[1, .2, 0], [0, -1, 0], [0, 0, -1]], // sheared camera axes
    [[2, 0, 0], [0, -1, 0], [0, 0, -1]],
    [[NaN, 0, 0], [0, -1, 0], [0, 0, -1]],
    [[1, 0, 0], [0, -1, 0], [0, 0, Infinity]],
    [['1', 0, 0], [0, -1, 0], [0, 0, -1]]];
  for (const r of invalid) {
    const frame = estimateDisplayFrame({ cameras: [camera(), camera(), { rotation_world_to_camera: r }] });
    assert.equal(frame.source, 'default');
    assert.equal(frame.reason, 'insufficient-camera-poses');
    assert.equal(frame.cameraCount, 2);
  }
  const frame = estimateDisplayFrame({ cameras: [camera(), camera(), camera(), ...invalid.map((r) => ({ rotation_world_to_camera: r }))] });
  assert.equal(frame.source, 'camera-poses');
  assert.equal(frame.cameraCount, 3);
  assert.equal(frame.inlierCount, 3);
});

test('missing pose metadata falls back without throwing', () => {
  for (const input of [undefined, null, {}, { cameras: null }, { cameras: {} }, { cameras: [] }, { cameras: [null, {}, { center: [0, 0, 0] }] }]) {
    const frame = estimateDisplayFrame(input);
    assert.equal(frame.source, 'default');
    assert.equal(frame.cameraCount, 0);
    closeVector(frame.up, [0, 1, 0]);
    closeVector(frame.forward, [0, 0, -1]);
  }
});

test('exactly three valid camera poses are sufficient', () => {
  const expectedUp = unit([-.1, -.98, .2]);
  const frame = estimateDisplayFrame({ cameras: [camera(expectedUp), camera(expectedUp), camera(expectedUp)] });
  assert.equal(frame.source, 'camera-poses');
  assert.equal(frame.cameraCount, 3);
  assert.equal(frame.inlierCount, 3);
  closeVector(frame.up, expectedUp);
  assert.ok(Math.abs(Math.hypot(...frame.up) - 1) < 1e-12);
});

test('capture viewpoint preserves translated camera centres and OpenCV forward/up after world rotation', () => {
  const q = rotation([1, 2, 3], .77); const translation = [17, -2, 4];
  const original = { ...camera(), center: [2, 3, -4] };
  const transformed = { center: apply(q, original.center).map((x, i) => x + translation[i]),
    rotation_world_to_camera: multiply(original.rotation_world_to_camera, transpose(q)) };
  const pose = captureViewPose(transformed);
  closeVector(pose.center, transformed.center);
  closeVector(pose.forward, apply(q, [0, 0, -1]));
  closeVector(pose.up, apply(q, [0, 1, 0]));
  assert.notEqual(pose.center, transformed.center);
});

test('capture viewpoint rejects invalid centres and camera handedness', () => {
  for (const center of [undefined, null, [0, 0], [0, 0, NaN], ['1', 0, 0]]) assert.equal(captureViewPose({ ...camera(), center }), null);
  assert.equal(captureViewPose({ center: [0, 0, 0], rotation_world_to_camera: [[1, 0, 0], [0, -1, 0], [0, 0, 1]] }), null);
});

test('declared native Y-up stays upright when all capture cameras pitch thirty degrees', () => {
  const a = Math.PI / 6; const pose = camera([0, Math.cos(a), Math.sin(a)], [0, Math.sin(a), -Math.cos(a)]);
  const frame = estimateDisplayFrame({ up_axis: 'Y', cameras: [pose, pose, pose] });
  closeVector(frame.up, [0, 1, 0]); closeVector(frame.forward, [0, 0, -1]);
  assert.equal(frame.source, 'declared-up-axis');
  // The same poses in an arbitrary world still use the image-up estimate.
  const arbitrary = estimateDisplayFrame({ cameras: [pose, pose, pose] });
  closeVector(arbitrary.up, [0, Math.cos(a), Math.sin(a)]);
});
