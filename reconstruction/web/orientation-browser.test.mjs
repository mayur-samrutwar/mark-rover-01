import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.dirname(fileURLToPath(import.meta.url));
const rawGLB = await readFile(path.join(root, 'fixtures/textured-cube.glb'));
function scaleGLB(source, factor) {
  const jsonLength = source.readUInt32LE(12);
  const data = JSON.parse(source.subarray(20, 20 + jsonLength).toString());
  const binaryHeader = 20 + jsonLength;
  const binary = Buffer.from(source.subarray(binaryHeader + 8, binaryHeader + 8 + source.readUInt32LE(binaryHeader)));
  const modified = new Set();
  for (const mesh of data.meshes) for (const primitive of mesh.primitives) {
    const index = primitive.attributes.POSITION;
    if (modified.has(index)) continue;
    modified.add(index);
    const accessor = data.accessors[index]; const view = data.bufferViews[accessor.bufferView];
    assert.equal(accessor.componentType, 5126); assert.equal(accessor.type, 'VEC3');
    for (let vertex = 0; vertex < accessor.count; vertex++) for (let axis = 0; axis < 3; axis++) {
      const offset = (view.byteOffset || 0) + (accessor.byteOffset || 0) + vertex * (view.byteStride || 12) + axis * 4;
      binary.writeFloatLE(binary.readFloatLE(offset) * factor, offset);
    }
    for (const field of ['min', 'max']) if (accessor[field]) accessor[field] = accessor[field].map((v) => v * factor);
  }
  let json = Buffer.from(JSON.stringify(data)); while (json.length % 4) json = Buffer.concat([json, Buffer.from(' ')]);
  const header = Buffer.alloc(12); header.write('glTF'); header.writeUInt32LE(2, 4); header.writeUInt32LE(12 + 8 + json.length + 8 + binary.length, 8);
  const jsonHeader = Buffer.alloc(8); jsonHeader.writeUInt32LE(json.length); jsonHeader.write('JSON', 4);
  const binHeader = Buffer.alloc(8); binHeader.writeUInt32LE(binary.length); binHeader.write('BIN\0', 4);
  return Buffer.concat([header, jsonHeader, json, binHeader, binary]);
}
const metricGLB = scaleGLB(rawGLB, 3);
function boxPLY(factor) {
  const vertices = [[-1,-2,-.5],[1,-2,-.5],[1,1,-.5],[-1,1,-.5],[-1,-2,.5],[1,-2,.5],[1,1,.5],[-1,1,.5]];
  const faces = [[0,1,2],[0,2,3],[4,6,5],[4,7,6],[0,4,5],[0,5,1],[3,2,6],[3,6,7],[1,5,6],[1,6,2],[0,3,7],[0,7,4]];
  return 'ply\nformat ascii 1.0\nelement vertex 8\nproperty float x\nproperty float y\nproperty float z\nelement face 12\nproperty list uchar int vertex_indices\nend_header\n' + vertices.map((p) => p.map((v) => v * factor).join(' ')).join('\n') + '\n' + faces.map((f) => '3 ' + f.join(' ')).join('\n') + '\n';
}
const cameras = { units: 'unknown', cameras: [-.16, 0, .16].map((angle, index) => ({ image: `image-${index}.jpg`, center: [index - 1, 0, -4], rotation_world_to_camera: [[Math.cos(angle),0,Math.sin(angle)],[0,1,0],[-Math.sin(angle),0,Math.cos(angle)]] })) };
const artifactRoot = '/artifacts/orientation-fixture/exports/';
const rawCamera = { name: 'cameras.json', kind: 'cameras', url: artifactRoot + 'cameras.json', units: 'unknown' };
const metricCamera = { name: 'cameras.json', kind: 'cameras', url: artifactRoot + 'scaled/cameras.json', units: 'metres' };
const models = [
  { name: 'scene.glb', kind: 'visual_mesh', url: artifactRoot + 'scene.glb', units: 'unknown' },
  { name: 'scene.glb', kind: 'visual_mesh', url: artifactRoot + 'scaled/scene.glb', units: 'metres' },
  { name: 'surface.ply', kind: 'surface_mesh', url: artifactRoot + 'surface.ply', units: 'unknown' },
  { name: 'surface.ply', kind: 'surface_mesh', url: artifactRoot + 'scaled/surface.ply', units: 'metres' },
];
const requests = [];
const server = createServer(async (request, response) => {
  const url = new URL(request.url, 'http://localhost'); requests.push(url.pathname);
  response.setHeader('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob: data:; connect-src 'self' blob: data:; object-src 'none'; base-uri 'none'");
  response.setHeader('X-Content-Type-Options', 'nosniff');
  if (url.pathname === '/') { response.writeHead(200, { 'Content-Type': 'text/html' }); return response.end('<!doctype html><html><head><meta charset="utf-8"><link rel="icon" href="/favicon.svg"><link rel="stylesheet" href="/fixture.css"></head><body><div id="stage"></div></body></html>'); }
  if (url.pathname === '/fixture.css') { response.writeHead(200, { 'Content-Type': 'text/css' }); return response.end('body{margin:0}#stage{width:960px;height:720px;position:relative}#stage canvas{position:absolute;inset:0}'); }
  if (url.pathname === rawCamera.url || url.pathname === metricCamera.url) { response.writeHead(200, { 'Content-Type': 'application/json' }); return response.end(JSON.stringify(url.pathname.includes('/scaled/') ? { ...cameras, units: 'metres', cameras: cameras.cameras.map((camera) => ({ ...camera, center: camera.center.map((v) => v * 3) })) } : cameras)); }
  if (url.pathname === models[0].url || url.pathname === models[1].url) { response.writeHead(200, { 'Content-Type': 'model/gltf-binary' }); return response.end(url.pathname === models[0].url ? rawGLB : metricGLB); }
  if (url.pathname === models[2].url || url.pathname === models[3].url) { response.writeHead(200, { 'Content-Type': 'application/octet-stream' }); return response.end(boxPLY(url.pathname === models[2].url ? 1 : 3)); }
  const relative = decodeURIComponent(url.pathname).slice(1);
  if (relative.includes('..')) { response.writeHead(403); return response.end(); }
  try { const content = await readFile(path.join(root, relative)); response.writeHead(200, { 'Content-Type': ({ '.mjs': 'text/javascript', '.js': 'text/javascript', '.svg': 'image/svg+xml' })[path.extname(relative)] || 'application/octet-stream' }); response.end(content); }
  catch { response.writeHead(404); response.end(); }
});
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
  browser = await chromium.launch({ headless: true, ...(process.env.CHROMIUM_PATH ? { executablePath: process.env.CHROMIUM_PATH } : {}), args: ['--enable-unsafe-swiftshader'] });
  const page = await browser.newPage({ viewport: { width: 1080, height: 800 } });
  const errors = []; page.on('pageerror', (error) => errors.push(error.message)); page.on('console', (message) => { if (message.type() === 'error') errors.push(message.text()); });
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.evaluate(async () => {
    const THREE = await import('/vendor/three.module.min.js');
    const { SceneViewer } = await import('/viewer.mjs');
    window.testState = { THREE, points: [] };
    window.testState.viewer = new SceneViewer(document.querySelector('#stage'), { onPoints: (points) => { window.testState.points = points; } });
    window.testState.readPositions = () => { const positions = []; window.testState.viewer.model.traverse((object) => { if (object.geometry?.attributes?.position) positions.push(Array.from(object.geometry.attributes.position.array)); }); return positions; };
    window.testState.projection = () => { const { viewer } = window.testState; viewer.camera.updateMatrixWorld(true); const center = new THREE.Box3().setFromObject(viewer.model).getCenter(new THREE.Vector3()); const origin = center.clone().project(viewer.camera); const above = center.clone().add(new THREE.Vector3(0,-1,0).multiplyScalar(viewer.radius * .4)).project(viewer.camera); return { rise: above.y - origin.y, dot: viewer.camera.up.dot(new THREE.Vector3(0,-1,0)), source: viewer.displayFrame.source }; };
  });
  for (const [index, model] of models.entries()) {
    await page.evaluate(async ({ model, artifacts }) => { await window.testState.viewer.load(model, artifacts); window.testState.before = window.testState.readPositions(); }, { model, artifacts: [metricCamera, rawCamera, ...models] });
    const initial = await page.evaluate(() => window.testState.projection());
    assert.ok(initial.dot > .99, `model ${index}: camera.up must follow image-up -Y`); assert.equal(initial.source, 'camera-poses'); assert.ok(initial.rise > .05, `model ${index}: world up must project screen-up`);
    const capture = await page.evaluate(() => { const { viewer } = window.testState; const expected = viewer.captureCameras[0]; const available = viewer.captureView(0); viewer.camera.updateMatrixWorld(true); return { available, center: viewer.camera.position.toArray(), expected: expected.center, forward: viewer.camera.getWorldDirection(new window.testState.THREE.Vector3()).toArray(), expectedForward: expected.rotation_world_to_camera[2], fov: viewer.camera.fov, unchanged: JSON.stringify(window.testState.before) === JSON.stringify(window.testState.readPositions()) }; });
    assert.equal(capture.available, true); assert.equal(capture.fov, 60); assert.equal(capture.unchanged, true);
    assert.ok(Math.hypot(...capture.center.map((v, i) => v - capture.expected[i])) < 1e-8);
    assert.ok(Math.hypot(...capture.forward.map((v, i) => v - capture.expectedForward[i])) < 1e-8);
    assert.ok(Math.abs(capture.center[2] - (model.units === 'metres' ? -12 : -4)) < 1e-8, 'capture centre must use the same units as the selected mesh');
    await page.evaluate(() => { const { viewer } = window.testState; viewer.fit(); for (let i = 0; i < 60; i++) viewer.controls.update(); window.testState.orbitBefore = viewer.camera.position.clone().sub(viewer.controls.target).toArray(); });
    const canvas = page.locator('#stage canvas'); const bounds = await canvas.boundingBox();
    await page.mouse.move(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2); await page.mouse.down(); await page.mouse.move(bounds.x + bounds.width / 2 + 100, bounds.y + bounds.height / 2, { steps: 8 }); await page.mouse.up();
    await page.waitForTimeout(350);
    const orbit = await page.evaluate(() => { const { THREE, viewer, orbitBefore } = window.testState; for (let i = 0; i < 90; i++) viewer.controls.update(); const before = new THREE.Vector3(...orbitBefore).normalize(); const after = viewer.camera.position.clone().sub(viewer.controls.target).normalize(); return { ...window.testState.projection(), signedRotation: before.cross(after).dot(new THREE.Vector3(0,-1,0)), positionsUnchanged: JSON.stringify(window.testState.before) === JSON.stringify(window.testState.readPositions()) }; });
    assert.ok(orbit.rise > .05); assert.ok(orbit.dot > .99); assert.ok(orbit.signedRotation < -.05, `model ${index}: orbit must rotate about inferred up, not stale +Y`); assert.equal(orbit.positionsUnchanged, true);
    const pick = await page.evaluate(() => {
      const { THREE, viewer } = window.testState; viewer.fit(); viewer.camera.updateMatrixWorld(true); viewer.setPicking(true);
      const bounds = new THREE.Box3().setFromObject(viewer.model); const rect = viewer.renderer.domElement.getBoundingClientRect();
      const ndc = new THREE.Vector2(.08, .03); const ray = new THREE.Raycaster(); ray.setFromCamera(ndc, viewer.camera); const expected = ray.ray.intersectBox(bounds, new THREE.Vector3());
      viewer.pick({ clientX: rect.left + (ndc.x + 1) / 2 * rect.width, clientY: rect.top + (1 - ndc.y) / 2 * rect.height });
      const point = window.testState.points[0]; viewer.setPicking(false);
      return { expected: expected?.toArray(), point, positionsUnchanged: JSON.stringify(window.testState.before) === JSON.stringify(window.testState.readPositions()), metricExtent: bounds.getSize(new THREE.Vector3()).toArray() };
    });
    assert.ok(pick.expected && pick.point, `model ${index}: surface point should be selected`);
    assert.ok(Math.hypot(...pick.expected.map((value, axis) => value - pick.point[axis])) < 1e-5, `model ${index}: raycast point must remain in model coordinates`); assert.equal(pick.positionsUnchanged, true);
    if (model.units === 'metres') assert.ok(Math.max(...pick.metricExtent) >= 6, 'metric mesh must retain its already-scaled vertex coordinates');
  }
  // A new job without poses must reset a prior inferred up, rather than inherit it.
  const fallback = await page.evaluate(async (model) => { const { viewer } = window.testState; await viewer.load(model, [model]); return { up: viewer.camera.up.toArray(), source: viewer.displayFrame.source }; }, models[0]);
  assert.deepEqual(fallback.up, [0,1,0]); assert.equal(fallback.source, 'default');
  const cameraRequests = requests.filter((url) => url.endsWith('/cameras.json')); assert.ok(cameraRequests.length <= models.length, 'one metadata request at most per artifact load'); assert.equal(cameraRequests.filter((url) => url.includes('/scaled/')).length, 2, 'scaled models must use camera centres in the same metric frame');
  assert.deepEqual(errors, []);
  await page.evaluate(() => window.testState.viewer.dispose());
  console.log(JSON.stringify({ test: 'viewer orientation browser regression', camera_up: 'negative Y from poses', fit_and_orbit: 'passed', control_recreation: 'correct orbit handedness', artifact_switches: models.length, raw_and_metric_vertices: 'unchanged', measurement_coordinates: 'preserved', capture_viewpoints: 'position, direction and scale verified', missing_metadata_reset: 'passed', csp: 'passed', errors }));
} finally { await browser?.close(); await new Promise((resolve) => server.close(resolve)); }
