import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.dirname(fileURLToPath(import.meta.url));
const rawGLB = await readFile(path.join(root, 'fixtures/textured-cube.glb'));
function scaledGLB(source, factor) {
  const length = source.readUInt32LE(12);
  const document = JSON.parse(source.subarray(20, 20 + length).toString());
  const binaryOffset = 20 + length;
  const binary = Buffer.from(source.subarray(binaryOffset + 8, binaryOffset + 8 + source.readUInt32LE(binaryOffset)));
  const modified = new Set();
  for (const mesh of document.meshes) for (const primitive of mesh.primitives) {
    const index = primitive.attributes.POSITION;
    if (modified.has(index)) continue;
    modified.add(index);
    const accessor = document.accessors[index]; const view = document.bufferViews[accessor.bufferView];
    assert.equal(accessor.componentType, 5126); assert.equal(accessor.type, 'VEC3');
    for (let vertex = 0; vertex < accessor.count; vertex++) for (let axis = 0; axis < 3; axis++) {
      const offset = (view.byteOffset || 0) + (accessor.byteOffset || 0) + vertex * (view.byteStride || 12) + axis * 4;
      binary.writeFloatLE(binary.readFloatLE(offset) * factor, offset);
    }
    for (const field of ['min', 'max']) if (accessor[field]) accessor[field] = accessor[field].map((v) => v * factor);
  }
  let json = Buffer.from(JSON.stringify(document)); while (json.length % 4) json = Buffer.concat([json, Buffer.from(' ')]);
  const header = Buffer.alloc(12); header.write('glTF'); header.writeUInt32LE(2, 4); header.writeUInt32LE(28 + json.length + binary.length, 8);
  const jsonHeader = Buffer.alloc(8); jsonHeader.writeUInt32LE(json.length); jsonHeader.write('JSON', 4);
  const binHeader = Buffer.alloc(8); binHeader.writeUInt32LE(binary.length); binHeader.write('BIN\0', 4);
  return Buffer.concat([header, jsonHeader, json, binHeader, binary]);
}
const metricGLB = scaledGLB(rawGLB, 3);
const cases = [];
const files = new Map();
for (const declared of [false, true]) for (const metric of [false, true]) {
  const prefix = `/artifacts/explore-${declared ? 'declared' : 'inferred'}/exports/${metric ? 'scaled/' : ''}`;
  const cameras = { ...(declared ? { up_axis: 'Y' } : {}), units: metric ? 'metres' : 'unknown', cameras: [-.12, 0, .12].map((angle, index) => ({
    image: `image-${index}.jpg`, center: [index - 1, 0, -4].map((v) => v * (metric ? 3 : 1)),
    rotation_world_to_camera: declared
      ? [[-Math.cos(angle), 0, -Math.sin(angle)], [0, -1, 0], [-Math.sin(angle), 0, Math.cos(angle)]]
      : [[Math.cos(angle), 0, Math.sin(angle)], [0, 1, 0], [-Math.sin(angle), 0, Math.cos(angle)]],
  })) };
  const artifact = { name: 'scene.glb', kind: 'visual_mesh', url: prefix + 'scene.glb', units: cameras.units };
  const metadata = { name: 'cameras.json', kind: 'cameras', url: prefix + 'cameras.json', units: cameras.units };
  files.set(artifact.url, [metric ? metricGLB : rawGLB, 'model/gltf-binary']);
  files.set(metadata.url, [JSON.stringify(cameras), 'application/json']);
  cases.push({ declared, metric, artifact, metadata, up: declared ? [0, 1, 0] : [0, -1, 0] });
}
const requests = [];
const server = createServer(async (request, response) => {
  const url = new URL(request.url, 'http://localhost'); requests.push({ method: request.method, path: url.pathname });
  response.setHeader('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob: data:; connect-src 'self' blob: data:; object-src 'none'; base-uri 'none'");
  response.setHeader('X-Content-Type-Options', 'nosniff');
  if (request.method !== 'GET') { response.writeHead(405); return response.end(); }
  if (url.pathname === '/') { response.writeHead(200, { 'Content-Type': 'text/html' }); return response.end('<!doctype html><html><head><meta charset="utf-8"><link rel="icon" href="/favicon.svg"><link rel="stylesheet" href="/fixture.css"></head><body><div id="stage"></div><input id="outside" aria-label="Outside viewer"><button id="outside-button">Outside button</button></body></html>'); }
  if (url.pathname === '/fixture.css') { response.writeHead(200, { 'Content-Type': 'text/css' }); return response.end('body{margin:0}#stage{width:960px;height:640px;position:relative}#stage canvas{position:absolute;inset:0}'); }
  if (files.has(url.pathname)) { const [content, type] = files.get(url.pathname); response.writeHead(200, { 'Content-Type': type }); return response.end(content); }
  const relative = decodeURIComponent(url.pathname).slice(1);
  if (relative.includes('..')) { response.writeHead(403); return response.end(); }
  try { const content = await readFile(path.join(root, relative)); response.writeHead(200, { 'Content-Type': ({ '.mjs': 'text/javascript', '.js': 'text/javascript', '.svg': 'image/svg+xml' })[path.extname(relative)] || 'application/octet-stream' }); response.end(content); }
  catch { response.writeHead(404); response.end(); }
});
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
  browser = await chromium.launch({ headless: true, ...(process.env.CHROMIUM_PATH ? { executablePath: process.env.CHROMIUM_PATH } : {}), args: ['--enable-unsafe-swiftshader'] });
  const page = await browser.newPage({ viewport: { width: 1040, height: 760 } });
  const errors = []; const outbound = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => { if (message.type() === 'error') errors.push(message.text()); });
  const origin = `http://127.0.0.1:${server.address().port}`;
  page.on('request', (request) => { if (!request.url().startsWith(origin + '/') && !/^(blob:|data:)/.test(request.url())) outbound.push(request.url()); });
  await page.goto(origin + '/');
  await page.evaluate(async () => {
    const THREE = await import('/vendor/three.module.min.js'); const { SceneViewer } = await import('/viewer.mjs');
    const viewer = new SceneViewer(document.querySelector('#stage'));
    window.testState = { THREE, viewer,
      positions: () => { const arrays = []; viewer.model?.traverse((object) => { if (object.geometry?.attributes?.position) arrays.push(Array.from(object.geometry.attributes.position.array)); }); return arrays; },
      snapshot: () => ({ position: viewer.camera.position.toArray(), forward: viewer.camera.getWorldDirection(new THREE.Vector3()).toArray(), keys: [...viewer.explore.keys], mode: viewer.navigationMode, enabled: viewer.explore.enabled }),
    };
  });
  const snapshot = () => page.evaluate(() => window.testState.snapshot());
  const distance = (a, b) => Math.hypot(...a.map((v, i) => v - b[i]));
  const direction = (a, b) => { const d = b.map((v, i) => v - a[i]); const n = Math.hypot(...d); return d.map((v) => v / n); };
  const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
  const canvas = page.locator('#stage canvas');
  const enter = async () => { await page.evaluate(() => window.testState.viewer.setNavigationMode('explore')); await canvas.focus(); };
  for (const [index, fixture] of cases.entries()) {
    await page.evaluate(async ({ artifact, metadata }) => { const state = window.testState; await state.viewer.load(artifact, [metadata, artifact]); state.viewer.captureView(0); state.originalModel = state.viewer.model; state.originalPositions = JSON.stringify(state.positions()); }, fixture);
    const scale = await page.evaluate(() => window.testState.viewer.radius);
    await enter(); let start = await snapshot();
    assert.equal(start.mode, 'explore'); assert.equal(start.enabled, true);
    await page.keyboard.down('w'); await page.waitForTimeout(150); await page.keyboard.up('w');
    let end = await snapshot();
    const horizontalForward = start.forward.map((v, i) => v - fixture.up[i] * dot(start.forward, fixture.up));
    const norm = Math.hypot(...horizontalForward); const expectedForward = horizontalForward.map((v) => v / norm);
    assert.ok(distance(start.position, end.position) > scale * .004, `case ${index}: W moves the camera`);
    assert.ok(dot(direction(start.position, end.position), expectedForward) > .995, `case ${index}: W uses horizontal camera heading`);
    assert.equal(end.keys.length, 0, 'key release clears movement');
    const stopped = end.position; await page.waitForTimeout(100);
    assert.ok(distance(stopped, (await snapshot()).position) < 1e-8, 'camera stops after release');
    start = await snapshot(); await page.keyboard.down('e'); await page.waitForTimeout(100); await page.keyboard.up('e'); end = await snapshot();
    assert.ok(dot(direction(start.position, end.position), fixture.up) > .999, `case ${index}: E follows the file up axis`);
    start = await snapshot(); await page.keyboard.down('q'); await page.waitForTimeout(100); await page.keyboard.up('q'); end = await snapshot();
    assert.ok(dot(direction(start.position, end.position), fixture.up) < -.999, `case ${index}: Q descends`);
    // Use real browser key events, then compare isolated update steps without RAF timing noise.
    await page.keyboard.down('w');
    const advance = (dt) => page.evaluate((seconds) => { const { viewer } = window.testState; const before = viewer.camera.position.clone(); viewer.explore.update(seconds); return viewer.camera.position.distanceTo(before); }, dt);
    const straight = await advance(.02); await page.keyboard.down('d'); const diagonal = await advance(.02);
    assert.ok(Math.abs(diagonal / straight - 1) < .001, 'diagonal travel is normalized');
    await page.keyboard.down('Shift'); const fast = await advance(.02); await page.keyboard.up('Shift');
    assert.ok(Math.abs(fast / diagonal - 3) < .001, 'Shift moves at three times the base speed');
    assert.ok(await advance(5) <= scale * .2 * .2 + 1e-8, 'long frame pauses cannot cause a large jump');
    await page.keyboard.up('d'); await page.keyboard.up('w');
    // Strafe has the correct handedness under either coordinate-up convention.
    start = await snapshot(); await page.keyboard.down('d'); await page.waitForTimeout(100); await page.keyboard.up('d'); end = await snapshot();
    const expectedRight = [expectedForward[1] * fixture.up[2] - expectedForward[2] * fixture.up[1], expectedForward[2] * fixture.up[0] - expectedForward[0] * fixture.up[2], expectedForward[0] * fixture.up[1] - expectedForward[1] * fixture.up[0]];
    assert.ok(dot(direction(start.position, end.position), expectedRight) > .995, 'D moves camera-right');
    for (const [key, expected] of [['s', expectedForward], ['a', expectedRight]]) {
      start = await snapshot(); await page.keyboard.down(key); await page.waitForTimeout(80); await page.keyboard.up(key); end = await snapshot();
      assert.ok(dot(direction(start.position, end.position), expected) < -.995, `${key.toUpperCase()} moves in the opposite direction`);
    }
    start = await snapshot(); const bounds = await canvas.boundingBox();
    await page.mouse.move(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2); await page.mouse.down();
    await page.mouse.move(bounds.x + bounds.width / 2 + 90, bounds.y + bounds.height / 2 + 35, { steps: 6 }); await page.mouse.up();
    end = await snapshot();
    assert.ok(dot(start.forward, end.forward) < .999, 'mouse drag changes gaze');
    assert.ok(distance(start.position, end.position) < 1e-8, 'looking does not translate camera');
    assert.ok(Math.abs(dot(end.forward, fixture.up)) < .9999, 'look direction stays away from vertical singularity');
    // Blur must release a key even if its keyup is delivered outside the viewer.
    await page.keyboard.down('w'); await page.waitForTimeout(40); await page.locator('#outside').focus();
    end = await snapshot(); assert.equal(end.keys.length, 0, 'canvas blur releases held keys');
    await page.keyboard.up('w'); await page.locator('#outside').fill(''); const outsideStart = (await snapshot()).position;
    await page.keyboard.type('wasdqe'); await page.waitForTimeout(100);
    assert.equal(await page.locator('#outside').inputValue(), 'wasdqe', 'normal text input remains usable');
    assert.ok(distance(outsideStart, (await snapshot()).position) < 1e-8, 'typing outside the viewer does not move it');
    await canvas.focus(); await page.keyboard.down('w');
    await page.evaluate(() => window.dispatchEvent(new Event('blur')));
    assert.equal((await snapshot()).keys.length, 0, 'window blur clears keys'); await page.keyboard.up('w');
    await canvas.focus(); await page.keyboard.down('w');
    await page.evaluate(() => { Object.defineProperty(document, 'hidden', { configurable: true, value: true }); document.dispatchEvent(new Event('visibilitychange')); delete document.hidden; });
    assert.equal((await snapshot()).keys.length, 0, 'hidden document clears keys'); await page.keyboard.up('w');
    start = await snapshot(); await page.keyboard.press('Escape'); end = await snapshot();
    assert.equal(end.mode, 'orbit'); assert.equal(end.enabled, false); assert.equal(end.keys.length, 0);
    assert.ok(distance(start.position, end.position) < 1e-8, 'Escape returns to orbit without a camera jump');
    await page.waitForTimeout(150); assert.ok(distance(end.position, (await snapshot()).position) < 1e-7, 'orbit damping does not snap back');
    // Bookmarks still inspect one unchanged 3D mesh rather than switching photographs.
    const bookmark = await page.evaluate(() => { const state = window.testState; state.viewer.setNavigationMode('explore'); const success = state.viewer.captureView(1); return { success, mode: state.viewer.navigationMode, sameModel: state.viewer.model === state.originalModel, sameGeometry: JSON.stringify(state.positions()) === state.originalPositions, position: state.viewer.camera.position.toArray(), expected: state.viewer.captureCameras[1].center }; });
    assert.equal(bookmark.success, true); assert.equal(bookmark.mode, 'orbit'); assert.equal(bookmark.sameModel, true); assert.equal(bookmark.sameGeometry, true);
    assert.ok(distance(bookmark.position, bookmark.expected) < 1e-8, 'camera bookmark matches selected model units');
    for (const action of ['fit', 'picking']) {
      await enter(); await page.keyboard.down('w');
      await page.evaluate((name) => { const { viewer } = window.testState; if (name === 'fit') viewer.fit(); else viewer.setPicking(true); }, action);
      end = await snapshot(); assert.equal(end.mode, 'orbit', `${action} exits explore mode`); assert.equal(end.keys.length, 0);
      await page.keyboard.up('w'); await page.evaluate(() => window.testState.viewer.setPicking(false));
    }
    assert.equal(await page.evaluate(() => JSON.stringify(window.testState.positions()) === window.testState.originalPositions), true, 'exploration never changes model vertices');
  }
  await enter(); await page.keyboard.down('w');
  await page.evaluate(async ({ artifact, metadata }) => window.testState.viewer.load(artifact, [metadata, artifact]), cases[0]);
  const reloaded = await snapshot(); assert.equal(reloaded.mode, 'orbit'); assert.equal(reloaded.keys.length, 0); assert.equal(reloaded.enabled, false);
  await page.keyboard.up('w');
  await enter(); await page.keyboard.down('w');
  await page.evaluate(() => window.testState.viewer.clear());
  const cleared = await snapshot(); assert.equal(cleared.mode, 'orbit'); assert.equal(cleared.keys.length, 0); assert.equal(cleared.enabled, false);
  await page.keyboard.up('w');
  await page.evaluate(() => window.testState.viewer.setNavigationMode('explore'));
  assert.equal((await snapshot()).enabled, false, 'an empty scene cannot activate exploration');
  assert.deepEqual(errors, []); assert.deepEqual(outbound, []);
  assert.ok(requests.every((request) => request.method === 'GET'), 'viewer input sends no mutations or rover commands');
  assert.ok(!requests.some((request) => /(?:drive|motor|command|serial|rover-control)/i.test(request.path)), 'no physical rover endpoints are requested');
  await page.evaluate(() => window.testState.viewer.dispose());
  console.log(JSON.stringify({ test: 'scene exploration browser regression', coordinateFrames: ['inferred negative-Y', 'declared positive-Y'], units: ['raw', 'metres'], movement: 'horizontal WASD, Q/E vertical, Shift speed, normalized diagonal, bounded elapsed time', release: 'keyup, canvas blur, window blur, hidden tab, Escape, fit, picking, load, clear', mouseLook: 'passed', geometry: 'unchanged', captureBookmarks: 'same model and correct camera units', physicalRoverRequests: 0, errors }));
} finally { await browser?.close(); await new Promise((resolve) => server.close(resolve)); }
