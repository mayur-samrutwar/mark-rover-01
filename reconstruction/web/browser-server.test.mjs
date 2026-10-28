import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.dirname(fileURLToPath(import.meta.url));
const service = spawn(process.env.PYTHON || 'python3', [path.join(root, 'fixtures/serve_fixture.py')], { stdio: ['ignore', 'pipe', 'pipe'] });
let diagnostic = ''; service.stderr.on('data', (data) => { diagnostic += data; });
let browser;
try {
  const port = await new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error('Fixture service did not start: ' + diagnostic)), 20000);
    let output = '';
    service.stdout.on('data', (chunk) => { output += chunk; const line = output.split('\n')[0]; try { const message = JSON.parse(line); if (message.port) { clearTimeout(timeout); resolve(message.port); } } catch {} });
    service.once('exit', (code) => { clearTimeout(timeout); reject(new Error(`Fixture service exited (${code}): ${diagnostic}`)); });
  });
  const base = `http://127.0.0.1:${port}`;
  browser = await chromium.launch({ headless: true, ...(process.env.CHROMIUM_PATH ? { executablePath: process.env.CHROMIUM_PATH } : {}), args: ['--enable-unsafe-swiftshader'] });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const errors = []; const external = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => { if (message.type() === 'error') errors.push(message.text()); });
  page.on('request', (request) => { if (!request.url().startsWith(base + '/') && !request.url().startsWith('data:') && !request.url().startsWith('blob:')) external.push(request.url()); });
  const response = await page.goto(base + '/');
  const csp = response.headers()['content-security-policy'];
  assert.match(csp, /script-src 'self'/);
  assert.match(csp, /connect-src[^;]*blob:/, 'embedded GLB images are fetched from blob URLs');
  await page.getByText('Connected', { exact: true }).waitFor();
  // Inspect the viewer instance only inside this test; production exposes no global.
  await page.evaluate(async () => { const { SceneViewer } = await import('/viewer.mjs'); const load = SceneViewer.prototype.load; SceneViewer.prototype.load = async function(...args) { const value = await load.apply(this, args); window.__fixtureViewer = this; return value; }; });
  await page.locator('#reconstruct').click();
  await page.waitForFunction(() => document.querySelector('#model').value.endsWith('sparse.ply'));
  await page.waitForFunction(() => document.querySelector('#model').value.endsWith('scene.glb'));
  await page.locator('#viewer-loading').waitFor({ state: 'hidden' });
  await page.locator('#viewer-empty').waitFor({ state: 'hidden' });
  const model = await page.evaluate(() => { let meshes = 0; let textured = 0; let textureWidth = 0; window.__fixtureViewer.model.traverse((object) => { if (object.isMesh) { meshes++; if (object.material.map) { textured++; textureWidth = object.material.map.image.width; } } }); return { meshes, textured, textureWidth }; });
  assert.deepEqual(model, { meshes: 1, textured: 1, textureWidth: 8 });
  assert.match(await page.locator('#metrics').textContent(), /Aligned3\/3Vertices8Faces12/);
  assert.equal(await page.locator('#scale-badge').textContent(), 'Scale unknown');
  await page.screenshot({ path: process.env.RECONSTRUCTION_SCREENSHOT || '/private/tmp/rover-reconstruction-textured-glb.png', fullPage: true });
  await page.locator('#view-mode').selectOption('solid');
  assert.equal(await page.evaluate(() => { let maps = 0; window.__fixtureViewer.model.traverse((object) => { if (object.material?.map) maps++; }); return maps; }), 0);
  await page.locator('#view-mode').selectOption('texture');
  assert.equal(await page.evaluate(() => { let maps = 0; window.__fixtureViewer.model.traverse((object) => { if (object.material?.map) maps++; }); return maps; }), 1);
  const artifact = await page.request.get(base + await page.locator('#model').inputValue());
  assert.equal(artifact.status(), 200);
  assert.equal(artifact.headers()['content-type'], 'model/gltf-binary');
  assert.deepEqual(errors, []); assert.deepEqual(external, []);
  console.log(JSON.stringify({ test: 'actual API browser integration', immutable_snapshot: '3 JPEG fixture inputs', progressive_sparse_to_glb: 'passed', embedded_texture: '8x8 rendered', geometry_toggle: 'passed', metrics: 'nested values displayed', csp: 'passed', external_requests: 0, hardware_motion: false, reconstruction_engines: 'fixture only; no quality claims' }));
} finally {
  await browser?.close();
  if (service.exitCode === null) { const exited = once(service, 'exit'); service.kill('SIGTERM'); const timer = setTimeout(() => service.kill('SIGKILL'), 5000); await exited; clearTimeout(timer); }
}
