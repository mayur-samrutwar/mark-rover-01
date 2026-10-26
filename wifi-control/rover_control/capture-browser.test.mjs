// Runs real capture handlers/storage on ephemeral loopback ports. Camera images
// and rover responses are fixtures; this test never connects to rover hardware.
import assert from 'node:assert/strict';
import { spawn, spawnSync } from 'node:child_process';
import { mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { once } from 'node:events';
const { chromium, request: apiRequest } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.dirname(fileURLToPath(import.meta.url));
const temporary = await mkdtemp(path.join(tmpdir(), 'rover-capture-browser-'));
const python = process.env.PYTHON || 'python3';
const certificate = path.join(temporary, 'certificate.pem'), key = path.join(temporary, 'key.pem');
const openssl = spawnSync('openssl', ['req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '1', '-subj', '/CN=localhost', '-keyout', key, '-out', certificate], {encoding: 'utf8'});
assert.equal(openssl.status, 0, 'OpenSSL must be available to create a temporary test certificate');
const harness = String.raw`
import base64, io, json, ssl, sys, threading
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, sys.argv[1])
from PIL import Image
from capture_server import CaptureStore
from serve import CaptureHTTPServer, make_handler
temporary = Path(sys.argv[2])
config = SimpleNamespace(port=0, https_port=0, setup_port=0, rover='http://127.0.0.1:1')
store = CaptureStore(temporary / 'captures', pairing={'code': '123456'})
tls = {'host': '127.0.0.1'}
operator = CaptureHTTPServer(('127.0.0.1', 0), make_handler(config, store))
config.port = operator.server_address[1]
phone = CaptureHTTPServer(('127.0.0.1', 0), make_handler(config, store, 'phone', tls))
config.https_port = phone.server_address[1]
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(str(temporary / 'certificate.pem'), str(temporary / 'key.pem'))
phone.socket = context.wrap_socket(phone.socket, server_side=True)
store.pairing.update(url=f'https://127.0.0.1:{config.https_port}/phone/?code=123456', setup_url=f'http://127.0.0.1:{config.port}/')
image = Image.new('RGB', (640, 480), '#809087')
buffer = io.BytesIO(); image.save(buffer, format='JPEG')
for server in [operator, phone]:
    threading.Thread(target=server.serve_forever, daemon=True).start()
print(json.dumps({'operator': f'http://127.0.0.1:{config.port}', 'phone': f'https://127.0.0.1:{config.https_port}', 'jpeg': base64.b64encode(buffer.getvalue()).decode()}), flush=True)
threading.Event().wait()
`;
const server = spawn(python, ['-u', '-c', harness, root, temporary], {env: {...process.env, PYTHONDONTWRITEBYTECODE: '1'}, stdio: ['ignore', 'pipe', 'pipe']});
let browser, api, heartbeat, stderr = '';
server.stderr.on('data', data => stderr += data);
const ready = new Promise((resolve, reject) => {
  let output = '';
  server.stdout.on('data', data => {
    output += data;
    const line = output.split('\n')[0];
    if (output.includes('\n')) { try { resolve(JSON.parse(line)); } catch (error) { reject(error); } }
  });
  server.once('exit', code => reject(new Error(`Capture test server exited (${code}): ${stderr}`)));
});
const wait = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds));
async function until(predicate, message, timeout = 10000) {
  const end = Date.now() + timeout;
  while (Date.now() < end) { if (await predicate()) return; await wait(100); }
  assert.fail(message);
}
try {
  let startupTimer;
  const {operator, phone, jpeg} = await Promise.race([
    ready,
    new Promise((_, reject) => {startupTimer = setTimeout(() => reject(new Error(`Capture server startup timed out: ${stderr}`)), 10000);}),
  ]).finally(() => clearTimeout(startupTimer));
  const image = Buffer.from(jpeg, 'base64');
  api = await apiRequest.newContext({ignoreHTTPSErrors: true});
  const pair = await api.post(phone + '/capture/phone/pair', {data: {code: '123456', device: 'Browser integration fixture'}});
  assert.equal(pair.status(), 200);
  const {token} = await pair.json();
  const phoneHeaders = {'X-Capture-Token': token};
  const status = () => api.post(phone + '/capture/phone/status', {headers: phoneHeaders, data: {camera_ready: true, visibility: 'visible', width: 640, height: 480}});
  await status();
  heartbeat = setInterval(() => { void status().catch(() => {}); }, 1000);
  const preview = await api.post(phone + '/capture/phone/preview', {headers: {...phoneHeaders, 'Content-Type': 'image/jpeg'}, data: image});
  assert.equal(preview.status(), 200);
  const state = async () => (await api.get(operator + '/capture/api/state')).json();
  const upload = command => api.post(phone + '/capture/phone/upload?command_id=' + command.id, {headers: {...phoneHeaders, 'Content-Type': 'image/jpeg', 'X-Frame-Metadata': JSON.stringify({captured_at: new Date().toISOString(), source: 'test-fixture'})}, data: image});
  const commands = async () => (await api.get(phone + '/capture/phone/commands', {headers: phoneHeaders})).json();
  browser = await chromium.launch({headless: true, ...(process.env.CHROMIUM_PATH ? {executablePath: process.env.CHROMIUM_PATH} : {})});
  const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
  const pageErrors = [], roverRequests = [];
  page.on('pageerror', error => pageErrors.push(error.message));
  await page.addInitScript(() => {document.hasFocus = () => true;});
  let armed = false, sequence = 0, left = 0, right = 0;
  const roverState = type => ({protocol: 'rover-wifi-v1', type, ready: true, armed, seq: sequence, left, right});
  const roverPattern = new RegExp('^' + operator.replace(/[.*+?^${}()|[\]\\]/g, '\\$&') + '/api/');
  await page.route(roverPattern, async route => {
    const request = route.request(), pathname = new URL(request.url()).pathname;
    roverRequests.push({path: pathname, body: request.postData()});
    if (pathname === '/api/session') {armed = true; sequence = left = right = 0; return route.fulfill({json: {...roverState('hello'), token: 'a'.repeat(32)}});}
    if (pathname === '/api/stop') {armed = false; left = right = 0; return route.fulfill({json: roverState('stopped')});}
    if (pathname === '/api/command') {[, sequence, left, right] = request.postData().split(' ').map(Number); return route.fulfill({json: roverState('state')});}
    return route.fulfill({json: roverState('info')});
  });
  await page.goto(operator);
  await page.waitForSelector('body.capture-enabled');
  assert.equal(await page.locator('#capture-position').textContent(), 'No scan');
  await page.locator('#capture-new').click();
  await page.waitForFunction(() => document.getElementById('capture-position').textContent === 'Position 01');
  await page.locator('#connect').click();
  await page.waitForFunction(() => document.getElementById('connection-state').textContent === 'Connected');
  await page.locator('#capture-photo').click();
  await until(async () => (await state()).capture?.status === 'queued', 'Capture must queue on real backend');
  assert.equal(await page.locator('.drive-key').first().isDisabled(), true);
  const movementBefore = roverRequests.filter(request => request.path === '/api/command').length;
  await page.keyboard.down('w'); await wait(150); await page.keyboard.up('w');
  assert.equal(roverRequests.filter(request => request.path === '/api/command').length, movementBefore, 'Capture must not permit motion');
  const first = (await commands()).commands[0];
  assert.ok(first.request_id);
  const saved = await upload(first); assert.equal(saved.status(), 200);
  await page.waitForFunction(() => document.getElementById('capture-count').textContent === '1 photo');
  await page.waitForFunction(() => !document.querySelector('.drive-key').disabled);
  const photo = (await saved.json()).photo;
  assert.equal((await api.get(operator + photo.url)).headers()['content-type'], 'image/jpeg');
  assert.equal((await api.get(operator + photo.thumbnail_url)).status(), 200);
  assert.equal(await page.locator('#capture-photos a').count(), 1);
  await page.locator('#capture-finish').click();
  await until(async () => (await state()).active_station_id === null, 'Finishing clears active station');
  await page.locator('#capture-next').click();
  await page.waitForFunction(() => document.getElementById('capture-position').textContent === 'Position 02');

  // Commit a capture to the actual server, lose its response, and temporarily
  // deny cancellation. The UI must remain stopped until cancellation is proved.
  let denyCancellation = true, lostCommand;
  await page.route(operator + '/capture/api/capture/cancel', route => denyCancellation ? route.fulfill({status: 503, json: {error: 'Fixture cancellation outage'}}) : route.continue());
  await page.route(operator + '/capture/api/capture', async route => {
    const response = await route.fetch(); lostCommand = (await response.json()).capture;
    await route.abort('failed');
  });
  await page.locator('#capture-photo').click();
  await until(() => !!lostCommand, 'Unknown-outcome fixture must have queued capture');
  await page.waitForFunction(() => document.getElementById('capture-error').textContent.includes('Cancellation not confirmed'));
  assert.equal((await state()).capture.status, 'queued');
  assert.equal(await page.locator('.drive-key').first().isDisabled(), true, 'Lost POST response cannot resume driving');
  denyCancellation = false;
  await until(async () => (await state()).capture.status === 'cancelled', 'Cancellation retries until backend acknowledgement');
  await page.waitForFunction(() => !document.querySelector('.drive-key').disabled);
  assert.equal((await upload(lostCommand)).status(), 409, 'A late image for a cancelled capture is rejected');
  await page.unroute(operator + '/capture/api/capture');
  await page.unroute(operator + '/capture/api/capture/cancel');

  // Cancellation must also reserve a request that has not reached the server.
  // Releasing the delayed original POST after driving resumes cannot queue it.
  let releaseDelayed, delayedRequestId, delayedResult;
  const delayedGate = new Promise(resolve => {releaseDelayed = resolve;});
  await page.route(operator + '/capture/api/capture', async route => {
    delayedRequestId = JSON.parse(route.request().postData()).request_id;
    await delayedGate;
    const response = await route.fetch(); delayedResult = await response.json();
    await route.fulfill({response});
  });
  await page.locator('#capture-photo').click();
  await until(() => !!delayedRequestId, 'Delayed capture request must reach browser transport');
  await page.locator('#capture-photo').click();
  await page.waitForFunction(() => document.getElementById('capture-photo-label').textContent === 'Capture');
  releaseDelayed();
  await until(() => !!delayedResult, 'Delayed request must eventually receive its tombstone');
  assert.equal(delayedResult.capture.status, 'cancelled');
  assert.equal(delayedResult.capture.request_id, delayedRequestId);
  assert.equal((await commands()).commands.length, 0, 'Delayed original request cannot queue after cancellation');
  assert.equal((await upload(delayedResult.capture)).status(), 409);
  await page.unroute(operator + '/capture/api/capture');

  // Reload with a capture queued by another instance. State reconciliation must
  // acquire the drive lock before the phone can finish that transaction.
  const reloadRequest = await api.post(operator + '/capture/api/capture', {data: {station_id: (await state()).active_station_id, request_id: 'reload-fixture'}});
  assert.equal(reloadRequest.status(), 202);
  await page.reload();
  await page.waitForFunction(() => document.getElementById('capture-photo-label').textContent === 'Cancel');
  assert.equal(await page.locator('.drive-key').first().isDisabled(), true);
  await page.locator('#capture-photo').click();
  await until(async () => (await state()).capture.status === 'cancelled', 'Queued capture adopted after reload can be cancelled');
  await page.waitForFunction(() => document.getElementById('capture-photo-label').textContent === 'Capture');

  // Camera failures are terminal, and navigation away sends a cancellation.
  await page.locator('#capture-photo').click();
  await until(async () => (await state()).capture.status === 'queued', 'Failure fixture must queue');
  const failed = (await state()).capture;
  await api.post(phone + '/capture/phone/error', {headers: phoneHeaders, data: {command_id: failed.id, error: 'Fixture camera failure'}});
  await page.waitForFunction(() => document.getElementById('capture-error').textContent === 'Fixture camera failure');
  await page.locator('#capture-photo').click();
  await until(async () => (await state()).capture.status === 'queued', 'Pagehide fixture must queue');
  await page.goto('about:blank');
  await until(async () => (await state()).capture.status === 'cancelled', 'Pagehide must request cancellation');
  const current = await state();
  const exported = await api.get(operator + current.export_url);
  assert.equal(exported.status(), 200);
  assert.equal(exported.headers()['content-type'], 'application/zip');
  assert.equal((await exported.body()).subarray(0, 2).toString(), 'PK');
  await page.goto(operator);
  await page.waitForSelector('body.capture-enabled');
  for (const width of [390, 760, 1000, 1440]) {
    await page.setViewportSize({width, height: 1000});
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false, `No horizontal overflow at ${width}px`);
  }
  assert.deepEqual(pageErrors, []);
  assert.equal(stderr, '', 'Actual capture handlers must not emit server errors');
  console.log('Capture browser integration passed: real filesystem backend, fake phone pairing/preview/upload, capture stop lock, position groups, thumbnails/export, lost-response cancellation recovery, delayed-request cancellation tombstone, rejected late upload, reload adoption, camera failure, pagehide cancellation, responsive layout. No hardware used.');
} finally {
  clearInterval(heartbeat);
  await browser?.close(); await api?.dispose();
  server.kill('SIGTERM');
  if (server.exitCode === null && server.signalCode === null) await once(server, 'exit');
  await rm(temporary, {recursive: true, force: true});
}
