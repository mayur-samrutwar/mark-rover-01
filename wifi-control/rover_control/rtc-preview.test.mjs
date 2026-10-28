// Real browser-to-browser WebRTC over isolated HTTP/TLS capture servers.
// Video comes from an animated canvas; original photographs are fixture JPEGs.
// Rover requests are intercepted and the server refuses outbound network calls.
import assert from 'node:assert/strict';
import { spawn, spawnSync } from 'node:child_process';
import { mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { once } from 'node:events';
import { createHash } from 'node:crypto';

const { chromium, request: apiRequest } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.dirname(fileURLToPath(import.meta.url));
const temporary = await mkdtemp(path.join(tmpdir(), 'rover-rtc-preview-'));
const certificate = path.join(temporary, 'certificate.pem'), key = path.join(temporary, 'key.pem');
const openssl = spawnSync('openssl', ['req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '1', '-subj', '/CN=localhost', '-keyout', key, '-out', certificate], {encoding: 'utf8'});
assert.equal(openssl.status, 0, 'OpenSSL must create a temporary test certificate');
const harness = String.raw`
import base64, io, json, ssl, sys, threading
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, sys.argv[1])
from PIL import Image
from capture_server import CaptureStore
import serve

def reject_hardware(*args, **kwargs):
    raise AssertionError('WebRTC tests must not contact the real rover')
serve.rover_request = reject_hardware
temporary = Path(sys.argv[2])
config = SimpleNamespace(port=0, https_port=0, setup_port=0, rover='http://must-not-contact.invalid')
store = CaptureStore(temporary / 'captures', pairing={'code': '123456'})
tls = {'host': '127.0.0.1'}
operator = serve.CaptureHTTPServer(('127.0.0.1', 0), serve.make_handler(config, store))
config.port = operator.server_address[1]
phone = serve.CaptureHTTPServer(('127.0.0.1', 0), serve.make_handler(config, store, 'phone', tls))
config.https_port = phone.server_address[1]
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(str(temporary / 'certificate.pem'), str(temporary / 'key.pem'))
phone.socket = context.wrap_socket(phone.socket, server_side=True)
store.pairing.update(url=f'https://127.0.0.1:{config.https_port}/phone/?code=123456', setup_url=f'http://127.0.0.1:{config.port}/')
image = Image.new('RGB', (1600, 1200), '#809087')
buffer = io.BytesIO(); image.save(buffer, format='JPEG')
for server in (operator, phone):
    threading.Thread(target=server.serve_forever, daemon=True).start()
print(json.dumps({'operator': f'http://127.0.0.1:{config.port}', 'phone': f'https://127.0.0.1:{config.https_port}', 'jpeg': base64.b64encode(buffer.getvalue()).decode()}), flush=True)
threading.Event().wait()
`;
const server = spawn(process.env.PYTHON || 'python3', ['-u', '-c', harness, root, temporary], {env: {...process.env, PYTHONDONTWRITEBYTECODE: '1'}, stdio: ['ignore', 'pipe', 'pipe']});
let stderr = '', browser, api;
server.stderr.on('data', data => {stderr += data;});
const ready = new Promise((resolve, reject) => {
  let output = '';
  server.stdout.on('data', data => {
    output += data;
    if (output.includes('\n')) {
      try { resolve(JSON.parse(output.split('\n')[0])); } catch (error) { reject(error); }
    }
  });
  server.once('exit', code => reject(new Error(`Isolated server exited (${code}): ${stderr}`)));
});
const wait = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds));
async function until(predicate, message, timeout = 20000) {
  const end = Date.now() + timeout;
  while (Date.now() < end) {
    if (await predicate()) return;
    await wait(100);
  }
  assert.fail(message);
}

try {
  let startupTimer;
  const addresses = await Promise.race([
    ready,
    new Promise((_, reject) => {startupTimer = setTimeout(() => reject(new Error(`Server startup timed out: ${stderr}`)), 10000);}),
  ]).finally(() => clearTimeout(startupTimer));
  const {operator, phone, jpeg} = addresses;
  api = await apiRequest.newContext({ignoreHTTPSErrors: true});
  browser = await chromium.launch({
    headless: true,
    ...(process.env.CHROMIUM_PATH ? {executablePath: process.env.CHROMIUM_PATH} : {}),
    args: ['--autoplay-policy=no-user-gesture-required', '--disable-background-timer-throttling', '--disable-renderer-backgrounding'],
  });
  const context = await browser.newContext({ignoreHTTPSErrors: true, viewport: {width: 1440, height: 1000}});
  const pageErrors = [], roverRequests = [];
  await context.addInitScript(({fixtureJPEG}) => {
    Object.defineProperty(document, 'hidden', {get: () => false});
    Object.defineProperty(document, 'visibilityState', {get: () => 'visible'});
    document.hasFocus = () => true;
    window.__rtcPeers = [];
    window.__blockRTC = false;
    const NativePeer = window.RTCPeerConnection;
    window.RTCPeerConnection = class extends NativePeer {
      constructor(configuration) {
        if (window.__blockRTC) throw new Error('Fixture direct-video outage');
        super(configuration);
        window.__rtcPeers.push(this);
      }
    };
    if (location.protocol !== 'https:') return;
    navigator.mediaDevices.getUserMedia = async () => {
      const canvas = document.createElement('canvas');
      canvas.width = 640; canvas.height = 360;
      const ctx = canvas.getContext('2d');
      let frame = 0;
      const paint = () => {
        ctx.fillStyle = '#204050'; ctx.fillRect(0, 0, 640, 360);
        ctx.fillStyle = '#ffcc55'; ctx.fillRect((frame++ * 8) % 580, 130, 60, 100);
        ctx.fillStyle = '#ffffff'; ctx.font = '32px sans-serif'; ctx.fillText(String(frame), 20, 50);
      };
      paint();
      const timer = setInterval(paint, 1000 / 30);
      const stream = canvas.captureStream(30);
      stream.getVideoTracks()[0].addEventListener('ended', () => clearInterval(timer));
      return stream;
    };
    window.ImageCapture = class {
      async takePhoto() {
        const bytes = Uint8Array.from(atob(fixtureJPEG), character => character.charCodeAt(0));
        return new Blob([bytes], {type: 'image/jpeg'});
      }
    };
  }, {fixtureJPEG: jpeg});
  const phonePage = await context.newPage();
  const operatorPage = await context.newPage();
  for (const page of [phonePage, operatorPage]) page.on('pageerror', error => pageErrors.push(error.message));
  await operatorPage.route(operator + '/api/**', async route => {
    const pathname = new URL(route.request().url()).pathname;
    roverRequests.push(pathname);
    const type = pathname === '/api/stop' ? 'stopped' : 'info';
    return route.fulfill({json: {protocol: 'rover-wifi-v1', type, ready: true, armed: false, seq: 0, left: 0, right: 0}});
  });
  await phonePage.goto(phone + '/phone/?code=123456');
  await phonePage.locator('#pair-button').click();
  await phonePage.locator('#start-camera').click();
  await phonePage.waitForFunction(() => document.getElementById('camera-state').textContent === 'Ready');
  await operatorPage.goto(operator);
  await operatorPage.waitForSelector('body.capture-enabled');
  const state = async () => (await api.get(operator + '/capture/api/state')).json();
  const rtc = async () => (await api.get(operator + '/capture/api/rtc')).json();
  const live = async () => operatorPage.evaluate(() => {
    const video = document.getElementById('capture-video');
    return Boolean(video && !video.hidden && video.readyState >= 2 && video.videoWidth);
  });
  const decoded = async () => operatorPage.evaluate(async () => {
    const peer = window.__rtcPeers.findLast(peer => peer.connectionState === 'connected');
    if (!peer) return null;
    const reports = await peer.getStats();
    const incoming = [...reports.values()].find(report => report.type === 'inbound-rtp' && (report.kind || report.mediaType) === 'video');
    return incoming ? {frames: incoming.framesDecoded, bytes: incoming.bytesReceived, timestamp: incoming.timestamp} : null;
  });
  await until(live, 'Direct video must deliver decoded frames to the actual operator page');
  assert.equal(await operatorPage.locator('#capture-preview-label').textContent(), 'Live');
  await until(async () => (await rtc()).rtc?.operator_state === 'playing', 'Viewer must report verified playing state');
  const firstGeneration = (await rtc()).rtc.connection_id;
  const before = await decoded();
  assert.ok(before && Number.isFinite(before.frames));
  await wait(3200);
  const after = await decoded();
  const fps = (after.frames - before.frames) * 1000 / (after.timestamp - before.timestamp);
  assert.ok(after.bytes > before.bytes && after.frames > before.frames, 'Real RTP video bytes and decoded frames must advance');
  assert.ok(fps > 15, `Animated local video should exceed 15 decoded fps; observed ${fps.toFixed(1)}`);
  // A second viewer cannot own the single direct stream, but must receive
  // advancing JPEG previews while the first viewer continues playing video.
  const secondViewer = await context.newPage();
  await secondViewer.goto(operator);
  await until(async () => await secondViewer.locator('#capture-preview').isVisible()
    && await secondViewer.locator('#capture-preview-label').textContent() === 'Preview',
    'A second viewer must receive a fresh fallback preview during live video');
  const sharedPreviewSeq = (await state()).preview.seq;
  await until(async () => (await state()).preview.seq > sharedPreviewSeq,
    'Fallback frames must keep advancing while another viewer owns direct video');
  assert.ok(await live(), 'The first viewer must retain direct video');
  await secondViewer.getByRole('button', {name: 'Switch to full-speed live video'}).click();
  await until(async () => await secondViewer.locator('#capture-preview-label').textContent() === 'Live',
    'A viewer must be able to explicitly take over full-speed video');
  assert.notEqual((await rtc()).rtc.connection_id, firstGeneration);
  await secondViewer.close();
  await until(live, 'The first viewer must recover direct video after the second closes');
  for (const page of [phonePage, operatorPage]) {
    const configurations = await page.evaluate(() => window.__rtcPeers.map(peer => peer.getConfiguration().iceServers));
    assert.ok(configurations.length);
    assert.ok(configurations.every(servers => servers.length === 0), 'No external STUN or TURN service may be configured');
  }

  // Native still capture remains separate from the bounded video stream.
  await operatorPage.locator('#capture-new').click();
  await operatorPage.waitForFunction(() => document.getElementById('capture-position').textContent === 'Position 01');
  await operatorPage.locator('#capture-photo').click();
  await until(async () => (await state()).capture?.status === 'complete', 'Still photograph must commit through the unchanged upload transaction');
  const photo = (await state()).stations[0].photos[0];
  assert.equal(photo.width, 1600); assert.equal(photo.height, 1200);
  const original = await (await api.get(operator + photo.url)).body();
  assert.equal(createHash('sha256').update(original).digest('hex'), createHash('sha256').update(Buffer.from(jpeg, 'base64')).digest('hex'));
  await until(live, 'Still capture must leave the live camera stream usable');

  // Reload establishes a new generation without restarting the phone camera.
  await operatorPage.reload();
  await until(live, 'Reloaded operator page must establish a new direct-video connection');
  assert.notEqual((await rtc()).rtc.connection_id, firstGeneration);
  const secondGeneration = (await rtc()).rtc.connection_id;
  await phonePage.locator('#stop-camera').click();
  await until(async () => !(await state()).phone.camera_ready, 'Camera stop must reach the server');
  await until(async () => !await live(), 'Stopped source cannot retain a Live preview');
  await phonePage.locator('#start-camera').click();
  await until(live, 'Camera restart must renegotiate direct video');
  assert.notEqual((await rtc()).rtc.connection_id, secondGeneration);

  // Force peer construction to fail after disconnect; JPEG fallback must keep
  // updating and later return to direct video without touching motor controls.
  await operatorPage.evaluate(() => {window.__blockRTC = true; window.__rtcPeers.forEach(peer => peer.close());});
  await until(async () => !await live(), 'Closed direct connection must leave Live mode');
  await until(async () => {
    const preview = await operatorPage.locator('#capture-preview').getAttribute('src');
    return !!preview && await operatorPage.locator('#capture-preview').isVisible();
  }, 'JPEG fallback must be displayed during the direct-video outage');
  const previewBefore = (await state()).preview.seq;
  await until(async () => (await state()).preview.seq > previewBefore, 'Fallback JPEG frames must continue arriving');
  await operatorPage.evaluate(() => {window.__blockRTC = false;});
  await until(live, 'Direct video must recover after the temporary outage', 30000);
  assert.equal(await operatorPage.locator('#capture-preview-label').textContent(), 'Live');
  assert.deepEqual(pageErrors, [], 'Both real pages must remain free of uncaught JavaScript errors');
  assert.ok(!roverRequests.includes('/api/command') && !roverRequests.includes('/api/session'), 'Preview tests must not initiate driving');
  console.log(JSON.stringify({test: 'WebRTC live preview', decoded_fps: Number(fps.toFixed(1)), original: `${photo.width}x${photo.height}`, reload: 'passed', camera_restart: 'passed', jpeg_fallback: 'passed', recovery: 'passed', external_ice_servers: 0, hardware_motion: false}));
} finally {
  await api?.dispose();
  await browser?.close();
  const exited = once(server, 'exit').catch(() => {});
  server.kill('SIGTERM');
  await Promise.race([exited, wait(2000)]);
  if (server.exitCode === null) server.kill('SIGKILL');
  await rm(temporary, {recursive: true, force: true});
}
