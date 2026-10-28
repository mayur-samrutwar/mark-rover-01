import {createPhoneVideo} from './phone-video.mjs';

const $ = id => document.getElementById(id);
const video = $('camera-video');
const TOKEN_KEY = 'rover-capture-phone-token';
const device = navigator.userAgentData?.platform || navigator.userAgent;
const previewCanvas = document.createElement('canvas');
const frameCanvas = document.createElement('canvas');
let token = sessionStorage.getItem(TOKEN_KEY) || '';
let stream = null;
let imageCapture = null;
let cameraStarting = false;
let cameraGeneration = 0;
let cameraError = '';
let previewError = '';
let connected = false;
let commandPolling = false;
let statusPosting = false;
let previewPosting = false;
let lastPreviewAttempt = -Infinity;
let captureBusy = false;
let wakeLock = null;
let capturedCount = 0;
let pairingBusy = false;
let ended = false;
const activeCommands = new Set();
const finishedCommands = new Set();
const phoneVideo = createPhoneVideo({
  request,
  onState: ({error = ''}) => {
    if (previewError === error) return;
    previewError = error;
    void postStatus();
  },
  getSource: () => ({
    stream, generation: cameraGeneration,
    ready: Boolean(token) && stream?.getVideoTracks()[0]?.readyState === 'live' && !document.hidden && !ended,
  }),
});

function cameraReady() {
  const track = stream?.getVideoTracks()[0];
  return !document.hidden && Boolean(track && track.readyState === 'live' && !track.muted && video.readyState >= 2 && video.videoWidth && video.videoHeight);
}

function feedback(message = '', error = false, help = '') {
  $('feedback').textContent = message;
  $('feedback').classList.toggle('error', error);
  $('help').textContent = help;
  $('help').hidden = !help;
}

function paint() {
  const ready = cameraReady();
  $('connection-state').textContent = !token ? 'Not paired' : connected ? 'Paired' : 'Reconnecting';
  $('connection-dot').className = `dot ${token ? connected ? 'online' : 'busy' : ''}`;
  $('pairing-form').hidden = Boolean(token);
  $('change-pairing').hidden = !token;
  $('pair-button').disabled = pairingBusy;
  $('pair-button').textContent = pairingBusy ? 'Pairing…' : 'Pair';
  $('start-camera').hidden = !token || Boolean(stream);
  $('start-camera').disabled = cameraStarting || !connected;
  $('start-camera').textContent = cameraStarting ? 'Opening…' : 'Start camera';
  $('stop-camera').hidden = !stream;
  $('camera-placeholder').hidden = Boolean(stream);
  document.querySelector('.camera').classList.toggle('live', Boolean(stream));
  $('resolution').textContent = stream ? `${video.videoWidth || '—'} × ${video.videoHeight || '—'} preview` : '—';
  $('camera-state').textContent = document.hidden ? 'Paused' : ready ? 'Ready' : stream ? 'Opening camera' : 'Rear camera';
  $('capture-count').textContent = `${capturedCount} captured`;
}

function unpair(message = 'Pair again using the code on your laptop.') {
  token = '';
  connected = false;
  sessionStorage.removeItem(TOKEN_KEY);
  stopCamera();
  feedback(message, Boolean(message));
  paint();
}

async function request(path, {method = 'GET', body, headers = {}, timeout = 8000} = {}) {
  const requestToken = token;
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeout);
  try {
    const response = await fetch(path, {
      method, body, cache: 'no-store', credentials: 'omit', signal: controller.signal,
      headers: {'X-Capture-Token': requestToken, ...headers},
    });
    const data = await response.json().catch(() => ({}));
    if (response.status === 401 || response.status === 403) {
      if (requestToken === token) unpair();
    }
    if (!response.ok) {
      const error = new Error(data.error || `Request failed (${response.status}).`);
      error.status = response.status;
      throw error;
    }
    if (requestToken === token && token && !connected) { connected = true; paint(); }
    return data;
  } catch (error) {
    if (!error.status && requestToken === token && token && connected) { connected = false; paint(); }
    throw error;
  } finally {
    clearTimeout(timer);
  }
}

function jsonRequest(path, body) {
  return request(path, {method: 'POST', body: JSON.stringify(body), headers: {'Content-Type': 'application/json'}});
}

async function postStatus() {
  if (!token || statusPosting || ended) return;
  statusPosting = true;
  try {
    await jsonRequest('/capture/phone/status', {
      camera_ready: cameraReady(), width: video.videoWidth || 0, height: video.videoHeight || 0,
      visibility: document.visibilityState, device, error: cameraError || (previewError ? `Video preview: ${previewError}` : ''),
    });
  } catch { /* The pairing indicator shows connectivity; the next heartbeat retries. */ }
  finally { statusPosting = false; }
}

async function acquireWakeLock() {
  if (!cameraReady() || wakeLock || !navigator.wakeLock) return;
  try {
    const held = await navigator.wakeLock.request('screen');
    if (!cameraReady()) { await held.release(); return; }
    wakeLock = held;
    held.addEventListener('release', () => { if (wakeLock === held) wakeLock = null; });
  } catch { /* Unsupported or declined wake locks do not prevent capture. */ }
}

function releaseWakeLock() {
  const held = wakeLock;
  wakeLock = null;
  if (held) void held.release().catch(() => {});
}

function cameraFailureMessage(error) {
  if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) return ['Camera access needs HTTPS.', 'Open the HTTPS phone address shown on your laptop. Install and trust the supplied local certificate on this phone first.'];
  if (error.name === 'NotAllowedError' || error.name === 'PermissionDeniedError') return ['Camera permission was denied.', 'Allow Camera in Chrome’s site settings for this address, then tap Start camera again.'];
  if (error.name === 'NotFoundError') return ['No camera is available.', 'Check that this page is open on the phone and that a camera is available.'];
  if (error.name === 'NotReadableError') return ['The camera is being used elsewhere.', 'Close other camera apps or browser tabs, then try again.'];
  return ['Could not open the camera.', error.message || 'Check camera access and try again.'];
}

async function startCamera() {
  if (!token || stream || cameraStarting) return;
  cameraStarting = true;
  cameraError = '';
  feedback();
  paint();
  const generation = ++cameraGeneration;
  let nextStream = null;
  try {
    if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) throw new Error('Camera access is unavailable.');
    nextStream = await navigator.mediaDevices.getUserMedia({audio: false, video: {facingMode: {ideal: 'environment'}, width: {ideal: 1920}, height: {ideal: 1080}, frameRate: {ideal: 30}}});
    if (generation !== cameraGeneration || !token || ended) { nextStream.getTracks().forEach(track => track.stop()); return; }
    stream = nextStream;
    video.srcObject = stream;
    await video.play();
    const track = stream.getVideoTracks()[0];
    imageCapture = null;
    if (typeof ImageCapture === 'function') {
      try { imageCapture = new ImageCapture(track); }
      catch { /* A regular video frame remains available when ImageCapture is unsupported. */ }
    }
    track.addEventListener('ended', () => {
      if (stream !== nextStream) return;
      stopCamera();
      cameraError = 'Camera disconnected.';
      feedback('Camera disconnected. Tap Start camera to reconnect.', true);
      void postStatus();
    });
    track.addEventListener('mute', () => { paint(); void postStatus(); });
    track.addEventListener('unmute', () => { paint(); void postStatus(); });
    await acquireWakeLock();
    feedback('Keep this page open while capturing.');
  } catch (error) {
    if (nextStream) nextStream.getTracks().forEach(track => track.stop());
    if (generation === cameraGeneration) {
      stream = null; imageCapture = null; video.srcObject = null;
      const [message, help] = cameraFailureMessage(error);
      cameraError = message;
      feedback(message, true, help);
    }
  } finally {
    cameraStarting = false;
    paint();
    void postStatus();
  }
}

function stopCamera() {
  ++cameraGeneration;
  phoneVideo.close();
  const previous = stream;
  stream = null;
  imageCapture = null;
  video.srcObject = null;
  previous?.getTracks().forEach(track => track.stop());
  releaseWakeLock();
  paint();
  void postStatus();
}

function canvasBlob(canvas, quality) {
  return new Promise((resolve, reject) => canvas.toBlob(blob => blob ? resolve(blob) : reject(new Error('Could not encode the image.')), 'image/jpeg', quality));
}

function boundedSize(width, height, maxEdge) {
  const scale = Math.min(1, maxEdge / Math.max(width, height));
  return [Math.max(1, Math.round(width * scale)), Math.max(1, Math.round(height * scale))];
}

async function preview() {
  if (!token || !cameraReady() || previewPosting || captureBusy || ended) return;
  // A direct stream serves one viewer. Keep other open control tabs supplied
  // with fresh fallback frames without encoding JPEGs at the full video rate.
  const now = performance.now();
  if (phoneVideo.isStreaming() && now - lastPreviewAttempt < 1000) return;
  lastPreviewAttempt = now;
  previewPosting = true;
  try {
    [previewCanvas.width, previewCanvas.height] = boundedSize(video.videoWidth, video.videoHeight, 640);
    previewCanvas.getContext('2d').drawImage(video, 0, 0, previewCanvas.width, previewCanvas.height);
    const blob = await canvasBlob(previewCanvas, .64);
    if (blob.size <= 512 * 1024 && cameraReady()) {
      await request('/capture/phone/preview', {method: 'POST', body: blob, headers: {'Content-Type': 'image/jpeg'}, timeout: 3000});
    }
  } catch { /* A dropped preview is expendable; commands and original photos are separate. */ }
  finally { previewPosting = false; }
}

async function takePhoto() {
  if (!cameraReady()) throw new Error('Camera is not ready. Keep the phone page open.');
  const settings = stream.getVideoTracks()[0].getSettings();
  const capturedAt = new Date().toISOString();
  let blob = null;
  let source = 'video-frame';
  if (imageCapture) {
    try { blob = await imageCapture.takePhoto(); source = 'image-capture'; }
    catch { /* Some Android camera configurations expose ImageCapture but reject takePhoto. */ }
  }
  if (!blob) {
    if (!cameraReady()) throw new Error('Camera became unavailable before capture.');
    [frameCanvas.width, frameCanvas.height] = boundedSize(video.videoWidth, video.videoHeight, 4096);
    frameCanvas.getContext('2d').drawImage(video, 0, 0, frameCanvas.width, frameCanvas.height);
    blob = await canvasBlob(frameCanvas, .93);
  }
  let width, height;
  const bitmap = await createImageBitmap(blob);
  try {
    width = bitmap.width; height = bitmap.height;
    if (blob.type !== 'image/jpeg' || Math.max(width, height) > 4096) {
      [width, height] = boundedSize(width, height, 4096);
      frameCanvas.width = width; frameCanvas.height = height;
      frameCanvas.getContext('2d').drawImage(bitmap, 0, 0, width, height);
      blob = await canvasBlob(frameCanvas, .93);
    }
  } finally { bitmap.close(); }
  if (blob.size > 20 * 1024 * 1024) throw new Error('Captured image exceeds the 20 MB upload limit.');
  return {blob, metadata: {width, height, captured_at: capturedAt, camera_settings: settings, source, device}};
}

const delay = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds));

async function reportCaptureError(command, message) {
  if (!token) return;
  try { await jsonRequest('/capture/phone/error', {command_id: command.id, error: message}); }
  catch { /* Expired or disconnected commands will also expire at the server. */ }
}

function rememberFinished(id) {
  finishedCommands.add(id);
  if (finishedCommands.size > 1000) finishedCommands.delete(finishedCommands.values().next().value);
}

async function handleCapture(command) {
  const id = String(command.id);
  if (captureBusy || activeCommands.has(id) || finishedCommands.has(id)) return;
  activeCommands.add(id);
  captureBusy = true;
  const captureToken = token;
  try {
    if (!Number.isFinite(command.expires_at) || command.expires_at <= Date.now() / 1000) throw new Error('Capture request expired.');
    $('upload-state').textContent = 'Capturing';
    const photo = await takePhoto();
    $('capture-flash').classList.remove('flash');
    void $('capture-flash').offsetWidth;
    $('capture-flash').classList.add('flash');
    let uploadError;
    for (let attempt = 0; attempt < 8; attempt++) {
      if (!token || token !== captureToken || ended) throw new Error('Capture pairing changed before upload.');
      const remaining = command.expires_at * 1000 - Date.now();
      if (remaining <= 0) throw new Error('Upload expired. Keep the connection available and capture again.');
      $('upload-state').textContent = attempt ? 'Retrying upload' : 'Uploading';
      try {
        await request(`/capture/phone/upload?command_id=${encodeURIComponent(id)}`, {
          method: 'POST', body: photo.blob,
          headers: {'Content-Type': 'image/jpeg', 'X-Frame-Metadata': JSON.stringify(photo.metadata)},
          timeout: Math.max(1, Math.min(15000, remaining)),
        });
        rememberFinished(id);
        capturedCount++;
        $('upload-state').textContent = 'Saved on laptop';
        feedback(`${photo.metadata.width} × ${photo.metadata.height} · saved`);
        paint();
        return;
      } catch (error) {
        uploadError = error;
        if (error.status && error.status < 500 && error.status !== 408 && error.status !== 429) throw error;
        const pause = Math.min(4000, 500 * 2 ** attempt);
        if (Date.now() + pause >= command.expires_at * 1000) throw error;
        await delay(pause);
      }
    }
    throw uploadError || new Error('Upload failed.');
  } catch (error) {
    // Retrying an upload reuses one exposure. Failed requests must never trigger
    // another shutter action when the command is redelivered by the server.
    rememberFinished(id);
    $('upload-state').textContent = 'Capture failed';
    feedback(error.message || 'Capture failed.', true);
    if (token === captureToken) await reportCaptureError(command, error.message || 'Capture failed.');
  } finally {
    activeCommands.delete(id);
    captureBusy = false;
  }
}

async function pollCommands() {
  if (!token || commandPolling || captureBusy || ended) return;
  commandPolling = true;
  try {
    const data = await request('/capture/phone/commands', {timeout: 4000});
    for (const command of data.commands || []) {
      if (command.type !== 'capture' || !command.id) continue;
      if (finishedCommands.has(String(command.id))) continue;
      await handleCapture(command);
      break;
    }
  } catch { /* The connection indicator and subsequent poll handle interruption. */ }
  finally { commandPolling = false; }
}

$('pairing-form').addEventListener('submit', async event => {
  event.preventDefault();
  const code = $('pairing-code').value.replace(/\D/g, '');
  if (!/^\d{6}$/.test(code) || pairingBusy) return;
  pairingBusy = true;
  feedback();
  paint();
  try {
    const data = await request('/capture/phone/pair', {method: 'POST', body: JSON.stringify({code, device}), headers: {'Content-Type': 'application/json'}});
    if (typeof data.token !== 'string' || !data.token) throw new Error('Pairing response was incomplete.');
    token = data.token;
    sessionStorage.setItem(TOKEN_KEY, token);
    connected = true;
    cameraError = '';
    feedback();
    void postStatus();
  } catch (error) { feedback(error.message || 'Could not pair.', true); }
  finally { pairingBusy = false; paint(); }
});

$('pairing-code').addEventListener('input', () => { $('pairing-code').value = $('pairing-code').value.replace(/\D/g, '').slice(0, 6); });
$('change-pairing').addEventListener('click', () => unpair(''));
$('start-camera').addEventListener('click', () => void startCamera());
$('stop-camera').addEventListener('click', () => { cameraError = ''; stopCamera(); feedback(); });
video.addEventListener('loadeddata', () => { paint(); void postStatus(); void acquireWakeLock(); });
video.addEventListener('resize', () => { paint(); void postStatus(); });
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { releaseWakeLock(); phoneVideo.close(); }
  else void acquireWakeLock();
  paint();
  void postStatus();
});
window.addEventListener('online', () => { void postStatus(); void pollCommands(); });
window.addEventListener('offline', () => { connected = false; paint(); });
window.addEventListener('pagehide', () => {
  ended = true;
  stopCamera();
  if (token) void fetch('/capture/phone/status', {method: 'POST', keepalive: true, headers: {'Content-Type': 'application/json', 'X-Capture-Token': token}, body: JSON.stringify({camera_ready: false, width: 0, height: 0, visibility: 'hidden', device, error: ''})}).catch(() => {});
});
window.addEventListener('pageshow', () => { ended = false; paint(); void postStatus(); });

const prefilledCode = new URL(location.href).searchParams.get('code');
if (/^\d{6}$/.test(prefilledCode || '')) $('pairing-code').value = prefilledCode;
if (!window.isSecureContext) feedback('Camera access needs HTTPS.', true, 'Open the HTTPS phone address shown on your laptop and trust its local certificate.');
paint();
void postStatus();
setInterval(() => void pollCommands(), 300);
setInterval(() => void postStatus(), 2000);
setInterval(() => void preview(), 300);

setInterval(() => void phoneVideo.poll(), 500);
