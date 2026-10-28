import { DRIVE_KEYS, mixKeys, directionName } from './controls.mjs';
import { WiFiLink } from './wifi-link.mjs';

const $ = id => document.getElementById(id);
const keyboard = new Set(), pointers = new Map();
const keyAliases = {ArrowUp:'KeyW',ArrowLeft:'KeyA',ArrowDown:'KeyS',ArrowRight:'KeyD'};
const state = {
  port: null, queue: null,
  ready: false, latched: false, connecting: false, closing: false,
  sequence: 0, lastAck: 0, lastSequence: 0, left: 0, right: 0,
  session: false, arming: false, polling: null, nextPoll: 0, generation: 0,
  idleFailures: 0, reconnecting: false, captureLocked: false,
};
let handshakeTimer;
const keys = () => new Set([...keyboard].map(code=>keyAliases[code] || code).concat([...pointers.values()]));

function log(message) {
  console.info(message);
}
function showConnection(title, detail, kind = '') {
  $('connection-state').textContent = title;
  $('connection-state').title = detail;
  $('connection-dot').className = 'dot ' + kind;
}
function paint() {
  const held = keys();
  document.querySelectorAll('.drive-key').forEach(button => {
    button.classList.toggle('active', state.ready && held.has(button.dataset.code));
    button.disabled = !state.ready || state.captureLocked;
  });
  $('connect').disabled = state.connecting || state.closing || state.captureLocked;
  $('connect').textContent = state.port ? 'Disconnect' : 'Connect';
  $('emergency').disabled = !state.port;
  $('resume').hidden = !state.latched || !state.port;
  $('resume').disabled = state.captureLocked;
  $('speed').disabled = state.latched || state.captureLocked;
  document.body.classList.toggle('latched', state.latched);
  $('motion').textContent = state.latched ? 'Stopped / locked' : directionName(state.left, state.right);
}
function command() {
  if (!state.ready || state.latched || state.captureLocked || !state.queue) return;
  const [left, right] = document.hidden || !document.hasFocus() ? [0, 0] : mixKeys(keys(), Number($('speed').value));
  // Idle is a connected network link, not an active motor lease. Release sends
  // STOP immediately. A fresh handshake is required for the next held input.
  if (!left && !right) {
    if (state.session) {
      state.session = false;
      state.left = state.right = 0;
      void state.port.stop();
    }
    return;
  }
  if (!state.session) {
    if (!state.arming) void synchronize(true);
    return;
  }
  if (++state.sequence > 2147483647) { emergency('Sequence limit reached. Resume to reconnect.'); return; }
  state.queue.send(`M ${state.sequence} ${left} ${right}`);
}
function releaseAll() {
  keyboard.clear(); pointers.clear(); command(); paint();
}
function emergency(reason = 'Emergency stop. Controls locked.') {
  const first = !state.latched;
  ++state.generation; state.polling = null;
  keyboard.clear(); pointers.clear();
  state.ready = false; state.latched = true;
  state.reconnecting = false;
  state.session = state.arming = false;
  state.left = state.right = 0;
  state.queue?.send('STOP');
  clearTimeout(handshakeTimer);
  showConnection('Stopped / locked', 'Release keys, then Resume controls to continue.', 'error');
  if (first) log(reason);
  paint();
}
function transportFailure(reason) {
  // Network recovery restores idle controls only. Operator emergency remains a
  // separate latch, and neither path retains a direction or a motor lease.
  if (!state.port || state.closing || state.latched) return;
  const stopLease = !state.reconnecting || state.session || state.arming;
  ++state.generation; state.polling = null;
  keyboard.clear(); pointers.clear();
  state.ready = false; state.reconnecting = true;
  state.session = state.arming = false;
  state.left = state.right = 0;
  clearTimeout(handshakeTimer);
  if (stopLease) void state.port.stop();
  showConnection('Reconnecting', reason + ' Checking the rover over Wi-Fi.');
  log(reason); paint();
}
function synchronize(preserveHeldInput = false) {
  if (!preserveHeldInput) { keyboard.clear(); pointers.clear(); }
  const generation = ++state.generation, link = state.port;
  state.polling = null; state.ready = false;
  state.latched = false; state.session = false; state.arming = true;
  state.reconnecting = false; state.idleFailures = 0;
  state.sequence = state.lastSequence = 0;
  state.left = state.right = 0;
  if (!preserveHeldInput) showConnection('Connecting', 'Checking the rover on your home network…');
  const connection = state.queue.send('SYNC');
  clearTimeout(handshakeTimer);
  handshakeTimer = setTimeout(() => {
    if (state.arming && state.port === link && state.generation === generation)
      transportFailure('No Wi-Fi firmware response.');
  }, 2500);
  paint();
  return connection;
}
function receive(line) {
  let message;
  try { message = JSON.parse(line); } catch { return; }
  if (message.protocol !== 'rover-wifi-v1') return;
  if (message.type === 'hello') {
    clearTimeout(handshakeTimer);
    if (state.latched) { state.queue?.send('STOP'); return; }
    if (message.ready !== true || message.armed !== true || message.left !== 0 || message.right !== 0) {
      transportFailure('ESP32 motor control is unavailable.'); return;
    }
    state.ready = true; state.session = true; state.arming = false; state.lastAck = performance.now();
    $('response-age').textContent = '0 ms';
    state.left = state.right = 0;
    showConnection('Connected', 'Hold WASD or the on-screen buttons to drive.', 'online');
    log('ESP32 ready. Motors stopped.');
    command(); paint(); return;
  }
  if (message.type === 'timeout' || message.type === 'fault') {
    transportFailure(message.type === 'timeout' ? 'ESP32 watchdog stopped the rover.' : 'ESP32 rejected a command; stopped.'); return;
  }
  if (message.type === 'state' && state.ready) {
    if (!Number.isInteger(message.seq) || message.seq <= state.lastSequence || message.seq > state.sequence) return;
    if (!message.armed || !message.ready || !Number.isInteger(message.left) || !Number.isInteger(message.right) || Math.abs(message.left) > 255 || Math.abs(message.right) > 255) {
      transportFailure('Invalid rover state; stopped.'); return;
    }
    state.lastAck = performance.now(); state.lastSequence = message.seq;
    state.left = message.left; state.right = message.right;
    paint();
  }
}
async function disconnect() {
  if (state.closing) return;
  const link = state.port;
  ++state.generation; state.polling = null;
  state.closing = true; releaseAll(); state.ready = false;
  state.left = state.right = 0; clearTimeout(handshakeTimer); paint();
  await link?.close();
  if (state.port !== link) return;
  state.port = state.queue = null;
  state.session = state.arming = false;
  state.reconnecting = false; state.idleFailures = 0;
  state.closing = state.connecting = state.latched = false;
  showConnection('Disconnected', 'Connect to the rover over your home Wi-Fi.');
  log('Wi-Fi controls disconnected. Motors stopped.'); paint();
}
async function connect() {
  if (state.port) { emergency('Disconnected by operator.'); await disconnect(); return; }
  state.connecting = true;
  const link = new WiFiLink(message => {
    if (state.port === link && !state.closing) receive(JSON.stringify(message));
  }, error => {
    if (state.port === link && !state.closing) transportFailure('Wi-Fi link: ' + error.message);
  });
  state.port = state.queue = link;
  paint();
  try { await synchronize(); }
  finally { if (state.port === link && !state.closing) { state.connecting = false; paint(); } }
}

document.addEventListener('keydown', event => {
  const editable = event.target.closest?.('input, textarea, select, [contenteditable="true"]');
  if ((event.code === 'Space' || event.code === 'Escape') && state.port) {
    event.preventDefault(); emergency(); return;
  }
  if (!DRIVE_KEYS.has(keyAliases[event.code] || event.code) || editable || event.ctrlKey || event.metaKey || event.altKey) return;
  event.preventDefault();
  if (!state.ready || state.latched || state.captureLocked || event.repeat) return;
  keyboard.add(event.code); command(); paint();
});
document.addEventListener('keyup', event => {
  if (!DRIVE_KEYS.has(keyAliases[event.code] || event.code)) return;
  keyboard.delete(event.code); command(); paint();
});
window.addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
window.addEventListener('pagehide', () => { releaseAll(); state.port?.beaconStop(); });
document.addEventListener('focusin', event => {
  if (event.target.matches('input, textarea, select, [contenteditable="true"]')) releaseAll();
});
window.addEventListener('contextmenu', releaseAll);
for (const button of document.querySelectorAll('.drive-key')) {
  button.addEventListener('pointerdown', event => {
    if (!state.ready || state.latched || state.captureLocked || event.button !== 0) return;
    event.preventDefault(); button.setPointerCapture(event.pointerId);
    pointers.set(event.pointerId, button.dataset.code); command(); paint();
  });
  const release = event => { pointers.delete(event.pointerId); command(); paint(); };
  button.addEventListener('pointerup', release);
  button.addEventListener('pointercancel', release);
  button.addEventListener('lostpointercapture', release);
}
$('speed').addEventListener('input', () => {
  $('speed-value').value = `${Math.round(Number($('speed').value) / 255 * 100)}%`;
  $('speed').style.setProperty('--strength',`${Number($('speed').value) / 255 * 100}%`);
  command();
});
$('connect').addEventListener('click', connect);
$('emergency').addEventListener('click', () => emergency());
$('resume').addEventListener('click', () => { if (state.port && !state.closing) synchronize(); });
function pollIdleLink() {
  if (!state.port || state.latched || state.closing || state.connecting || state.session || state.arming || state.polling || performance.now() < state.nextPoll) return;
  const link = state.port, pollStarted = performance.now(), generation = state.generation;
  // A late response/finally from an older link or control attempt owns no UI state.
  const poll = {}; state.polling = poll; state.nextPoll = pollStarted + 1000;
  void link.request('/api/info',undefined,null,2000).then(message => {
    if (state.port !== link || state.generation !== generation || state.latched || state.closing || state.session || state.arming) return;
    if (message.protocol !== 'rover-wifi-v1' || message.ready !== true || message.armed !== false || message.left !== 0 || message.right !== 0) {
      transportFailure(message.armed ? 'Rover is busy; release its other controller.' : 'Rover motor control is unavailable.'); return;
    }
    state.lastAck = performance.now(); state.idleFailures = 0;
    $('response-age').textContent = `${Math.round(performance.now() - pollStarted)} ms`;
    if (state.reconnecting) {
      // Reconnection never restores a held direction or an armed motor session.
      keyboard.clear(); pointers.clear(); state.session = false;
      state.ready = true; state.reconnecting = false;
      showConnection('Connected','Hold a direction to drive.','online'); paint();
    }
  }).catch(() => {
    if (state.port !== link || state.generation !== generation || state.latched || state.closing || state.session || state.arming) return;
    // One slow idle request is not a lost Wi-Fi connection. Retry independently
    // of the motor watchdog; repeated failures disable input until health returns.
    if (++state.idleFailures >= 3) {
      transportFailure('Wi-Fi health checks failed.');
    }
  }).finally(() => { if (state.polling === poll) state.polling = null; });
}
setInterval(() => {
  if (state.ready) {
    const age = performance.now() - state.lastAck;
    if (state.session || state.arming) $('response-age').textContent = `${Math.round(age)} ms`;
    if (state.session && age > 500) { transportFailure('No rover response for 500 ms while driving.'); return; }
    command();
  } else $('response-age').textContent = '—';
  pollIdleLink();
}, 100);
// Capture never generates motion. Release held input, invalidate the motor
// lease and keep drive input disabled until the camera transaction finishes.
window.roverCaptureLock = async locked => {
  if (locked) {
    state.captureLocked = true;
    keyboard.clear(); pointers.clear();
    state.session = state.arming = false;
    state.left = state.right = 0;
    clearTimeout(handshakeTimer);
    paint();
    await state.port?.stop();
    // The firmware also disarms independently if STOP cannot be delivered.
    // Waiting past that bound avoids exposing while the last lease is live.
    await new Promise(resolve => setTimeout(resolve, 400));
  } else {
    keyboard.clear(); pointers.clear();
    state.captureLocked = false;
    paint();
  }
};
if (['127.0.0.1', 'localhost', 'rover.localhost'].includes(location.hostname)) {
  void import('./capture.mjs').then(module => module.initCapture())
    .catch(error => console.warn('Capture interface unavailable:', error.message));
}
showConnection('Disconnected', 'Connect to the rover over your home Wi-Fi.');
paint();
