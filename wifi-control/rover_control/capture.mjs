import {createPreviewLink} from './preview-link.mjs';

const $ = id => document.getElementById(id);
const TERMINAL = new Set(['complete', 'error', 'expired', 'cancelled']);
const sleep = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds));

export function initCapture() {
  if (!$('capture-panel') || document.body.dataset.captureInitialized) return;
  document.body.dataset.captureInitialized = 'true';
  let snapshot = null, selectedStation = null, busy = false, pending = null;
  let stopped = false, captureLocked = false, captureStarted = 0, lockTimer = null;
  let requestId = null, cancelling = false, cancelInFlight = false;
  let previewSeq = null, photosKey = '', stationsKey = '', errorMessage = '';
  let previewLoading = false, lastPreview = 0;
  let previewState = {mode: 'offline', framesPerSecond: null, reason: ''};
  const previewLink = createPreviewLink({video: $('capture-video'), onState(state) {
    previewState = state;
    if (snapshot?.enabled && !stopped) updatePreview();
  }});

  async function request(path, body, timeout = 5000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(path, {
        method: body === undefined ? 'GET' : 'POST',
        headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
        body: body === undefined ? undefined : JSON.stringify(body),
        cache: 'no-store', signal: controller.signal,
      });
      const data = await response.json().catch(() => null);
      if (!response.ok) throw new Error(data?.error || `Request failed (${response.status})`);
      if (!data) throw new Error('Invalid capture server response');
      return data;
    } catch (error) {
      if (error.name === 'AbortError') throw new Error('Capture server did not respond');
      throw error;
    } finally { clearTimeout(timer); }
  }

  function showError(message = '') {
    errorMessage = message;
    $('capture-error').textContent = message;
    $('capture-error').hidden = !message;
  }

  function setEnabled(enabled) {
    document.body.classList.toggle('capture-enabled', enabled);
    $('capture-panel').hidden = $('capture-connection').hidden = !enabled;
  }

  const activeStation = () => snapshot?.stations?.find(station => station.id === snapshot.active_station_id);
  const viewingStation = () => snapshot?.stations?.find(station => station.id === selectedStation) || activeStation() || snapshot?.stations?.at(-1);
  const isPending = () => busy || !!pending || (snapshot?.capture && !TERMINAL.has(snapshot.capture.status));
  const stationNumber = station => String(station?.number || 1).padStart(2, '0');
  function safeURL(value) {
    if (!value) return null;
    try { const url = new URL(value, location.href); return ['http:', 'https:'].includes(url.protocol) ? url.href : null; }
    catch { return null; }
  }
  function setLink(id, value) {
    const url = safeURL(value), element = $(id);
    element.hidden = !url;
    if (url) element.href = url;
  }

  function renderStations() {
    const stations = snapshot.stations || [];
    const key = JSON.stringify(stations.map(station => [station.id, station.number, station.status, selectedStation]));
    if (stationsKey === key) return;
    stationsKey = key;
    const tabs = $('capture-stations');
    tabs.replaceChildren();
    tabs.setAttribute('role', 'tablist');
    for (const station of stations) {
      const button = document.createElement('button');
      button.type = 'button'; button.className = 'station-tab';
      button.classList.toggle('ready', station.status === 'ready');
      button.textContent = stationNumber(station);
      button.setAttribute('role', 'tab');
      button.setAttribute('aria-selected', String(station.id === viewingStation()?.id));
      button.setAttribute('aria-label', `Position ${station.number}${station.status === 'ready' ? ', finished' : ', open'}`);
      button.addEventListener('click', () => { selectedStation = station.id; stationsKey = ''; render(); });
      tabs.append(button);
    }
  }

  function renderPhotos() {
    const station = viewingStation(), photos = station?.photos || [];
    const key = JSON.stringify([station?.id, photos.map(photo => [photo.id, photo.url, photo.thumbnail_url])]);
    if (photosKey === key) return;
    photosKey = key;
    const strip = $('capture-photos'); strip.replaceChildren();
    if (!photos.length) {
      const empty = document.createElement('p'); empty.className = 'photo-empty';
      empty.textContent = station?.status === 'ready' ? 'No photos at this position' : 'Captured photos appear here';
      strip.append(empty); return;
    }
    photos.forEach((photo, index) => {
      const url = safeURL(photo.url), thumbnail = safeURL(photo.thumbnail_url || photo.url);
      if (!url || !thumbnail) return;
      const link = document.createElement('a'); link.className = 'photo-card';
      link.href = url; link.target = '_blank'; link.rel = 'noopener';
      let filename;
      try { filename = decodeURIComponent(new URL(url).pathname.split('/').pop()); } catch { filename = `Photo ${index + 1}`; }
      const dimensions = photo.width && photo.height ? `, ${photo.width} × ${photo.height}` : '';
      link.setAttribute('aria-label', `${filename}${dimensions}, open original`);
      link.title = `${filename}${dimensions}`;
      const image = document.createElement('img'); image.src = thumbnail; image.loading = 'lazy';
      image.alt = `Position ${station.number}, photo ${index + 1}`;
      const number = document.createElement('span'); number.textContent = String(index + 1).padStart(2, '0');
      link.append(image, number); strip.append(link);
    });
  }

  function updatePreview() {
    const preview = snapshot.preview || {}, frame = $('capture-preview');
    const phone = snapshot.phone || {};
    const connected = phone.connected && phone.visibility !== 'hidden' &&
      (phone.camera_ready || (previewLink.isPlaying && phone.width > 0 && phone.height > 0));
    const live = connected && previewState.mode === 'live';
    const stale = !connected || !preview.url || Number(preview.age_ms) > 3000;
    document.querySelector('.camera-frame').classList.toggle('stale', !live && stale);
    const hasPreview = !!frame.getAttribute('src') && lastPreview > 0;
    $('capture-video').hidden = !live;
    frame.hidden = live || !hasPreview;
    $('capture-placeholder').hidden = live || hasPreview;
    $('capture-preview-label').hidden = !live && !hasPreview;
    $('capture-preview-label').textContent = live ? 'Live' : stale ? 'Preview paused' : 'Preview';
    $('capture-preview-label').title = live
      ? `Direct Wi-Fi video${previewState.framesPerSecond ? ` · ${previewState.framesPerSecond} fps` : ''}`
      : `JPEG fallback${previewState.reason ? ` · ${previewState.reason}` : ''}`;
    $('capture-placeholder-text').textContent = snapshot.phone?.connected ? 'Start the phone camera' : 'Waiting for phone';
    const url = safeURL(preview.url);
    if (!live && url && !previewLoading && preview.seq !== previewSeq && connected) {
      previewLoading = true;
      const image = new Image();
      const requestedSeq = preview.seq;
      image.onload = () => {
        previewLoading = false;
        if (!stopped) {
          frame.src = image.src;
          previewSeq = requestedSeq; lastPreview = Date.now();
          updatePreview();
        }
      };
      image.onerror = () => { previewLoading = false; };
      const resource = new URL(url); resource.searchParams.set('seq', String(requestedSeq ?? Date.now()));
      image.src = resource.href;
    }
  }

  function render() {
    if (!snapshot?.enabled) return;
    const station = viewingStation(), active = activeStation(), phone = snapshot.phone || {};
    const controlsBusy = isPending();
    const capturing = captureLocked || !!pending || (!!snapshot.capture && !TERMINAL.has(snapshot.capture.status));
    const connected = !!phone.connected, cameraReady = connected && !!phone.camera_ready;
    $('capture-position').textContent = station ? `Position ${stationNumber(station)}` : 'No scan';
    const count = station?.photos?.length || 0;
    $('capture-count').textContent = `${count} ${count === 1 ? 'photo' : 'photos'}`;
    const latest = station?.photos?.at(-1);
    $('capture-resolution').textContent = latest?.width && latest?.height ? `${latest.width.toLocaleString()} × ${latest.height.toLocaleString()}` : phone.width && phone.height ? `${phone.width.toLocaleString()} × ${phone.height.toLocaleString()}` : '—';
    $('capture-message').textContent = cancelling ? 'Confirming cancellation · controls paused' : capturing ? 'Capture in progress · controls paused' : !snapshot.session ? 'Create a scan to begin' : station?.id !== active?.id ? 'Viewing saved position' : station?.status === 'ready' ? 'Position saved' : !connected ? 'Connect the phone camera' : !cameraReady ? 'Start the camera on the phone' : latest ? 'Original saved on laptop' : 'Ready to capture';
    $('capture-photo').disabled = requestId && captureLocked ? cancelling : controlsBusy || !cameraReady || !active || active.status !== 'open' || station?.id !== active.id;
    $('capture-photo-label').textContent = cancelling ? 'Cancelling…' : requestId && captureLocked ? 'Cancel' : 'Capture';
    $('capture-new').disabled = controlsBusy;
    $('capture-finish').hidden = !active || active.status !== 'open' || station?.id !== active.id;
    $('capture-finish').disabled = controlsBusy || !(active?.photos?.length);
    $('capture-next').hidden = !snapshot.session || (!!active && active.status !== 'ready');
    $('capture-next').disabled = controlsBusy;
    $('capture-phone-state').textContent = connected ? cameraReady ? 'Camera ready' : 'Phone connected' : 'Phone offline';
    $('capture-phone-dot').className = `dot${cameraReady ? ' online' : ''}`;
    $('capture-phone-state').title = phone.error || (connected ? 'Phone connected to the capture server' : 'Open the phone page on the same Wi-Fi network');
    setLink('capture-phone-url', snapshot.pairing?.url);
    const phoneLinkLabel = snapshot.pairing?.code ? `Phone · ${snapshot.pairing.code} ` : 'Phone page ';
    if ($('capture-phone-url').dataset.label !== phoneLinkLabel) {
      const arrow = document.createElement('span'); arrow.textContent = '↗'; arrow.setAttribute('aria-hidden', 'true');
      $('capture-phone-url').replaceChildren(document.createTextNode(phoneLinkLabel), arrow);
      $('capture-phone-url').dataset.label = phoneLinkLabel;
    }
    $('capture-phone-url').title = snapshot.pairing?.code ? `Phone capture page · Pairing code ${snapshot.pairing.code}` : 'Phone capture page';
    setLink('capture-setup', snapshot.pairing?.setup_url);
    setLink('capture-setup-empty', snapshot.pairing?.setup_url || snapshot.pairing?.url);
    $('capture-copy').disabled = !safeURL(snapshot.pairing?.url);
    setLink('capture-export', snapshot.export_url);
    renderStations(); renderPhotos();
    previewLink.update(phone); updatePreview();
  }

  async function unlockCapture() {
    clearTimeout(lockTimer); lockTimer = null;
    if (!captureLocked) return;
    captureLocked = false;
    try { await window.roverCaptureLock?.(false); }
    catch (error) { showError(`Capture finished. Controls could not be released: ${error.message}`); }
  }

  async function finishCapture(result) {
    if (snapshot?.capture?.request_id === requestId || snapshot?.capture?.id === result?.id) snapshot.capture = result;
    requestId = pending = null; busy = cancelling = false;
    if (result?.status === 'complete') showError();
    else if (result?.status === 'cancelled') showError('Capture cancelled.');
    else showError(result?.error || (result?.status === 'expired' ? 'Capture expired. Check the phone camera and try again.' : 'The photo was not saved.'));
    await unlockCapture(); render();
  }

  async function cancelCapture() {
    if (!requestId || cancelInFlight) return;
    cancelling = true; cancelInFlight = true; render();
    const identifier = requestId;
    try {
      const result = await request('/capture/api/capture/cancel', {request_id: identifier});
      if (requestId !== identifier) return;
      // The server reserves this request ID even if the original POST is still
      // in flight, so acknowledging cancellation prevents a late capture.
      if (!TERMINAL.has(result.capture?.status) || result.capture?.request_id !== identifier) throw new Error('Capture cancellation was not confirmed');
      await finishCapture(result.capture);
    } catch {
      if (requestId === identifier) showError('Cancellation not confirmed. Controls stay paused while the server reconnects.');
    } finally { cancelInFlight = false; render(); }
  }

  function scheduleCancellation() {
    clearTimeout(lockTimer);
    lockTimer = setTimeout(() => { if (requestId) void cancelCapture(); }, Math.max(0, 30000 - (Date.now() - captureStarted)));
  }

  async function inspectCapture() {
    const capture = snapshot.capture;
    if (!requestId && capture?.status === 'queued' && capture.request_id) {
      // A page reload must not leave a previously queued camera transaction
      // running alongside newly enabled drive controls.
      requestId = capture.request_id; pending = capture.id; busy = true;
      captureStarted = Date.now(); captureLocked = true;
      await window.roverCaptureLock?.(true);
      scheduleCancellation();
    }
    if (!requestId) return;
    const matches = capture?.request_id === requestId || (pending && capture?.id === pending);
    if (matches && TERMINAL.has(capture.status)) {
      await finishCapture(capture);
    } else {
      if (matches) pending = capture.id;
      if (cancelling || Date.now() - captureStarted >= 30000) void cancelCapture();
    }
  }

  async function refresh() {
    snapshot = await request('/capture/api/state');
    if (!snapshot.enabled) { setEnabled(false); previewLink.update({}); return false; }
    setEnabled(true);
    if (!selectedStation || !snapshot.stations?.some(station => station.id === selectedStation)) selectedStation = snapshot.active_station_id || snapshot.stations?.at(-1)?.id;
    await inspectCapture(); render(); return true;
  }

  async function mutate(path, body = {}) {
    if (isPending()) return;
    busy = true; showError(); render();
    try {
      await request(path, body);
      selectedStation = null; stationsKey = ''; photosKey = '';
      await refresh();
    } catch (error) { showError(error.message); }
    finally { busy = false; render(); }
  }

  $('capture-photo').addEventListener('click', async () => {
    if (requestId && captureLocked) { void cancelCapture(); return; }
    const station = activeStation();
    if (isPending() || !station || station.status !== 'open' || !snapshot.phone?.camera_ready || !snapshot.phone?.connected) return;
    busy = true; captureStarted = Date.now(); requestId = crypto.randomUUID();
    const identifier = requestId;
    let postAttempted = false;
    showError(); render();
    try {
      if (typeof window.roverCaptureLock !== 'function') throw new Error('Capture stop control is unavailable. Reload this page.');
      captureLocked = true; render();
      await Promise.race([
        window.roverCaptureLock(true),
        sleep(6000).then(() => { throw new Error('Could not confirm the rover has stopped'); }),
      ]);
      if (requestId !== identifier || cancelling) return;
      postAttempted = true; scheduleCancellation();
      const result = await request('/capture/api/capture', {station_id: station.id, request_id: identifier}, 10000);
      if (requestId !== identifier) return;
      if (!result.capture?.id) throw new Error('The server did not acknowledge the capture');
      pending = result.capture.id;
      if (TERMINAL.has(result.capture.status)) { await finishCapture(result.capture); return; }
      await refresh();
    } catch (error) {
      if (requestId !== identifier) return;
      if (postAttempted) {
        showError(`${error.message}. Confirming cancellation before resuming controls.`);
        await cancelCapture();
      } else {
        requestId = pending = null; busy = false; showError(error.message);
        await unlockCapture(); render();
      }
    }
  });

  $('capture-new').addEventListener('click', () => mutate('/capture/api/sessions'));
  $('capture-next').addEventListener('click', () => mutate('/capture/api/stations'));
  $('capture-finish').addEventListener('click', () => {
    const station = activeStation();
    if (station) void mutate(`/capture/api/stations/${encodeURIComponent(station.id)}/finish`);
  });
  $('capture-copy').addEventListener('click', async () => {
    const url = safeURL(snapshot?.pairing?.url);
    if (!url) return;
    try {
      await navigator.clipboard.writeText(url);
      $('capture-copy').title = 'Copied'; $('capture-copy').setAttribute('aria-label', 'Phone page link copied');
      setTimeout(() => { $('capture-copy').title = 'Copy phone page link'; $('capture-copy').setAttribute('aria-label', 'Copy phone page link'); }, 2000);
    } catch { showError(`Open this address on the phone: ${url}`); }
  });
  window.addEventListener('pagehide', () => {
    stopped = true; clearTimeout(lockTimer); previewLink.close();
    if (requestId) {
      // Best effort only: a replacement page adopts any still-queued command.
      const body = new Blob([JSON.stringify({request_id: requestId})], {type: 'application/json'});
      navigator.sendBeacon?.('/capture/api/capture/cancel', body);
    }
  });

  async function poll() {
    while (!stopped) {
      try {
        const enabled = await refresh();
        if (!enabled) return;
        if (errorMessage === 'Capture server unavailable. Reconnecting…') showError();
      } catch {
        if (snapshot?.enabled) {
          showError('Capture server unavailable. Reconnecting…');
          $('capture-phone-state').textContent = 'Server offline';
          $('capture-phone-dot').className = 'dot error';
          $('capture-photo').disabled = true;
          previewLink.update({});
          $('capture-preview-label').textContent = 'Preview paused';
          document.querySelector('.camera-frame').classList.add('stale');
        } else { setEnabled(false); }
      }
      await sleep(500);
    }
  }
  void poll();
}
