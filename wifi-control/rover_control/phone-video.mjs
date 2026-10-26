const RTC_PATH = '/capture/phone/rtc';

// The camera track is shared with ImageCapture. Preview limits belong on the
// sender, so negotiating video never lowers the resolution of still photos.
export function createPhoneVideo({request, getSource, onState = () => {}, PeerConnection = globalThis.RTCPeerConnection, now = Date.now, gatherTimeout = 3000, negotiationTimeout = 3000, tuningTimeout = 1000}) {
  let active = null;
  let polling = false;
  let generation = 0;

  const sourceMatches = entry => {
    const source = getSource();
    return active === entry && source.ready && source.generation === entry.sourceGeneration &&
      source.stream?.getVideoTracks()[0] === entry.track && entry.track.readyState === 'live';
  };

  function notify(entry, state, stage, error = '') {
    if (entry && !sourceMatches(entry)) return;
    if (entry) entry.stage = stage;
    try { onState({state, stage, error}); }
    catch { /* A diagnostic callback must not interrupt signaling. */ }
  }

  function close() {
    generation++;
    const previous = active;
    active = null;
    if (!previous) return;
    previous.cancelGather?.();
    if (previous.pc) {
      previous.pc.onconnectionstatechange = null;
      previous.pc.oniceconnectionstatechange = null;
      previous.pc.close();
    }
    notify(null, 'closed', 'closed');
    // Never stop the shared camera track here: still capture owns its lifetime.
  }

  async function bounded(operation, label, timeout) {
    let timer;
    try {
      return await Promise.race([
        Promise.resolve().then(operation),
        new Promise((_, reject) => { timer = setTimeout(() => reject(new Error(`${label} timed out after ${timeout} ms.`)), timeout); }),
      ]);
    } finally { clearTimeout(timer); }
  }

  async function step(entry, stage, operation) {
    notify(entry, 'connecting', stage);
    return bounded(operation, stage, negotiationTimeout);
  }

  async function report(entry, state, error = '') {
    if (!sourceMatches(entry) || entry.reportedState === state) return;
    entry.reportedState = state;
    try {
      await request(RTC_PATH, {
        method: 'POST', timeout: 2000,
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: 'status', connection_id: entry.id, state, ...(error ? {error} : {})}),
      });
    } catch { /* A stale connection or interrupted status report is expendable. */ }
  }

  function gatheringComplete(entry) {
    if (entry.pc.iceGatheringState === 'complete') return Promise.resolve();
    return new Promise(resolve => {
      let timer;
      const finish = () => {
        clearTimeout(timer);
        entry.pc.removeEventListener('icegatheringstatechange', changed);
        entry.cancelGather = null;
        resolve();
      };
      const changed = () => { if (entry.pc.iceGatheringState === 'complete') finish(); };
      entry.cancelGather = finish;
      entry.pc.addEventListener('icegatheringstatechange', changed);
      timer = setTimeout(finish, gatherTimeout);
      changed();
    });
  }

  async function configureSender(entry, sender) {
    if (!sourceMatches(entry) || !sender?.getParameters || !sender.setParameters) return;
    try {
      const parameters = sender.getParameters();
      const settings = entry.track.getSettings?.() || {};
      const maxEdge = Math.max(settings.width || 0, settings.height || 0);
      parameters.encodings ||= [{}];
      if (!parameters.encodings.length) parameters.encodings.push({});
      for (const encoding of parameters.encodings) {
        encoding.maxBitrate = 2_000_000;
        encoding.maxFramerate = 30;
        encoding.scaleResolutionDownBy = Math.max(1, maxEdge / 1280);
      }
      await bounded(() => sender.setParameters(parameters), 'Preview encoder settings', tuningTimeout);
    } catch { /* Optional encoder settings must never block the SDP answer. */ }
  }

  async function answer(rtc, source) {
    close();
    const entry = {
      id: rtc.connection_id, sourceGeneration: source.generation,
      track: source.stream.getVideoTracks()[0], pc: null,
      operatorPlaying: rtc.operator_state === 'playing', expiresAt: rtc.expires_at,
      lastSignalAt: now(), reportedState: '', failed: false, stage: 'Creating video connection',
    };
    active = entry;
    try {
      notify(entry, 'connecting', entry.stage);
      // Keep construction inside the guarded path: older Android WebViews may
      // expose RTCPeerConnection yet reject its configuration synchronously.
      entry.pc = new PeerConnection({iceServers: [], bundlePolicy: 'max-bundle'});
      const connectionChanged = () => {
        if (!sourceMatches(entry) || entry.failed) return;
        const state = entry.pc.connectionState;
        if (state === 'connected') {
          notify(entry, 'connected', 'Video connected');
          void report(entry, 'connected');
        } else if (state === 'failed' || entry.pc.iceConnectionState === 'failed') {
          entry.failed = true;
          const message = 'The local video connection failed.';
          notify(entry, 'failed', 'Connecting video', message);
          void report(entry, 'failed', message);
        } else if (state === 'closed') {
          notify(entry, 'closed', 'Video closed');
          void report(entry, 'closed');
        } else void report(entry, 'connecting');
      };
      entry.pc.onconnectionstatechange = connectionChanged;
      entry.pc.oniceconnectionstatechange = connectionChanged;
      void report(entry, 'connecting');
      notify(entry, 'connecting', 'Attaching camera track');
      const sender = entry.pc.addTrack(entry.track, source.stream);
      await step(entry, 'Reading operator video offer', () => entry.pc.setRemoteDescription(rtc.offer));
      if (!sourceMatches(entry)) return;
      const localAnswer = await step(entry, 'Creating video answer', () => entry.pc.createAnswer());
      if (!sourceMatches(entry)) return;
      await step(entry, 'Applying video answer', () => entry.pc.setLocalDescription(localAnswer));
      if (!sourceMatches(entry)) return;
      notify(entry, 'connecting', 'Gathering local network addresses');
      await gatheringComplete(entry);
      if (!sourceMatches(entry)) return;
      notify(entry, 'connecting', 'Sending video answer');
      await bounded(() => request(RTC_PATH, {
        method: 'POST', timeout: 4000,
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: 'answer', connection_id: entry.id, sdp: entry.pc.localDescription.sdp}),
      }), 'Sending video answer', 4500);
      if (!sourceMatches(entry)) return;
      notify(entry, entry.pc.connectionState === 'connected' ? 'connected' : 'connecting', 'Video answer sent');
      // Android implementations can leave setParameters pending while starting
      // an encoder. Its optional tuning runs only after signaling is complete.
      void configureSender(entry, sender);
    } catch (error) {
      if (!sourceMatches(entry)) return;
      entry.failed = true;
      const message = `${entry.stage}: ${error.message || 'Could not start local video.'}`;
      notify(entry, 'failed', entry.stage, message);
      void report(entry, 'failed', message);
      entry.cancelGather?.();
      if (entry.pc) {
        entry.pc.onconnectionstatechange = null;
        entry.pc.oniceconnectionstatechange = null;
        entry.pc.close();
      }
    }
  }

  async function poll() {
    const source = getSource();
    if (!source.ready || !PeerConnection) { if (active) close(); return; }
    if (active && !sourceMatches(active)) close();
    if (polling) return;
    polling = true;
    const epoch = generation;
    try {
      const {rtc} = await request(RTC_PATH, {timeout: 2000});
      const current = getSource();
      if (epoch !== generation || !current.ready || current.generation !== source.generation || current.stream !== source.stream) return;
      if (!rtc?.connection_id || !rtc.offer?.sdp || !Number.isFinite(rtc.expires_at) || rtc.expires_at * 1000 <= now()) {
        if (active) close();
        return;
      }
      if (!active || active.id !== rtc.connection_id) {
        await answer(rtc, current);
      } else {
        active.operatorPlaying = rtc.operator_state === 'playing';
        active.expiresAt = rtc.expires_at;
        active.lastSignalAt = now();
      }
    } catch (error) {
      // Auth invalidation closes the camera through the caller. Other failures
      // let the JPEG fallback resume while the operator retries a new offer.
      if ((error.status === 401 || error.status === 403) && active) close();
    } finally { polling = false; }
  }

  function isStreaming() {
    return Boolean(active && !active.failed && sourceMatches(active) && active.pc?.connectionState === 'connected' &&
      active.operatorPlaying && active.expiresAt * 1000 > now() && now() - active.lastSignalAt < 2000);
  }

  return {poll, close, isStreaming};
}
