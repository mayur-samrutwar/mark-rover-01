// A separate, receive-only preview connection. Preview negotiation never arms
// the rover, acquires a drive lock, or changes capture transactions.
export function createPreviewLink({video, onState = () => {}}) {
  const Peer = globalThis.RTCPeerConnection;
  let stopped = false, ready = false, canStart = false, connection = null, generation = 0;
  let nextAttempt = 0, failures = 0, stepping = false;
  let takeOverRequested = false;
  let current = {mode: 'offline', framesPerSecond: null, reason: ''};
  const controllers = new Set();
  const now = () => performance.now();
  const isCurrent = item => !stopped && connection === item && item.generation === generation;

  function publish(mode, reason = '', framesPerSecond = null) {
    const state = {mode, reason, framesPerSecond};
    if (JSON.stringify(state) === JSON.stringify(current)) return;
    current = state; onState({...state});
  }

  async function request(body) {
    const controller = new AbortController(); controllers.add(controller);
    const timeout = setTimeout(() => controller.abort(), 1800);
    try {
      const response = await fetch('/capture/api/rtc', {
        method: body ? 'POST' : 'GET', cache: 'no-store', signal: controller.signal,
        headers: body ? {'Content-Type': 'application/json'} : undefined,
        body: body ? JSON.stringify(body) : undefined,
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result?.error || 'Preview connection unavailable');
      return result;
    } finally { clearTimeout(timeout); controllers.delete(controller); }
  }

  function notifyClosed(item, state = 'closed') {
    // This acknowledgement is best effort. The short server lease also resumes
    // JPEG fallback if the tab, process, or network disappears without it.
    void fetch('/capture/api/rtc', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({action: 'status', connection_id: item.id, state}),
      keepalive: true,
    }).catch(() => {});
  }

  function dispose(state = 'closed') {
    const item = connection; connection = null; generation += 1;
    for (const controller of controllers) controller.abort();
    controllers.clear();
    if (!item) return;
    if (item.frameCallback !== null) video.cancelVideoFrameCallback?.(item.frameCallback);
    item.cancelGather?.();
    item.pc.ontrack = item.pc.onconnectionstatechange = null;
    item.pc.close();
    video.pause(); video.srcObject = null;
    if (item.offerAttempted) notifyClosed(item, state);
  }

  function fail(item, reason) {
    if (!isCurrent(item)) return;
    dispose('failed');
    failures += 1;
    nextAttempt = now() + Math.min(20000, 1500 * 2 ** Math.min(failures - 1, 4));
    publish('fallback', reason);
  }

  function recordFrame(item, frames) {
    if (!isCurrent(item) || video.readyState < 2) return;
    const time = now();
    if (item.totalFrames !== null && frames <= item.totalFrames) return;
    const previous = item.totalFrames;
    item.totalFrames = frames; item.lastFrame = time;
    item.framesInWindow += previous === null ? 1 : frames - previous;
    if (time - item.fpsAt >= 1000) {
      item.fps = Math.max(1, Math.round(item.framesInWindow * 1000 / (time - item.fpsAt)));
      item.framesInWindow = 0; item.fpsAt = time;
    }
    failures = 0;
    publish('live', '', item.fps);
  }

  function watchFrames(item) {
    if (typeof video.requestVideoFrameCallback !== 'function') return;
    const receive = (_time, metadata) => {
      if (!isCurrent(item)) return;
      recordFrame(item, Number(metadata.presentedFrames) || (item.totalFrames || 0) + 1);
      item.frameCallback = video.requestVideoFrameCallback(receive);
    };
    item.frameCallback = video.requestVideoFrameCallback(receive);
  }

  function gatherIce(item) {
    return new Promise(resolve => {
      let finished = false;
      const finish = () => {
        if (finished) return;
        finished = true; clearTimeout(timeout);
        item.pc.removeEventListener('icegatheringstatechange', changed);
        item.cancelGather = null; resolve();
      };
      const changed = () => { if (item.pc.iceGatheringState === 'complete') finish(); };
      const timeout = setTimeout(finish, 3000);
      item.cancelGather = finish;
      item.pc.addEventListener('icegatheringstatechange', changed);
      changed();
    });
  }

  async function begin() {
    if (connection || stopped || !ready || !canStart) return;
    if (!Peer || !video) { publish('fallback', 'Live video is unavailable in this browser'); return; }
    const created = now();
    let peer;
    try { peer = new Peer({iceServers: []}); }
    catch { nextAttempt = now() + 20000; publish('fallback', 'Live video is unavailable in this browser'); return; }
    const item = {
      id: crypto.randomUUID(), generation: ++generation,
      pc: peer, created, offerAttempted: false, offered: false, answered: false,
      lastFrame: 0, totalFrames: null, frameCallback: null, cancelGather: null,
      framesInWindow: 0, fpsAt: created, fps: null, nextPoll: 0, nextHeartbeat: 0,
      lastVideoTime: 0,
      takeOver: takeOverRequested,
    };
    takeOverRequested = false;
    connection = item; publish('connecting');
    item.pc.onconnectionstatechange = () => {
      if (['failed', 'closed'].includes(item.pc.connectionState)) fail(item, 'Live video interrupted');
    };
    item.pc.ontrack = event => {
      if (!isCurrent(item) || event.track.kind !== 'video') return;
      video.srcObject = event.streams[0] || new MediaStream([event.track]);
      watchFrames(item);
      void video.play().catch(() => fail(item, 'Browser could not play live video'));
    };
    try {
      item.pc.addTransceiver('video', {direction: 'recvonly'});
      const offer = await item.pc.createOffer();
      if (!isCurrent(item)) return;
      await item.pc.setLocalDescription(offer);
      await gatherIce(item);
      if (!isCurrent(item)) return;
      // Use the gathered description, not createOffer's pre-candidate SDP.
      if (item.takeOver) {
        const {rtc} = await request();
        if (!isCurrent(item)) return;
        if (rtc && rtc.operator_state !== 'closed' && rtc.connection_id !== item.id) {
          await request({action: 'status', connection_id: rtc.connection_id, state: 'closed'});
          if (!isCurrent(item)) return;
        }
      }
      item.offerAttempted = true;
      await request({action: 'offer', connection_id: item.id, sdp: item.pc.localDescription.sdp});
      if (!isCurrent(item)) { notifyClosed(item); return; }
      item.offered = true; item.nextHeartbeat = 0;
    } catch { fail(item, 'Live video could not connect'); }
  }

  async function readFrames(item) {
    if (!isCurrent(item) || typeof video.requestVideoFrameCallback === 'function') return;
    let count = video.getVideoPlaybackQuality?.().totalVideoFrames;
    if (!Number.isFinite(count) || count === 0) {
      try {
        const stats = await item.pc.getStats();
        stats.forEach(report => {
          if (report.type === 'inbound-rtp' && (report.kind || report.mediaType) === 'video' && Number.isFinite(report.framesDecoded)) count = report.framesDecoded;
        });
      } catch { /* currentTime remains a fallback for older WebKit builds */ }
    }
    if (!isCurrent(item)) return;
    if (Number.isFinite(count) && count > 0) recordFrame(item, count);
    else if (video.readyState >= 2 && video.currentTime > item.lastVideoTime + 0.001) {
      item.lastVideoTime = video.currentTime;
      recordFrame(item, (item.totalFrames || 0) + 1);
    }
  }

  async function step() {
    if (stepping || stopped || !ready) return;
    stepping = true;
    try {
      if (!connection) { if (now() >= nextAttempt) void begin(); return; }
      const item = connection;
      if ((item.lastFrame && now() - item.lastFrame > 2500) || (!item.lastFrame && now() - item.created > 15000)) {
        fail(item, 'Live video stalled'); return;
      }
      if (!item.offered) return;
      await readFrames(item);
      if (!isCurrent(item)) return;
      if (now() >= item.nextHeartbeat) {
        item.nextHeartbeat = now() + 1000;
        const playing = item.lastFrame > 0 && now() - item.lastFrame < 2000;
        await request({action: 'status', connection_id: item.id, state: playing ? 'playing' : 'connecting',
          ...(item.totalFrames !== null ? {frames_decoded: Math.round(item.totalFrames)} : {}),
          ...(item.fps !== null ? {frames_per_second: item.fps} : {}),
        });
        if (!isCurrent(item)) return;
      }
      if (!item.answered && now() >= item.nextPoll) {
        item.nextPoll = now() + 400;
        const result = await request();
        if (!isCurrent(item)) return;
        const rtc = result.rtc;
        if (!rtc || rtc.connection_id !== item.id || ['failed', 'closed'].includes(rtc.phone_state)) {
          fail(item, 'Phone preview reconnecting'); return;
        }
        if (rtc.answer?.type === 'answer' && rtc.answer.sdp) {
          await item.pc.setRemoteDescription(rtc.answer);
          if (isCurrent(item)) item.answered = true;
        }
      }
    } catch { if (connection) fail(connection, 'Live video connection interrupted'); }
    finally { stepping = false; }
  }

  const timer = setInterval(() => { void step(); }, 250);
  return {
    takeOver() {
      if (stopped || !canStart) return;
      takeOverRequested = true;
      dispose(); nextAttempt = 0; failures = 0;
      void step();
    },
    update(phone = {}) {
      if (stopped) return;
      const visible = !!phone.connected && phone.visibility !== 'hidden';
      canStart = visible && !!phone.camera_ready;
      // Taking a full-resolution still may briefly mute the shared camera
      // track. Preserve an existing peer while camera dimensions remain valid;
      // a stopped camera, hidden page or disconnected phone still closes it.
      const available = visible && (canStart || (!!connection && phone.width > 0 && phone.height > 0));
      if (!available) {
        ready = false; dispose(); nextAttempt = 0; failures = 0;
        publish('offline'); return;
      }
      if (!ready) { ready = true; nextAttempt = 0; void step(); }
    },
    close() {
      if (stopped) return;
      stopped = true; ready = canStart = false; clearInterval(timer); dispose(); publish('offline');
    },
    get isPlaying() { return current.mode === 'live'; },
    get state() { return {...current}; },
  };
}
