const $ = (id) => document.getElementById(id);
const API_BASE = new URL('.', import.meta.url).pathname.replace(/\/$/, '');
const ACTIVE = new Set(['queued', 'running']);
const TERMINAL = new Set(['completed', 'failed', 'cancelled', 'interrupted']);
const storedJob = localStorage.getItem('reconstruction-job') || '';
const state = { health: null, scans: [], jobs: [], job: null, selected: storedJob, selectedExplicit: !!storedJob && localStorage.getItem('reconstruction-job-explicit') === storedJob, viewer: null, viewerPromise: null, artifactKey: '', modelManual: false, refreshPromise: null, busy: false, offline: false, poll: null, methodInitialized: false, completionPending: '' };
const dateFormat = new Intl.DateTimeFormat(undefined, { month: 'short', day: 'numeric', hour: '2-digit', minute: '2-digit' });
const timeFormat = new Intl.DateTimeFormat(undefined, { hour: '2-digit', minute: '2-digit' });
const titleCase = (value) => String(value || '').replaceAll('_', ' ').replace(/\b\w/g, (x) => x.toUpperCase());
const formatDate = (value, format = dateFormat) => { const date = new Date(value); return Number.isFinite(date.getTime()) ? format.format(date) : 'Unknown date'; };
const safeURL = (value) => { try { const url = new URL(value, location.href); return url.origin === location.origin && url.pathname.startsWith('/artifacts/') ? url.href : null; } catch { return null; } };
const text = (tag, value, className = '') => { const el = document.createElement(tag); el.textContent = String(value); if (className) el.className = className; return el; };
const showError = (id, error) => { $(id).textContent = error?.message || String(error || 'Request failed'); $(id).hidden = !error; };

async function request(path, options = {}) {
  const { timeoutMs = 12000, ...fetchOptions } = options;
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetch(API_BASE + path, { ...fetchOptions, credentials: 'same-origin', cache: 'no-store', signal: controller.signal, headers: { ...(options.body ? { 'Content-Type': 'application/json' } : {}), ...options.headers } });
    let data;
    try { data = await response.json(); } catch { throw new Error(`Server returned an invalid response (${response.status})`); }
    if (!response.ok) throw new Error(typeof data.error === 'string' ? data.error : data.error?.message || `Request failed (${response.status})`);
    return data;
  } catch (error) { if (error.name === 'AbortError') throw new Error('Server response timed out'); throw error; }
  finally { clearTimeout(timer); }
}
const post = (path, body = {}, options = {}) => request(path, { ...options, method: 'POST', body: JSON.stringify(body) });

function renderHealth() {
  const h = state.health;
  $('server-dot').className = `dot ${state.offline ? 'error' : h?.status === 'ready' ? 'ready' : ''}`;
  $('server-state').textContent = state.offline ? 'Server unavailable' : h?.status === 'ready' ? 'Connected' : h ? 'Tools unavailable' : 'Connecting';
  const c = h?.engines?.colmap; const m = h?.engines?.openmvs; const native = h?.engines?.photogrammetry;
  $('native-status').textContent = native?.available ? 'Apple photogrammetry ready' : 'Apple photogrammetry unavailable';
  const nativeOption = $('method').querySelector('[value="photogrammetry"]');
  nativeOption.disabled = !native?.available;
  if (!state.methodInitialized && h) {
    state.methodInitialized = true;
    if (native?.available) { $('method').value = 'photogrammetry'; document.querySelector('input[name="quality"][value="full"]').checked = true; }
  }
  renderQuality();
  $('colmap-status').textContent = c?.available ? 'COLMAP ready' : 'COLMAP unavailable';
  $('colmap-status').title = c?.version || '';
  $('openmvs-status').textContent = m?.available ? 'OpenMVS ready' : 'OpenMVS unavailable';
  $('openmvs-status').title = m?.version || '';
  $('metal-status').textContent = m?.metal_compiled === true ? 'Metal available' : m?.metal_compiled === false ? 'Metal unavailable' : 'Metal unverified';
  const capture = h?.capture_url;
  if (API_BASE === '/reconstruction') $('capture-link').href = '/';
  else try { const url = new URL(capture); if (['localhost', '127.0.0.1', '[::1]'].includes(url.hostname) && ['http:', 'https:'].includes(url.protocol)) $('capture-link').href = url.href; } catch {}
  $('footer-detail').textContent = 'Local processing';
  renderStart();
}
function renderStart() {
  const scan = state.scans.find((x) => x.id === $('scan').value);
  const available = $('method').value === 'photogrammetry' ? state.health?.engines?.photogrammetry?.available : state.health?.engines?.colmap?.available && state.health?.engines?.openmvs?.available;
  $('method').disabled = state.busy;
  $('reconstruct').disabled = state.busy || state.offline || !available || !scan || scan.usable === false || Number(scan.photo_count) < 3;
  if (scan) {
    const count = Number(scan.photo_count) || 0; const positions = Number(scan.position_count) || 0;
    $('scan-detail').textContent = `${count} photo${count === 1 ? '' : 's'} · ${positions} position${positions === 1 ? '' : 's'}${scan.warning ? ' · ' + scan.warning : ''}`;
  } else $('scan-detail').textContent = state.scans.length ? 'Select a scan' : 'Capture images from the camera page';
}
function renderScans() {
  const select = $('scan'); const old = select.value;
  select.replaceChildren();
  if (!state.scans.length) { const option = text('option', 'No scans yet'); option.value = ''; select.append(option); }
  for (const scan of state.scans) { const option = text('option', `${formatDate(scan.created_at)} · ${Number(scan.photo_count) || 0} photos`); option.value = scan.id; option.title = scan.id; select.append(option); }
  if (state.scans.some((x) => x.id === old)) select.value = old;
  select.disabled = !state.scans.length || state.busy;
  renderStart();
}
const runLabel = (job) => job.scene_specific_fit === true ? 'Scene-specific fit · inferred' : (job.label === 'Refined shapes' ? 'Proportion correction' : job.label) || (job.method === 'photogrammetry' ? 'Photogrammetry' : titleCase(job.quality || 'draft'));
function renderRuns() {
  const runs = $('runs'); runs.replaceChildren();
  if (!state.jobs.length) { runs.append(text('p', 'No runs yet', 'muted empty-runs')); return; }
  for (const job of state.jobs) {
    const button = document.createElement('button'); button.type = 'button'; button.className = 'run-item'; button.setAttribute('aria-pressed', String(job.id === state.selected)); button.dataset.jobId = job.id;
    const dot = text('span', '', `dot ${job.status}`); dot.setAttribute('aria-hidden', 'true');
    const status = job.status === 'completed' && job.metrics?.surface_topology?.assessment === 'fragmented' ? 'Fragmented' : job.status === 'completed' ? 'Finished' : titleCase(job.status);
    button.append(dot, text('span', `${runLabel(job)} · ${status}`));
    const time = text('time', formatDate(job.created_at, timeFormat)); time.dateTime = job.created_at || ''; button.append(time);
    button.title = `${formatDate(job.created_at)} · ${job.id}`;
    button.addEventListener('click', () => selectJob(job.id, { explicit: true })); runs.append(button);
  }
}
function renderStages(job) {
  $('stage-panel').hidden = !job;
  if (!job) return;
  $('run-label').textContent = `${runLabel(job)} run`;
  $('run-status').textContent = job.status === 'completed' && job.metrics?.surface_topology?.assessment === 'fragmented' ? 'Fragmented surface' : job.status === 'completed' ? 'Processing finished' : titleCase(job.status);
  $('progress-bar').style.width = `${Math.round(Math.min(1, Math.max(0, Number(job.progress) || 0)) * 100)}%`;
  $('stages').replaceChildren();
  for (const stage of job.stages || []) {
    const li = document.createElement('li'); li.className = stage.status || '';
    const marker = text('span', stage.status === 'completed' ? '✓' : stage.status === 'failed' ? '!' : '', 'stage-marker'); marker.setAttribute('aria-hidden', 'true');
    const label = text('div', stage.label || titleCase(stage.id));
    if (stage.message) label.append(text('span', stage.message, 'stage-message'));
    li.append(marker, label); $('stages').append(li);
  }
  $('cancel').hidden = !ACTIVE.has(job.status); $('retry').hidden = !['failed', 'cancelled', 'interrupted'].includes(job.status);
  $('cancel').disabled = state.busy; $('retry').disabled = state.busy || state.health?.status !== 'ready';
  const backend = job.backend?.active;
  $('run-backend').textContent = backend === 'metal' ? 'Metal GPU' : backend === 'cpu' ? 'CPU' : backend === 'apple' ? 'Apple photogrammetry' : '';
}
function renderMetrics(job) {
  const metrics = $('metrics'); metrics.replaceChildren();
  if (!job) return;
  const values = job.metrics || {};
  const metric = (name, value) => { if (value === undefined || value === null) return; const item = text('span', name); item.append(text('b', value)); metrics.append(item); };
  const registered = values.registered_images ?? values.registered_cameras ?? values.aligned_images;
  const total = values.input_images ?? values.total_images ?? values.image_count;
  if (registered !== undefined) metric('Aligned', total ? `${registered}/${total}` : registered);
  let vertices = values.vertices ?? values.vertex_count ?? values.visual_mesh?.vertices;
  let faces = values.faces ?? values.face_count ?? values.triangles ?? values.visual_mesh?.triangles;
  let components = values.surface_topology?.components;
  const selected = models(job).find((a) => a.url === $('model').value);
  const completion = job.completion?.[isMetricArtifact(selected) ? 'scaled' : 'raw'];
  if (completion && ['completed_visual_mesh', 'completed_surface_mesh', 'inferred_surface'].includes(selected?.kind)) {
    const addedVertices = (completion.loops || []).filter((loop) => loop.status === 'filled').reduce((sum, loop) => sum + Number(loop.boundary_vertices || 0), 0);
    const addedFaces = Number(completion.added_triangles) || 0;
    if (selected.kind === 'inferred_surface') { vertices = addedVertices; faces = addedFaces; components = completion.filled_loops; }
    else { vertices = Number(completion.original_vertices ?? vertices) + addedVertices; faces = Number(completion.original_triangles ?? faces) + addedFaces; components = completion.topology_after?.components ?? components; }
  }
  if (vertices !== undefined) metric('Vertices', Number(vertices).toLocaleString());
  if (faces !== undefined) metric('Faces', Number(faces).toLocaleString());
  if (components !== undefined) metric('Surface pieces', Number(components).toLocaleString());
  const elapsed = values.elapsed_seconds ?? values.duration_seconds;
  if (elapsed !== undefined) metric('Time', elapsed < 60 ? `${Math.round(elapsed)}s` : `${Math.round(elapsed / 60)} min`);
  if (job.calibration?.status === 'scaled') metric('Scale', 'metres');
}
function usableArtifacts(job) { return (job?.artifacts || []).filter((a) => safeURL(a.url)); }
function models(job) { return usableArtifacts(job).filter((a) => /\.(ply|obj|glb|gltf)(?:[?#]|$)/i.test(a.url)); }
function isMetricArtifact(a) { return a?.units === 'metres' || a?.units === 'meters' || !!a?.url?.includes('/exports/scaled/'); }
function artifactLabel(a) {
  const name = (a.name || '').toLowerCase();
  const inferred = a.kind === 'completed_visual_mesh' ? (a.contains_inferred === false ? 'Gap fill · no changes' : 'Gap fill · estimated') : a.kind === 'completed_surface_mesh' ? (a.contains_inferred === false ? 'Gap-filled surface · no changes' : 'Gap-filled surface · estimated') : a.kind === 'inferred_surface' || name === 'inferred.glb' ? 'Inferred surfaces only' + (name.endsWith('.ply') ? ' · PLY' : '') : null;
  const label = inferred || (a.label === 'Refined model' ? 'Proportion-corrected model' : a.label) || (name.includes('collision') ? 'Collision candidate' : /\.glb$/i.test(name) ? 'Original model' : name.includes('texture') ? 'Textured mesh' : name.includes('refin') ? 'Refined mesh' : name.includes('surface') ? 'Surface mesh' : name.includes('dense') ? 'Dense cloud' : name.includes('sparse') ? 'Camera alignment' : a.name || 'Model');
  return isMetricArtifact(a) ? 'Metric · ' + label : label;
}
function selectedSpace() { return isMetricArtifact(models(state.job).find((a) => a.url === $('model').value)) ? 'scaled' : 'raw'; }
function renderCompletion() {
  const job = state.job; renderMetrics(job); const space = selectedSpace();
  const waiting = !!job?.completing || state.completionPending === job?.id;
  $('complete').textContent = waiting ? 'Filling…' : 'Fill gaps';
  const source = usableArtifacts(job).some((a) => /\/scene\.glb$/i.test(a.url) && isMetricArtifact(a) === (space === 'scaled'));
  $('complete').disabled = !source || job?.status !== 'completed' || state.busy || state.offline || !!job?.scaling || waiting;
  const report = job?.completion?.[space];
  const count = (value) => Array.isArray(value) ? value.length : Number(value) || 0;
  const selected = models(job).find((a) => a.url === $('model').value);
  const displayingFill = ['completed_visual_mesh', 'completed_surface_mesh', 'inferred_surface'].includes(selected?.kind);
  $('completion-summary').hidden = !report || !displayingFill;
  $('completion-summary').textContent = report && displayingFill ? `Partial gap fill · ${count(report.filled_loops)} boundary loops filled · ${count(report.skipped_loops)} unfilled boundar${count(report.skipped_loops) === 1 ? 'y' : 'ies'} · ${count(report.added_triangles)} estimated triangles` : '';
}

function renderArtifacts(job) {
  const current = $('model').value;
  const available = models(job);
  $('model').replaceChildren();
  for (const a of available) { const option = text('option', artifactLabel(a)); option.value = a.url; $('model').append(option); }
  if (!available.length) { $('model').append(text('option', 'No model')); $('model').disabled = true; }
  else { $('model').disabled = false; if (state.modelManual && available.some((a) => a.url === current)) $('model').value = current; else { const chosen = available.find((a) => a.kind === 'completed_visual_mesh' && isMetricArtifact(a)) || available.find((a) => /\.glb$/i.test(a.url) && isMetricArtifact(a)) || available.find((a) => isMetricArtifact(a)) || available.find((a) => a.kind === 'completed_visual_mesh') || available.find((a) => /\.glb$/i.test(a.url)) || available.find((a) => /texture/i.test(a.name)) || available.find((a) => /refin/i.test(a.name)) || available.at(-1); $('model').value = chosen.url; } }
  $('artifacts').replaceChildren();
  for (const a of usableArtifacts(job)) {
    if (/\.(png|jpg|jpeg|mtl|json|txt)$/i.test(a.name || '')) continue;
    const link = document.createElement('a'); link.href = safeURL(a.url); link.download = a.name || ''; link.textContent = (isMetricArtifact(a) ? 'Metric · ' : '') + (a.name || 'Download'); link.title = Number(a.size) ? `${(a.size / 1048576).toFixed(1)} MB` : ''; link.append(text('span', '↓')); $('artifacts').append(link);
  }
  showSelectedModel();
}
function renderJob() {
  const job = state.job; renderStages(job); renderMetrics(job); renderArtifacts(job);
  showError('run-error', job?.error || null);
  $('warnings').replaceChildren();
  const warnings = [...(job?.warnings || [])];
  if (job && ['Refined shapes', 'Proportion correction'].includes(job.label)) warnings.unshift('This run adjusts overall proportions only. Local furniture shapes and floor/ceiling coverage remain unresolved.');
  for (const warning of warnings) $('warnings').append(text('p', warning));
  $('warnings').hidden = !warnings.length;
  $('log-panel').hidden = !job;
  const logs = job?.log_tail || []; const log = $('logs'); const atBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 35; log.textContent = logs.join('\n'); if (atBottom) log.scrollTop = log.scrollHeight;
  $('log-count').textContent = logs.length ? `${logs.length} lines` : '';
  const hasModel = models(job).length > 0;
  $('viewer-message').textContent = job && ACTIVE.has(job.status) ? (job.stages || []).find((x) => x.status === 'running')?.label || 'Processing scan' : job?.status === 'failed' ? 'Reconstruction stopped' : job?.status === 'cancelled' ? 'Run cancelled' : 'No model yet';
  $('viewer-note').textContent = job && ACTIVE.has(job.status) ? 'The model appears as stages finish' : job?.status === 'failed' ? 'See the processing log for details' : 'Choose a scan to begin';
  $('viewer-empty').hidden = hasModel && !!state.viewer?.hasModel;
  const selectedModel = models(job).find((a) => a.url === $('model').value);
  $('scale-badge').textContent = isMetricArtifact(selectedModel) ? 'Metric scale' : 'Scale unknown';
  $('scale-badge').hidden = !state.viewer?.hasModel;
  $('measure').disabled = !state.viewer?.hasModel || job?.status !== 'completed' || state.busy || !!job.scaling || !!job.completing;
  renderCompletion();
}
async function ensureViewer() {
  if (state.viewer) return state.viewer;
  if (!state.viewerPromise) state.viewerPromise = import('./viewer.mjs').then(({ SceneViewer }) => { state.viewer = new SceneViewer($('viewer'), { onPoints: handlePoints, onNavigationChange: renderNavigation, onViewChange: () => { $('viewpoint').value = 'free'; } }); return state.viewer; }).catch((error) => { state.viewerPromise = null; throw error; });
  return state.viewerPromise;
}
async function showSelectedModel() {
  const artifact = models(state.job).find((x) => x.url === $('model').value);
  const key = artifact ? `${state.selected}:${artifact.url}` : '';
  $('scale-badge').textContent = isMetricArtifact(artifact) ? 'Metric scale' : 'Scale unknown';
  if (key === state.artifactKey) return;
  state.artifactKey = key;
  cancelScale();
  if (!artifact) { state.viewer?.clear(); $('viewer-empty').hidden = false; $('viewer-hint').hidden = true; $('scale-badge').hidden = true; $('view-mode').disabled = true; $('fit').disabled = true; $('measure').disabled = true; $('walk').disabled = true; $('viewpoint').hidden = true; return; }
  $('viewer-loading').hidden = false;
  try {
    const viewer = await ensureViewer();
    if (state.artifactKey !== key) return;
    await viewer.load({ ...artifact, url: safeURL(artifact.url) }, usableArtifacts(state.job), () => state.artifactKey === key);
    if (state.artifactKey !== key) return;
    viewer.setMode($('view-mode').value);
    $('viewpoint').replaceChildren(); const overview = text('option', 'Overview'); overview.value = 'overview'; $('viewpoint').append(overview); const free = text('option', 'Free view'); free.value = 'free'; free.disabled = true; $('viewpoint').append(free);
    viewer.captureCameras.forEach((camera, index) => { const option = text('option', `Camera position ${index + 1}`); option.value = String(index); option.title = camera.image || ''; $('viewpoint').append(option); });
    $('viewpoint').hidden = !viewer.captureCameras.length;
    if (state.job?.method === 'photogrammetry' && viewer.captureCameras.length && !state.modelManual) { $('viewpoint').value = '0'; viewer.captureView(0); }
    $('viewer-empty').hidden = true; $('viewer-hint').hidden = false; $('scale-badge').hidden = false;
    $('view-mode').disabled = false; $('fit').disabled = false; $('walk').disabled = false; renderNavigation(viewer.navigationMode); $('measure').disabled = state.job?.status !== 'completed' || state.busy || !!state.job.scaling || !!state.job.completing;
  } catch (error) {
    if (state.artifactKey !== key) return;
    $('viewer-empty').hidden = false; $('viewer-message').textContent = 'Model preview unavailable'; $('viewer-note').textContent = error.message;
    $('fit').disabled = true; $('view-mode').disabled = true; $('measure').disabled = true; $('walk').disabled = true; $('viewpoint').hidden = true;
  } finally { if (state.artifactKey === key) $('viewer-loading').hidden = true; }
}
async function selectJob(id, { explicit = false } = {}) {
  if (explicit) { state.selectedExplicit = true; localStorage.setItem('reconstruction-job-explicit', id); }
  if (id !== state.selected) { state.selected = id; state.modelManual = false; localStorage.setItem('reconstruction-job', id); state.job = null; renderRuns(); renderJob(); }
  try { const result = await request(`/api/jobs/${encodeURIComponent(id)}`); if (state.selected !== id) return; state.job = result.job; renderJob(); }
  catch (error) { if (state.selected === id) showError('run-error', error); }
}
async function refresh() {
  if (state.refreshPromise) return state.refreshPromise;
  state.refreshPromise = (async () => {
    const result = await Promise.allSettled([request('/api/health'), request('/api/scans'), request('/api/jobs')]);
    state.offline = result[0].status === 'rejected';
    if (!state.offline) state.health = result[0].value;
    renderHealth();
    if (result[1].status === 'fulfilled') { state.scans = result[1].value.scans || []; renderScans(); }
    if (result[2].status === 'fulfilled') {
      state.jobs = result[2].value.jobs || [];
      const selected = state.jobs.find((x) => x.id === state.selected);
      // Experiments remain inspectable, but automatic fallback is a reusable run.
      // A separate marker distinguishes a deliberate selection from legacy defaults.
      if (!selected || (selected.scene_specific_fit === true && !state.selectedExplicit)) {
        state.selected = state.jobs.find((x) => x.scene_specific_fit !== true)?.id || '';
        state.selectedExplicit = false;
        localStorage.removeItem('reconstruction-job-explicit');
        if (state.selected) localStorage.setItem('reconstruction-job', state.selected);
        else localStorage.removeItem('reconstruction-job');
      }
      renderRuns();
      if (state.selected) await selectJob(state.selected); else { state.job = null; renderJob(); }
    }
  })().finally(() => { state.refreshPromise = null; });
  return state.refreshPromise;
}
async function runAction(action) {
  if (state.busy || !state.job) return;
  state.busy = true; renderStart(); renderStages(state.job); showError('run-error', null);
  try { const result = await post(`/api/jobs/${encodeURIComponent(state.job.id)}/${action}`); if (result.job?.id) await selectJob(result.job.id); await refresh(); }
  catch (error) { showError('run-error', error); }
  finally { state.busy = false; renderStart(); renderStages(state.job); }
}
let scalePoints = [];
function handlePoints(points) {
  scalePoints = points;
  $('scale-instruction').textContent = points.length === 2 ? 'Enter the measured distance' : points.length === 1 ? 'Select the second point' : 'Select two points on the model';
  $('save-scale').disabled = points.length !== 2 || state.busy || !!state.job?.scaling || !!state.job?.completing;
  $('scale-current').textContent = points.length === 2 ? 'Applying scale creates metric exports.' : 'Use a known physical distance.';
}
function cancelScale() { state.viewer?.setPicking(false); $('walk').disabled = !state.viewer?.hasModel; $('scale-panel').hidden = true; scalePoints = []; $('distance').value = ''; $('measure').textContent = 'Set scale'; }
$('measure').addEventListener('click', () => { if (!$('scale-panel').hidden) return cancelScale(); $('scale-panel').hidden = false; $('measure').textContent = 'Picking points'; state.viewer?.setPicking(true); $('walk').disabled = true; handlePoints([]); });
$('cancel-scale').addEventListener('click', cancelScale);
$('scale-form').addEventListener('submit', async (event) => {
  event.preventDefault(); const distance = Number($('distance').value);
  if (scalePoints.length !== 2 || !Number.isFinite(distance) || distance <= 0 || state.busy || state.job?.status !== 'completed') return;
  state.busy = true; $('save-scale').disabled = true; $('save-scale').textContent = 'Applying…'; $('cancel-scale').disabled = true; $('measure').disabled = true; showError('run-error', null);
  try { const result = await post(`/api/jobs/${encodeURIComponent(state.job.id)}/scale`, { point_a: scalePoints[0], point_b: scalePoints[1], distance_m: distance, source_space: new URL($('model').value, location.href).pathname.split('/').slice(3).join('/').startsWith('exports/scaled/') ? 'scaled' : 'raw' }, { timeoutMs: 120000 }); cancelScale(); state.artifactKey = ''; state.modelManual = false; $('model').value = ''; if (result.job) { state.job = result.job; renderJob(); } else await refresh(); }
  catch (error) { showError('run-error', error); }
  finally { state.busy = false; $('save-scale').disabled = scalePoints.length !== 2 || !!state.job?.scaling || !!state.job?.completing; $('save-scale').textContent = 'Apply'; $('cancel-scale').disabled = false; $('measure').disabled = !state.viewer?.hasModel || state.job?.status !== 'completed' || !!state.job?.scaling || !!state.job?.completing; renderStart(); renderCompletion(); }
});
$('complete').addEventListener('click', async () => {
  if ($('complete').disabled || !state.job) return;
  const id = state.job.id; const sourceSpace = selectedSpace();
  state.busy = true; state.completionPending = id; cancelScale(); renderStart(); renderJob(); showError('run-error', null);
  try {
    const result = await post(`/api/jobs/${encodeURIComponent(id)}/complete`, { source_space: sourceSpace }, { timeoutMs: 120000 });
    if (state.selected !== id) return;
    state.job = result.job;
    const completed = models(state.job).find((a) => a.kind === 'completed_visual_mesh' && isMetricArtifact(a) === (sourceSpace === 'scaled'));
    // Manual selection is retained across polling; finishing an export explicitly selects its new variant.
    state.modelManual = true; renderJob();
    if (completed) { $('model').value = completed.url; state.artifactKey = ''; showSelectedModel(); renderCompletion(); }
  } catch (error) { if (state.selected === id) showError('run-error', error); }
  finally { state.busy = false; state.completionPending = ''; renderStart(); renderJob(); }
});
$('new-job').addEventListener('submit', async (event) => {
  event.preventDefault(); if ($('reconstruct').disabled) return;
  state.busy = true; renderStart(); showError('create-error', null);
  try { const result = await post('/api/jobs', { scan_id: $('scan').value, method: $('method').value, quality: new FormData($('new-job')).get('quality') }); if (result.job?.id) { state.modelManual = false; state.selected = result.job.id; localStorage.setItem('reconstruction-job', state.selected); } await refresh(); }
  catch (error) { showError('create-error', error); }
  finally { state.busy = false; renderStart(); }
});
$('scan').addEventListener('change', renderStart);
function renderQuality() {
  const full = document.querySelector('input[name="quality"]:checked')?.value === 'full';
  $('quality-detail').textContent = $('method').value === 'photogrammetry' ? (full ? 'Original images · maximum detail' : 'Reduced mesh · faster preview') : (full ? 'CPU · higher detail' : 'Metal when available · quick inspection');
}
for (const input of document.querySelectorAll('input[name="quality"]')) input.addEventListener('change', renderQuality);
$('method').addEventListener('change', () => { renderQuality(); renderStart(); });
$('refresh').addEventListener('click', refresh); $('cancel').addEventListener('click', () => runAction('cancel')); $('retry').addEventListener('click', () => runAction('retry'));
$('model').addEventListener('change', () => { state.modelManual = true; showSelectedModel(); renderCompletion(); }); $('view-mode').addEventListener('change', () => state.viewer?.setMode($('view-mode').value)); $('fit').addEventListener('click', () => { state.viewer?.fit(); $('viewpoint').value = 'overview'; });
function renderNavigation(mode = 'orbit') {
  const exploring = mode === 'explore';
  $('walk').setAttribute('aria-pressed', String(exploring));
  $('walk').textContent = exploring ? 'Exit walk' : 'Walk';
  $('viewer-hint').textContent = exploring ? 'WASD move · Q/E down/up · Shift faster · drag to look · Esc exit' : 'Drag to orbit · scroll to zoom · right-drag to pan';
}
$('walk').addEventListener('click', () => { state.viewer?.setNavigationMode(state.viewer.navigationMode === 'explore' ? 'orbit' : 'explore'); });
$('viewpoint').addEventListener('change', () => { if ($('viewpoint').value === 'overview') state.viewer?.fit(); else if ($('viewpoint').value !== 'free') state.viewer?.captureView(Number($('viewpoint').value)); });
await refresh();
state.poll = setInterval(() => { if (!document.hidden) refresh(); }, 2500);
document.addEventListener('visibilitychange', () => { if (!document.hidden) refresh(); });
window.addEventListener('pagehide', () => { clearInterval(state.poll); state.viewer?.dispose(); });
