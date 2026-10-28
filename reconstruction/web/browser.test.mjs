import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.dirname(fileURLToPath(import.meta.url));
const now = new Date().toISOString();
const texturedCube = await readFile(path.join(root, 'fixtures/textured-cube.glb'));
const cube = `ply\nformat ascii 1.0\nelement vertex 8\nproperty float x\nproperty float y\nproperty float z\nelement face 12\nproperty list uchar int vertex_indices\nend_header\n-1 -1 -1\n1 -1 -1\n1 1 -1\n-1 1 -1\n-1 -1 1\n1 -1 1\n1 1 1\n-1 1 1\n3 0 1 2\n3 0 2 3\n3 4 6 5\n3 4 7 6\n3 0 4 5\n3 0 5 1\n3 3 2 6\n3 3 6 7\n3 1 5 6\n3 1 6 2\n3 0 3 7\n3 0 7 4\n`;
const state = { jobs: [], next: 0, bodies: [], missing: false, keepRunning: false, scaled: false, requests: [] };
const stages = ['Prepare images', 'Camera alignment', 'Dense depth', 'Surface mesh', 'Texture', 'Exports'];
const job = () => state.jobs[0];
const server = createServer(async (req, res) => {
  res.setHeader('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data: blob:; connect-src 'self' blob: data:; object-src 'none'; base-uri 'none'");
  const url = new URL(req.url, 'http://localhost');
  state.requests.push(`${req.method} ${url.pathname}`);
  let body = ''; for await (const chunk of req) body += chunk;
  const json = (value, status = 200) => { res.writeHead(status, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' }); res.end(JSON.stringify(value)); };
  if (url.pathname === '/api/health') return json({ status: state.missing ? 'missing-tools' : 'ready', capture_url: 'http://127.0.0.1:8765/', engines: { colmap: { available: !state.missing, version: 'test' }, openmvs: { available: !state.missing, version: 'test', metal_compiled: true } } });
  if (url.pathname === '/api/scans') return json({ scans: [{ id: 'fixture-scan', created_at: now, photo_count: 15, position_count: 2, finished_positions: 2, usable: true }] });
  if (url.pathname === '/api/jobs' && req.method === 'GET') return json({ jobs: state.jobs });
  if (url.pathname === '/api/jobs' && req.method === 'POST') {
    const input = JSON.parse(body); state.bodies.push(input); const id = `job-${++state.next}`;
    state.jobs.unshift({ id, scan_id: input.scan_id, quality: input.quality, status: 'running', stage: 'dense', stages: stages.map((label, i) => ({ id: `stage${i}`, label, status: i < 2 ? 'completed' : i === 2 ? 'running' : 'pending' })), progress: 0.4, created_at: now, updated_at: now, backend: { requested: 'metal', active: 'metal' }, calibration: { status: 'unscaled' }, metrics: { registered_images: 15, input_images: 15 }, artifacts: [{name:'sparse.ply',kind:'point_cloud',url:`/artifacts/${id}/sparse.ply`,size:cube.length}], warnings: [], log_tail: ['Prepared 15 photographs', 'Metal fixture backend selected'] });
    const created = job();
    if (!state.keepRunning) setTimeout(() => { created.status = 'completed'; created.progress = 1; created.stage = 'exports'; created.stages.forEach((s) => { s.status = 'completed'; }); created.artifacts = [...created.artifacts, {name:'cameras.json',kind:'cameras',url:`/artifacts/${id}/exports/cameras.json`}, { name: 'scene_refined.ply', kind: 'mesh', url: `/artifacts/${id}/scene_refined.ply`, size: cube.length }, {name:'scene.glb',kind:'visual_mesh',url:`/artifacts/${id}/scene.glb`,size:texturedCube.length}]; created.metrics.visual_mesh = {vertices:8,triangles:12}; }, 400);
    return json({ job: created }, 201);
  }
  const match = url.pathname.match(/^\/api\/jobs\/(job-\d+)(?:\/(cancel|retry|scale|complete))?$/);
  if (match) {
    const current = state.jobs.find((j) => j.id === match[1]); if (!current) return json({ error: 'Not found' }, 404);
    if (req.method === 'POST' && match[2] === 'cancel') { current.status = 'cancelled'; return json({ job: current }); }
    if (req.method === 'POST' && match[2] === 'retry') { current.status = 'running'; return json({ job: current }); }
    if (req.method === 'POST' && match[2] === 'complete') {
      const input = JSON.parse(body); state.bodies.push(input); assert.ok(['raw', 'scaled'].includes(input.source_space));
      current.completing = true; await new Promise(resolve => setTimeout(resolve, 250));
      const prefix = `/artifacts/${current.id}/exports/${input.source_space === 'scaled' ? 'scaled/' : ''}completed/fixture-v1/`;
      const units = input.source_space === 'scaled' ? 'metres' : 'arbitrary';
      current.artifacts = current.artifacts.filter(a => !a.url.startsWith(prefix));
      current.artifacts.push({name:'scene-completed.glb',kind:'completed_visual_mesh',units,url:prefix+'scene-completed.glb',size:texturedCube.length}, {name:'inferred.glb',kind:'inferred_surface',units,url:prefix+'inferred.glb',size:texturedCube.length});
      current.completion ||= {}; current.completion[input.source_space] = {source_space:input.source_space,filled_loops:2,skipped_loops:1,added_triangles:4,original_vertices:8,original_triangles:12,loops:[{status:'filled',boundary_vertices:4},{status:'filled',boundary_vertices:4}],topology_after:{components:1},collision_validated:false};
      current.completing = false; return json({job:current});
    }
    if (req.method === 'POST' && match[2] === 'scale') { const input = JSON.parse(body); state.bodies.push(input); assert.equal(input.point_a.length, 3); assert.equal(input.point_b.length, 3); assert.equal(input.distance_m, 2.5); assert.equal(input.source_space, state.scaled ? 'scaled' : 'raw'); current.calibration = { status: 'scaled', meters_per_unit: 1 }; if (!current.artifacts.some(a=>a.units==='metres')) current.artifacts.push({ name: 'scene_refined.ply', kind: 'mesh', units: 'metres', url: `/artifacts/${current.id}/exports/scaled/scene_refined.ply`, size: cube.length }); if (!current.artifacts.some(a=>a.units==='metres' && a.name==='scene.glb')) current.artifacts.push({name:'scene.glb',kind:'visual_mesh',units:'metres',url:`/artifacts/${current.id}/exports/scaled/scene.glb`,size:texturedCube.length}); if (!current.artifacts.some(a=>a.kind==='cameras' && a.units==='metres')) current.artifacts.push({name:'cameras.json',kind:'cameras',units:'metres',url:`/artifacts/${current.id}/exports/scaled/cameras.json`}); state.scaled = true; return json({ job: current }); }
    return json({ job: current });
  }
  if (/^\/artifacts\/job-\d+\/exports\/(?:scaled\/)?cameras\.json$/.test(url.pathname)) { const scaled = url.pathname.includes('/scaled/'); return json({up_axis:'Y',cameras:[{center:scaled?[2,0,4]:[1,0,2],rotation_world_to_camera:[[1,0,0],[0,1,0],[0,0,1]],translation:scaled?[-2,0,-4]:[-1,0,-2],image:'fixture.jpg'}]}); }
  if (/^\/artifacts\/job-\d+\/exports\/(?:scaled\/)?completed\/fixture-v1\/(?:scene-completed|inferred)\.glb$/.test(url.pathname)) { res.writeHead(200, { 'Content-Type': 'model/gltf-binary' }); return res.end(texturedCube); }
  if (/^\/artifacts\/job-\d+\/(?:exports\/scaled\/)?scene.glb$/.test(url.pathname)) { res.writeHead(200, { 'Content-Type': 'model/gltf-binary' }); return res.end(texturedCube); }
  if (/^\/artifacts\/job-\d+\/(?:exports\/scaled\/)?(?:scene_refined|sparse).ply$/.test(url.pathname)) { res.writeHead(200, { 'Content-Type': 'application/octet-stream' }); return res.end(cube); }
  const relative = url.pathname === '/' ? 'index.html' : decodeURIComponent(url.pathname).slice(1);
  if (relative.includes('..')) { res.writeHead(403); return res.end(); }
  try { const file = await readFile(path.join(root, relative)); const ext = path.extname(relative); res.writeHead(200, { 'Content-Type': ({ '.html': 'text/html', '.svg':'image/svg+xml', '.css': 'text/css', '.mjs': 'text/javascript', '.js': 'text/javascript' })[ext] || 'application/octet-stream' }); res.end(file); }
  catch { res.writeHead(404); res.end(); }
});
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
let browser;
try {
  browser = await chromium.launch({ headless: true, ...(process.env.CHROMIUM_PATH ? { executablePath: process.env.CHROMIUM_PATH } : {}), args: ['--enable-unsafe-swiftshader'] });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 }, deviceScaleFactor: 1 });
  const errors = []; const remote = [];
  page.on('pageerror', (e) => errors.push(e.message));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  page.on('request', (r) => { if (!r.url().startsWith(`http://127.0.0.1:${server.address().port}/`) && !r.url().startsWith('data:') && !r.url().startsWith('blob:')) remote.push(r.url()); });
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.getByText('Connected', { exact: true }).waitFor();
  await page.evaluate(async () => { const {SceneViewer}=await import('/viewer.mjs'); const load=SceneViewer.prototype.load; SceneViewer.prototype.load=async function(...args){const result=await load.apply(this,args);window.__testViewer=this;return result;}; });
  assert.equal(await page.locator('#reconstruct').isEnabled(), true);
  assert.match(await page.locator('#scan-detail').textContent(), /15 photos · 2 positions/);
  await page.getByText('Full', { exact: true }).click();
  await page.locator('#reconstruct').click();
  await page.locator('#cancel').waitFor({ state: 'visible' });
  assert.equal(state.bodies[0].quality, 'full'); assert.equal(state.bodies[0].scan_id, 'fixture-scan');
  await page.locator('#viewer canvas').waitFor();
  await page.locator('#viewer-empty').waitFor({ state: 'hidden', timeout: 15000 });
  await page.waitForFunction(()=>document.querySelector('#model').value.endsWith('scene.glb'));
  await page.locator('#measure').waitFor({ state: 'visible' });
  assert.equal(await page.locator('#measure').isEnabled(), true);
  assert.match(await page.locator('#metrics').textContent(), /Aligned15\/15Vertices8Faces12/);
  assert.equal(await page.locator('#artifacts a').count(), 3);
  job().metrics.surface_topology = { assessment: 'fragmented', components: 25 };
  await page.locator('#refresh').click();
  await page.getByText('Fragmented surface', { exact: true }).waitFor();
  assert.match(await page.locator('#metrics').textContent(), /Surface pieces25/);

  await page.locator('#viewer-loading').waitFor({state:'hidden'});
  assert.equal(await page.evaluate(()=>{let textured=0;window.__testViewer.model.traverse(o=>{if(o.material?.map?.image?.width===8)textured++});return textured}),1,'GLB embedded texture must load under CSP');
  assert.equal(await page.evaluate(()=>{let sided=true;window.__testViewer.model.traverse(o=>{if(o.isMesh)for(const m of [o.material].flat())sided&&=m.side===2;});return sided}),true,'open mesh sheets must display from either side');
  assert.equal(await page.locator('#complete').isEnabled(), true);
  await page.locator('#complete').click();
  await page.getByRole('button',{name:'Filling…',exact:true}).waitFor();
  assert.equal(await page.locator('#complete').isDisabled(), true);
  await page.waitForFunction(() => document.querySelector('#model').value.endsWith('/completed/fixture-v1/scene-completed.glb'));
  await page.locator('#viewer-loading').waitFor({state:'hidden'});
  assert.equal(state.bodies.at(-1).source_space,'raw');
  assert.deepEqual(await page.evaluate(()=>window.__testViewer.captureCameras[0].center),[1,0,2],'completed raw model must use raw cameras');
  assert.match(await page.locator('#completion-summary').textContent(),/2 boundary loops filled · 1 unfilled boundary · 4 estimated triangles/);
  job().label='Refined shapes';
  await page.locator('#refresh').click();
  await page.getByRole('button',{name:/Proportion correction · Fragmented/}).waitFor();
  assert.match(await page.locator('#warnings').textContent(),/Local furniture shapes and floor\/ceiling coverage remain unresolved/);
  assert.match(await page.locator('#metrics').textContent(), /Vertices16Faces16Surface pieces1/);
  const rawCompleted = `/artifacts/${job().id}/exports/completed/fixture-v1/scene-completed.glb`;
  const rawInferred = `/artifacts/${job().id}/exports/completed/fixture-v1/inferred.glb`;
  assert.match(await page.locator(`#model option[value="${rawCompleted}"]`).textContent(), /Gap fill · estimated/);
  await page.locator('#model').selectOption(rawInferred);
  await page.locator('#viewer-loading').waitFor({state:'hidden'});
  await page.locator('#refresh').click();
  await page.waitForTimeout(200);
  assert.equal(await page.locator('#model').inputValue(),rawInferred,'polling must preserve inferred-only inspection');
  assert.match(await page.locator('#metrics').textContent(), /Vertices8Faces4Surface pieces2/);
  await page.locator('#model').selectOption(`/artifacts/${job().id}/scene.glb`);
  await page.locator('#viewer-loading').waitFor({state:'hidden'});
  await page.locator('#refresh').click();
  await page.waitForTimeout(200);
  assert.equal(await page.locator('#model').inputValue(),`/artifacts/${job().id}/scene.glb`,'polling must preserve the original-model selection');
  assert.equal(await page.locator('#completion-summary').isHidden(),true,'gap-fill report must not describe an original model as completed');
  await page.locator('#view-mode').selectOption('wireframe');
  await page.locator('#view-mode').selectOption('solid');
  await page.locator('#fit').click();
  await page.locator('#measure').click();
  const canvas = page.locator('#viewer canvas'); const bounds = await canvas.boundingBox();
  await page.mouse.click(bounds.x + bounds.width / 2 - 35, bounds.y + bounds.height / 2);
  await page.mouse.click(bounds.x + bounds.width / 2 + 35, bounds.y + bounds.height / 2);
  await page.getByText('Enter the measured distance', { exact: true }).waitFor();
  await page.locator('#distance').fill('2.5'); await page.locator('#save-scale').click();
  await page.locator('#model').selectOption(`/artifacts/${job().id}/exports/scaled/scene.glb`);
  await page.getByText('Metric scale', { exact: true }).waitFor(); assert.equal(state.scaled, true);
  await page.locator('#model').selectOption(`/artifacts/${job().id}/scene.glb`);
  assert.equal(await page.locator('#scale-badge').textContent(),'Scale unknown');
  await page.locator('#model').selectOption(`/artifacts/${job().id}/exports/scaled/scene.glb`);
  assert.equal(await page.locator('#scale-badge').textContent(),'Metric scale');
  await page.locator('#viewer-loading').waitFor({state:'hidden'});
  await page.locator('#measure').click();
  const metricBounds=await canvas.boundingBox();
  await page.mouse.click(metricBounds.x+metricBounds.width/2-35,metricBounds.y+metricBounds.height/2);
  await page.mouse.click(metricBounds.x+metricBounds.width/2+35,metricBounds.y+metricBounds.height/2);
  await page.getByText('Enter the measured distance',{exact:true}).waitFor();
  await page.locator('#distance').fill('2.5'); await page.locator('#save-scale').click();
  await page.locator('#scale-panel').waitFor({state:'hidden'});
  assert.equal(state.bodies.at(-1).source_space,'scaled');
  await page.locator('#complete').click();
  await page.waitForFunction(() => document.querySelector('#model').value.includes('/exports/scaled/completed/'));
  await page.locator('#viewer-loading').waitFor({state:'hidden'});
  assert.equal(state.bodies.at(-1).source_space,'scaled');
  assert.equal(await page.locator('#scale-badge').textContent(),'Metric scale');
  assert.deepEqual(await page.evaluate(()=>window.__testViewer.captureCameras[0].center),[2,0,4],'nested metric completion must use metric cameras');
  await page.locator('#refresh').click();
  await page.waitForTimeout(200);
  assert.match(await page.locator('#model').inputValue(),/\/exports\/scaled\/completed\/fixture-v1\/scene-completed\.glb$/);
  assert.equal(job().completion.raw.source_space,'raw','raw completion must survive a separate metric variant');
  await page.locator('#log-panel summary').click();
  assert.match(await page.locator('#logs').textContent(), /Prepared 15 photographs/);
  await page.screenshot({ path: '/private/tmp/rover-reconstruction-desktop.png', fullPage: true });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({ path: '/private/tmp/rover-reconstruction-mobile.png', fullPage: true });
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth), true, 'mobile page must not overflow horizontally');
  state.keepRunning = true;
  await page.setViewportSize({ width: 1440, height: 1000 });
  await page.locator('#reconstruct').click(); await page.locator('#cancel').waitFor({ state: 'visible' }); await page.locator('#cancel').click();
  await page.locator('#retry').waitFor({ state: 'visible' });
  assert.equal(job().status, 'cancelled');
  await page.locator('#retry').click(); await page.locator('#cancel').waitFor({ state: 'visible' }); assert.equal(job().status, 'running');
  state.missing = true; await page.locator('#refresh').click(); await page.getByText('Tools unavailable', { exact: true }).waitFor(); assert.equal(await page.locator('#reconstruct').isDisabled(), true);
  assert.deepEqual(errors, []); assert.deepEqual(remote, []);
  console.log(JSON.stringify({ test: 'reconstruction browser', create: 'passed', native_ply_viewer: 'passed', textured_glb: 'passed', embedded_texture_csp: 'passed', geometry_mode: 'passed', scale_point_selection: 'passed', cancel_retry: 'passed', gap_completion: 'raw and metric variants', inferred_only: 'passed', manual_model_polling: 'preserved', mobile_layout: 'passed', missing_engines: 'passed', external_requests: remote.length, hardware_motion: false, screenshots: ['/private/tmp/rover-reconstruction-desktop.png', '/private/tmp/rover-reconstruction-mobile.png'] }));
} finally { await browser?.close(); await new Promise((resolve) => server.close(resolve)); }
