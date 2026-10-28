import { captureViewPose, estimateDisplayFrame } from './orientation.mjs';
import { ExploreControls } from './explore-controls.mjs';
import * as THREE from '/vendor/three.module.min.js';
import { OrbitControls } from './vendor/controls/OrbitControls.js';
import { PLYLoader } from './vendor/loaders/PLYLoader.js';
import { OBJLoader } from './vendor/loaders/OBJLoader.js';
import { MTLLoader } from './vendor/loaders/MTLLoader.js';
import { GLTFLoader } from './vendor/loaders/GLTFLoader.js';

const allowedResource = (value) => {
  if (/^(blob:|data:)/.test(value)) return value;
  const url = new URL(value, location.href);
  if (url.origin !== location.origin || !url.pathname.startsWith('/artifacts/')) throw new Error('External model resources are disabled');
  return url.href;
};
function disposeObject(root) {
  const geometries = new Set(); const materials = new Set(); const textures = new Set();
  root.traverse((object) => {
    if (object.geometry) geometries.add(object.geometry);
    for (const material of [object.material, object.userData.originalMaterial, object.userData.displayMaterial].flat().filter(Boolean)) materials.add(material);
  });
  for (const material of materials) { for (const value of Object.values(material)) if (value?.isTexture) textures.add(value); material.dispose(); }
  for (const geometry of geometries) geometry.dispose();
  for (const texture of textures) texture.dispose();
}
export class SceneViewer {
  constructor(container, { onPoints, onNavigationChange, onViewChange } = {}) {
    this.container = container; this.onPoints = onPoints; this.onNavigationChange = onNavigationChange; this.onViewChange = onViewChange; this.navigationMode = 'orbit'; this.hasModel = false; this.model = null; this.loadVersion = 0; this.picking = false; this.points = []; this.radius = 1; this.needsRender = true; this.displayFrame = estimateDisplayFrame(null); this.captureCameras = [];
    try { this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'low-power' }); }
    catch { throw new Error('WebGL is unavailable. Model files can still be downloaded.'); }
    this.renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2)); this.renderer.setClearColor(0xf7f8fa); this.renderer.outputColorSpace = THREE.SRGBColorSpace; this.renderer.toneMapping = THREE.ACESFilmicToneMapping; this.renderer.toneMappingExposure = 1.15;
    this.renderer.domElement.setAttribute('aria-label', 'Interactive reconstructed model'); this.renderer.domElement.tabIndex = 0; this.renderer.domElement.setAttribute('aria-describedby', 'viewer-hint'); container.prepend(this.renderer.domElement);
    this.scene = new THREE.Scene(); this.camera = new THREE.PerspectiveCamera(42, 1, 0.001, 10000);
    this.hemisphere = new THREE.HemisphereLight(0xffffff, 0x8996a3, 2.5); this.scene.add(this.hemisphere);
    this.keyLight = new THREE.DirectionalLight(0xffffff, 2.2); this.keyLight.position.set(1, 3, 2); this.scene.add(this.keyLight);
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.4));
    this.resetControls();
    this.explore = new ExploreControls(this.camera, this.renderer.domElement, {
      getUp: () => this.displayFrame.up, getForward: () => this.displayFrame.forward, getRadius: () => this.radius,
      onChange: () => { this.needsRender = true; this.onViewChange?.(); }, onExit: () => this.setNavigationMode('orbit'),
    });
    this.markers = new THREE.Group(); this.scene.add(this.markers);
    this.observer = new ResizeObserver(() => this.resize()); this.observer.observe(container); this.resize();
    this.down = null;
    this.pointerDown = (event) => { this.down = { x: event.clientX, y: event.clientY, button: event.button }; };
    this.pointerUp = (event) => { if (this.picking && this.down?.button === 0 && Math.hypot(event.clientX - this.down.x, event.clientY - this.down.y) < 5) this.pick(event); this.down = null; };
    this.renderer.domElement.addEventListener('pointerdown', this.pointerDown); this.renderer.domElement.addEventListener('pointerup', this.pointerUp);
    this.lastFrame = performance.now();
    this.frame = (time = performance.now()) => {
      this.raf = requestAnimationFrame(this.frame);
      const dt = Math.min(.05, Math.max(0, (time - this.lastFrame) / 1000)); this.lastFrame = time;
      if (document.hidden) { this.explore.stop(); return; }
      if (this.navigationMode === 'explore') this.explore.update(dt); else this.controls.update();
      if (this.needsRender) { this.renderer.render(this.scene, this.camera); this.needsRender = false; }
    }; this.frame();
  }
  resetControls() {
    this.controls?.dispose();
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enabled = this.navigationMode === 'orbit';
    this.controls.addEventListener('start', () => this.onViewChange?.());
    this.controls.enableDamping = true; this.controls.dampingFactor = 0.09; this.controls.screenSpacePanning = true;
    this.controls.addEventListener('change', () => { this.needsRender = true; });
  }
  setNavigationMode(mode) {
    if (!['orbit', 'explore'].includes(mode) || (mode === 'explore' && (!this.hasModel || this.picking))) return false;
    if (mode === this.navigationMode) return true;
    this.navigationMode = mode;
    if (mode === 'explore') {
      // Pending orbit damping stays suspended; fresh controls are created on exit.
      this.controls.enabled = false; this.explore.activate();
    } else {
      this.explore.deactivate();
      const direction = this.camera.getWorldDirection(new THREE.Vector3());
      this.camera.up.fromArray(this.displayFrame.up); this.resetControls();
      this.controls.target.copy(this.camera.position).addScaledVector(direction, this.radius * .1);
      this.controls.minDistance = this.radius * .001; this.controls.maxDistance = this.radius * 100;
      this.controls.update();
    }
    this.container.classList.toggle('exploring', mode === 'explore');
    this.lastFrame = performance.now(); this.needsRender = true; this.onNavigationChange?.(mode); return true;
  }
  async readDisplayFrame(artifacts, selectedArtifact) {
    const scaled = /\/exports\/scaled\//.test(selectedArtifact.url);
    const metadata = artifacts.find((item) => item.kind === 'cameras' && /\/exports\/scaled\//.test(item.url) === scaled);
    if (!metadata) return { frame: estimateDisplayFrame(null), cameras: [] };
    const controller = new AbortController(); const timer = setTimeout(() => controller.abort(), 4000);
    try {
      const response = await fetch(allowedResource(metadata.url), { signal: controller.signal, credentials: 'same-origin', cache: 'no-store' });
      if (!response.ok || Number(response.headers.get('Content-Length')) > 4 * 1024 * 1024) return { frame: estimateDisplayFrame(null), cameras: [] };
      const data = await response.text();
      if (data.length > 4 * 1024 * 1024) return { frame: estimateDisplayFrame(null), cameras: [] };
      const document = JSON.parse(data);
      return { frame: estimateDisplayFrame(document), cameras: (document.cameras || []).filter((camera) => captureViewPose(camera)) };
    } catch { return { frame: estimateDisplayFrame(null), cameras: [] }; }
    finally { clearTimeout(timer); }
  }
  applyDisplayFrame(frame) {
    this.displayFrame = frame;
    this.camera.up.fromArray(frame.up);
    // OrbitControls caches its up-axis quaternion in its constructor.
    this.resetControls();
    this.hemisphere.position.copy(this.camera.up);
    const back = new THREE.Vector3().fromArray(frame.forward).negate();
    const right = new THREE.Vector3().crossVectors(this.camera.up, back).normalize();
    this.keyLight.position.copy(right).addScaledVector(this.camera.up, 3).addScaledVector(back, 2);
  }
  resize() { const width = this.container.clientWidth; const height = this.container.clientHeight; if (!width || !height) return; this.camera.aspect = width / height; this.camera.updateProjectionMatrix(); this.renderer.setSize(width, height, false); this.needsRender = true; }
  async load(artifact, artifacts, stillCurrent = () => true) {
    this.setNavigationMode('orbit');
    const version = ++this.loadVersion;
    const manager = new THREE.LoadingManager(); manager.setURLModifier(allowedResource);
    const url = allowedResource(artifact.url); const extension = new URL(url).pathname.split('.').at(-1).toLowerCase(); let model;
    try {
      const metadata = await this.readDisplayFrame(artifacts, artifact);
      if (version !== this.loadVersion || !stillCurrent()) return;
      if (extension === 'ply') {
        const geometry = await new PLYLoader(manager).loadAsync(url);
        const hasFaces = geometry.index !== null;
        if (hasFaces) { geometry.computeVertexNormals(); model = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: geometry.hasAttribute('color') ? 0xffffff : 0xc1c9d1, vertexColors: geometry.hasAttribute('color'), roughness: 0.94, metalness: 0, side: THREE.DoubleSide })); }
        else { model = new THREE.Points(geometry, new THREE.PointsMaterial({ size: 0.015, vertexColors: geometry.hasAttribute('color'), color: geometry.hasAttribute('color') ? 0xffffff : 0x879eb3, sizeAttenuation: true })); }
      } else if (extension === 'obj') {
        const loader = new OBJLoader(manager);
        const base = new URL('.', url).href;
        const expected = new URL(url).pathname.replace(/\.obj$/i, '.mtl');
        const material = artifacts.find((a) => { const u = new URL(a.url, location.href); return u.pathname === expected; }) || artifacts.find((a) => /\.mtl$/i.test(a.url) && new URL('.', new URL(a.url, location.href)).href === base);
        if (material) { const materials = await new MTLLoader(manager).loadAsync(allowedResource(material.url)); materials.preload(); loader.setMaterials(materials); }
        model = await loader.loadAsync(url);
      } else if (extension === 'glb' || extension === 'gltf') model = (await new GLTFLoader(manager).loadAsync(url)).scene;
      else throw new Error('Unsupported model format');
      if (version !== this.loadVersion || !stillCurrent()) { disposeObject(model); return; }
      this.clear(false); this.captureCameras = metadata.cameras; this.applyDisplayFrame(metadata.frame); this.model = model; this.scene.add(model); this.hasModel = true;
      model.traverse((object) => {
        if (!object.material) return;
        // Photogrammetry surfaces are often open sheets. Show either side when
        // inspecting from outside; this does not fill missing geometry.
        for (const material of [object.material].flat()) {
          material.side = THREE.DoubleSide;
          if ('metalness' in material) { material.metalness = 0; material.roughness = 1; }
          if (material.map && material.color) material.color.set(0xffffff);
          material.needsUpdate = true;
        }
        object.userData.originalMaterial = object.material;
      });
      const bounds = new THREE.Box3().setFromObject(model);
      if (bounds.isEmpty() || ![bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z].every(Number.isFinite)) throw new Error('The model has no finite geometry');
      this.radius = Math.max(bounds.getSize(new THREE.Vector3()).length() / 2, 0.001);
      model.traverse((object) => { if (object.isPoints) object.material.size = this.radius * 0.004; });
      this.fit();
    } catch (error) { if (model && model !== this.model) disposeObject(model); if (version === this.loadVersion) this.clear(false); throw error; }
  }
  fit() {
    if (!this.model) return;
    this.setNavigationMode('orbit');
    this.applyDisplayFrame(this.displayFrame); this.camera.fov = 42;
    const bounds = new THREE.Box3().setFromObject(this.model); const center = bounds.getCenter(new THREE.Vector3());
    const fov = this.camera.fov * Math.PI / 180; const effectiveFov = Math.min(fov, 2 * Math.atan(Math.tan(fov / 2) * this.camera.aspect));
    const distance = this.radius / Math.sin(effectiveFov / 2) * 1.15;
    this.camera.near = Math.max(this.radius / 10000, 0.00001); this.camera.far = Math.max(this.radius * 1000, 100); this.camera.updateProjectionMatrix();
    const up = this.camera.up;
    const back = new THREE.Vector3().fromArray(this.displayFrame.forward).negate();
    const right = new THREE.Vector3().crossVectors(up, back).normalize();
    const direction = back.addScaledVector(right, .6).addScaledVector(up, .55).normalize();
    this.camera.position.copy(center).addScaledVector(direction, distance);
    this.controls.target.copy(center); this.controls.minDistance = this.radius * 0.01; this.controls.maxDistance = this.radius * 100; this.controls.update(); this.needsRender = true;
  }
  captureView(index) {
    const pose = captureViewPose(this.captureCameras[index]);
    if (!this.model || !pose) return false;
    this.setNavigationMode('orbit');
    this.camera.up.fromArray(pose.up); this.resetControls();
    // Preserve the physical capture position. This inspection view uses a
    // 60-degree vertical field of view, rather than claiming pixel alignment
    // with portrait source photographs in a landscape viewport.
    this.camera.fov = 60; this.camera.updateProjectionMatrix();
    this.camera.position.fromArray(pose.center);
    this.controls.target.copy(this.camera.position).addScaledVector(new THREE.Vector3().fromArray(pose.forward), this.radius * .1);
    this.controls.minDistance = this.radius * .001; this.controls.maxDistance = this.radius * 100;
    this.controls.update(); this.needsRender = true; return true;
  }
  setMode(mode) {
    if (!this.model) return;
    this.model.traverse((object) => {
      if (!object.material) return;
      const original = object.userData.originalMaterial;
      if (object.userData.displayMaterial) { for (const material of [object.userData.displayMaterial].flat()) material.dispose(); delete object.userData.displayMaterial; }
      if (mode === 'texture' || !object.isMesh) { object.material = original; return; }
      const material = new THREE.MeshStandardMaterial({ color: mode === 'wireframe' ? 0x879aab : 0xb7c4d1, roughness: 0.94, metalness: 0, side: THREE.DoubleSide, wireframe: mode === 'wireframe', polygonOffset: true, polygonOffsetFactor: 1 });
      object.userData.displayMaterial = material; object.material = material;
    }); this.needsRender = true;
  }
  setPicking(enabled) { if (enabled) this.setNavigationMode('orbit'); this.picking = !!enabled; this.container.classList.toggle('picking', this.picking); this.points = []; this.clearMarkers(); this.onPoints?.([]); }
  pick(event) {
    if (!this.model) return;
    const rect = this.renderer.domElement.getBoundingClientRect(); const mouse = new THREE.Vector2(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
    const ray = new THREE.Raycaster(); ray.params.Points.threshold = this.radius * 0.015; ray.setFromCamera(mouse, this.camera);
    const hit = ray.intersectObject(this.model, true)[0]; if (!hit) return;
    if (this.points.length === 2) { this.points = []; this.clearMarkers(); }
    const point = hit.point; this.points.push(point.toArray());
    const marker = new THREE.Mesh(new THREE.SphereGeometry(this.radius * 0.009, 14, 10), new THREE.MeshBasicMaterial({ color: 0x5e829d, depthTest: false, transparent: true, opacity: 0.95 })); marker.renderOrder = 100; marker.position.copy(point); this.markers.add(marker);
    if (this.points.length === 2) { const geometry = new THREE.BufferGeometry().setFromPoints(this.points.map((p) => new THREE.Vector3(...p))); const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x5e829d, depthTest: false })); line.renderOrder = 100; this.markers.add(line); }
    this.needsRender = true; this.onPoints?.(this.points.map((p) => [...p]));
  }
  clearMarkers() { while (this.markers.children.length) { const child = this.markers.children[0]; this.markers.remove(child); disposeObject(child); } this.needsRender = true; }
  clear(invalidate = true) { this.setNavigationMode('orbit'); this.explore.stop(); if (invalidate) this.loadVersion++; if (this.model) { this.scene.remove(this.model); disposeObject(this.model); this.model = null; } this.hasModel = false; this.captureCameras = []; this.setPicking(false); this.needsRender = true; }
  dispose() { cancelAnimationFrame(this.raf); this.observer.disconnect(); this.clear(); this.explore.dispose(); this.controls.dispose(); this.renderer.domElement.removeEventListener('pointerdown', this.pointerDown); this.renderer.domElement.removeEventListener('pointerup', this.pointerUp); this.renderer.dispose(); this.renderer.domElement.remove(); }
}
