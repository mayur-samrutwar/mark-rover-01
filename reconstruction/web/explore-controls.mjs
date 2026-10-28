import * as THREE from '/vendor/three.module.min.js';

const MOVE_KEYS = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE', 'ShiftLeft', 'ShiftRight']);
const MAX_PITCH = Math.PI / 2 - .02;

// Camera-only inspection. The reconstructed mesh and its coordinate frame stay
// untouched; these controls do not implement gravity or collision physics.
export class ExploreControls {
  constructor(camera, element, { getUp, getForward, getRadius, onChange, onExit }) {
    Object.assign(this, { camera, element, getUp, getForward, getRadius, onChange, onExit });
    this.enabled = false; this.keys = new Set(); this.drag = null;
    this.up = new THREE.Vector3(0, 1, 0); this.forward = new THREE.Vector3(0, 0, -1); this.pitch = 0;
    this.keyDown = (event) => {
      if (!this.enabled || document.activeElement !== this.element || event.ctrlKey || event.metaKey || event.altKey) return;
      if (event.code === 'Escape') { event.preventDefault(); this.onExit(); return; }
      if (MOVE_KEYS.has(event.code)) { event.preventDefault(); this.keys.add(event.code); }
    };
    this.keyUp = (event) => { this.keys.delete(event.code); };
    this.onBlur = () => this.stop();
    this.onVisibility = () => { if (document.hidden) this.stop(); };
    this.pointerDown = (event) => {
      if (!this.enabled || event.button !== 0) return;
      event.preventDefault(); this.element.focus({ preventScroll: true });
      this.drag = { id: event.pointerId, x: event.clientX, y: event.clientY };
      this.element.setPointerCapture(event.pointerId); this.element.classList.add('looking');
    };
    this.pointerMove = (event) => {
      if (!this.enabled || this.drag?.id !== event.pointerId) return;
      const dx = event.clientX - this.drag.x; const dy = event.clientY - this.drag.y;
      this.drag.x = event.clientX; this.drag.y = event.clientY;
      this.forward.applyAxisAngle(this.up, -dx * .003).normalize();
      this.pitch = THREE.MathUtils.clamp(this.pitch - dy * .003, -MAX_PITCH, MAX_PITCH);
      this.look(); this.onChange();
    };
    this.pointerUp = (event) => { if (this.drag?.id === event.pointerId) this.releaseDrag(); };
    element.addEventListener('keydown', this.keyDown);
    window.addEventListener('keyup', this.keyUp);
    element.addEventListener('blur', this.onBlur); window.addEventListener('blur', this.onBlur);
    document.addEventListener('visibilitychange', this.onVisibility);
    element.addEventListener('pointerdown', this.pointerDown);
    element.addEventListener('pointermove', this.pointerMove);
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) element.addEventListener(type, this.pointerUp);
  }
  activate() {
    this.stop(); this.up.fromArray(this.getUp()).normalize();
    const direction = this.camera.getWorldDirection(new THREE.Vector3()).normalize();
    this.pitch = THREE.MathUtils.clamp(Math.asin(THREE.MathUtils.clamp(direction.dot(this.up), -1, 1)), -MAX_PITCH, MAX_PITCH);
    this.forward.copy(direction).addScaledVector(this.up, -direction.dot(this.up));
    if (this.forward.lengthSq() < 1e-8) this.forward.fromArray(this.getForward()).addScaledVector(this.up, -new THREE.Vector3().fromArray(this.getForward()).dot(this.up));
    this.forward.normalize(); this.enabled = true; this.look();
    this.element.focus({ preventScroll: true });
  }
  deactivate() { this.enabled = false; this.stop(); }
  releaseDrag() {
    const drag = this.drag; this.drag = null; this.element.classList.remove('looking');
    if (drag && this.element.hasPointerCapture(drag.id)) this.element.releasePointerCapture(drag.id);
  }
  stop() { this.keys.clear(); this.releaseDrag(); }
  look() {
    const direction = this.forward.clone().multiplyScalar(Math.cos(this.pitch)).addScaledVector(this.up, Math.sin(this.pitch));
    this.camera.up.copy(this.up); this.camera.lookAt(this.camera.position.clone().add(direction));
  }
  update(dt) {
    if (!this.enabled || !Number.isFinite(dt)) return;
    if (document.hidden || document.activeElement !== this.element) { this.stop(); return; }
    const axis = (positive, negative) => Number(this.keys.has(positive)) - Number(this.keys.has(negative));
    const right = new THREE.Vector3().crossVectors(this.forward, this.up).normalize();
    const move = this.forward.clone().multiplyScalar(axis('KeyW', 'KeyS')).addScaledVector(right, axis('KeyD', 'KeyA')).addScaledVector(this.up, axis('KeyE', 'KeyQ'));
    if (move.lengthSq() === 0) return;
    const boost = this.keys.has('ShiftLeft') || this.keys.has('ShiftRight') ? 3 : 1;
    this.camera.position.addScaledVector(move.normalize(), Math.max(.001, this.getRadius()) * .2 * boost * THREE.MathUtils.clamp(dt, 0, .05));
    this.look(); this.onChange();
  }
  dispose() {
    this.deactivate();
    this.element.removeEventListener('keydown', this.keyDown); window.removeEventListener('keyup', this.keyUp);
    this.element.removeEventListener('blur', this.onBlur); window.removeEventListener('blur', this.onBlur);
    document.removeEventListener('visibilitychange', this.onVisibility);
    this.element.removeEventListener('pointerdown', this.pointerDown); this.element.removeEventListener('pointermove', this.pointerMove);
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) this.element.removeEventListener(type, this.pointerUp);
  }
}
