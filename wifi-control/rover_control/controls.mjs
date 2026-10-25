export const DRIVE_KEYS = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD']);

export function mixKeys(keys, speed = 128) {
  const w = keys.has('KeyW'), s = keys.has('KeyS');
  const a = keys.has('KeyA'), d = keys.has('KeyD');
  if ((w && s) || (a && d)) return [0, 0];
  const throttle = Number(w) - Number(s);
  const turn = Number(d) - Number(a);
  if (!throttle && !turn) return [0, 0];
  const yaw = turn * (throttle ? 0.5 : 1);
  const left = throttle + yaw, right = throttle - yaw;
  const scale = Math.max(1, Math.abs(left), Math.abs(right));
  const duty = Math.max(0, Math.min(255, Math.round(speed)));
  return [Math.round(left / scale * duty), Math.round(right / scale * duty)];
}

export function directionName(left, right) {
  if (!left && !right) return 'Stopped';
  if (left === right) return left > 0 ? 'Forward' : 'Reverse';
  if (left < 0 && right > 0) return 'Turn left';
  if (left > 0 && right < 0) return 'Turn right';
  return 'Arc ' + (left < right ? 'left' : 'right');
}

// At most one write in flight and one replaceable latest command. No movement queue.
export class LatestWriter {
  constructor(writer, onError) {
    this.writer = writer; this.onError = onError;
    this.pending = null; this.sending = false; this.closed = false;
  }
  send(line) {
    if (this.closed) return;
    this.pending = line;
    if (!this.sending) void this.flush();
  }
  async flush() {
    this.sending = true;
    try {
      while (this.pending !== null && !this.closed) {
        const line = this.pending; this.pending = null;
        await this.writer.write(new TextEncoder().encode(line + '\n'));
      }
    } catch (error) {
      this.closed = true; this.pending = null; this.onError(error);
    } finally { this.sending = false; }
  }
}
