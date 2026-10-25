import test from 'node:test';
import assert from 'node:assert/strict';
import { mixKeys, LatestWriter } from './controls.mjs';

test('hold-to-drive wheel mixing and opposing-key stop', () => {
  const mix = (...keys) => mixKeys(new Set(keys), 128);
  assert.deepEqual(mix(), [0, 0]);
  assert.deepEqual(mix('KeyW'), [128, 128]);
  assert.deepEqual(mix('KeyS'), [-128, -128]);
  assert.deepEqual(mix('KeyA'), [-128, 128]);
  assert.deepEqual(mix('KeyD'), [128, -128]);
  assert.deepEqual(mix('KeyW', 'KeyS'), [0, 0]);
  assert.deepEqual(mix('KeyW', 'KeyA', 'KeyD'), [0, 0]);
  const leftArc = mix('KeyW', 'KeyA');
  assert.ok(leftArc[0] > 0 && leftArc[0] < leftArc[1]);
  assert.deepEqual(mix('KeyW', 'KeyD'), leftArc.toReversed());
  assert.deepEqual(mix('KeyS', 'KeyA'), [-128, -43]);
  for (let mask = 0; mask < 16; mask++) {
    const keys = ['KeyW','KeyA','KeyS','KeyD'].filter((_, i) => mask & (1 << i));
    assert.ok(mixKeys(new Set(keys), 255).every(value => Math.abs(value) <= 255));
  }
});

test('a blocked transport retains only the latest command, including emergency stop', async () => {
  const writes = [], finish = [];
  const writer = new LatestWriter({write(bytes) {
    writes.push(new TextDecoder().decode(bytes));
    return new Promise(resolve => finish.push(resolve));
  }}, error => { throw error; });
  writer.send('M 1 128 128');
  writer.send('M 2 200 200');
  writer.send('M 3 -128 -128');
  writer.send('STOP');
  assert.equal(writes.length, 1);
  finish.shift()();
  await new Promise(resolve => setImmediate(resolve));
  assert.deepEqual(writes, ['M 1 128 128\n', 'STOP\n']);
  finish.shift()();
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(writer.sending, false);
});

test('transport failure discards queued movement', async () => {
  let reported;
  const queue = new LatestWriter({async write() { throw new Error('unplugged'); }}, e => { reported = e; });
  queue.send('M 1 128 128');
  queue.send('M 2 128 128');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(reported.message, 'unplugged');
  assert.equal(queue.pending, null);
  assert.equal(queue.closed, true);
});
