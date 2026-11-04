import test from 'node:test';
import assert from 'node:assert/strict';
import { encodeFrame } from './protocol.js';

// Exercise the actual browser controller with serial streams; no hardware or browser dependency.
test('browser controller covers session boundaries, 500ms reception watchdog, calibration progress, END recovery and imports', async () => {
  const elements = new Map();
  class Element {
    constructor() { this.children = []; this.events = {}; this.style = {}; this.value = ''; this.disabled = false; this._text = ''; }
    set textContent(v) { this._text = String(v); this.children = []; }
    get textContent() { return this._text + this.children.map(c => c.textContent ?? '').join(''); }
    addEventListener(name, fn) { this.events[name] = fn; }
    append(...children) { for (const child of children) { child.parent = this; this.children.push(child); } }
    prepend(child) { child.parent = this; this.children.unshift(child); }
    replaceChildren(...children) { this._text = ''; this.children = []; this.append(...children); }
    get lastChild() { return this.children.at(-1); }
    remove() { if (this.parent) this.parent.children.splice(this.parent.children.indexOf(this), 1); }
    getBoundingClientRect() { return { width: 400, height: 180 }; }
    getContext() { return new Proxy({}, { get: () => () => {} }); }
  }
  const element = id => { if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); };
  const original = new Map();
  function replace(name, value) { original.set(name, Object.getOwnPropertyDescriptor(globalThis, name)); Object.defineProperty(globalThis, name, { value, configurable: true, writable: true }); }
  let controller, opened = false; const commands = [];
  const serialPort = {
    async open(options) { assert.equal(options.baudRate, 460800); opened = true; this.readable = new ReadableStream({ start(c) { controller = c; } }); this.writable = new WritableStream({ write(bytes) { commands.push(new TextDecoder().decode(bytes).trim()); } }); },
    async setSignals() {}, async close() { opened = false; },
  };
  const timers = [];
  replace('document', { getElementById: element, createElement: () => new Element(), createTextNode: text => ({ textContent: text }) });
  replace('window', { addEventListener() {} }); replace('location', { search: '' });
  replace('navigator', { serial: { async requestPort() { return serialPort; } } });
  let wallTime = 0, renderFrame;
  replace('requestAnimationFrame', callback => { renderFrame = callback; return 0; });
  replace('performance', { now: () => wallTime });
  const tick = time => { wallTime = time; renderFrame(time); };
  element('windowSelect').value = '6'; element('speedSelect').value = '1';
  const realTimeout = globalThis.setTimeout;
  replace('setTimeout', (fn, delay) => { timers.push({ fn, delay }); return timers.length; });
  const settle = () => new Promise(resolve => realTimeout(resolve, 0));
  const meta = { session_id: 71, state: 'idle', calibrated: true, sample_hz: 200, g: 9.80665 };
  function data(seq) {
    const bytes = new Uint8Array(82), v = new DataView(bytes.buffer);
    v.setUint32(0, 71, true); v.setUint32(4, seq, true); v.setBigUint64(8, BigInt(1e6 + seq * 5000), true); v.setUint16(16, 3, true);
    v.setInt16(22, 8192, true); v.setFloat32(38, 9.80665, true); v.setFloat32(54, 1, true);
    return encodeFrame(1, bytes);
  }
  const summary = { session_id: 71, state: 'idle', calibrated: true, generated: 3, sent: 3, sample_fail: 0, stale_samples: 0, missed_deadlines: 0, queue_drops: 0, integration_gaps: 0, dt_count: 2, dt_mean_us: 5000, dt_std_us: 0, dt_min_us: 5000, dt_max_us: 5000 };
  try {
    await import('./app.js');
    assert.equal(element('startBtn').disabled, true);
    await element('connectBtn').events.click(); assert.equal(opened, true); assert.deepEqual(commands, ['INFO']);
    controller.enqueue(encodeFrame(2, { ...meta, state: 'calibrating', calibrated: false }));
    controller.enqueue(encodeFrame(4, { session_id: 71, state: 'calibrating', calibrated: false, calibration_progress: 120, calibration_reset_reason: 'accel_change', calibration_resets: 2 }));
    await settle(); assert.ok(element('deviceState').textContent.includes('120 / 600')); assert.ok(element('deviceState').textContent.includes('检测到移动'));
    tick(600); assert.ok(!element('connectionBadge').className.includes('stale')); // Calibration is not a DATA stream.
    controller.enqueue(encodeFrame(2, meta)); await settle(); assert.equal(element('startBtn').disabled, false);
    await element('startBtn').events.click(); assert.equal(commands.at(-1), 'START'); assert.equal(element('startBtn').disabled, true);
    controller.enqueue(encodeFrame(2, { ...meta, state: 'recording' })); await settle();
    tick(1099); assert.ok(!element('connectionBadge').className.includes('stale'));
    tick(1101); assert.ok(element('connectionBadge').className.includes('stale')); assert.ok(element('notice').textContent.includes('500 ms')); assert.ok(element('notice').className.includes('error'));
    controller.enqueue(data(0)); await settle(); tick(1135);
    assert.ok(!element('connectionBadge').className.includes('stale')); assert.ok(element('notice').textContent.includes('已恢复')); assert.equal(element('quatW').textContent, '1.00000');
    tick(1701); assert.ok(element('connectionBadge').className.includes('stale'));
    controller.enqueue(data(0)); await settle(); tick(1735); // A duplicate is not a new valid sample and cannot keep the receiver alive.
    assert.ok(element('connectionBadge').className.includes('stale'));
    controller.enqueue(data(2)); controller.enqueue(data(1)); await settle(); tick(1769);
    assert.ok(!element('connectionBadge').className.includes('stale'));
    await settle(); assert.equal(element('calibrateBtn').disabled, true); assert.equal(element('csvBtn').disabled, false);
    element('stopBtn').events.click(); await settle(); tick(2500); assert.ok(!element('connectionBadge').className.includes('stale')); // STOP suppresses waiting-for-END false alarms.
    controller.enqueue(encodeFrame(3, summary)); await settle();
    assert.equal(element('frameCount').textContent, '3'); assert.equal(element('lossRate').textContent, '0.000');
    assert.ok(element('qualityList').textContent.includes('1 / 1 / 1')); // Duplicate, out-of-order, and timestamp anomaly.
    assert.equal(element('startBtn').disabled, false); assert.equal(element('playBtn').disabled, false);
    tick(5000); assert.ok(!element('connectionBadge').className.includes('stale'));
    controller.enqueue(encodeFrame(3, summary)); await settle(); assert.equal(element('frameCount').textContent, '3');
    controller.enqueue(encodeFrame(2, { ...meta, state: 'calibrating', calibrated: false }));
    controller.enqueue(encodeFrame(3, summary)); await settle(); // Replayed old END must not turn an active calibration into idle.
    assert.ok(element('deviceState').textContent.includes('静态校准中')); assert.equal(element('startBtn').disabled, true);
    controller.enqueue(encodeFrame(2, meta)); await settle();
    element('stopBtn').events.click(); await settle(); assert.equal(commands.at(-1), 'STOP');
    await element('disconnectBtn').events.click(); assert.equal(opened, false); assert.equal(element('frameCount').textContent, '3'); assert.equal(element('jsonBtn').disabled, false);
    tick(7000); assert.ok(!element('connectionBadge').className.includes('stale'));

    // Root capture tool format, with no END: never turn missing loss into zero.
    const capture = { meta: { ...meta, session_id: 99 }, frames: [{ session_id: 99, seq: 0, t_us: 5000, flags: 3, raw_counts: [0, 0, 8192, 0, 0, 0], raw: [0, 0, 9.80665, 0, 0, 0], filtered: [0, 0, 9.80665, 0, 0, 0], q: [1, 0, 0, 0], position: [0, 0, 0] }], diagnostics: { crc_errors: 2 }, source: 'hardware', captured_at: '2026-11-04T12:00:00Z', end: null };
    await element('importFile').events.change({ target: { files: [{ name: 'recording.json', size: 1000, async text() { return JSON.stringify(capture); } }] } });
    assert.equal(element('frameCount').textContent, '1'); assert.equal(element('lossRate').textContent, '未知');
    assert.ok(element('sourceBanner').textContent.includes('离线回放')); assert.ok(element('qualityList').textContent.includes('2 / 0'));
    tick(10000); assert.ok(!element('connectionBadge').className.includes('stale')); assert.ok(!element('notice').textContent.includes('500 ms'));
  } finally {
    if (opened) await element('disconnectBtn').events.click();
    for (const [name, descriptor] of original) { if (descriptor) Object.defineProperty(globalThis, name, descriptor); else delete globalThis[name]; }
  }
});
