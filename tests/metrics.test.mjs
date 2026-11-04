import test from 'node:test';
import assert from 'node:assert/strict';
import { analyzeSession } from '../web/metrics.js';

const meta = { session_id: 17, sample_hz: 200, g: 9.80665, calibrated: true };
function makeFrames(seconds, bias = 1) {
  return Array.from({ length: Math.round(seconds * 200) + 1 }, (_, seq) => ({
    session_id: 17, seq, t_us: 1e6 + seq * 5000, flags: 3,
    raw: [0, 0, 9.80665, bias, 0, 0], filtered: [0, 0, 9.80665, 0, 0, 0],
  }));
}
function makeEnd(frames, extra = {}) {
  return { session_id: 17, generated: frames.length, sent: frames.length,
    first_t_us: frames[0].t_us, last_t_us: frames.at(-1).t_us,
    sample_fail: 0, stale_samples: 0, missed_deadlines: 0, queue_drops: 0,
    integration_gaps: 0, calibrated: true, dt_count: frames.length - 1,
    dt_mean_us: 5000, dt_std_us: 0, dt_min_us: 5000, dt_max_us: 5000, ...extra };
}

test('constant 1 degree/s raw bias gives 300 degrees, processed gives zero', () => {
  const frames = makeFrames(300);
  const result = analyzeSession(frames, meta, makeEnd(frames));
  assert.equal(result.complete.drift300, true);
  assert.ok(Math.abs(result.drift300.raw_deg[0] - 300) < 1e-6);
  assert.deepEqual(result.drift300.filtered_deg, [0, 0, 0]);
  assert.equal(result.static60.axes[3].raw_error, 1);
  assert.equal(result.static60.axes[2].raw_error, 0);
  assert.equal(result.sampling.actual_hz, 200);
  assert.equal(result.transmission.loss_pct, 0);
});

test('tail and head losses are included using END generated count', () => {
  const full = makeFrames(300);
  const result = analyzeSession(full.slice(2, -3), meta, makeEnd(full));
  assert.equal(result.transmission.lost, 5);
  assert.equal(result.transmission.head_missing, 2);
  assert.equal(result.transmission.tail_missing, 3);
  assert.equal(result.complete.drift300, false);
  assert.equal(result.transmission.complete, true);
});

test('five-minute record with sensor failure cannot claim complete drift', () => {
  const frames = makeFrames(300);
  const result = analyzeSession(frames, meta, makeEnd(frames, { sample_fail: 1 }));
  assert.equal(result.complete.drift300, false);
  assert.match(result.drift300.reasons.join(' '), /读取失败/);
});

test('no END leaves packet loss unknown and short runs cannot pass', () => {
  const result = analyzeSession(makeFrames(10), meta);
  assert.equal(result.transmission.loss_pct, null);
  assert.equal(result.complete.session, false);
  assert.equal(result.complete.static60, false);
  assert.equal(result.complete.drift300, false);
});

test('duplicates do not increase unique reception and host gaps do not corrupt device dt', () => {
  const full = makeFrames(60);
  const received = [full[0], full[0], ...full.slice(2)];
  const result = analyzeSession(received, meta, makeEnd(full), { duplicates: 1 });
  assert.equal(result.transmission.duplicates, 1);
  assert.equal(result.transmission.received_unique, full.length - 1);
  assert.equal(result.transmission.lost, 1);
  assert.equal(result.sampling.host_gap_count, 1);
  assert.equal(result.sampling.dt_max_us, 5000);
  assert.equal(result.complete.static60, false);
});

test('integration does not bridge a sequence gap', () => {
  const full = makeFrames(1);
  const result = analyzeSession(full.filter(f => f.seq !== 100), meta, makeEnd(full));
  assert.equal(result.drift300.skipped_intervals, 1);
  assert.ok(Math.abs(result.drift300.raw_deg[0] - 0.99) < 1e-8);
});

test('acceleration error uses mg and reference preserves gravity', () => {
  const frames = makeFrames(60);
  frames.forEach(f => { f.raw[0] = 0.00980665; f.raw[2] += 0.0196133; });
  const result = analyzeSession(frames, meta, makeEnd(frames));
  assert.ok(Math.abs(result.static60.axes[0].raw_error - 1) < 1e-9);
  assert.ok(Math.abs(result.static60.axes[2].raw_error - 2) < 1e-9);
});

test('inconsistent END cannot create negative loss', () => {
  const frames = makeFrames(1);
  const result = analyzeSession(frames, meta, makeEnd(frames, { generated: frames.length - 1 }));
  assert.equal(result.transmission.loss_pct, null);
  assert.equal(result.transmission.complete, false);
});

test('60000 ideal samples cover 300 seconds but integrate the actual 299.995 seconds', () => {
  const frames = makeFrames(300).slice(0, -1);
  const result = analyzeSession(frames, meta, makeEnd(frames));
  assert.equal(frames.length, 60000);
  assert.equal(result.coverage_s, 300);
  assert.equal(result.complete.drift300, true);
  assert.ok(Math.abs(result.drift300.raw_deg[0] - 299.995) < 1e-6);
});

test('an unreported long read interval also invalidates completeness', () => {
  const frames = makeFrames(60);
  frames.forEach(f => { if (f.seq >= 6000) f.t_us += 30000; });
  const result = analyzeSession(frames, meta, makeEnd(frames));
  assert.equal(result.complete.static60, false);
  assert.equal(result.drift300.skipped_intervals, 1);
  assert.match(result.reasons.join(' '), /20ms/);
});
