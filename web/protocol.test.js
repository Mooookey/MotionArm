import test from 'node:test';
import assert from 'node:assert/strict';
import { FrameParser, TYPES, crc16, encodeFrame, decodeData } from './protocol.js';
import { rotateVector, quaternionToEuler } from './math.js';
import { CSV_COLUMNS, framesToCsv, validateRecording } from './recording.js';

function dataFrame(seq = 0, extra = {}) {
  const bytes = new Uint8Array(82), v = new DataView(bytes.buffer);
  v.setUint32(0, 27, true); v.setUint32(4, seq, true); v.setBigUint64(8, BigInt(1000000 + seq * 5000), true); v.setUint16(16, 3, true);
  const raw = extra.raw_counts ?? [8192, -8192, 16384, 655, -655, 0];
  raw.forEach((x, i) => v.setInt16(18 + i * 2, x, true));
  (extra.filtered ?? [9.80665, -9.80665, 19.6133, 10, -10, 0]).forEach((x, i) => v.setFloat32(30 + i * 4, x, true));
  (extra.q ?? [1, 0, 0, 0]).forEach((x, i) => v.setFloat32(54 + i * 4, x, true));
  (extra.position ?? [.1, -.2, .3]).forEach((x, i) => v.setFloat32(70 + i * 4, x, true));
  return encodeFrame(TYPES.DATA, bytes);
}
function concat(...arrays) {
  const out = new Uint8Array(arrays.reduce((s, a) => s + a.length, 0)); let offset = 0;
  for (const a of arrays) { out.set(a, offset); offset += a.length; } return out;
}
function near(actual, expected, tolerance = 1e-6) { assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} ≈ ${expected}`); }

test('CRC16/CCITT-FALSE matches standard check vector', () => {
  assert.equal(crc16(new TextEncoder().encode('123456789')), 0x29b1);
});

test('DATA offsets, little endian uint64 timestamp and physical units match protocol', () => {
  const bytes = dataFrame(234); assert.equal(bytes.length, 90);
  const f = decodeData(bytes); assert.equal(f.session_id, 27); assert.equal(f.seq, 234); assert.equal(f.t_us, 2170000); assert.equal(f.flags, 3);
  assert.deepEqual(f.raw_counts, [8192, -8192, 16384, 655, -655, 0]);
  assert.deepEqual(f.raw, [9.80665, -9.80665, 19.6133, 10, -10, 0]);
  assert.deepEqual(f.q, [1, 0, 0, 0]); near(f.position[1], -.2);
});

test('single-pose acceleration norm correction is not applied again to raw or processed wire values', () => {
  const f = decodeData(dataFrame(), { g: 9.80665, accel_lsb_per_g: 8192, gyro_lsb_per_dps: 65.5, accel_norm_scale: .84 });
  near(f.raw[0], 9.80665); near(f.filtered[0], 9.80665);
});

test('parser handles every byte arriving separately and calls callback once per frame', () => {
  const callbacks = [], parser = new FrameParser(frame => callbacks.push(frame.kind));
  const input = concat(encodeFrame(2, { session_id: 27, accel_lsb_per_g: 8192, gyro_lsb_per_dps: 65.5, g: 9.80665 }), dataFrame(0), dataFrame(1), encodeFrame(3, { session_id: 27, generated: 2 }));
  const received = [];
  for (const byte of input) received.push(...parser.push(Uint8Array.of(byte)));
  assert.deepEqual(callbacks, ['META', 'DATA', 'DATA', 'END']); assert.equal(received[2].data.seq, 1);
  assert.equal(parser.buffer.length, 0); assert.equal(parser.diagnostics.crc_errors, 0);
});

test('parser handles multiple frames in one read and random split/coalesced reads', () => {
  const input = concat(...Array.from({ length: 300 }, (_, i) => dataFrame(i)));
  const parser = new FrameParser(), received = [];
  let offset = 0, seed = 31;
  while (offset < input.length) { seed = (seed * 1664525 + 1013904223) >>> 0; const length = 1 + seed % 700; received.push(...parser.push(input.subarray(offset, offset + length))); offset += length; }
  assert.equal(received.length, 300); assert.deepEqual(received.map(f => f.data.seq), Array.from({ length: 300 }, (_, i) => i));
});

test('CRC damage, boot noise and damaged length recover to the next valid frame', () => {
  const broken = dataFrame(0); broken[39] ^= 0x01;
  const badLength = Uint8Array.of(0xa5, 0x5a, 1, 2, 0xff, 0x0f, 0, 0, 0);
  const parser = new FrameParser();
  const frames = parser.push(concat(new TextEncoder().encode('boot message\r\n'), broken, badLength, dataFrame(1), encodeFrame(3, { generated: 2 })));
  assert.deepEqual(frames.map(f => f.kind), ['DATA', 'END']); assert.equal(frames[0].data.seq, 1);
  assert.equal(parser.diagnostics.crc_errors, 1); assert.ok(parser.diagnostics.invalid_frames >= 1); assert.ok(parser.diagnostics.discarded_bytes > 0);
});

test('invalid JSON and non-finite DATA are rejected without hiding later frames', () => {
  const invalidJson = encodeFrame(TYPES.META, new TextEncoder().encode('{bad json}'));
  const invalidObject = encodeFrame(TYPES.STATUS, [1, 2]);
  const parser = new FrameParser();
  const found = parser.push(concat(invalidJson, invalidObject, dataFrame(0, { filtered: [NaN, 0, 0, 0, 0, 0] }), dataFrame(1)));
  assert.equal(found.length, 1); assert.equal(found[0].data.seq, 1); assert.equal(parser.diagnostics.json_errors, 2); assert.equal(parser.diagnostics.invalid_frames, 1);
});

test('unsafe uint64 timestamp and invalid DATA lengths are rejected', () => {
  const bytes = dataFrame(0), v = new DataView(bytes.buffer); v.setBigUint64(14, BigInt(Number.MAX_SAFE_INTEGER) + 1n, true);
  assert.throws(() => decodeData(bytes), /safe integer/);
  assert.throws(() => decodeData(new Uint8Array(89)), /90 bytes/);
  assert.throws(() => encodeFrame(1, new Uint8Array(81)), /length/);
  assert.throws(() => encodeFrame(2, new Uint8Array(4090)), /length/);
});

test('sensor-to-world right handed quaternion rotates +X to +Y for +90° yaw', () => {
  const q = [Math.SQRT1_2, 0, 0, Math.SQRT1_2], result = rotateVector(q, [1, 0, 0]);
  near(result[0], 0); near(result[1], 1); near(result[2], 0);
  const e = quaternionToEuler(q); near(e.roll, 0); near(e.pitch, 0); near(e.yaw, 90);
});

test('positive roll and pitch follow body XYZ, Z up; quaternion magnitude is normalized', () => {
  const roll = rotateVector([2 * Math.SQRT1_2, 2 * Math.SQRT1_2, 0, 0], [0, 1, 0]); near(roll[2], 1);
  const pitch = rotateVector([Math.SQRT1_2, 0, Math.SQRT1_2, 0], [1, 0, 0]); near(pitch[2], -1);
  const e = quaternionToEuler([Math.SQRT1_2, 0, Math.SQRT1_2, 0]); near(e.pitch, 90, 2e-6); assert.ok(Object.values(e).every(Number.isFinite));
  assert.deepEqual(quaternionToEuler([0, 0, 0, 0]), { roll: 0, pitch: 0, yaw: 0 });
});

test('recording CSV retains exact sequence/timestamp and has consistent columns', () => {
  const f = decodeData(dataFrame(10)), csv = framesToCsv([f]); const rows = csv.slice(1).trim().split('\r\n');
  assert.deepEqual(rows[0].split(','), CSV_COLUMNS); assert.equal(rows[1].split(',').length, CSV_COLUMNS.length);
  assert.deepEqual(rows[1].split(',').slice(0, 4), ['27', '10', '1050000', '3']);
  assert.equal(validateRecording({ meta: { session_id: 27 }, end: { session_id: 27 }, frames: [f] }).frames.length, 1);
});

test('recording import rejects mixed sessions and non-finite pose data', () => {
  const f = decodeData(dataFrame(0));
  assert.throws(() => validateRecording({ meta: { session_id: 28 }, frames: [f] }), /不一致/);
  assert.throws(() => validateRecording({ frames: [f, { ...f, session_id: 28 }] }), /混合会话/);
  assert.throws(() => validateRecording({ frames: [{ ...f, q: [1, NaN, 0, 0] }] }), /q/);
});
