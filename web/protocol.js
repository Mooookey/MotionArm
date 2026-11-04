// Protocol v1. All offsets and units are defined in ../PROTOCOL.md.
export const TYPES = Object.freeze({ DATA: 1, META: 2, END: 3, STATUS: 4, ERROR: 5 });
export const BAUD_RATE = 460800;
export const TYPE_NAMES = Object.freeze({ 1: 'DATA', 2: 'META', 3: 'END', 4: 'STATUS', 5: 'ERROR' });
const encoder = new TextEncoder();
const decoder = new TextDecoder('utf-8', { fatal: true });

export function crc16(bytes) {
  let crc = 0xffff;
  for (const value of bytes) {
    crc ^= value << 8;
    for (let bit = 0; bit < 8; bit++) crc = ((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1) & 0xffff;
  }
  return crc;
}

export function encodeFrame(type, payload) {
  if (!TYPE_NAMES[type]) throw new Error('Unknown frame type');
  const body = payload instanceof Uint8Array ? payload : encoder.encode(JSON.stringify(payload));
  const length = body.length + 8;
  if (length > 4096 || (type === TYPES.DATA && length !== 90)) throw new Error('Invalid frame length');
  const bytes = new Uint8Array(length);
  const view = new DataView(bytes.buffer);
  bytes.set([0xa5, 0x5a, 1, type]);
  view.setUint16(4, length, true);
  bytes.set(body, 6);
  view.setUint16(length - 2, crc16(bytes.subarray(0, length - 2)), true);
  return bytes;
}

export function decodeData(bytes, meta = {}) {
  if (!(bytes instanceof Uint8Array) || bytes.length !== 90) throw new Error('DATA frame must be 90 bytes');
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const timestamp = v.getBigUint64(14, true);
  if (timestamp > BigInt(Number.MAX_SAFE_INTEGER)) throw new Error('Timestamp exceeds safe integer range');
  const raw_counts = Array.from({ length: 6 }, (_, i) => v.getInt16(24 + i * 2, true));
  const filtered = Array.from({ length: 6 }, (_, i) => v.getFloat32(36 + i * 4, true));
  const q = Array.from({ length: 4 }, (_, i) => v.getFloat32(60 + i * 4, true));
  const position = Array.from({ length: 3 }, (_, i) => v.getFloat32(76 + i * 4, true));
  if (![...filtered, ...q, ...position].every(Number.isFinite)) throw new Error('Non-finite DATA');
  const accelScale = (meta.g ?? 9.80665) / (meta.accel_lsb_per_g ?? 8192);
  const gyroScale = 1 / (meta.gyro_lsb_per_dps ?? 65.5);
  const raw = raw_counts.map((n, i) => n * (i < 3 ? accelScale : gyroScale));
  if (!raw.every(Number.isFinite)) throw new Error('Invalid raw scale');
  return {
    session_id: v.getUint32(6, true), seq: v.getUint32(10, true), t_us: Number(timestamp),
    flags: v.getUint16(22, true), raw_counts, raw, filtered, q, position,
  };
}

function validCandidate(bytes, offset) {
  if (offset + 8 > bytes.length || bytes[offset] !== 0xa5 || bytes[offset + 1] !== 0x5a || bytes[offset + 2] !== 1 || !TYPE_NAMES[bytes[offset + 3]]) return false;
  const length = bytes[offset + 4] | bytes[offset + 5] << 8;
  if (length < 8 || length > 4096 || offset + length > bytes.length || (bytes[offset + 3] === 1 && length !== 90)) return false;
  const expected = bytes[offset + length - 2] | bytes[offset + length - 1] << 8;
  return crc16(bytes.subarray(offset, offset + length - 2)) === expected;
}

export class FrameParser {
  constructor(onFrame = null) {
    this.onFrame = typeof onFrame === 'function' ? onFrame : null;
    this.buffer = new Uint8Array();
    this.meta = {};
    this.diagnostics = { crc_errors: 0, invalid_frames: 0, json_errors: 0, discarded_bytes: 0 };
  }

  push(chunk) {
    if (!(chunk instanceof Uint8Array)) chunk = new Uint8Array(chunk);
    const joined = new Uint8Array(this.buffer.length + chunk.length);
    joined.set(this.buffer); joined.set(chunk, this.buffer.length);
    const frames = [];
    let offset = 0;
    while (offset + 2 <= joined.length) {
      if (joined[offset] !== 0xa5 || joined[offset + 1] !== 0x5a) {
        offset++; this.diagnostics.discarded_bytes++; continue;
      }
      if (offset + 6 > joined.length) break;
      const type = joined[offset + 3];
      const length = joined[offset + 4] | joined[offset + 5] << 8;
      if (joined[offset + 2] !== 1 || !TYPE_NAMES[type] || length < 8 || length > 4096 || (type === 1 && length !== 90)) {
        offset++; this.diagnostics.invalid_frames++; this.diagnostics.discarded_bytes++; continue;
      }
      if (offset + length > joined.length) {
        // A damaged length must not hide a complete, CRC-valid later frame.
        let recovered = -1;
        for (let i = offset + 1; i + 8 <= joined.length; i++) if (validCandidate(joined, i)) { recovered = i; break; }
        if (recovered < 0) break;
        this.diagnostics.invalid_frames++; this.diagnostics.discarded_bytes += recovered - offset;
        offset = recovered; continue;
      }
      const bytes = joined.slice(offset, offset + length);
      const expected = bytes[length - 2] | bytes[length - 1] << 8;
      if (crc16(bytes.subarray(0, length - 2)) !== expected) {
        offset++; this.diagnostics.crc_errors++; this.diagnostics.discarded_bytes++; continue;
      }
      offset += length;
      let data;
      try {
        if (type === 1) data = decodeData(bytes, this.meta);
        else {
          data = JSON.parse(decoder.decode(bytes.subarray(6, length - 2)));
          if (!data || typeof data !== 'object' || Array.isArray(data)) throw new Error('JSON payload must be an object');
          if (type === 2) this.meta = data;
        }
      } catch {
        if (type === 1) this.diagnostics.invalid_frames++;
        else this.diagnostics.json_errors++;
        continue;
      }
      const frame = { type, kind: TYPE_NAMES[type], data, bytes };
      frames.push(frame);
      this.onFrame?.(frame);
    }
    this.buffer = joined.slice(offset);
    return frames;
  }
}
