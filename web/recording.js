import { quaternionToEuler } from './math.js';

export const CSV_COLUMNS = [
  'session_id', 'seq', 't_us', 'flags',
  ...['ax', 'ay', 'az', 'gx', 'gy', 'gz'].map(a => `raw_${a}_counts`),
  ...['ax_mps2', 'ay_mps2', 'az_mps2', 'gx_dps', 'gy_dps', 'gz_dps'].map(a => `raw_${a}`),
  ...['ax_mps2', 'ay_mps2', 'az_mps2', 'gx_dps', 'gy_dps', 'gz_dps'].map(a => `filtered_${a}`),
  'q_w', 'q_x', 'q_y', 'q_z', 'position_x_m', 'position_y_m', 'position_z_m', 'roll_deg', 'pitch_deg', 'yaw_deg',
];

export function framesToCsv(frames) {
  const lines = [CSV_COLUMNS.join(',')];
  for (const f of frames) {
    const e = quaternionToEuler(f.q);
    lines.push([f.session_id, f.seq, f.t_us, f.flags, ...(f.raw_counts ?? Array(6).fill('')), ...f.raw, ...f.filtered, ...f.q, ...f.position, e.roll, e.pitch, e.yaw].join(','));
  }
  return '\ufeff' + lines.join('\r\n') + '\r\n';
}

export function validateRecording(recording) {
  if (!recording || typeof recording !== 'object' || !Array.isArray(recording.frames) || !recording.frames.length) throw new Error('记录中没有 frames 数据');
  const frameSession = recording.frames[0].session_id;
  for (const f of recording.frames) {
    if (!Number.isInteger(f.session_id) || f.session_id !== frameSession || !Number.isInteger(f.seq) || f.seq < 0 || f.seq > 0xffffffff || !Number.isSafeInteger(f.t_us) || f.t_us < 0) throw new Error('记录包含无效序号、时间戳或混合会话');
    for (const [name, length] of [['raw', 6], ['filtered', 6], ['q', 4], ['position', 3]]) {
      if (!Array.isArray(f[name]) || f[name].length !== length || !f[name].every(Number.isFinite)) throw new Error(`记录中的 ${name} 字段无效`);
    }
  }
  if (recording.meta?.session_id != null && recording.meta.session_id !== frameSession) throw new Error('META 与数据的会话编号不一致');
  if (recording.end?.session_id != null && recording.end.session_id !== frameSession) throw new Error('END 与数据的会话编号不一致');
  return recording;
}
