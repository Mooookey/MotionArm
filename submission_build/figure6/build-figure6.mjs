import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';

const output = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(output, '../..');
const inputPath = path.join(root, 'results/selections/continuous_pose.json');
const inputBytes = fs.readFileSync(inputPath);
const record = JSON.parse(inputBytes.toString('utf8').replace(/^\uFEFF/, ''));
const frames = record.frames;
const selection = record.selection;
const meta = record.meta;
const sourceStartUs = record.source_end.first_t_us;
const startUs = 461985936, endUs = 505735936;
const xMin = 23, xMax = 66.75;
if (frames.length !== 8751 || frames[0].t_us !== startUs || frames.at(-1).t_us !== endUs || selection.source_elapsed_start_s !== xMin || selection.source_elapsed_end_s !== xMax) throw new Error('Selection boundary changed; review source provenance before regenerating');
if (sourceStartUs !== 438985936 || meta.session_id !== 3623475967) throw new Error('Unexpected source session');
for (let i = 1; i < frames.length; i++) if (frames[i].seq !== frames[i - 1].seq + 1 || frames[i].t_us - frames[i - 1].t_us !== 5000) throw new Error('Selection is not continuous; do not hide gaps');
const esc = value => String(value).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]);
const fmt = (v, digits = 3) => Number.isFinite(v) ? Number(v.toFixed(digits)).toString() : 'unknown';
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const axisNames = ['ax', 'ay', 'az', 'gx', 'gy', 'gz'];
const rawColor = '#2563eb', processedColor = '#059669';
const W = 1200, H = 1215;

// Reuses the report renderer's extrema-preserving display reduction. Statistics
// and provenance retain all 8,751 frames; neither time nor value range is cropped.
function minMaxPoints(points, maxPoints = 950) {
  if (points.length <= maxPoints) return points;
  const stride = Math.ceil(points.length / (maxPoints / 2));
  const result = [points[0]];
  for (let offset = 0; offset < points.length; offset += stride) {
    const group = points.slice(offset, offset + stride);
    let min = 0, max = 0;
    for (let i = 1; i < group.length; i++) { if (group[i].y < group[min].y) min = i; if (group[i].y > group[max].y) max = i; }
    for (const index of min <= max ? [min, max] : [max, min]) if (group[index] !== result.at(-1)) result.push(group[index]);
  }
  if (result.at(-1) !== points.at(-1)) result.push(points.at(-1));
  return result;
}
function plot(axis, leftColumn, row, correctedReference) {
  const boxX = leftColumn ? 28 : 612;
  const left = boxX + 78, right = boxX + 550;
  const top = 245 + row * 265, height = 164, bottom = top + height;
  const before = frames.map(f => ({ x: (f.t_us - sourceStartUs) / 1e6, y: correctedReference ? (axis < 3 ? f.raw[axis] * meta.accel_norm_scale : f.raw[axis] - meta.gyro_bias_dps[axis - 3]) : f.raw[axis] }));
  const after = frames.map(f => ({ x: (f.t_us - sourceStartUs) / 1e6, y: f.filtered[axis] }));
  let lo = Infinity, hi = -Infinity;
  for (const point of [...before, ...after]) { lo = Math.min(lo, point.y); hi = Math.max(hi, point.y); }
  const pad = Math.max((hi - lo) * .12, axis < 3 ? .03 : .08); lo -= pad; hi += pad;
  const sx = x => left + (x - xMin) / (xMax - xMin) * (right - left);
  const sy = y => bottom - (y - lo) / (hi - lo) * height;
  let svg = `<text class="panel" x="${left}" y="${top - 20}">${axisNames[axis]} <tspan class="unit">${axis < 3 ? '加速度 · m/s²' : '角速度 · °/s'}</tspan></text>`;
  for (let i = 0; i <= 3; i++) {
    const y = top + i / 3 * height;
    svg += `<line class="grid" x1="${left}" y1="${y}" x2="${right}" y2="${y}"/><text class="axis" text-anchor="end" x="${left - 10}" y="${y + 6}">${fmt(hi - (hi - lo) * i / 3, 2)}</text>`;
  }
  for (let i = 0; i <= 3; i++) {
    const x = left + i / 3 * (right - left), t = xMin + (xMax - xMin) * i / 3;
    svg += `<line class="grid" x1="${x}" y1="${top}" x2="${x}" y2="${bottom}"/><text class="axis" text-anchor="${i === 0 ? 'start' : i === 3 ? 'end' : 'middle'}" x="${x}" y="${bottom + 27}">${fmt(t, 2)}</text>`;
  }
  svg += `<rect class="border" x="${left}" y="${top}" width="${right - left}" height="${height}"/>`;
  for (const [points, color] of [[before, rawColor], [after, processedColor]]) {
    const d = minMaxPoints(points).map((p, i) => `${i ? 'L' : 'M'}${sx(p.x).toFixed(3)},${sy(p.y).toFixed(3)}`).join(' ');
    svg += `<path d="${d}" fill="none" stroke="${color}" stroke-width="1.45" stroke-linejoin="round"/>`;
  }
  return svg + `<text class="axis" text-anchor="end" x="${right}" y="${bottom + 53}">源会话 elapsed / s</text>`;
}
function makeFigure(correctedReference) {
  const title = correctedReference ? '图 6b · 校准后参考与固件低通响应' : '图 6 · 主展示段六轴原始 / 处理响应';
  let svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}" role="img" aria-label="${title}"><style>text{font-family:Segoe UI,Microsoft YaHei,sans-serif;fill:#26334a}.title{font-size:29px;font-weight:700}.sub{font-size:16px;fill:#52647d}.legend{font-size:18px}.panel{font-size:22px;font-weight:700}.unit{font-size:18px;font-weight:400}.axis{font-size:17px;fill:#566a83}.grid{stroke:#e1e8f0;stroke-width:1}.border{stroke:#bfccdb;fill:none}.note{font-size:16px;fill:#4b607c}</style><rect width="100%" height="100%" fill="white"/><text class="title" x="30" y="43">${esc(title)}</text>`;
  svg += `<text class="sub" x="30" y="77">源会话 ${meta.session_id} · t_us ${startUs}–${endUs} µs · 源 elapsed 23–66.75 s</text>`;
  svg += '<text class="sub" x="30" y="107">选段 43.750 s / 8751 帧 / 200 Hz；含暂停与分轴动作，不等于全程持续非零转动。</text>';
  svg += `<text class="sub" x="30" y="137">κ = ${meta.accel_norm_scale.toFixed(9)}；陀螺 bias [${meta.gyro_bias_dps.map(v => v.toFixed(6)).join(', ')}] °/s；fc = 20 Hz</text>`;
  const firstLegend = correctedReference ? '校准后参考（无软件低通）' : 'raw（未扣偏 / 未乘 κ）';
  const secondLegend = 'processed（ESP32 校准 + 低通）';
  svg += `<line x1="32" y1="177" x2="65" y2="177" stroke="${rawColor}" stroke-width="4"/><text class="legend" x="75" y="183">${firstLegend}</text><line x1="550" y1="177" x2="583" y2="177" stroke="${processedColor}" stroke-width="4"/><text class="legend" x="593" y="183">${secondLegend}</text>`;
  for (let row = 0; row < 3; row++) { svg += plot(row, true, row, correctedReference); svg += plot(row + 3, false, row, correctedReference); }
  svg += `<text class="note" x="30" y="1096">processed：加速度先乘 κ；角速度先减启动静态 bias；再执行 20 Hz 一阶低通。</text>`;
  svg += `<text class="note" x="30" y="1123">${correctedReference ? '蓝线由保留的 raw 与固定校准参数推导，便于分离低通影响；不是另一组独立测量。' : 'raw 同样已经过传感器硬件 DLPF；均值 / 峰值变化不能全部归因于软件低通。'}</text>`;
  svg += '<text class="note" x="30" y="1150">横轴保留源会话时间原点；所有选段样本参与。图线显示抽样保留每桶极值，不裁切时间或幅值。</text>';
  svg += '<text class="note" x="30" y="1177">本图属于动态人工动作对照，不是标准阶跃试验；κ 单姿态模长校准不能证明三轴独立精度。</text></svg>';
  return svg;
}
for (const [name, corrected] of [['06-filter-response-raw-processed.svg', false], ['06b-calibrated-reference-vs-lpf.svg', true]]) fs.writeFileSync(path.join(output, name), makeFigure(corrected));
const figureMetadata = {
  created_at: new Date().toISOString(), source: 'hardware', recording_kind: 'dynamic',
  input_file: path.relative(root, inputPath).replaceAll('\\', '/'), input_sha256: sha(inputBytes),
  source_session: selection, source_session_start_t_us: sourceStartUs,
  source_elapsed_start_s: xMin, source_elapsed_end_s: xMax, sample_count: frames.length,
  axis_time_origin: 'source_end.first_t_us, never the first selected frame',
  transformation: { raw: 'unchanged physical-unit samples after shared hardware DLPF', processed: 'original transmitted filtered values, no recomputation', derived_reference: 'raw accel * accel_norm_scale; raw gyro - gyro_bias_dps, no additional LPF', accel_norm_scale: meta.accel_norm_scale, gyro_bias_dps: meta.gyro_bias_dps, filter_hz: meta.filter_hz },
  display: { time_and_amplitude_cropped: false, all_source_samples_used_for_ranges: true, extrema_preserving_display_reduction: true, full_png_not_cropped: true },
};
fs.writeFileSync(path.join(output, 'figure-metadata.json'), JSON.stringify(figureMetadata, null, 2) + '\n');
console.log(JSON.stringify({ output, figures: ['06-filter-response-raw-processed.svg', '06b-calibrated-reference-vs-lpf.svg'], source_elapsed: [xMin, xMax], source_t_us: [startUs, endUs], samples: frames.length }, null, 2));
