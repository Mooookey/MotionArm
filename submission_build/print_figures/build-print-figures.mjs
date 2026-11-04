import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import { analyzeSession } from '../../web/metrics.js';

const out = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(out, '../..');
const read = name => JSON.parse(fs.readFileSync(path.join(root, name), 'utf8').replace(/^\uFEFF/, ''));
const dynamic = read('results/selections/continuous_pose.json');
const esc = value => String(value).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]);
const fmt = (value, digits = 3) => Number.isFinite(value) ? Number(value.toFixed(digits)).toString() : '未知';
const C = { raw: '#2563eb', processed: '#059669', grid: '#e2e9f1', gray: '#64748b' };
const width = 800;
function begin(title, subtitles, height) {
  let s = `<svg xmlns="http://www.w3.org/2000/svg" width="800" height="${height}" viewBox="0 0 800 ${height}" role="img" aria-label="${esc(title)}"><style>text{font-family:Segoe UI,Microsoft YaHei,sans-serif;fill:#26344b;font-size:18px}.title{font-size:25px;font-weight:700}.subtitle{font-size:16px;fill:#4d617b}.panel{font-size:21px;font-weight:600}.axis{font-size:18px;fill:#516780}.legend{font-size:18px}.note{font-size:16px;fill:#4c607b}.grid{stroke:#e0e8f2;stroke-width:1}.border{stroke:#bccbdc;fill:none}</style><rect width="100%" height="100%" fill="white"/><text class="title" x="24" y="35">${esc(title)}</text>`;
  subtitles.forEach((text, i) => { s += `<text class="subtitle" x="24" y="${68 + i * 27}">${esc(text)}</text>`; });
  return s;
}
function legend(items, y) {
  return items.map((item, i) => `<line x1="${26 + i * 375}" y1="${y - 6}" x2="${54 + i * 375}" y2="${y - 6}" stroke="${item.color}" stroke-width="4"/><text class="legend" x="${64 + i * 375}" y="${y}">${esc(item.label)}</text>`).join('');
}
function note(text, y) { return `<text class="note" x="24" y="${y}">${esc(text)}</text>`; }
function range(values, zero = false) {
  let lo = zero ? 0 : Infinity, hi = zero ? 0 : -Infinity;
  for (const x of values) if (Number.isFinite(x)) { lo = Math.min(lo, x); hi = Math.max(hi, x); }
  let span = hi - lo; if (span < 1e-10) span = Math.max(Math.abs(hi) * .1, .01);
  return [lo - span * .12, hi + span * .18];
}
// Same extrema preservation policy as tools/build-report.mjs; data/ranges remain complete.
function reduce(points, target = 1200) {
  if (points.length <= target) return points;
  const result = [points[0]], stride = Math.ceil(points.length / (target / 2));
  for (let i = 0; i < points.length; i += stride) {
    const bucket = points.slice(i, i + stride);
    if (bucket.some(p => !p)) { result.push(null); continue; }
    let min = 0, max = 0;
    for (let j = 1; j < bucket.length; j++) { if (bucket[j].y < bucket[min].y) min = j; if (bucket[j].y > bucket[max].y) max = j; }
    for (const j of min <= max ? [min, max] : [max, min]) if (bucket[j] !== result.at(-1)) result.push(bucket[j]);
  }
  if (result.at(-1) !== points.at(-1)) result.push(points.at(-1)); return result;
}
function linePanel({ name, unit, top, series, xMin, xMax, height = 155, reference = null, xLabel = '源会话 elapsed / s', zero = false }) {
  const left = 100, right = 770, bottom = top + height;
  const values = series.flatMap(s => s.points.filter(Boolean).map(p => p.y)); if (Number.isFinite(reference)) values.push(reference);
  const [lo, hi] = range(values, zero);
  const sx = x => left + (x - xMin) / Math.max(.005, xMax - xMin) * (right - left), sy = y => bottom - (y - lo) / (hi - lo) * height;
  let svg = `<text class="panel" x="${left}" y="${top - 19}">${esc(name)} · ${esc(unit)}</text>`;
  for (let i = 0; i <= 3; i++) {
    const y = top + height * i / 3;
    svg += `<line class="grid" x1="${left}" y1="${y}" x2="${right}" y2="${y}"/><text class="axis" text-anchor="end" x="${left - 11}" y="${y + 6}">${fmt(hi - (hi - lo) * i / 3, 2)}</text>`;
  }
  for (let i = 0; i <= 4; i++) {
    const x = left + (right - left) * i / 4;
    svg += `<line class="grid" x1="${x}" y1="${top}" x2="${x}" y2="${bottom}"/><text class="axis" text-anchor="${i === 0 ? 'start' : i === 4 ? 'end' : 'middle'}" x="${x}" y="${bottom + 27}">${fmt(xMin + (xMax - xMin) * i / 4, 2)}</text>`;
  }
  if (Number.isFinite(reference)) svg += `<line x1="${left}" y1="${sy(reference)}" x2="${right}" y2="${sy(reference)}" stroke="#9bacbf" stroke-dasharray="6 5"/>`;
  svg += `<rect class="border" x="${left}" y="${top}" width="${right - left}" height="${height}"/>`;
  for (const item of series) {
    let d = '', open = false;
    for (const point of reduce(item.points)) { if (!point) { open = false; continue; } d += `${open ? 'L' : 'M'}${sx(point.x).toFixed(3)},${sy(point.y).toFixed(3)} `; open = true; }
    svg += `<path d="${d}" fill="none" stroke="${item.color}" stroke-width="1.6" stroke-linejoin="round"/>`;
  }
  return svg + `<text class="axis" text-anchor="end" x="${right}" y="${bottom + 53}">${esc(xLabel)}</text>`;
}
function save(name, svg) { fs.writeFileSync(path.join(out, name), svg + '</svg>'); console.log(name); }

const sourceStart = dynamic.source_end.first_t_us;
if (dynamic.frames[0].t_us !== 461985936 || dynamic.frames.at(-1).t_us !== 505735936) throw new Error('Dynamic selection changed');
let six = begin('图 6 · 主展示段三轴角速度响应', [
  `真实硬件 · 源会话 ${dynamic.meta.session_id} · 8751 帧 / 200 Hz`,
  't_us 461985936–505735936 µs；源会话 elapsed 23–66.75 s',
  '完整选段 43.750 s，含分轴动作和暂停；横轴不在选段起点归零。',
], 1095);
six += legend([{ label: 'raw（未扣 bias）', color: C.raw }, { label: 'processed（扣偏 + LPF）', color: C.processed }], 164);
six += note('处理公式：ω_processed = LPF_20Hz(ω_raw − bias)。', 198);
for (let axis = 0; axis < 3; axis++) six += linePanel({ name: `${'XYZ'[axis]} 轴角速度`, unit: '°/s', top: 246 + axis * 245, xMin: 23, xMax: 66.75, height: 145, series: [
  { color: C.raw, points: dynamic.frames.map(f => ({ x: (f.t_us - sourceStart) / 1e6, y: f.raw[axis + 3] })) },
  { color: C.processed, points: dynamic.frames.map(f => ({ x: (f.t_us - sourceStart) / 1e6, y: f.filtered[axis + 3] })) },
] });
six += note(`bias = [${dynamic.meta.gyro_bias_dps.map(v => fmt(v, 6)).join(', ')}] °/s。`, 988);
six += note(`κ = ${dynamic.meta.accel_norm_scale} 仅用于加速度，不影响本图 gyro。`, 1016);
six += note('raw 仍含共同硬件 DLPF；差异包含扣偏与低通，不全是低通衰减。', 1044);
six += note('人工动作非标准阶跃。显示保留每桶极值，未裁切时间或幅值。', 1072);
save('06-main-segment-gyro-response.svg', six);

if (process.argv.includes('--only6')) process.exit(0);

// Figures 01–05 are appended below by this same reproducible print renderer.
