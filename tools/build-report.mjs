import fs from 'node:fs';
import path from 'node:path';
import { analyzeSession } from '../web/metrics.js';

// Dependency-free report generation. Every plotted x value is derived from the
// ESP32 timestamp; the host date below is only the report creation date.
const input = process.argv[2];
if (!input) {
  console.error('Usage: node tools/build-report.mjs <recording.json> [--kind static|dynamic|unknown]');
  process.exit(1);
}
const inputPath = path.resolve(input);
const record = JSON.parse(fs.readFileSync(inputPath, 'utf8').replace(/^\uFEFF/, ''));
if (!Array.isArray(record.frames) || !record.frames.length) throw new Error('Recording has no frames');
const finite = Number.isFinite;
const escape = value => String(value ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]);
const fmt = (value, digits = 4) => !finite(value) ? '—' : value !== 0 && Math.abs(value) < 0.0001 ? value.toExponential(3) : value.toLocaleString('en-US', { maximumFractionDigits: digits });
const source = record.source ?? 'unknown';
const meta = record.meta ?? {};
const end = record.end ?? null;
const diagnostics = { ...(record.diagnostics ?? {}), source };
const metrics = analyzeSession(record.frames, meta, end, diagnostics);
const seen = new Set();
const frames = record.frames.filter(f => {
  if (f.session_id !== metrics.session_id || !Number.isInteger(f.seq) || !finite(f.t_us) ||
      !Array.isArray(f.raw) || f.raw.length !== 6 || !f.raw.every(finite) ||
      !Array.isArray(f.filtered) || f.filtered.length !== 6 || !f.filtered.every(finite) || seen.has(f.seq)) return false;
  seen.add(f.seq); return true;
}).sort((a, b) => a.seq - b.seq);
if (!frames.length) throw new Error('No finite frames belonging to the selected session');
const outDir = path.dirname(inputPath);
const figuresDir = path.join(outDir, 'figures');
fs.mkdirSync(figuresDir, { recursive: true });
const kindIndex = process.argv.indexOf('--kind');
let kind = kindIndex >= 0 ? process.argv[kindIndex + 1] : record.recording_kind ?? record.capture?.test_type;
if (!['static', 'dynamic', 'unknown'].includes(kind)) kind = /(?:^|[\\/])static[_\\/]/i.test(inputPath) ? 'static' : 'unknown';
const kindLabel = { static: '静态记录', dynamic: '动态动作记录（动作条件需人工确认）', unknown: '未标注动作类型的记录' }[kind];
const sourceLabel = source === 'hardware' ? '硬件采集' : source === 'demo' ? '模拟演示 · 不得作为硬件验收证据' : `来源未确认（${source}）`;
const startUs = end?.session_id === metrics.session_id && finite(end.first_t_us) ? end.first_t_us : frames[0].t_us;
const elapsed = f => (f.t_us - startUs) / 1e6;
const totalSeconds = Math.max(metrics.duration_s, elapsed(frames.at(-1)), 0.005);
const staticSeconds = Math.min(60, metrics.coverage_s);
const timeLabel = `ESP32 t_us ${fmt(startUs, 0)} 起；设备时间覆盖 ${fmt(metrics.coverage_s, 3)} s；${kindLabel}；${sourceLabel}`;
const staticLabel = `ESP32 elapsed 0–${fmt(staticSeconds, 3)} s（${metrics.static60.samples} 帧）；${kindLabel}；${sourceLabel}`;
const rawColor = '#2563eb', filteredColor = '#059669';
const width = 1200;

function svgStart(title, subtitle, height) {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}" role="img" aria-label="${escape(title)}"><style>text{font-family:Segoe UI,Microsoft YaHei,sans-serif;fill:#24334b;font-size:14px}.title{font-size:25px;font-weight:700}.sub{font-size:13px;fill:#5b6c84}.axis{font-size:12px;fill:#586a83}.panel{font-size:16px;font-weight:600}.grid{stroke:#e3e9f2;stroke-width:1}.border{stroke:#ccd6e4;fill:none}.legend{font-size:13px}.note{font-size:13px;fill:#5b6c84}</style><rect width="100%" height="100%" fill="#fff"/><text class="title" x="36" y="38">${escape(title)}</text><text class="sub" x="36" y="64">${escape(subtitle)}</text>`;
}
function rangeFor(values, zero = false) {
  let low = zero ? 0 : Infinity, high = zero ? 0 : -Infinity;
  for (const value of values) if (finite(value)) { low = Math.min(low, value); high = Math.max(high, value); }
  if (!finite(low) || !finite(high)) return [-1, 1];
  let span = high - low;
  if (span < 1e-12) span = Math.max(Math.abs(high) * 0.1, 0.001);
  return [low - span * 0.10, high + span * 0.14];
}
function legend(items, y = 88) {
  return items.map((item, index) => `<line x1="${38 + index * 215}" y1="${y - 4}" x2="${65 + index * 215}" y2="${y - 4}" stroke="${item.color}" stroke-width="3"/><text class="legend" x="${73 + index * 215}" y="${y}">${escape(item.label)}</text>`).join('');
}
// Keep both extrema of each pixel bucket so short spikes remain visible.
function decimate(points, maximum = 1500) {
  if (points.length <= maximum) return points;
  const result = [];
  const stride = Math.max(1, Math.ceil(points.length / (maximum / 2)));
  for (let i = 0; i < points.length; i += stride) {
    const group = points.slice(i, i + stride);
    if (group.some(p => p === null)) { result.push(null); continue; }
    let min = 0, max = 0;
    for (let j = 1; j < group.length; j++) { if (group[j].y < group[min].y) min = j; if (group[j].y > group[max].y) max = j; }
    if (min <= max) { result.push(group[min]); if (max !== min) result.push(group[max]); }
    else { result.push(group[max]); result.push(group[min]); }
  }
  if (points.at(-1) !== result.at(-1)) result.push(points.at(-1));
  return result;
}
function linePanel({ name, unit, top, series, xMax = totalSeconds, xMin = 0, plotHeight = 158, zero = false, reference = null }) {
  const left = 100, right = 1165, bottom = top + plotHeight;
  const yValues = series.flatMap(s => s.points.filter(Boolean).map(p => p.y));
  if (finite(reference)) yValues.push(reference);
  const [lo, hi] = rangeFor(yValues, zero);
  const sx = x => left + (x - xMin) / Math.max(xMax - xMin, 0.005) * (right - left);
  const sy = y => bottom - (y - lo) / (hi - lo) * plotHeight;
  let result = `<text class="panel" x="${left}" y="${top - 12}">${escape(name)} (${escape(unit)})</text>`;
  for (let i = 0; i <= 4; i++) {
    const y = top + i * plotHeight / 4, value = hi - i * (hi - lo) / 4;
    result += `<line class="grid" x1="${left}" y1="${y}" x2="${right}" y2="${y}"/><text class="axis" text-anchor="end" x="${left - 12}" y="${y + 4}">${escape(fmt(value, 5))}</text>`;
  }
  for (let i = 0; i <= 5; i++) {
    const x = left + i * (right - left) / 5;
    result += `<line class="grid" x1="${x}" y1="${top}" x2="${x}" y2="${bottom}"/><text class="axis" text-anchor="middle" x="${x}" y="${bottom + 20}">${escape(fmt(xMin + i * (xMax - xMin) / 5, 2))}</text>`;
  }
  if (finite(reference)) result += `<line x1="${left}" y1="${sy(reference)}" x2="${right}" y2="${sy(reference)}" stroke="#9bacc2" stroke-dasharray="5 5"/>`;
  result += `<rect class="border" x="${left}" y="${top}" width="${right - left}" height="${plotHeight}"/>`;
  for (const item of series) {
    let d = '', open = false;
    for (const point of decimate(item.points)) {
      if (!point || !finite(point.x) || !finite(point.y)) { open = false; continue; }
      d += `${open ? 'L' : 'M'}${sx(point.x).toFixed(2)},${sy(point.y).toFixed(2)} `; open = true;
    }
    result += `<path d="${d.trim()}" fill="none" stroke="${item.color}" stroke-width="1.4" stroke-linejoin="round"/>`;
  }
  return result + `<text class="axis" text-anchor="end" x="${right}" y="${bottom + 40}">ESP32 elapsed_s</text>`;
}
function barPanel({ name, unit, top, axes, rawField, filteredField }) {
  const left = 100, right = 1165, plotHeight = 170, bottom = top + plotHeight;
  const [lo, hi] = rangeFor(axes.flatMap(axis => [axis[rawField], axis[filteredField]]), true);
  const sy = y => bottom - (y - lo) / (hi - lo) * plotHeight;
  let result = `<text class="panel" x="${left}" y="${top - 14}">${escape(name)} (${escape(unit)})</text>`;
  for (let i = 0; i <= 4; i++) {
    const y = top + i * plotHeight / 4, value = hi - i * (hi - lo) / 4;
    result += `<line class="grid" x1="${left}" y1="${y}" x2="${right}" y2="${y}"/><text class="axis" text-anchor="end" x="${left - 12}" y="${y + 4}">${escape(fmt(value, 5))}</text>`;
  }
  result += `<line x1="${left}" y1="${sy(0)}" x2="${right}" y2="${sy(0)}" stroke="#93a6c1"/>`;
  axes.forEach((axis, index) => {
    const center = left + (index + 0.5) * (right - left) / axes.length;
    for (const [offset, field, color] of [[-75, rawField, rawColor], [9, filteredField, filteredColor]]) {
      const value = axis[field];
      if (!finite(value)) continue;
      const y = Math.min(sy(0), sy(value)), height = Math.max(1, Math.abs(sy(value) - sy(0)));
      result += `<rect x="${center + offset}" y="${y}" width="65" height="${height}" rx="2" fill="${color}"/><text text-anchor="middle" class="axis" x="${center + offset + 32.5}" y="${value >= 0 ? y - 5 : y + height + 16}">${escape(fmt(value, 5))}</text>`;
    }
    result += `<text class="axis" text-anchor="middle" x="${center}" y="${bottom + 28}">${escape(axis.name)}</text>`;
  });
  return result;
}
function saveFigure(filename, svg) {
  fs.writeFileSync(path.join(figuresDir, filename), svg + '</svg>');
  return `figures/${filename}`;
}

const axes = metrics.static60.axes;
const errorFigure = saveFigure('01-measurement-error.svg', svgStart(kind === 'static' ? '01 · 静态测量误差' : '01 · 均值相对参考的差值（非静态验收）', staticLabel, 645) +
  legend([{ label: '原始', color: rawColor }, { label: '处理后', color: filteredColor }]) +
  barPanel({ name: '加速度均值相对 [0,0,g] 的误差', unit: 'mg', top: 135, axes: axes.slice(0, 3), rawField: 'raw_error', filteredField: 'filtered_error' }) +
  barPanel({ name: '角速度均值相对 0 的误差', unit: '°/s', top: 395, axes: axes.slice(3), rawField: 'raw_error', filteredField: 'filtered_error' }) +
  `<text class="note" x="36" y="621">静态/水平条件需人工确认；processed 单姿态 κ 归一不能证明三轴精度。60s 数据完整性：${metrics.static60.complete ? '通过' : '未通过/不足'}</text>`);

const stdFigure = saveFigure('02-static-variation.svg', svgStart(kind === 'static' ? '02 · 静态六轴波动' : '02 · 六轴波动（含动态动作，非静态验收）', staticLabel, 645) +
  legend([{ label: '原始标准差', color: rawColor }, { label: '处理后标准差', color: filteredColor }]) +
  barPanel({ name: '加速度样本标准差', unit: 'm/s²', top: 135, axes: axes.slice(0, 3), rawField: 'raw_std', filteredField: 'filtered_std' }) +
  barPanel({ name: '角速度样本标准差', unit: '°/s', top: 395, axes: axes.slice(3), rawField: 'raw_std', filteredField: 'filtered_std' }) +
  '<text class="note" x="36" y="621">同一前60s窗口；标准差采用 N−1；加速度 κ 同样影响幅值，变化不全由低通产生。</text>');

const driftRaw = [[], [], []], driftFiltered = [[], [], []];
let cumulativeRaw = [0, 0, 0], cumulativeFiltered = [0, 0, 0];
for (let i = 0; i < frames.length; i++) {
  const f = frames[i], x = elapsed(f);
  if (x > 300 && i > 0 && elapsed(frames[i - 1]) >= 300) break;
  if (i > 0) {
    const a = frames[i - 1], dtUs = f.t_us - a.t_us;
    if (f.seq !== a.seq + 1 || dtUs <= 0 || dtUs > 20000 || (f.flags & 4)) {
      for (let axis = 0; axis < 3; axis++) { driftRaw[axis].push(null); driftFiltered[axis].push(null); }
    } else {
      const usable = Math.max(0, Math.min(x, 300) - Math.max(elapsed(a), 0));
      const fraction = Math.min(1, usable * 1e6 / dtUs);
      for (let axis = 0; axis < 3; axis++) {
        const r0 = a.raw[axis + 3], r1 = r0 + (f.raw[axis + 3] - r0) * fraction;
        const f0 = a.filtered[axis + 3], f1 = f0 + (f.filtered[axis + 3] - f0) * fraction;
        cumulativeRaw[axis] += (r0 + r1) * 0.5 * usable;
        cumulativeFiltered[axis] += (f0 + f1) * 0.5 * usable;
      }
    }
  }
  for (let axis = 0; axis < 3; axis++) {
    driftRaw[axis].push({ x: Math.min(x, 300), y: cumulativeRaw[axis] });
    driftFiltered[axis].push({ x: Math.min(x, 300), y: cumulativeFiltered[axis] });
  }
}
let driftSvg = svgStart(kind === 'static' ? '03 · 三轴角速度独立积分漂移' : '03 · 三轴角速度独立积分（动态记录，非静止漂移）', timeLabel, 845) + legend([{ label: '原始角速度积分', color: rawColor }, { label: '处理后角速度积分', color: filteredColor }]);
for (let axis = 0; axis < 3; axis++) driftSvg += linePanel({ name: `${'XYZ'[axis]} 轴累计角度`, unit: '°', top: 130 + axis * 220, xMax: Math.min(totalSeconds, 300), series: [
  { color: rawColor, points: driftRaw[axis] }, { color: filteredColor, points: driftFiltered[axis] },
] });
driftSvg += `<text class="note" x="36" y="817">实际积分 ${fmt(metrics.drift300.duration_s, 3)} s；完整5min：${metrics.drift300.complete ? '通过' : '未通过/不足'}。缺样和长间断不桥接；此处不是融合欧拉角。</text>`;
const driftFigure = saveFigure('03-angle-drift.svg', driftSvg);

const lossPoints = frames.map((f, index) => ({ x: elapsed(f), y: f.seq + 1 - (index + 1) }));
if (metrics.transmission.lost !== null) lossPoints.push({ x: totalSeconds, y: metrics.transmission.lost });
const tr = metrics.transmission;
let lossSvg = svgStart('04 · 传输完整性与序号缺失', timeLabel, 485) + linePanel({ name: '按序号发现的累计缺帧（END 补入头尾）', unit: '帧', top: 120, plotHeight: 210, zero: true, series: [{ color: '#dc2626', points: lossPoints }] });
lossSvg += `<text x="36" y="405">生成 ${fmt(tr.generated, 0)} · 有效去重接收 ${fmt(tr.received_unique, 0)} · 丢失 ${fmt(tr.lost, 0)} · 丢包率 ${fmt(tr.loss_pct, 6)}%</text>`;
lossSvg += `<text class="note" x="36" y="433">重复 ${fmt(tr.duplicates, 0)} · 倒序 ${fmt(tr.out_of_order, 0)} · CRC错误 ${fmt(tr.crc_errors, 0)} · 异常 ${fmt(tr.invalid_frames, 0)} · END ${tr.complete ? '已确认' : '未确认，丢包率未知'}</text>`;
lossSvg += '<text class="note" x="36" y="460">曲线定位收到后发现的序号缺口；没有收到的样本不能恢复其精确丢失时刻。</text>';
const lossFigure = saveFigure('04-packet-completeness.svg', lossSvg);

const intervalPoints = [];
for (let i = 1; i < frames.length; i++) {
  if (frames[i].seq === frames[i - 1].seq + 1 && frames[i].t_us > frames[i - 1].t_us)
    intervalPoints.push({ x: elapsed(frames[i]), y: frames[i].t_us - frames[i - 1].t_us });
  else intervalPoints.push(null);
}
const sampling = metrics.sampling;
let timingSvg = svgStart('05 · 实际采样频率与间隔', timeLabel, 500) +
  linePanel({ name: '连续序号的相邻设备采样间隔', unit: 'µs', top: 120, plotHeight: 210, reference: 1e6 / (meta.sample_hz || 200), series: [{ color: '#7c3aed', points: intervalPoints }] });
timingSvg += `<text x="36" y="405">${sampling.source === 'device' ? '设备端汇总' : '电脑端连续序号估计'}：${fmt(sampling.actual_hz, 4)} Hz · mean ${fmt(sampling.dt_mean_us, 3)} µs · std ${fmt(sampling.dt_std_us, 3)} µs</text>`;
timingSvg += `<text class="note" x="36" y="433">min / max ${fmt(sampling.dt_min_us, 0)} / ${fmt(sampling.dt_max_us, 0)} µs · 错过周期 ${fmt(sampling.missed_deadlines, 0)} · 读取失败 ${fmt(sampling.sample_fail, 0)} · 无新样本 ${fmt(sampling.stale_samples, 0)}</text>`;
timingSvg += `<text class="note" x="36" y="460">接收序号缺口 ${fmt(sampling.host_gap_count, 0)} 段，已从本图间隔中排除；灰虚线为目标名义周期。</text>`;
const timingFigure = saveFigure('05-sampling-stability.svg', timingSvg);

const responseTitle = kind === 'dynamic' ? '06 · 动态原始/处理曲线（需说明动作与响应）' : '06 · 原始/处理曲线（本记录不能完成动态响应验收）';
let responseSvg = svgStart(responseTitle, timeLabel, 1505) + legend([{ label: '原始', color: rawColor }, { label: '处理后', color: filteredColor }]);
for (let axis = 0; axis < 6; axis++) responseSvg += linePanel({ name: ['ax', 'ay', 'az', 'gx', 'gy', 'gz'][axis], unit: axis < 3 ? 'm/s²' : '°/s', top: 130 + axis * 220, series: [
  { color: rawColor, points: frames.map(f => ({ x: elapsed(f), y: f.raw[axis] })) },
  { color: filteredColor, points: frames.map(f => ({ x: elapsed(f), y: f.filtered[axis] })) },
] });
responseSvg += `<text class="note" x="36" y="1475">${kind === 'dynamic' ? '应结合明确动作检查延迟和峰值衰减；记录类型标签不是动作有效性的自动证明。' : '静态或未标注动作数据仅供查看，必须另录规定移动/转动动作才能提交动态响应结果。'}</text>`;
const responseFigure = saveFigure('06-raw-filtered-curves.svg', responseSvg);

const table = (headers, rows) => `<div class="table-scroll"><table><thead><tr>${headers.map(h => `<th>${escape(h)}</th>`).join('')}</tr></thead><tbody>${rows.map(row => `<tr>${row.map(value => `<td>${escape(value)}</td>`).join('')}</tr>`).join('')}</tbody></table></div>`;
const figure = (url, alt) => `<a href="${escape(url)}"><img src="${escape(url)}" alt="${escape(alt)}" loading="lazy"/></a>`;
const badge = (pass, yes, no) => `<span class="badge ${pass ? 'pass' : 'pending'}">${escape(pass ? yes : no)}</span>`;
const numberOrUnknown = (value, unit = '', digits = 4) => value === null || value === undefined ? '未知' : `${fmt(value, digits)}${unit}`;
const reasons = metrics.reasons.length ? `<ul>${metrics.reasons.map(r => `<li>${escape(r)}</li>`).join('')}</ul>` : '<p>设备结束摘要与数据完整性检查未发现采集错误。</p>';
const html = `<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IMU 会话 ${escape(metrics.session_id)} 评测报告</title><style>
:root{color-scheme:light;font-family:Segoe UI,Microsoft YaHei,sans-serif;color:#24334b;background:#eef3f9}*{box-sizing:border-box}body{margin:0}main{max-width:1260px;margin:0 auto;padding:36px 24px 70px}header{padding:28px 30px;border-radius:18px;background:#12233f;color:#fff}h1{font-size:30px;margin:0 0 12px}header p{color:#cfdbed}h2{font-size:23px;margin:0 0 14px}p,li{line-height:1.75}section{background:#fff;border:1px solid #dce5f0;border-radius:14px;margin-top:22px;padding:26px}a{color:#2563eb}img{display:block;width:100%;height:auto;border:1px solid #e4ebf3;border-radius:8px;margin:18px 0}.badge{display:inline-block;border-radius:20px;padding:5px 11px;font-size:13px;margin:4px 5px 4px 0}.pass{background:#e1f4eb;color:#14613c}.pending{background:#fff1db;color:#8c540a}.demo{background:#a22834;color:#fff;padding:12px;border-radius:8px}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(185px,1fr));gap:12px;margin-top:20px}.card{border:1px solid #dce5f0;padding:16px;border-radius:10px}.card strong{font-size:24px;display:block;margin-top:8px}.small{font-size:13px;color:#5b6c84}.formula{padding:12px 15px;background:#f2f6fc;border-radius:8px;font-family:Consolas,Microsoft YaHei,monospace}.table-scroll{overflow:auto}table{width:100%;border-collapse:collapse;font-size:14px}th,td{text-align:left;padding:10px 12px;border-bottom:1px solid #e3eaf2;white-space:nowrap}th{background:#f3f7fc}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f3f7fc;padding:16px;border-radius:8px;font-size:13px}.warning{border-left:4px solid #d97706;padding-left:15px;color:#825018}@media print{body{background:#fff}main{max-width:none;padding:0}header{background:#fff;color:#17243c;border:1px solid #ddd}header p{color:#456}section{break-inside:avoid;padding:16px;margin-top:14px}a{color:inherit;text-decoration:none}img{max-height:24cm}h2{font-size:20px}}
</style></head><body><main><header><h1>六轴 IMU 采集与性能评测</h1><p>Session ${escape(metrics.session_id)} · ${escape(kindLabel)} · ${escape(sourceLabel)}</p><p>所有曲线横轴均为 ESP32 采样时间；报告生成时间 ${escape(new Date().toISOString())} 仅用于文件管理。</p></header>
${source !== 'hardware' ? `<p class="demo">${escape(sourceLabel)}。本报告不应作为真实硬件评测结果提交。</p>` : ''}
<div class="cards"><div class="card">设备时间覆盖<strong>${fmt(metrics.coverage_s, 3)} s</strong></div><div class="card">有效去重接收<strong>${fmt(tr.received_unique, 0)} 帧</strong></div><div class="card">实际采样频率<strong>${numberOrUnknown(sampling.actual_hz, ' Hz', 3)}</strong></div><div class="card">丢包率<strong>${numberOrUnknown(tr.loss_pct, '%', 6)}</strong></div><div class="card">processed 加速度 κ<strong>${numberOrUnknown(meta.accel_norm_scale, '', 6)}</strong><span class="small">${escape(meta.accel_calibration_method ?? '方法未提供')}</span></div></div>
<section><h2>记录条件与完整性</h2>${badge(tr.complete, 'END 已确认', 'END 未确认，结果不完整')}${kind === 'static' ? badge(metrics.static60.complete, '60s 时间与数据完整', '60s 条件不足/数据异常') + badge(metrics.drift300.complete, '5min 时间与数据完整', '5min 条件不足/数据异常') : badge(false, '', '本记录不用于静态误差/静止漂移验收')}
<p class="warning">完整标记只证明时长和数据链路检查通过，不能自动证明水平、静止或规定动作已完成。近似水平时，X/Y 加速度均值包含重力投影，不可把全部误差归因于传感器零偏。</p><p class="warning">单姿态重力模长比例 κ = g / ‖mean(raw a)‖ 仅用于 processed 加速度；raw 完整保留。它不能分离 offset、各轴 gain 或倾斜，不能代替六面标定。同姿态归一后重力模长接近 g 不是独立精度证明；校准数据与评测记录分开，测试内固定 κ。</p>${reasons}<p class="small">输入：${escape(path.basename(inputPath))}；记录时间：${escape(record.captured_at ?? '未提供')}；设备首时间戳：${fmt(startUs, 0)} µs；原始时间跨度 ${fmt(metrics.duration_s, 3)} s。时间覆盖包含最后一个名义周期，积分只使用实际相邻时间戳。</p></section>
<section><h2>1. ${kind === 'static' ? '测量误差' : '均值相对参考的差值（非静态验收）'}</h2>${kind === 'static' ? '' : '<p class="warning">本记录含动态动作，以下均值相对水平参考的差值不是传感器静态误差。</p>'}<p class="formula">角速度误差 = mean(ω)；加速度误差 mg = 1000 × (mean(a) − a_ref) / g；a_ref = [0,0,g]。</p><p>取设备会话前 ${fmt(staticSeconds, 3)} 秒；校准与滤波前后使用同一窗口。水平固定且 +Z 向上时参考成立。processed 加速度包含 κ 修正，同姿态均值误差下降不能直接证明精确三轴标定。</p>${table(['轴', '原始均值', '处理均值', '原始误差', '处理误差', '误差单位'], axes.map(a => [a.name, fmt(a.raw_mean, 6), fmt(a.filtered_mean, 6), fmt(a.raw_error, 5), fmt(a.filtered_error, 5), a.error_unit]))}${figure(errorFigure, '均值相对参考差值')}</section>
<section><h2>2. 数据波动</h2><p class="formula">样本标准差 s = sqrt(Σ(x−mean)²/(N−1))；变化比例 = (1 − s_filtered/s_raw) × 100%。</p><p>原始值仍包含共同的硬件 DLPF；这里对比额外软件处理的效果。加速度标准差也受 κ 比例影响，不能把全部变化归因于低通。原始标准差为零时变化比例不定义。${kind === 'static' ? '' : '本记录含动态动作，标准差包含动作本身的变化，不能解读为纯静态噪声。'}</p>${table(['轴', '原始标准差', '处理标准差', '变化比例', '单位'], axes.map(a => [a.name, fmt(a.raw_std, 7), fmt(a.filtered_std, 7), numberOrUnknown(a.reduction_pct, '%', 3), a.unit]))}${figure(stdFigure, '同一窗口六轴标准差')}</section>
<section><h2>3. ${kind === 'static' ? '五分钟静止角度漂移' : '角速度独立积分（非静止漂移）'}</h2>${kind === 'static' ? '' : '<p class="warning">动态转动产生的累计角度不是静止漂移；本节只作数据查看。</p>'}<p class="formula">θ = Σ((ω[k−1]+ω[k])/2) × (t[k]−t[k−1])；ω 为 °/s，dt 为秒。</p><p>三轴独立积分原始/处理角速度，不使用融合后的 Euler。实际积分 ${fmt(metrics.drift300.duration_s, 6)} s；跳过异常区间 ${fmt(metrics.drift300.skipped_intervals, 0)} 个。${kind === 'static' && metrics.drift300.complete ? '时长与数据完整性通过；仍需确认整个记录期间静止。' : '本次不能宣称完整五分钟静止漂移验收通过。'}</p>${table(['轴', '原始累计角度 °', '处理累计角度 °'], ['X', 'Y', 'Z'].map((a, i) => [a, fmt(metrics.drift300.raw_deg[i], 6), fmt(metrics.drift300.filtered_deg[i], 6)]))}${figure(driftFigure, '原始与处理三轴角速度积分曲线')}<ul>${metrics.drift300.reasons.map(r => `<li>${escape(r)}</li>`).join('')}</ul></section>
<section><h2>4. 数据丢包与异常</h2><p class="formula">丢包率 = (ESP32 END.generated − 有效且不重复接收数) / END.generated × 100%。</p>${table(['生成', '有效去重接收', '丢失', '丢包率', '重复', '倒序', 'CRC错误', '异常帧'], [[fmt(tr.generated, 0), fmt(tr.received_unique, 0), fmt(tr.lost, 0), numberOrUnknown(tr.loss_pct, '%', 6), fmt(tr.duplicates, 0), fmt(tr.out_of_order, 0), fmt(tr.crc_errors, 0), fmt(tr.invalid_frames, 0)]] )}<p>头部缺失 ${numberOrUnknown(tr.head_missing, ' 帧', 0)}，尾部缺失 ${numberOrUnknown(tr.tail_missing, ' 帧', 0)}。读取失败和无新样本独立统计，不用旧数据补齐。</p>${figure(lossFigure, '序号缺失随设备时间和最终统计')}</section>
<section><h2>5. 实际采样稳定性</h2><p class="formula">实际频率 = 1,000,000 / mean(Δt_us)。采样间隔优先采用设备摘要；接收端仅对连续序号计算。</p>${table(['统计来源', '实际频率 Hz', '间隔均值 µs', '间隔标准差 µs', '最小间隔 µs', '最大间隔 µs'], [[sampling.source, fmt(sampling.actual_hz, 6), fmt(sampling.dt_mean_us, 3), fmt(sampling.dt_std_us, 3), fmt(sampling.dt_min_us, 0), fmt(sampling.dt_max_us, 0)]] )}${table(['采样失败', '无新样本', '错过周期', '发送队列丢弃', '积分长间断', '接收序号缺口'], [[numberOrUnknown(sampling.sample_fail), numberOrUnknown(sampling.stale_samples), numberOrUnknown(sampling.missed_deadlines), numberOrUnknown(sampling.queue_drops), numberOrUnknown(sampling.integration_gaps), sampling.host_gap_count]])}${figure(timingFigure, '连续序号采样时间间隔')}</section>
<section><h2>6. 滤波响应与原始/处理曲线</h2><p class="warning">${kind === 'dynamic' ? '本记录标为动态动作，但动作、幅度和响应解释仍须人工补充。对比变动时刻、峰值和停止后的恢复过程，不能只比较平滑程度。' : '本记录是静态或未标注动作的数据；下图仅提供曲线参考，动态滤波响应验收尚未完成。请另录统一移动/转动动作。'}</p><p>一阶低通 y += α(x−y)，α = 1−exp(−2π·fc·dt)。陀螺处理包含启动零偏扣除，加速度处理包含 κ 比例；均值、峰值和波动变化不能全部归因于低通本身。</p>${table(['轴', '原始峰峰值', '处理峰峰值', '单位'], axes.map((a, i) => [a.name, fmt(metrics.response.raw_peak_to_peak[i], 6), fmt(metrics.response.filtered_peak_to_peak[i], 6), a.unit]))}${figure(responseFigure, '六轴原始和处理后采集曲线')}</section>
<section><h2>三维轴向视频与位移边界</h2><p>本静态报告不能替代实体视频。另录初始水平姿态、绕物理 +X/+Y/+Z 按右手方向各转约90°（每次先回初始），再连续组合转动至少30秒，同时拍到模块与模型。</p><p>位移为重力剔除后的短时积分估计；零速约束依赖起止静止的动作假设。六轴无绝对偏航参考，不能将短时演示作为长期定位精度证明。</p></section>
<section><h2>配置与可追溯数据</h2><p><a href="recording.json">原始/处理完整记录 JSON</a> · <a href="data.csv">CSV（若已解码生成）</a> · <a href="metrics.json">指标 JSON（若已解码生成）</a></p><details><summary>采样配置、校准值和算法参数</summary><pre>${escape(JSON.stringify(meta, null, 2))}</pre></details><details><summary>设备 END 摘要与接收诊断</summary><pre>${escape(JSON.stringify({ end, diagnostics }, null, 2))}</pre></details><ul>${metrics.notes.map(n => `<li>${escape(n)}</li>`).join('')}</ul><p class="small">图线为每像素桶保留极值的显示抽样；原始完整采样保存在 JSON/CSV 中，统计使用完整有效数据。</p></section>
</main></body></html>`;
fs.writeFileSync(path.join(outDir, 'report.html'), html);
console.log(JSON.stringify({ input: inputPath, report: path.join(outDir, 'report.html'), figures: [errorFigure, stdFigure, driftFigure, lossFigure, timingFigure, responseFigure], source, kind, complete: metrics.complete, samples: frames.length }, null, 2));
