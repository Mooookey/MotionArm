import fs from 'node:fs';
import path from 'node:path';

// Auxiliary dynamic comparison only. Positive lag means processed follows raw.
// This is not a step-response test or a measurement of display/USB latency.
const input = process.argv[2];
if (!input) {
  console.error('Usage: node tools/analyze-response.mjs <recording.json>');
  process.exit(1);
}
const inputPath = path.resolve(input);
const record = JSON.parse(fs.readFileSync(inputPath, 'utf8').replace(/^\uFEFF/, ''));
const finite = Number.isFinite;
const meta = record.meta ?? {};
const sessionId = meta.session_id ?? record.frames?.[0]?.session_id;
const sampleHz = finite(meta.sample_hz) && meta.sample_hz > 0 ? meta.sample_hz : 200;
const nominalUs = 1e6 / sampleHz;
const g = finite(meta.g) && meta.g > 0 ? meta.g : 9.80665;
const scaleKnown = finite(meta.accel_norm_scale) && meta.accel_norm_scale > 0;
const biasKnown = Array.isArray(meta.gyro_bias_dps) && meta.gyro_bias_dps.length === 3 && meta.gyro_bias_dps.every(finite);
const scale = scaleKnown ? meta.accel_norm_scale : 1;
const bias = biasKnown ? meta.gyro_bias_dps : [0, 0, 0];
const adjustmentKnown = scaleKnown && biasKnown;
const used = new Set();
const frames = (record.frames ?? []).filter(f => {
  if (f.session_id !== sessionId || !Number.isInteger(f.seq) || !finite(f.t_us) ||
    !Array.isArray(f.raw) || f.raw.length !== 6 || !f.raw.every(finite) ||
    !Array.isArray(f.filtered) || f.filtered.length !== 6 || !f.filtered.every(finite) || used.has(f.seq)) return false;
  used.add(f.seq); return true;
}).sort((a, b) => a.seq - b.seq);
if (!frames.length) throw new Error('No valid frames for response analysis');
const firstUs = frames[0].t_us;
const elapsed = f => (f.t_us - firstUs) / 1e6;
const axisNames = ['ax', 'ay', 'az', 'gx', 'gy', 'gz'];
const corrected = (f, axis) => axis < 3 ? f.raw[axis] * scale : f.raw[axis] - bias[axis - 3];
const saturated = (f, axis) => Array.isArray(f.raw_counts) && finite(f.raw_counts[axis])
  ? Math.abs(f.raw_counts[axis]) >= 32760
  : Math.abs(f.raw[axis]) >= (axis < 3 ? (meta.accel_range_g ?? 4) * g : (meta.gyro_range_dps ?? 500)) * 0.999;
const median = values => {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) / 2;
};
const round = (n, digits = 6) => finite(n) ? Number(n.toFixed(digits)) : null;
function moments(values) {
  let n = 0, mean = 0, m2 = 0, min = Infinity, max = -Infinity;
  for (const value of values) {
    n++; const delta = value - mean; mean += delta / n; m2 += delta * (value - mean);
    min = Math.min(min, value); max = Math.max(max, value);
  }
  return { mean, std: n > 1 ? Math.sqrt(m2 / (n - 1)) : 0, min, max, peak_to_peak: max - min };
}

// Continuous, stable segments: a missing seq, integration-gap flag, timestamp
// reversal or interval outside nominal +/-10% creates an explicit boundary.
const segments = [];
let segmentStart = 0, rejectedIntervals = 0;
for (let i = 1; i < frames.length; i++) {
  const dt = frames[i].t_us - frames[i - 1].t_us;
  if (frames[i].seq !== frames[i - 1].seq + 1 || dt < nominalUs * 0.9 || dt > nominalUs * 1.1 || (frames[i].flags & 4)) {
    if (i > segmentStart) segments.push([segmentStart, i]);
    segmentStart = i; rejectedIntervals++;
  }
}
segments.push([segmentStart, frames.length]);
const maxLagSamples = Math.max(1, Math.round(0.05 * sampleHz));
const windowSamples = Math.max(20, Math.round(2 * sampleHz));
const minimumSamples = Math.max(20, Math.round(0.75 * sampleHz));

function mostActiveWindow(axis) {
  let best = null;
  for (const [start, stop] of segments) {
    const count = Math.min(windowSamples, stop - start);
    if (count < minimumSamples) continue;
    let sum = 0, sumSquares = 0;
    for (let i = start; i < start + count; i++) { const x = corrected(frames[i], axis); sum += x; sumSquares += x * x; }
    for (let left = start; left + count <= stop; left++) {
      const variance = Math.max(0, (sumSquares - sum * sum / count) / Math.max(1, count - 1));
      if (!best || variance > best.variance) best = { start: left, stop: left + count, variance };
      if (left + count < stop) {
        const removed = corrected(frames[left], axis), added = corrected(frames[left + count], axis);
        sum += added - removed; sumSquares += added * added - removed * removed;
      }
    }
  }
  return best;
}

function correlation(window, axis, lag) {
  const start = window.start + Math.max(0, -lag);
  const stop = window.stop - Math.max(0, lag);
  let n = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
  const timeOffsets = [];
  for (let i = start; i < stop; i++) {
    const j = i + lag, x = corrected(frames[i], axis), y = frames[j].filtered[axis];
    n++; sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
    timeOffsets.push((frames[j].t_us - frames[i].t_us) / 1000);
  }
  const denominator = Math.sqrt(Math.max(0, sxx - sx * sx / n) * Math.max(0, syy - sy * sy / n));
  return { lag_samples: lag, lag_ms: median(timeOffsets), correlation: denominator > 1e-12 ? (sxy - sx * sy / n) / denominator : null, pairs: n };
}

const channels = axisNames.map((axis, index) => {
  const raw = moments(frames.map(f => corrected(f, index)));
  const processed = moments(frames.map(f => f.filtered[index]));
  const saturationCount = frames.reduce((n, f) => n + Number(saturated(f, index)), 0);
  const activityThreshold = index < 3 ? 0.1 : 1.0;
  const window = mostActiveWindow(index);
  const activityStd = window ? Math.sqrt(window.variance) : 0;
  const warnings = [];
  let estimate = null;
  if (saturationCount) warnings.push('原始 ADC 接近量程边界，存在饱和/削顶风险；不估计 lag。');
  else if (!window) warnings.push('没有至少 0.75 秒连续且采样间隔稳定的片段；不估计 lag。');
  else if (activityStd < activityThreshold) warnings.push(`活动量不足（窗口标准差低于 ${activityThreshold}${index < 3 ? 'm/s²' : '°/s'}）；不估计 lag。`);
  else {
    const candidates = [];
    for (let lag = -maxLagSamples; lag <= maxLagSamples; lag++) candidates.push(correlation(window, index, lag));
    const ranked = candidates.filter(c => finite(c.correlation)).sort((a, b) => b.correlation - a.correlation);
    const best = ranked[0];
    if (!best || best.correlation < 0.5) warnings.push('相关性不足，波形变化不能可靠对应；不估计 lag。');
    else if (Math.abs(best.lag_samples) === maxLagSamples) warnings.push('相关峰落在 ±50ms 搜索边界，无法确定峰位置；不报告 lag。');
    else {
      const neighbours = candidates.filter(c => Math.abs(c.lag_samples - best.lag_samples) === 1 && finite(c.correlation));
      const broadPeak = neighbours.some(c => best.correlation - c.correlation < 0.001);
      const offsets = [];
      for (let i = window.start + 1; i < window.stop; i++) offsets.push((frames[i].t_us - frames[i - 1].t_us) / 1000);
      estimate = { lag_samples: best.lag_samples, lag_ms: round(best.lag_ms, 1), correlation: round(best.correlation, 3),
        nominal_resolution_ms: round(median(offsets), 1), pairs: best.pairs, broad_peak: broadPeak };
      if (broadPeak) warnings.push('相关峰较宽，邻近一个采样周期的 lag 也可能成立；不作亚采样精度解释。');
      if (best.lag_samples < 0) warnings.push('辅助相关峰为负 lag，不能解释为因果滤波具有负延迟；需人工检查动作、窗口和噪声。');
    }
  }
  const positiveIndex = frames.reduce((best, f, i) => corrected(f, index) > corrected(frames[best], index) ? i : best, 0);
  const negativeIndex = frames.reduce((best, f, i) => corrected(f, index) < corrected(frames[best], index) ? i : best, 0);
  return { axis, unit: index < 3 ? 'm/s²' : '°/s', reference_adjustment: index < 3 ? `raw × κ (${scale})` : `raw − bias (${bias[index - 3]} °/s)`,
    raw_reference_peak_to_peak: round(raw.peak_to_peak), processed_peak_to_peak: round(processed.peak_to_peak),
    peak_to_peak_ratio: raw.peak_to_peak > 1e-12 ? round(processed.peak_to_peak / raw.peak_to_peak, 4) : null,
    saturation_samples: saturationCount, activity_std: round(activityStd), activity_threshold: activityThreshold,
    window_elapsed_s: window ? [round(elapsed(frames[window.start]), 3), round(elapsed(frames[window.stop - 1]), 3)] : null,
    lag: estimate, positive_peak: { value: round(raw.max), elapsed_s: round(elapsed(frames[positiveIndex]), 3) },
    negative_peak: { value: round(raw.min), elapsed_s: round(elapsed(frames[negativeIndex]), 3) }, warnings };
});
const notes = [
  '峰峰值使用完整有效记录；不同轴的相关估计可能选取不同的高活动2秒窗口。',
  '为尽量隔离软件低通，raw加速度先乘本次κ，raw陀螺先减本次零偏，再与processed比较。',
  '相关 lag 的定义为 corr(reference[i], processed[i+lag])；正数表示processed较晚。',
  '仅在连续序号且实际采样间隔处于名义±10%的片段中按索引配对；lag毫秒值来自真实t_us差值中位数。',
  '搜索范围约±50ms，不插值求亚采样峰值；常见200Hz数据分辨率约5ms，显示小数仅表示时间戳量化。',
  '相关峰为辅助波形对齐估计，不是严格阶跃响应，也不是USB、浏览器或模型端到端延迟。',
  '低活动、相关性不足、量程边界或搜索边界情况不报告lag；周期动作和宽峰仍存在歧义。',
  '本次操作约定：抬远端预期绕+X正转；抬左边预期绕+Y正转；上视逆时针预期绕+Z正转。是否符合需结合实体方向、动作时间和视频确认。',
  ...(adjustmentKnown ? [] : ['META缺少κ或零偏，缺失项按κ=1/bias=0作参考；不能宣称已隔离校准影响。']),
  ...(record.source === 'hardware' ? [] : [`数据来源为${record.source ?? 'unknown'}，不可作为已验证的硬件响应证据。`]),
];
const report = { source: record.source ?? 'unknown', input: path.basename(inputPath), session_id: sessionId,
  samples: frames.length, duration_s: round(elapsed(frames.at(-1)), 6), reference_adjustments_known: adjustmentKnown,
  accel_norm_scale: scale, gyro_bias_dps: bias, rejected_intervals: rejectedIntervals,
  method: 'peak_to_peak_and_auxiliary_integer_lag_correlation', channels, notes,
  manual_observation_required: true, axis_alignment_verified: false, end_to_end_latency_measured: false };
const format = (value, digits = 4) => finite(value) ? value.toFixed(digits).replace(/\.?0+$/, '') || '0' : '未估计';
const lines = [
  '# 动态原始/处理响应辅助分析', '',
  `数据来源：${report.source}；会话 ${sessionId}；${frames.length} 帧；设备时间跨度 ${format(report.duration_s, 3)} 秒。`, '',
  '**相关 lag 仅用于波形对齐，不是严格阶跃响应或端到端延迟。仍需结合规定动作和实物视频填写现象说明。**', '',
  '## 峰峰值与辅助相关峰', '',
  '| 轴 | 比较参考 | 参考峰峰值 | processed峰峰值 | 幅值比 | lag ms | 相关系数 | 估计窗口 s |',
  '|---|---|---|---|---|---|---|---|',
  ...channels.map(c => `| ${c.axis} (${c.unit}) | ${c.reference_adjustment} | ${format(c.raw_reference_peak_to_peak, 6)} | ${format(c.processed_peak_to_peak, 6)} | ${format(c.peak_to_peak_ratio, 4)} | ${c.lag ? format(c.lag.lag_ms, 1) : '未估计'} | ${c.lag ? format(c.lag.correlation, 3) : '—'} | ${c.window_elapsed_s ? c.window_elapsed_s.map(v => format(v, 3)).join('–') : '—'} |`), '',
  '## 主角速度轴与方向参考', '',
  '| 轴 | 最大正角速度 °/s（设备elapsed_s） | 最大负角速度 °/s（设备elapsed_s） | 本次动作约定 |',
  '|---|---|---|---|',
  ...channels.slice(3).map((c, i) => `| ${c.axis} | ${format(c.positive_peak.value, 3)} @ ${format(c.positive_peak.elapsed_s, 3)} | ${format(c.negative_peak.value, 3)} @ ${format(c.negative_peak.elapsed_s, 3)} | ${['抬远端预期X正', '抬左边预期Y正', '上视逆时针预期Z正'][i]} |`), '',
  '正负峰可能对应正向动作、回位或其他转动；这些数字不能自动证明模型与实体轴向一致。', '',
  '## 估计限制与通道提示', '',
  ...channels.flatMap(c => c.warnings.map(w => `- ${c.axis}：${w}`)),
  ...notes.map(note => `- ${note}`), '',
  '## 人工现象说明（待结合动作/视频填写）', '',
  '- 规定动作的时间段和幅度：待确认。',
  '- 原始/处理曲线的平滑、延迟、峰值和停止后恢复：待确认。',
  '- 四步轴对齐和至少30秒连续模型跟随：不能由本相关分析替代。', '',
];
const directory = path.dirname(inputPath);
fs.writeFileSync(path.join(directory, 'response.json'), JSON.stringify(report, null, 2));
fs.writeFileSync(path.join(directory, 'response.md'), lines.join('\n'));
console.log(JSON.stringify({ directory, session_id: sessionId, samples: frames.length, source: report.source,
  rejected_intervals: rejectedIntervals, channels: channels.map(c => ({ axis: c.axis, ratio: c.peak_to_peak_ratio, lag: c.lag, warnings: c.warnings })) }, null, 2));
