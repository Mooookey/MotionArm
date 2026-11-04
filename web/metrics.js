// Metrics use ESP32 sampling time, never browser arrival time. No dependencies.
const G = 9.80665;
const AXES = ['ax', 'ay', 'az', 'gx', 'gy', 'gz'];
const finite = value => typeof value === 'number' && Number.isFinite(value);
const counter = value => finite(value) && value >= 0 ? value : 0;

function stats(values) {
  let n = 0, mean = 0, m2 = 0, min = Infinity, max = -Infinity;
  for (const x of values) {
    if (!finite(x)) continue;
    n++;
    const delta = x - mean;
    mean += delta / n;
    m2 += delta * (x - mean);
    min = Math.min(min, x); max = Math.max(max, x);
  }
  return { n, mean: n ? mean : null, std: n > 1 ? Math.sqrt(m2 / (n - 1)) : n ? 0 : null,
    min: n ? min : null, max: n ? max : null };
}

function isDataFrame(frame) {
  return frame && Number.isInteger(frame.seq) && frame.seq >= 0 &&
    finite(frame.t_us) && frame.t_us >= 0 &&
    Array.isArray(frame.raw) && frame.raw.length === 6 && frame.raw.every(finite) &&
    Array.isArray(frame.filtered) && frame.filtered.length === 6 && frame.filtered.every(finite);
}

/**
 * Analyze a recording. Frames may already be deduplicated by the receiver.
 * Partial results retain numbers, but complete flags state whether they can be
 * presented as the prescribed 60 s / 300 s tests. All standard deviations here
 * use the sample estimator (N-1). Device dt_std_us is reported as supplied.
 */
export function analyzeSession(inputFrames = [], meta = {}, end = null, diagnostics = {}) {
  const framesInput = Array.isArray(inputFrames) ? inputFrames : [];
  const sessionId = meta.session_id ?? end?.session_id ?? framesInput[0]?.session_id ?? null;
  const bySeq = new Map();
  let invalid = 0, duplicatesSeen = 0, orderSeen = 0, timeErrorsSeen = 0, maxSeq = -1, previousTime = -1;
  for (const frame of framesInput) {
    if (!isDataFrame(frame) || (sessionId !== null && frame.session_id !== sessionId)) { invalid++; continue; }
    if (bySeq.has(frame.seq)) { duplicatesSeen++; continue; }
    if (frame.seq < maxSeq) orderSeen++;
    if (frame.seq > maxSeq && frame.t_us <= previousTime) timeErrorsSeen++;
    if (frame.seq > maxSeq) { maxSeq = frame.seq; previousTime = frame.t_us; }
    bySeq.set(frame.seq, frame);
  }
  const frames = [...bySeq.values()].sort((a, b) => a.seq - b.seq);
  const hz = finite(meta.sample_hz) && meta.sample_hz > 0 ? meta.sample_hz : 200;
  const nominalUs = 1e6 / hz;
  const g = finite(meta.g) && meta.g > 0 ? meta.g : G;
  const firstReceived = frames[0]?.t_us ?? null;
  const lastReceived = frames.at(-1)?.t_us ?? null;
  const matchingEnd = end && end.session_id === sessionId;
  const generated = matchingEnd && Number.isInteger(end.generated) && end.generated >= 0 ? end.generated : null;
  const startUs = matchingEnd && finite(end.first_t_us) && generated > 0 ? end.first_t_us : firstReceived;
  const stopUs = matchingEnd && finite(end.last_t_us) && generated > 0 ? end.last_t_us : lastReceived;
  const duration = startUs !== null && stopUs !== null ? Math.max(0, (stopUs - startUs) / 1e6) : 0;
  const coverage = frames.length ? duration + nominalUs / 1e6 : 0;
  const diagnosticsTimeErrors = counter(diagnostics.timestamp_errors);
  const duplicates = Math.max(duplicatesSeen, counter(diagnostics.duplicates));
  const outOfOrder = Math.max(orderSeen, counter(diagnostics.out_of_order));
  const timestampErrors = Math.max(timeErrorsSeen, diagnosticsTimeErrors);
  const crcErrors = counter(diagnostics.crc_errors);
  const invalidFrames = invalid + counter(diagnostics.invalid_frames) + counter(diagnostics.json_errors);

  let hostGapCount = 0, missingInterior = 0, badTimestampPairs = 0, longHostIntervals = 0;
  const consecutiveDt = [];
  for (let i = 1; i < frames.length; i++) {
    const previous = frames[i - 1], current = frames[i];
    const distance = current.seq - previous.seq;
    if (distance > 1) { hostGapCount++; missingInterior += distance - 1; }
    const dt = current.t_us - previous.t_us;
    if (dt <= 0) badTimestampPairs++;
    if (distance === 1 && dt > 20000) longHostIntervals++;
    if (distance === 1 && dt > 0) consecutiveDt.push(dt);
  }
  const hostDt = stats(consecutiveDt);
  const deviceDt = matchingEnd && counter(end.dt_count) > 0 && finite(end.dt_mean_us) && end.dt_mean_us > 0;
  const actualHz = deviceDt ? 1e6 / end.dt_mean_us : hostDt.mean > 0 ? 1e6 / hostDt.mean : null;
  const sampling = {
    source: deviceDt ? 'device' : 'host_consecutive_sequences', actual_hz: actualHz,
    dt_mean_us: deviceDt ? end.dt_mean_us : hostDt.mean,
    dt_std_us: deviceDt ? end.dt_std_us : hostDt.std,
    dt_min_us: deviceDt ? end.dt_min_us : hostDt.min,
    dt_max_us: deviceDt ? end.dt_max_us : hostDt.max,
    host_gap_count: hostGapCount, timestamp_errors: timestampErrors + badTimestampPairs,
    sample_fail: matchingEnd ? counter(end.sample_fail) : null,
    stale_samples: matchingEnd ? counter(end.stale_samples) : null,
    missed_deadlines: matchingEnd ? counter(end.missedDeadlines ?? end.missed_deadlines) : null,
    queue_drops: matchingEnd ? counter(end.queue_drops) : null,
    integration_gaps: matchingEnd ? counter(end.integration_gaps) : null,
  };
  const endConsistent = matchingEnd && generated !== null && generated >= frames.length &&
    (!frames.length || frames.at(-1).seq < generated);
  const lost = endConsistent ? generated - frames.length : null;
  const headMissing = frames.length ? frames[0].seq : 0;
  const tailMissing = endConsistent && frames.length ? Math.max(0, generated - frames.at(-1).seq - 1) : null;
  const transmission = {
    complete: !!endConsistent, generated, received_unique: frames.length, lost,
    loss_pct: endConsistent && generated > 0 ? lost / generated * 100 : null,
    duplicates, out_of_order: outOfOrder, invalid_frames: invalidFrames, crc_errors: crcErrors,
    missing_seq: endConsistent ? lost : missingInterior + headMissing,
    head_missing: headMissing, tail_missing: tailMissing,
  };

  const reasons = [];
  if (!end) reasons.push('缺少 ESP32 END 摘要，测试边界及丢包率尚未确认');
  else if (!matchingEnd) reasons.push('结束摘要与数据会话 ID 不一致');
  else if (!endConsistent) reasons.push('结束摘要生成数与接收序号不一致');
  if (!frames.length) reasons.push('没有有效数据帧');
  if (lost > 0) reasons.push(`缺失 ${lost} 帧（含测试头尾）`);
  if (sampling.sample_fail > 0) reasons.push(`传感器读取失败 ${sampling.sample_fail} 次`);
  if (sampling.stale_samples > 0) reasons.push(`传感器无新样本 ${sampling.stale_samples} 次`);
  if (sampling.missed_deadlines > 0) reasons.push(`错过采样周期 ${sampling.missed_deadlines} 次`);
  if (sampling.queue_drops > 0) reasons.push(`设备发送队列丢弃 ${sampling.queue_drops} 帧`);
  if (sampling.integration_gaps > 0) reasons.push(`积分长间断 ${sampling.integration_gaps} 次`);
  if (longHostIntervals > 0) reasons.push(`收到 ${longHostIntervals} 个超过 20ms 的连续序号采样间隔`);
  if (timestampErrors + badTimestampPairs > 0) reasons.push('采样时间戳异常');
  if (meta.calibrated === false || matchingEnd && end.calibrated === false || frames.some(f => !(f.flags & 1)))
    reasons.push('数据包含未完成校准的样本');

  const reference = [0, 0, g, 0, 0, 0];
  const staticFrames = startUs === null ? [] : frames.filter(f => f.t_us >= startUs && f.t_us < startUs + 60e6);
  const staticAxes = AXES.map((name, axis) => {
    const raw = stats(staticFrames.map(f => f.raw[axis]));
    const filtered = stats(staticFrames.map(f => f.filtered[axis]));
    const factor = axis < 3 ? 1000 / g : 1;
    return { name, unit: axis < 3 ? 'm/s²' : '°/s', reference: reference[axis],
      raw_mean: raw.mean, filtered_mean: filtered.mean, raw_std: raw.std, filtered_std: filtered.std,
      raw_error: raw.mean === null ? null : (raw.mean - reference[axis]) * factor,
      filtered_error: filtered.mean === null ? null : (filtered.mean - reference[axis]) * factor,
      error_unit: axis < 3 ? 'mg' : '°/s',
      reduction_pct: raw.std > 0 ? (1 - filtered.std / raw.std) * 100 : null };
  });
  // A complete static test is conservative: all session acquisition failures
  // invalidate the completeness flag because END does not timestamp each error.
  const acquisitionOK = endConsistent && lost === 0 && sampling.sample_fail === 0 && sampling.stale_samples === 0 &&
    sampling.missed_deadlines === 0 && sampling.queue_drops === 0 && sampling.integration_gaps === 0 &&
    timestampErrors + badTimestampPairs === 0 && longHostIntervals === 0 && !reasons.some(r => r.includes('未完成校准'));
  const static60 = { complete: !!(coverage >= 60 - 1e-6 && acquisitionOK), samples: staticFrames.length,
    duration_s: Math.min(60, coverage), axes: staticAxes };

  const rawIntegral = [0, 0, 0], filteredIntegral = [0, 0, 0];
  let skippedIntervals = 0, integratedSeconds = 0;
  const windowEndUs = startUs === null ? 0 : startUs + 300e6;
  for (let i = 1; i < frames.length; i++) {
    const a = frames[i - 1], b = frames[i];
    if (a.t_us >= windowEndUs) break;
    const dtUs = b.t_us - a.t_us;
    if (b.seq !== a.seq + 1 || dtUs <= 0 || dtUs > 20000 || (b.flags & 4)) { skippedIntervals++; continue; }
    const usableUs = Math.min(b.t_us, windowEndUs) - Math.max(a.t_us, startUs);
    if (usableUs <= 0) continue;
    const leftFraction = Math.max(0, startUs - a.t_us) / dtUs;
    const rightFraction = Math.min(1, (windowEndUs - a.t_us) / dtUs);
    const dtSeconds = usableUs / 1e6;
    for (let axis = 0; axis < 3; axis++) {
      const index = axis + 3;
      const ar = a.raw[index] + (b.raw[index] - a.raw[index]) * leftFraction;
      const br = a.raw[index] + (b.raw[index] - a.raw[index]) * rightFraction;
      const af = a.filtered[index] + (b.filtered[index] - a.filtered[index]) * leftFraction;
      const bf = a.filtered[index] + (b.filtered[index] - a.filtered[index]) * rightFraction;
      rawIntegral[axis] += (ar + br) * 0.5 * dtSeconds;
      filteredIntegral[axis] += (af + bf) * 0.5 * dtSeconds;
    }
    integratedSeconds += dtSeconds;
  }
  const driftReasons = [...reasons];
  if (coverage < 300 - 1e-6) driftReasons.push(`记录不足 5 分钟（设备时间覆盖 ${coverage.toFixed(3)} 秒）`);
  if (skippedIntervals) driftReasons.push(`未跨越 ${skippedIntervals} 个缺样或异常积分区间`);
  const drift300 = { complete: !!(coverage >= 300 - 1e-6 && acquisitionOK && skippedIntervals === 0),
    duration_s: integratedSeconds, raw_deg: rawIntegral, filtered_deg: filteredIntegral,
    skipped_intervals: skippedIntervals, reasons: driftReasons };
  const peakToPeak = field => AXES.map((_, axis) => {
    const axisStats = stats(frames.map(f => f[field][axis]));
    return axisStats.n ? axisStats.max - axisStats.min : null;
  });
  return {
    session_id: sessionId, source: diagnostics.source ?? 'hardware', duration_s: duration,
    coverage_s: coverage, sample_count: frames.length,
    complete: { session: !!endConsistent, static60: static60.complete, drift300: drift300.complete },
    reasons, static60, drift300, transmission, sampling,
    response: { raw_peak_to_peak: peakToPeak('raw'), filtered_peak_to_peak: peakToPeak('filtered'),
      note: '峰峰值仅为辅助量；动态响应须结合带设备时间轴的原始/处理曲线，说明延迟、幅值与动作。' },
    notes: [
      '静态参考要求水平固定且物理 +Z 向上；加速度误差为带符号的均值差。',
      '静态统计使用前 60 秒及样本标准差 N-1；不足时仅提供已有数据的参考结果。',
      '时间覆盖包含最后一个名义采样周期；积分时长只累计实际相邻时间戳间隔。',
      '采样间隔优先使用 ESP32 摘要；电脑端仅对连续序号计算间隔，丢帧间隔单列。',
      '漂移比较为角速度独立积分，不是融合欧拉角；长间断不桥接。',
      ...(diagnostics.source === 'demo' ? ['模拟数据，只用于界面演示，不是硬件验收证据。'] : []),
    ],
  };
}
