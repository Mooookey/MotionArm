import { FrameParser, TYPES, BAUD_RATE } from './protocol.js';
import { quaternionToEuler, rotateVector } from './math.js';
import { analyzeSession } from './metrics.js';
import { framesToCsv, validateRecording } from './recording.js';

const $ = id => document.getElementById(id);
const palette = { raw: '#62758a', filtered: '#9ce8bd', grid: '#263341', text: '#6e8399', x: '#ed8c8b', y: '#9ce8bd', z: '#88bff3' };
const demo = new URLSearchParams(location.search).get('demo') === '1';
let port = null, reader = null, writer = null, readTask = null;
let connected = false, closing = false, recording = false, source = demo ? 'demo' : 'hardware', imported = false;
let meta = {}, end = null, frames = [], diagnostics = {}, seen = new Set(), highestSeq = -1, previousTimestamp = null;
let parser = new FrameParser(handleFrame), cursor = 0, playing = false, playClock = 0, playTimestamp = 0;
let pendingStart = false, metrics = null, lastMetricTime = 0, lastDrawTime = 0, demoTimer = null, demoSeq = 0;
let lastValidDataAt = null, recordingStartedAt = null, receiveStale = false, stopping = false;
const RECEIVE_TIMEOUT_MS = 500;
const events = [];

function message(text, kind = '') { $('notice').textContent = text; $('notice').className = `notice ${kind}`; }
function log(text, kind = '') {
  const item = { time: new Date().toLocaleTimeString('zh-CN', { hour12: false }), text, kind };
  events.unshift(item); if (events.length > 100) events.pop();
  const row = document.createElement('div'); row.className = `event ${kind}`;
  const time = document.createElement('time'); time.textContent = item.time;
  const content = document.createElement('span'); content.textContent = text;
  row.append(time, content); $('eventLog').prepend(row);
  while ($('eventLog').children.length > 100) $('eventLog').lastChild.remove();
  $('logCount').textContent = `${events.length} EVENTS`;
}

function getDiagnostics() {
  const result = { ...diagnostics, source };
  for (const [key, value] of Object.entries(parser.diagnostics)) result[key] = (diagnostics[key] ?? 0) + value;
  return result;
}
function resetSession(newMeta = {}) {
  meta = { ...newMeta }; frames = []; end = null; diagnostics = {}; seen = new Set(); highestSeq = -1; previousTimestamp = null;
  cursor = 0; playing = false; metrics = null; lastMetricTime = 0;
  lastValidDataAt = null; recordingStartedAt = null; receiveStale = false; stopping = false;
  parser.diagnostics = { crc_errors: 0, invalid_frames: 0, json_errors: 0, discarded_bytes: 0 };
}

function addData(f) {
  if (meta.session_id != null && f.session_id !== meta.session_id) { diagnostics.foreign_frames = (diagnostics.foreign_frames ?? 0) + 1; return false; }
  if (meta.session_id == null) meta.session_id = f.session_id;
  if (seen.has(f.seq)) { diagnostics.duplicates = (diagnostics.duplicates ?? 0) + 1; return false; }
  if (f.seq < highestSeq) diagnostics.out_of_order = (diagnostics.out_of_order ?? 0) + 1;
  if (previousTimestamp != null && f.t_us <= previousTimestamp) diagnostics.timestamp_errors = (diagnostics.timestamp_errors ?? 0) + 1;
  seen.add(f.seq); highestSeq = Math.max(highestSeq, f.seq); previousTimestamp = f.t_us;
  frames.push(f);
  if (!imported && !playing) cursor = frames.length;
  return true;
}

function updateReceiveHealth(now) {
  // Host wall time measures reception health only; all plots/statistics retain device t_us.
  const watching = connected && recording && !stopping && !imported && !demo && String(meta.state).toLowerCase() === 'recording';
  const reference = lastValidDataAt ?? recordingStartedAt;
  const stale = watching && reference != null && now - reference > RECEIVE_TIMEOUT_MS;
  if (stale === receiveStale) return;
  receiveStale = stale;
  if (stale) {
    message('数据接收告警：超过 500 ms 未收到本会话的有效新 DATA，请检查设备与串口。接收看门狗使用电脑时间，不影响设备采样时间统计。', 'error');
    log('接收看门狗：超过 500 ms 无有效新 DATA', 'error');
  } else if (watching) {
    message('有效数据接收已恢复，继续记录。', 'success'); log('接收看门狗：有效数据恢复');
  } else if ($('notice').textContent.startsWith('数据接收告警：')) message('当前未进行实时记录，接收看门狗已暂停。');
  updateControls();
}

function handleFrame(frame) {
  const { type, data } = frame;
  if (type === TYPES.META) {
    const wasRecording = recording, wasCalibrated = meta.calibrated;
    const newSession = data.session_id !== meta.session_id;
    const active = ['recording', 'draining'].includes(String(data.state).toLowerCase());
    if (newSession || (pendingStart && String(data.state).toLowerCase() === 'recording')) {
      resetSession(data); imported = false; source = 'hardware'; pendingStart = false;
      log(`会话 ${data.session_id} 开始，目标 ${data.sample_hz ?? 200} Hz`);
    } else meta = { ...meta, ...data };
    recording = active;
    if (active && (!wasRecording || recordingStartedAt == null)) { recordingStartedAt = performance.now(); stopping = false; }
    $('metadataView').textContent = JSON.stringify(meta, null, 2);
    if (data.calibrated && !wasCalibrated && !active) message('静态校准已完成，可以开始记录。', 'success');
    log(`META · ${data.state ?? 'unknown'} · 校准 ${data.calibrated ? '完成' : '未完成'}`);
  } else if (type === TYPES.DATA) {
    if (end) { diagnostics.after_end_frames = (diagnostics.after_end_frames ?? 0) + 1; return; }
    if (!addData(data)) return;
    recording = true; lastValidDataAt = performance.now();
    if (frames.length === 1) updateControls();
    return; // Receive/store at 200 Hz; UI redraw remains capped at 30 Hz.
  } else if (type === TYPES.END) {
    if (data.session_id !== meta.session_id) { log(`忽略其他会话 END ${data.session_id}`); return; }
    const replay = end != null;
    end = data;
    // INFO/STOP may replay a completed session's END while a later calibration is running.
    if (!replay) { meta.state = data.state ?? 'idle'; meta.calibrated = data.calibrated ?? meta.calibrated; }
    recording = false; pendingStart = false; stopping = false; cursor = frames.length;
    updateReceiveHealth(performance.now());
    refreshMetrics(); log(`END · 生成 ${data.generated}，发送 ${data.sent}，有效接收 ${frames.length}`);
    if (!replay || String(meta.state).toLowerCase() === 'idle') message(`记录结束，共 ${frames.length.toLocaleString()} 个有效帧。可导出 CSV、JSON 或回放。`, 'success');
  } else if (type === TYPES.STATUS) {
    if (data.session_id === meta.session_id) {
      meta = { ...meta, state: data.state, calibrated: data.calibrated ?? meta.calibrated, latest_status: data };
      const wasRecording = recording;
      recording = ['recording', 'draining'].includes(String(data.state).toLowerCase());
      if (recording && !wasRecording) { recordingStartedAt = performance.now(); stopping = false; }
      $('metadataView').textContent = JSON.stringify(meta, null, 2);
    }
    log(`STATUS · ${data.state} · generated=${data.generated ?? '—'}`);
  } else if (type === TYPES.ERROR) {
    pendingStart = false;
    message(data.code === 'CAL_MOVING' ? '检测到移动，校准已重新计时。请保持传感器静止，等待 600 个稳定样本。' : `设备：${data.message ?? data.code}`, 'error');
    log(`ERROR ${data.code}: ${data.message}`, 'error');
  }
  updateControls();
}

async function sendCommand(command) {
  if (!writer) return;
  try { await writer.write(new TextEncoder().encode(command + '\n')); log(`→ ${command}`); }
  catch (error) { message(`命令发送失败：${error.message}`, 'error'); log(error.message, 'error'); }
}

async function connect() {
  if (!('serial' in navigator)) { message('当前浏览器未开放 Web Serial。请使用桌面 Edge 或 Chrome，并通过 localhost 打开页面。', 'error'); return; }
  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: BAUD_RATE, bufferSize: 65536 });
    try { await port.setSignals({ dataTerminalReady: false, requestToSend: false }); } catch { /* Some serial drivers do not expose control signals. */ }
    reader = port.readable.getReader(); writer = port.writable.getWriter();
    const keepSession = frames.length > 0 && !imported && source === 'hardware';
    const previousDiagnostics = getDiagnostics();
    parser = new FrameParser(handleFrame); connected = true; closing = false; imported = false; source = 'hardware';
    if (keepSession) diagnostics = previousDiagnostics;
    else resetSession();
    updateControls(); log(`USB 串口已连接 · ${BAUD_RATE} baud`);
    message('已连接。设备启动后获取信息；校准时请将板面水平静置、Z 轴向上。');
    readTask = readSerial(); await sendCommand('INFO');
    setTimeout(() => { if (connected && !recording) sendCommand('INFO'); }, 1600);
  } catch (error) {
    if (error.name !== 'NotFoundError') { message(`连接失败：${error.message}。请检查串口是否被占用。`, 'error'); log(error.message, 'error'); }
    if (port && !connected) { try { await port.close(); } catch {} port = null; }
    updateControls();
  }
}

async function readSerial() {
  try {
    while (reader && !closing) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value) parser.push(value);
    }
  } catch (error) {
    if (!closing) { message(`串口读取中断：${error.message}。未收到 END 的记录完整性未知。`, 'error'); log(error.message, 'error'); }
  } finally {
    try { reader?.releaseLock(); } catch {}
    reader = null;
    if (!closing) {
      connected = false; recording = false; pendingStart = false;
      try { writer?.releaseLock(); } catch {} writer = null;
      try { await port?.close(); } catch {} port = null;
      refreshMetrics(); updateControls();
    }
  }
}

async function disconnect() {
  stopping = true; updateReceiveHealth(performance.now());
  if (recording) { await sendCommand('STOP'); message('请求停止并等待 END；断开后若未收到 END，丢帧率仍为未知。'); await new Promise(resolve => setTimeout(resolve, 450)); }
  closing = true;
  try { await reader?.cancel(); await readTask; } catch {}
  try { writer?.releaseLock(); } catch {} writer = null;
  try { await port?.close(); } catch (error) { log(error.message, 'error'); }
  connected = false; recording = false; pendingStart = false; port = null; closing = false;
  log('串口已释放'); refreshMetrics(); updateControls();
}

function updateControls() {
  const state = String(meta.state).toLowerCase();
  const idle = connected && state === 'idle' && !recording && !pendingStart;
  $('connectBtn').disabled = connected || demo;
  $('disconnectBtn').disabled = !connected;
  $('startBtn').disabled = demo ? recording : !idle || !meta.calibrated;
  $('stopBtn').disabled = demo ? !recording : !connected;
  $('calibrateBtn').disabled = !connected || !['idle', 'error'].includes(state) || pendingStart || demo;
  $('zeroBtn').disabled = !idle || !meta.calibrated || demo;
  $('infoBtn').disabled = !connected;
  const hasFrames = frames.length > 0;
  for (const id of ['csvBtn', 'jsonBtn']) $(id).disabled = !hasFrames;
  $('importBtn').disabled = connected || recording;
  $('playBtn').disabled = !hasFrames || recording;
  $('timeline').disabled = !hasFrames || recording;
  $('liveBtn').disabled = !hasFrames || recording;
  $('playBtn').textContent = playing ? 'Ⅱ 暂停' : '▶ 回放';
  const badge = $('connectionBadge');
  badge.className = `live-badge ${connected ? 'connected' : ''} ${recording ? 'recording' : ''} ${receiveStale ? 'stale' : ''}`;
  const label = receiveStale ? '数据中断 >500 ms' : demo ? (recording ? '模拟演示中' : '模拟记录') : connected ? (recording ? '正在记录' : '设备已连接') : imported ? '记录回放' : '尚未连接';
  badge.replaceChildren(Object.assign(document.createElement('i'), {}), document.createTextNode(label));
  const stateNames = { idle: '待命', recording: '记录中', draining: '正在排空发送队列', calibrating: '静态校准中', error: '设备故障', ready: '待命', booting: '启动中' };
  $('deviceState').textContent = `${stateNames[String(meta.state).toLowerCase()] ?? (connected ? '等待设备状态' : imported ? '导入记录 · 离线查看' : '等待设备连接')}${meta.calibrated ? ' · 已校准' : connected ? ' · 需要静态校准' : ''}`;
  if (state === 'calibrating') {
    const status = meta.latest_status ?? {};
    const progress = status.calibration_progress ?? meta.calibration_samples ?? 0;
    const moved = ['accel_change', 'gyro_change', 'accel_std', 'gyro_std'].includes(status.calibration_reset_reason);
    $('deviceState').textContent = `静态校准中 · ${progress} / 600 个稳定样本${moved ? ' · 检测到移动，已重新计时' : ' · 请保持静止'}`;
  }
  $('baudRate').textContent = BAUD_RATE;
  $('accelScaleNote').textContent = Number.isFinite(meta.accel_norm_scale) ? `处理加速度比例 κ = ${meta.accel_norm_scale.toFixed(6)} · 原始值保留` : '全帧接收与记录 · 30 Hz 绘图';
  const banner = $('sourceBanner'); banner.hidden = !demo && !imported;
  banner.className = `source-banner ${imported && source !== 'demo' ? 'imported' : ''}`;
  banner.textContent = source === 'demo' ? '模拟演示 · 所有信号由浏览器生成，不是硬件测量，也不作为性能验收证据。' : `导入记录 · ${source === 'hardware' ? '来源：硬件采集' : '来源：' + source} · 当前为离线回放`;
}

function number(value, digits = 2, fallback = '—') { return Number.isFinite(value) ? value.toFixed(digits) : fallback; }
function clock(seconds) {
  const s = Math.max(0, seconds || 0);
  return `${Math.floor(s / 60).toString().padStart(2, '0')}:${(s % 60).toFixed(2).padStart(5, '0')}`;
}
function pair(label, value) {
  const row = document.createElement('div'); const dt = document.createElement('dt'); dt.textContent = label;
  const dd = document.createElement('dd'); dd.textContent = value; row.append(dt, dd); return row;
}

function refreshMetrics() {
  lastMetricTime = performance.now();
  if (!frames.length) return;
  try { metrics = analyzeSession(frames, meta, end, getDiagnostics()); }
  catch (error) { log(`评测失败：${error.message}`, 'error'); return; }
  const m = metrics, tr = m.transmission, sa = m.sampling;
  $('sampleHz').textContent = number(sa.actual_hz, 2);
  $('hzDetail').textContent = `间隔 ${number(sa.dt_mean_us / 1000, 3)} ms · σ ${number(sa.dt_std_us, 1)} µs`;
  $('frameCount').textContent = frames.length.toLocaleString();
  $('frameDetail').textContent = `Session ${meta.session_id ?? '—'} · ${end ? '已结束' : recording ? '接收中' : 'END 未确认'}`;
  $('duration').textContent = number(m.duration_s, 2);
  $('lossRate').textContent = tr.loss_pct == null ? '未知' : number(tr.loss_pct, 3);
  $('lossRate').className = tr.loss_pct == null ? 'pending' : '';
  $('lossDetail').textContent = tr.loss_pct == null ? '未收到 END · 无法确定总生成数' : `丢失 ${tr.lost} / 生成 ${tr.generated}`;
  $('qualityList').replaceChildren(
    pair('会话完整性', m.complete.session ? (m.reasons.length ? 'END 已确认 · 存在异常' : '完整') : '未完整 / 存在异常'),
    pair('有效唯一帧 / 生成帧', `${tr.received_unique} / ${tr.generated ?? '未知'}`),
    pair('CRC 错误 / 非法帧', `${tr.crc_errors ?? 0} / ${tr.invalid_frames ?? 0}`),
    pair('重复 / 倒序 / 时间异常', `${tr.duplicates ?? 0} / ${tr.out_of_order ?? 0} / ${diagnostics.timestamp_errors ?? 0}`),
    pair('采样失败 / 陈旧样本', `${sa.sample_fail ?? '—'} / ${sa.stale_samples ?? '—'}`),
    pair('错过周期 / 队列丢弃', `${sa.missed_deadlines ?? '—'} / ${sa.queue_drops ?? '—'}`),
    pair('积分间隙 / 接收时间间隙', `${sa.integration_gaps ?? '—'} / ${sa.host_gap_count ?? 0}`),
  );
  $('staticBadge').textContent = m.static60.complete ? '完整' : '未完成 · 参考值';
  $('staticBadge').className = `mini-chip ${m.static60.complete ? 'good' : 'warn'}`;
  $('staticRows').replaceChildren(...m.static60.axes.map(a => {
    const row = document.createElement('tr');
    for (const value of [a.name, number(a.raw_mean, 3), number(a.filtered_mean, 3), number(a.raw_std, 4), number(a.filtered_std, 4), number(a.filtered_error, 2), number(a.reduction_pct, 1) + (a.reduction_pct == null ? '' : '%')]) { const td = document.createElement('td'); td.textContent = value; row.append(td); }
    row.title = `单位 ${a.unit} · 原始均值 ${number(a.raw_mean, 5)} · 处理后均值 ${number(a.filtered_mean, 5)} · 误差 ${number(a.filtered_error, 3)} ${a.error_unit ?? ''}`;
    return row;
  }));
  $('driftBadge').textContent = m.drift300.complete ? '完整' : '未完成 · 参考值';
  $('driftBadge').className = `mini-chip ${m.drift300.complete ? 'good' : 'warn'}`;
  $('driftList').replaceChildren(
    pair('有效积分时长', `${number(m.drift300.duration_s, 2)} s / 300 s`),
    ...['X', 'Y', 'Z'].map((name, i) => pair(`${name} 原始 → 处理后`, `${number(m.drift300.raw_deg[i], 3)}° → ${number(m.drift300.filtered_deg[i], 3)}°`)),
    pair('略过异常间隔', `${m.drift300.skipped_intervals ?? 0}`),
  );
  $('assessmentNote').textContent = (m.reasons?.length ? '完整性提示：' + m.reasons.join('；') : '当前会话完整。静态与漂移测试仍需满足规定时长和放置条件。') + (source === 'demo' ? ' 模拟数据仅供界面演示。' : '');
  $('recordNote').textContent = `${imported ? '导入' : '接收'} ${frames.length.toLocaleString()} 帧 · ${end ? '已保存 END 汇总' : 'END 缺失，完整性未知'} · JSON 保存原始元数据、完整采样数据和评测结果。`;
}

function canvasContext(canvas) {
  const rect = canvas.getBoundingClientRect(), scale = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, rect.width), height = Math.max(1, rect.height);
  if (canvas.width !== Math.round(width * scale) || canvas.height !== Math.round(height * scale)) { canvas.width = Math.round(width * scale); canvas.height = Math.round(height * scale); }
  const ctx = canvas.getContext('2d'); ctx.setTransform(scale, 0, 0, scale, 0, 0); ctx.clearRect(0, 0, width, height);
  return { ctx, width, height };
}

function drawChart(axis, windowFrames, currentTime) {
  const { ctx, width: w, height: h } = canvasContext($('chart' + axis));
  const box = { x: 40, y: 12, w: Math.max(10, w - 52), h: Math.max(10, h - 35) };
  let min = Infinity, max = -Infinity;
  for (const f of windowFrames) { min = Math.min(min, f.raw[axis], f.filtered[axis]); max = Math.max(max, f.raw[axis], f.filtered[axis]); }
  if (!Number.isFinite(min)) { min = axis === 2 ? 8 : -1; max = axis === 2 ? 11 : 1; }
  const pad = Math.max((max - min) * .15, axis < 3 ? .03 : .06); min -= pad; max += pad;
  ctx.font = '8px Consolas, monospace'; ctx.lineWidth = 1;
  for (let i = 0; i <= 2; i++) {
    const y = box.y + i / 2 * box.h; ctx.strokeStyle = palette.grid; ctx.beginPath(); ctx.moveTo(box.x, y); ctx.lineTo(box.x + box.w, y); ctx.stroke();
    ctx.fillStyle = palette.text; ctx.textAlign = 'right'; ctx.fillText(number(max - i / 2 * (max - min), Math.max(Math.abs(min), Math.abs(max)) >= 100 ? 0 : 2), box.x - 7, y + 3);
  }
  const span = Number($('windowSelect').value), leftTime = currentTime - span * 1e6;
  for (let i = 0; i <= 3; i++) {
    const x = box.x + i / 3 * box.w; ctx.strokeStyle = '#202c38'; ctx.beginPath(); ctx.moveTo(x, box.y); ctx.lineTo(x, box.y + box.h); ctx.stroke();
    ctx.fillStyle = '#536b82'; ctx.textAlign = i === 0 ? 'left' : i === 3 ? 'right' : 'center'; ctx.fillText(`${number(-span + span * i / 3, 0)}s`, x, h - 6);
  }
  ctx.save(); ctx.beginPath(); ctx.rect(box.x, box.y - 1, box.w, box.h + 2); ctx.clip();
  for (const field of ['raw', 'filtered']) {
    ctx.strokeStyle = palette[field]; ctx.lineWidth = field === 'raw' ? 1 : 1.5; ctx.beginPath();
    let lastT = null;
    for (const f of windowFrames) {
      const x = box.x + (f.t_us - leftTime) / (span * 1e6) * box.w;
      const y = box.y + (max - f[field][axis]) / (max - min) * box.h;
      if (lastT == null || f.t_us - lastT > 20000 || f.t_us <= lastT) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      lastT = f.t_us;
    }
    ctx.stroke();
  }
  ctx.restore();
  if (!windowFrames.length) { ctx.fillStyle = '#4f657c'; ctx.textAlign = 'center'; ctx.font = '10px "Microsoft YaHei", sans-serif'; ctx.fillText('等待采样数据', box.x + box.w / 2, box.y + box.h / 2); }
}

function drawPose(q) {
  const { ctx, width: w, height: h } = canvasContext($('poseCanvas'));
  const camera = [4.8, -6.5, 4.7], length = Math.hypot(...camera), forward = camera.map(v => -v / length);
  const rightNorm = Math.hypot(forward[0], forward[1]);
  const right = [forward[1] / rightNorm, -forward[0] / rightNorm, 0];
  const up = [right[1] * forward[2], -right[0] * forward[2], right[0] * forward[1] - right[1] * forward[0]];
  const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
  function project(p) {
    const rel = p.map((v, i) => v - camera[i]), depth = dot(rel, forward), scale = Math.min(w, h) * 1.9 / depth;
    return [w * .5 + dot(rel, right) * scale, h * .53 - dot(rel, up) * scale, depth];
  }
  function line(a, b, color, lineWidth = 1, dash = []) {
    const aa = project(a), bb = project(b); ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.setLineDash(dash); ctx.beginPath(); ctx.moveTo(aa[0], aa[1]); ctx.lineTo(bb[0], bb[1]); ctx.stroke(); ctx.setLineDash([]);
  }
  for (let i = -3; i <= 3; i++) { line([i * .6, -1.8, -.25], [i * .6, 1.8, -.25], '#1d2d3b'); line([-1.8, i * .6, -.25], [1.8, i * .6, -.25], '#1d2d3b'); }
  line([0, 0, -.25], [0, 0, 1.7], '#344b62', 1, [3, 4]);
  const zLabel = project([0, 0, 1.85]); ctx.fillStyle = '#536e89'; ctx.font = '8px Consolas,monospace'; ctx.fillText('WORLD Z ↑', zLabel[0] - 22, zLabel[1]);
  const vertices = [[-1.25, -.75, -.12], [1.25, -.75, -.12], [1.25, .75, -.12], [-1.25, .75, -.12], [-1.25, -.75, .12], [1.25, -.75, .12], [1.25, .75, .12], [-1.25, .75, .12]].map(p => rotateVector(q, p));
  const faces = [ { ids: [0, 1, 2, 3], fill: '#29463f' }, { ids: [4, 5, 6, 7], fill: '#355f52' }, { ids: [0, 1, 5, 4], fill: '#1e3935' }, { ids: [1, 2, 6, 5], fill: '#254c40' }, { ids: [2, 3, 7, 6], fill: '#234035' }, { ids: [3, 0, 4, 7], fill: '#2c5147' } ];
  faces.sort((a, b) => b.ids.reduce((s, i) => s + project(vertices[i])[2], 0) - a.ids.reduce((s, i) => s + project(vertices[i])[2], 0));
  for (const face of faces) {
    ctx.beginPath(); face.ids.forEach((id, i) => { const p = project(vertices[id]); if (i) ctx.lineTo(p[0], p[1]); else ctx.moveTo(p[0], p[1]); }); ctx.closePath(); ctx.fillStyle = face.fill; ctx.fill(); ctx.strokeStyle = '#72ae9366'; ctx.lineWidth = 1; ctx.stroke();
  }
  const chip = [[-.28, -.28, .14], [.28, -.28, .14], [.28, .28, .14], [-.28, .28, .14]].map(p => project(rotateVector(q, p)));
  ctx.beginPath(); chip.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); ctx.closePath(); ctx.fillStyle = '#14272b'; ctx.fill(); ctx.strokeStyle = '#619582'; ctx.stroke();
  for (let i = 0; i < 3; i++) {
    const v = [0, 0, 0]; v[i] = 1.75; const endpoint = rotateVector(q, v), origin = rotateVector(q, [0, 0, .18]);
    line(origin, endpoint, [palette.x, palette.y, palette.z][i], 2);
    const p = project(endpoint); ctx.fillStyle = [palette.x, palette.y, palette.z][i]; ctx.font = 'bold 11px Consolas,monospace'; ctx.fillText(['X', 'Y', 'Z'][i], p[0] + 5, p[1] + 3);
  }
  ctx.fillStyle = '#536b81'; ctx.font = '8px Consolas,monospace'; ctx.fillText('BODY XYZ', 18, h - 11);
}

function render(time) {
  requestAnimationFrame(render);
  updateReceiveHealth(time);
  if (playing && frames.length) {
    const target = playTimestamp + (time - playClock) * 1000 * Number($('speedSelect').value);
    while (cursor < frames.length && frames[cursor].t_us <= target) cursor++;
    if (cursor >= frames.length) { playing = false; updateControls(); }
  }
  if (time - lastDrawTime < 1000 / 30) return;
  lastDrawTime = time;
  if (frames.length && time - lastMetricTime > 1500 && !imported && recording) refreshMetrics();
  const current = frames[Math.max(0, cursor - 1)], currentTime = current?.t_us ?? 0;
  const windowStart = currentTime - Number($('windowSelect').value) * 1e6;
  let start = Math.max(0, cursor - Math.ceil(Number($('windowSelect').value) * 240));
  while (start < cursor && frames[start].t_us < windowStart) start++;
  // Bound the visual workload, while preserving every frame in the recording.
  const subset = frames.slice(start, cursor);
  for (let axis = 0; axis < 6; axis++) drawChart(axis, subset, currentTime);
  drawPose(current?.q ?? [1, 0, 0, 0]);
  if (current) {
    const e = quaternionToEuler(current.q);
    for (const [i, name] of ['W', 'X', 'Y', 'Z'].entries()) $('quat' + name).textContent = number(current.q[i], 5);
    for (const key of ['roll', 'pitch', 'yaw']) $(key).replaceChildren(document.createTextNode(number(e[key], 1)), Object.assign(document.createElement('small'), { textContent: '°' }));
    for (const [i, axis] of ['X', 'Y', 'Z'].entries()) {
      $('pos' + axis).textContent = number(current.position[i], 4);
      const fraction = Math.min(.5, Math.abs(current.position[i]) * .25); const bar = $('bar' + axis);
      bar.style.width = `${fraction * 100}%`; bar.style.left = `${current.position[i] < 0 ? 50 - fraction * 100 : 50}%`;
    }
    $('stationaryBadge').textContent = current.flags & 2 ? '静止' : '运动';
    $('stationaryBadge').className = `mini-chip ${current.flags & 4 ? 'warn' : current.flags & 2 ? 'good' : ''}`;
    if (current.flags & 4) $('stationaryBadge').textContent = '积分间隙';
    $('cursorInfo').textContent = `t = ${number((current.t_us - frames[0].t_us) / 1e6, 3)}s · seq = ${current.seq}`;
  }
  $('timeline').max = frames.length; $('timeline').value = cursor;
  $('playTime').textContent = `${clock((currentTime - (frames[0]?.t_us ?? 0)) / 1e6)} / ${clock(((frames.at(-1)?.t_us ?? 0) - (frames[0]?.t_us ?? 0)) / 1e6)}`;
  if (recording) { $('frameCount').textContent = frames.length.toLocaleString(); $('duration').textContent = number(((frames.at(-1)?.t_us ?? 0) - (frames[0]?.t_us ?? 0)) / 1e6, 2); }
}

function download(name, data, type) {
  const url = URL.createObjectURL(data instanceof Blob ? data : new Blob([data], { type }));
  const a = document.createElement('a'); a.href = url; a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(url), 30000);
}
function filename(extension) { return `imu_${source}_${meta.session_id ?? 'session'}_${new Date().toISOString().replace(/[:.]/g, '-')}.${extension}`; }
function exportJson() {
  refreshMetrics();
  const record = { format: 'imu-lab-recording-v1', source, captured_at: new Date().toISOString(), meta, end, diagnostics: getDiagnostics(), frames, metrics };
  download(filename('json'), JSON.stringify(record), 'application/json'); log('已导出 JSON 记录和评测');
}
async function importRecording(file) {
  if (!file) return;
  if (file.size > 200 * 1024 * 1024) { message('记录超过 200 MB，请使用更短的单次会话。', 'error'); return; }
  try {
    const data = validateRecording(JSON.parse(await file.text()));
    resetSession(data.meta ?? { session_id: data.frames[0].session_id });
    meta.session_id ??= data.frames[0].session_id;
    imported = true; source = data.source ?? 'unknown'; recording = false;
    // Keep the supplied diagnostics, and independently count anomalies still present in the imported frames.
    for (const f of data.frames) addData(f);
    for (const [key, value] of Object.entries(data.diagnostics ?? {})) if (typeof value === 'number' && Number.isFinite(value)) diagnostics[key] = Math.max(diagnostics[key] ?? 0, value);
    end = data.end ?? null; cursor = frames.length;
    $('metadataView').textContent = JSON.stringify({ ...meta, recording_source: source, captured_at: data.captured_at }, null, 2);
    refreshMetrics(); updateControls(); message(`已导入 ${file.name}。${end ? 'END 汇总存在。' : '没有 END，丢帧率未知。'}`, 'success'); log(`导入 ${file.name} · ${frames.length} 个唯一帧`);
  } catch (error) { message(`导入失败：${error.message}`, 'error'); log(error.message, 'error'); }
  $('importFile').value = '';
}

function exportPng() {
  const out = document.createElement('canvas'); out.width = 1600; out.height = 1100;
  const ctx = out.getContext('2d'); ctx.fillStyle = '#0b1017'; ctx.fillRect(0, 0, out.width, out.height);
  ctx.fillStyle = '#9ce8bd'; ctx.font = 'bold 28px "Microsoft YaHei", sans-serif'; ctx.fillText('IMU LAB · 六轴信号记录', 48, 59);
  ctx.fillStyle = '#8da1b6'; ctx.font = '16px "Microsoft YaHei", sans-serif'; ctx.fillText(`${source === 'demo' ? '模拟数据 / 非硬件测量' : imported ? '导入硬件记录' : '硬件实时采集'} · Session ${meta.session_id ?? '—'} · ${frames.length} 帧 · ${end ? 'END 已确认' : '完整性未知'}`, 48, 94);
  const names = ['X 加速度 [m/s²]', 'X 角速度 [°/s]', 'Y 加速度 [m/s²]', 'Y 角速度 [°/s]', 'Z 加速度 [m/s²]', 'Z 角速度 [°/s]'];
  [0, 3, 1, 4, 2, 5].forEach((axis, i) => {
    const x = 48 + i % 2 * 766, y = 135 + Math.floor(i / 2) * 292;
    ctx.fillStyle = '#111923'; ctx.fillRect(x, y, 738, 268); ctx.fillStyle = '#a4b5c6'; ctx.font = '17px "Microsoft YaHei",sans-serif'; ctx.fillText(names[i], x + 20, y + 33);
    ctx.drawImage($('chart' + axis), x + 10, y + 48, 718, 204);
  });
  ctx.fillStyle = '#62758a'; ctx.fillRect(48, 1045, 25, 3); ctx.fillStyle = '#8da1b6'; ctx.font = '15px "Microsoft YaHei",sans-serif'; ctx.fillText('原始：硬件 DLPF 后', 82, 1052); ctx.fillStyle = '#9ce8bd'; ctx.fillRect(325, 1045, 25, 3); ctx.fillStyle = '#8da1b6'; ctx.fillText('处理后：ESP32 校准 / 软件滤波', 360, 1052);
  ctx.fillStyle = '#566f86'; ctx.font = '13px Consolas,monospace'; ctx.fillText(new Date().toISOString(), 1250, 1052);
  out.toBlob(blob => { if (blob) download(filename('png'), blob); }, 'image/png');
}

function startDemo() {
  resetSession({ session_id: 9001, state: 'recording', calibrated: true, sample_hz: 200, baud: BAUD_RATE, g: 9.80665, accel_lsb_per_g: 8192, gyro_lsb_per_dps: 65.5, accel_norm_scale: 1, firmware_version: 'BROWSER DEMO — NOT HARDWARE', coordinate_frame: 'right-handed body-to-world, world Z up' });
  source = 'demo'; recording = true; demoSeq = 0;
  $('metadataView').textContent = JSON.stringify(meta, null, 2);
  demoTimer = setInterval(() => {
    for (let i = 0; i < 10; i++) {
      const t = demoSeq / 200, half = Math.sin(t * .34) * .24, q = [Math.cos(half), Math.sin(half) * .7, 0, Math.sin(half) * Math.sqrt(.51)];
      const gravity = rotateVector([q[0], -q[1], -q[2], -q[3]], [0, 0, 9.80665]);
      const filtered = [...gravity.map((v, axis) => v + .05 * Math.sin(t * (3 + axis))), 8 * Math.cos(t * .34), 1.3 * Math.sin(t * .5), 5 * Math.cos(t * .34)];
      const raw = filtered.map((v, axis) => v + .22 * Math.sin(t * 73 + axis) + .08 * Math.sin(t * 39 - axis));
      const raw_counts = raw.map((v, axis) => Math.round(v * (axis < 3 ? 8192 / 9.80665 : 65.5)));
      addData({ session_id: meta.session_id, seq: demoSeq, t_us: 1000000 + demoSeq * 5000, flags: 1, raw_counts, raw, filtered, q, position: [.06 * Math.sin(t), .03 * Math.cos(t * .6), .02 * Math.sin(t * .4)] }); demoSeq++;
    }
    if (frames.length >= 12000) stopDemo();
  }, 50);
  log('模拟演示开始：浏览器合成信号，非硬件数据'); message('模拟演示不会访问串口。信号用于检查绘图、回放与导出功能。'); updateControls();
}
function stopDemo() {
  clearInterval(demoTimer); demoTimer = null; recording = false; meta.state = 'idle';
  end = { session_id: meta.session_id, state: 'idle', generated: frames.length, sent: frames.length, sample_fail: 0, stale_samples: 0, missed_deadlines: 0, queue_drops: 0, integration_gaps: 0, first_t_us: frames[0]?.t_us ?? 0, last_t_us: frames.at(-1)?.t_us ?? 0, dt_count: Math.max(0, frames.length - 1), dt_mean_us: 5000, dt_std_us: 0, dt_min_us: 5000, dt_max_us: 5000, calibrated: true };
  cursor = frames.length; refreshMetrics(); updateControls(); log('模拟记录结束');
}

$('connectBtn').addEventListener('click', connect);
$('disconnectBtn').addEventListener('click', disconnect);
$('startBtn').addEventListener('click', async () => {
  if (demo) { startDemo(); return; }
  pendingStart = true; playing = false; updateControls(); await sendCommand('START'); message('已发送 START，等待设备 META 确认新会话。');
  setTimeout(() => { if (pendingStart && connected) { pendingStart = false; updateControls(); sendCommand('INFO'); message('尚未确认 START，请查看设备状态或事件日志。', 'error'); } }, 4000);
});
$('stopBtn').addEventListener('click', () => { if (demo) stopDemo(); else { stopping = true; updateReceiveHealth(performance.now()); sendCommand('STOP'); message('请求 STOP，等待队列排空与 END 汇总。可重复点击以重取 END。'); } });
$('calibrateBtn').addEventListener('click', () => { sendCommand('CALIBRATE'); message('静态校准：保持板面水平、Z 轴向上，请勿移动，等待设备返回结果。'); });
$('zeroBtn').addEventListener('click', () => { sendCommand('ZERO'); message('已请求位移与速度归零。开始新的短时运动前请保持静止。'); });
$('infoBtn').addEventListener('click', () => sendCommand('INFO'));
$('csvBtn').addEventListener('click', () => { download(filename('csv'), framesToCsv(frames), 'text/csv;charset=utf-8'); log('已导出完整逐帧 CSV'); });
$('jsonBtn').addEventListener('click', exportJson);
$('pngBtn').addEventListener('click', exportPng);
$('importBtn').addEventListener('click', () => $('importFile').click());
$('importFile').addEventListener('change', event => importRecording(event.target.files[0]));
$('refreshMetricsBtn').addEventListener('click', refreshMetrics);
$('timeline').addEventListener('input', event => { playing = false; cursor = Math.max(1, Number(event.target.value)); updateControls(); });
$('playBtn').addEventListener('click', () => {
  if (!frames.length) return;
  playing = !playing; if (playing) { if (cursor >= frames.length) cursor = 1; playClock = performance.now(); playTimestamp = frames[Math.max(0, cursor - 1)].t_us; } updateControls();
});
$('speedSelect').addEventListener('change', () => { if (playing) { playClock = performance.now(); playTimestamp = frames[Math.max(0, cursor - 1)].t_us; } });
$('liveBtn').addEventListener('click', () => { playing = false; cursor = frames.length; updateControls(); });
window.addEventListener('beforeunload', event => { if (recording && !demo) { event.preventDefault(); event.returnValue = ''; } });
updateControls(); requestAnimationFrame(render);
if (demo) startDemo();
else if (!('serial' in navigator)) message('此浏览器未开放 Web Serial。可导入 JSON 查看记录；实时采集请在桌面 Edge / Chrome 的 localhost 页面进行。');
