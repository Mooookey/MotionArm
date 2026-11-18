<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useRoute, useRouter } from 'vue-router'
import { request } from './api'
import { isPose, useDevice } from './device'
import ArmScene from './ArmScene.vue'
import TelemetryChart from './TelemetryChart.vue'
import { fallbackGeometry, poseAt, referencePose } from './geometry'
import { nextAccelerationWarning } from './displayStability'
import { coordinatedFollowLabel, hasButtonClickCounters } from './controlPresentation'
import type { Geometry, Point, Preflight, Recording, RecordingDetail, SimulationConfig } from './types'

const device = useDevice(), route = useRoute(), router = useRouter()
const settings = computed(() => route.path === '/settings')
const library = computed(() => route.path === '/recordings')
const state = computed(() => device.snapshot)
const ports = ref<Array<{ device: string; description: string }>>([])
const port = ref(''), busy = ref(false), error = ref(''), message = ref('')
const ghost = ref(false), axes = ref(false), dimensions = ref(false)
const chartType = ref<'imu' | 'joints'>('imu'), paused = ref(false), frozen = ref<Point[]>([])
const recordings = ref<Recording[]>([]), detail = ref<RecordingDetail | null>(null)
const recordName = ref(''), preview = ref(false), cursor = ref(0), preflight = ref<Preflight | null>(null)
const defaultConfig: SimulationConfig = { deadzone_deg: 5, full_scale_deg: 30, limits_deg: [[-45, 45], [0, 60], [0, 60]], max_velocity_deg_s: [20, 15, 15], max_acceleration_deg_s2: [60, 60, 60], roll_sign: 1, pitch_sign: 1, neutral_deg: [0, 0] }
const form = ref<SimulationConfig>(structuredClone(defaultConfig))
const axisLabels = ['底座', '大臂', '小臂']
const axisKeys = ['base_yaw_deg', 'shoulder_down_deg', 'elbow_down_deg'] as const
const statusLabels: Record<string, string> = { disconnected: '未连接', connecting: '连接中', calibrating: '静置校准中', valid: '数据有效', stale: '数据已过期', invalid: '数据无效' }
const reasonLabels: Record<string, string> = {
  DATA_STALE: 'IMU 数据已过期，实时目标已保持',
  INPUT_DISCONNECTED: '请连接真实腕端 IMU',
  IMU_INVALID: 'IMU 姿态无效，请静置并松开两个按钮',
  OPERATOR_STOP: '软件已停止，请重新启用虚拟控制',
  ANGLE_AMBIGUOUS: '大角度姿态有歧义，回到启用参考附近恢复',
}
const inputStatus = computed(() => !device.fresh ? 'offline' : state.value?.input.status ?? 'disconnected')
const inputLabel = computed(() => inputStatus.value === 'offline' ? '服务未连接' : statusLabels[inputStatus.value] ?? inputStatus.value)
const buttonsFresh = computed(() => device.fresh && state.value?.input.age_ms != null && state.value.input.age_ms < 250)
const relativeMapping = computed(() => state.value?.control.mapping === 'COORDINATED_RELATIVE_ANGLE')
const clickCountersReady = computed(() => hasButtonClickCounters(state.value?.buttons))
const runLabel = computed(() => device.fresh ? coordinatedFollowLabel(!!state.value?.control.enabled, !!state.value?.control.armed) : '等待有效遥测')
const history = computed(() => preview.value || !!state.value?.replay.active)
const historicalFrames = computed(() => detail.value?.metadata?.executor === 'virtual' &&
  detail.value.metadata.angle_convention === 'base_yaw_shoulder_down_elbow_relative_down_gripper_open_ratio_v1'
  ? detail.value.frames.filter((f) => isPose(f.output_target) && isPose(f.requested)) : [])
const previewResult = computed(() => poseAt(historicalFrames.value, cursor.value))
const pose = computed(() => preview.value && previewResult.value.pose ? previewResult.value.pose : state.value?.joint_state.output_target ?? referencePose)
const requested = computed(() => preview.value ? previewResult.value.frame?.requested ?? pose.value : state.value?.joint_state.requested ?? referencePose)
const geometry = computed<Geometry>(() => {
  const recorded = detail.value?.metadata?.geometry as Geometry | undefined
  if (preview.value && recorded && Object.keys(fallbackGeometry).every((key) => typeof recorded[key as keyof Geometry] === 'number' && recorded[key as keyof Geometry] > 0)) return recorded
  if (state.value?.replay.active && state.value.replay.geometry) return state.value.replay.geometry
  return device.model?.geometry ?? fallbackGeometry
})
const activeLimits = computed(() => device.model?.config.limits_deg ?? defaultConfig.limits_deg)
const chartPoints = computed<Point[]>(() => preview.value ? historicalFrames.value.flatMap((f, index, all) => {
  const point: Point = { time: f.t_ms, roll: f.imu?.roll_deg ?? null, pitch: f.imu?.pitch_deg ?? null, requested: f.requested, output: f.output_target }
  return index && f.t_ms - all[index - 1]!.t_ms > 250 ? [{ time: f.t_ms - 1, roll: null, pitch: null, requested: null, output: null }, point] : [point]
}) : paused.value ? frozen.value : device.points)
const canStop = computed(() => device.fresh && (!!state.value?.control.enabled || !!state.value?.replay.active || !!state.value?.recording.active))
const canRecord = computed(() => device.realReady && state.value?.control.enabled && !state.value.replay.active && !preview.value)
const configLocked = computed(() => !device.fresh || !!state.value?.control.enabled || !!state.value?.replay.active || !!state.value?.recording.active)
const owner = computed(() => ({ NONE: '无控制权', WEARABLE: '手腕控制', REPLAY: '历史回放' })[state.value?.control.owner ?? 'NONE'] ?? '未知')
const reason = computed(() => {
  if (!device.fresh) return '等待本地 FastAPI 遥测服务'
  if (history.value) return '历史数据驱动三维预览，与实时腕端输入独立'
  if (state.value?.control.gate_status === 'ANGLE_AMBIGUOUS') return reasonLabels.ANGLE_AMBIGUOUS!
  const raw = state.value?.control.reason
  return raw ? reasonLabels[raw] ?? raw : state.value?.control.armed ? '跟随中；短按 A 暂停，B 仅切换夹爪' : state.value?.control.enabled ? '已暂停，短按 A 继续；参考保持为本次启用时的手势' : '放稳并启用回初始，再短按 A 开始跟随'
})
const diagnosticWarning = ref(false)
watch(() => [state.value?.imu.norm_g, buttonsFresh.value] as const, ([norm, fresh]) => {
  diagnosticWarning.value = nextAccelerationWarning(diagnosticWarning.value, norm, fresh)
}, { immediate: true })
const neutral = computed(() => state.value?.imu.neutral_deg ?? device.model?.config.neutral_deg ?? [0, 0])
const blockers = computed(() => state.value?.control.blockers ?? [])
const gateLabels: Record<string, string> = { REPLAY: '历史回放中', INPUT_INVALID: '等待有效输入', DISABLED: '请启用并回初始', ACTIVE: '跟随中 · 短按 A 暂停', RUNNING: '跟随中 · 短按 A 暂停', PAUSED: '已暂停 · 短按 A 继续', READY: '已暂停 · 短按 A 开始', BUTTON_PROTOCOL_REQUIRED: '需要按钮计数固件', ANGLE_AMBIGUOUS: '大角度有歧义 · 靠近启用参考' }
const gateLabel = computed(() => !device.fresh ? '等待本地服务' : gateLabels[state.value?.control.gate_status ?? ''] ?? runLabel.value)
const canInitialize = computed(() => device.realReady && relativeMapping.value && clickCountersReady.value && !state.value?.buttons.a && !state.value?.buttons.b &&
  !state.value?.control.armed && !state.value?.replay.active && !state.value?.recording.active && !preview.value)
const fmt = (n: number | null | undefined, digits = 1) => typeof n === 'number' && Number.isFinite(n) ? n.toFixed(digits) : '—'
const date = (n: number) => new Date(n).toLocaleString('zh-CN', { month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' })
const statusName = (s: string) => ({ completed: '已完成', interrupted: '已中断', recording: '录制中', invalid: '不可回放' })[s] ?? s
async function act(work: () => Promise<void>, success = '') {
  if (busy.value) return
  busy.value = true; error.value = ''; message.value = ''
  try { await work(); message.value = success } catch (cause) { error.value = cause instanceof Error ? cause.message : '请求失败' } finally { busy.value = false }
}
async function refreshPorts() {
  ports.value = await request('/api/input/ports')
  if (!ports.value.some((p) => p.device === port.value)) port.value = ports.value.length === 1 ? ports.value[0]!.device : ''
}
async function loadRecords() { recordings.value = await request('/api/recordings') }
function connect() { void act(async () => { await request('/api/input/connect', { port: port.value }) }, '已发起连接，等待腕端身份与校准数据。') }
function disconnect() { void act(async () => { await request('/api/input/disconnect', {}) }, '腕端串口已断开。') }
function initialize() {
  preview.value = false
  void act(async () => {
    await request('/api/virtual/enable', {})
    await device.loadConfig()
  }, '已捕获当前手势参考，并回到 0° / 45° / 45°、夹爪打开。短按 A 开始，B 控制夹爪。')
}
function stop() { void act(async () => { await request('/api/virtual/stop', {}) }, '虚拟执行端已停止。') }
function reset() { void act(async () => { await request('/api/virtual/reset', {}) }, '已复位到 45° 仿真启动目标。') }
function saveConfig() {
  void act(async () => { await request('/api/virtual/config', { ...form.value }); await device.loadConfig() }, '仿真参数已保存，需重新启用控制。')
}
function toggleChart() { if (!paused.value) frozen.value = [...device.points]; paused.value = !paused.value }
function startRecording() {
  void act(async () => { await request('/api/recordings/start', { name: recordName.value.trim() || '腕端示教 ' + new Date().toLocaleTimeString('zh-CN') }); await loadRecords() }, '正在录制 IMU、按钮和虚拟关节目标。')
}
function stopRecording() { void act(async () => { await request('/api/recordings/stop', {}); await loadRecords() }, '录制已归档。') }
function selectRecord(record: Recording) {
  void act(async () => {
    if (state.value?.recording.active) throw new Error('请先结束当前录制，再进入历史预览。')
    if (state.value?.replay.active) throw new Error('请先停止正在进行的历史回放。')
    if (state.value?.control.enabled) await request('/api/virtual/stop', {})
    const value = await request<RecordingDetail>('/api/recordings/' + record.id)
    detail.value = value; cursor.value = 0; preflight.value = null
    preview.value = historicalFrames.value.length > 0
  })
}
function exitPreview() { preview.value = false; preflight.value = null }
function replayRecord() {
  void act(async () => {
    if (!detail.value) return
    const result = await request<Preflight>('/api/replays/preflight', { record_id: detail.value.id })
    preflight.value = result
    if (!result.ok || !result.preflight_id) throw new Error(result.violations.join('；') || '该录制不能连续回放。')
    await request('/api/replays/start', { preflight_id: result.preflight_id })
    preview.value = false
  }, '已开始历史回放，实时腕端不参与控制。')
}
watch(() => device.model, (model) => { if (model) form.value = structuredClone(model.config) })
watch(() => route.path, () => { if (library.value) void loadRecords().catch(() => {}) })
watch(() => state.value?.recording.active, (active, before) => { if (before && !active) void loadRecords().catch(() => {}) })
let refreshTimer: ReturnType<typeof setInterval> | undefined
onMounted(() => {
  device.start()
  void refreshPorts().catch(() => {})
  void loadRecords().catch(() => {})
  refreshTimer = setInterval(() => { if (library.value) void loadRecords().catch(() => {}) }, 4000)
})
onBeforeUnmount(() => { device.stop(); clearInterval(refreshTimer) })
</script>

<template>
  <div class="app-layout">
    <aside class="sidebar">
      <RouterLink to="/" class="brand" aria-label="MotionArm 首页"><span class="brand-glyph"><svg viewBox="0 0 32 32"><path d="M6 27h20M10 25v-7l8-7 7 5M18 11l-3-6M23 14l4-4M9 18l-3-4"/><circle cx="10" cy="18" r="3"/><circle cx="18" cy="11" r="3"/></svg></span><span>MotionArm<small>MOTION CONTROL LAB</small></span></RouterLink>
      <div class="sidebar-label">工作空间</div>
      <nav class="main-nav">
        <RouterLink to="/"><span>◈</span>实时控制台<small>01</small></RouterLink>
        <RouterLink to="/recordings"><span>◷</span>录制与回放<small>02</small></RouterLink>
        <RouterLink to="/settings"><span>⚙</span>仿真设置<small>03</small></RouterLink>
      </nav>
      <div class="sidebar-foot"><span class="connected-line"><i class="status-dot" :class="{ good: device.fresh }"></i>{{ device.fresh ? '本地服务已连接' : '等待本地服务' }}</span><small>FastAPI · Vue 3 · Three.js</small><p>真实传感输入<br>虚拟执行与估算预览</p><span class="build-label">VIRTUAL CONTROL / V2</span></div>
    </aside>
    <div class="main-layout">
      <header class="topbar">
        <div class="breadcrumb">实验工作台 <span>/</span><strong>{{ settings ? '仿真设置' : library ? '录制与回放' : '实时控制台' }}</strong></div>
        <div class="top-status"><span class="chip"><i class="status-dot" :class="{ good: device.realReady }"></i>真实 IMU 输入</span><span class="chip virtual-chip">◈ 虚拟执行端</span><button class="stop-button" :disabled="busy || !canStop" @click="stop">■ 停止</button></div>
      </header>
      <main>
        <div class="page-heading"><div><span class="eyebrow">MOTIONARM / {{ settings ? 'SIMULATION PROFILE' : library ? 'MOTION ARCHIVE' : 'LIVE TELEMETRY' }}</span><h1>{{ settings ? '仿真参数' : library ? '动作记录与回放' : '机械臂三维控制台' }}<span v-if="history" class="history-pill">历史回放</span></h1><p>{{ settings ? '独立仿真配置，不作为实体机械臂的安全标定。' : '从真实手腕姿态到虚拟关节目标，观察每一次动作。' }}</p></div><span class="session-indicator"><i class="status-dot" :class="{ good: device.realReady, warning: inputStatus === 'stale' }"></i>{{ inputLabel }}<small>{{ state?.input.port || 'USB / 未选择端口' }}</small></span></div>
        <div v-if="device.incompatible" class="notice danger" role="alert">{{ device.incompatible }} <b>本界面实时控制已锁定。</b></div>
        <div v-if="!device.fresh && !device.incompatible" class="notice warning" role="status">遥测服务{{ device.connection === 'connecting' ? '连接中' : '已断开或过期' }}，实时入口已锁定。请检查本机 FastAPI 服务。</div>
        <div v-if="error" class="notice danger" role="alert">{{ error }}<button aria-label="关闭错误提示" @click="error = ''">×</button></div>
        <div v-if="message" class="notice success" role="status">{{ message }}<button aria-label="关闭提示" @click="message = ''">×</button></div>

        <template v-if="!settings">
          <div class="workspace-grid">
            <section class="panel input-panel">
              <div class="panel-heading"><h2>腕端输入</h2><span class="tiny-tag">USB SERIAL</span></div>
              <div class="port-picker"><label for="port">设备端口</label><div><select id="port" v-model="port" :disabled="busy || !!state?.control.enabled || !!state?.replay.active"><option value="">选择串口</option><option v-for="p in ports" :key="p.device" :value="p.device">{{ p.device }} · {{ p.description }}</option></select><button class="icon-button" :disabled="busy" aria-label="刷新串口" @click="act(refreshPorts)">↻</button></div><div class="connect-row"><button class="button secondary" :disabled="busy || !device.fresh || !port || !!state?.control.enabled || !!state?.replay.active" @click="connect">连接腕端</button><button class="text-button" :disabled="busy || !device.fresh || !state?.input.port" @click="disconnect">断开</button></div></div>
              <div class="sensor-state"><i class="status-dot" :class="{ good: device.realReady, warning: inputStatus === 'calibrating' || inputStatus === 'stale' }"></i><strong>{{ inputLabel }}</strong><span>{{ state?.imu.age_ms == null ? '—' : state.imu.age_ms + ' ms' }}</span></div>
              <div v-if="inputStatus === 'calibrating'" class="calibration-progress"><progress :value="state?.imu.calibration_samples ?? 0" max="250"></progress><small>请静置并松开 A / B · {{ state?.imu.calibration_samples ?? 0 }} / 250</small></div>
              <div class="orientation-readout"><div><span>ROLL <small>启用参考 / 底座</small></span><strong>{{ fmt(state?.imu.roll_deg) }}<em>°</em></strong><div class="angle-track"><i :style="{ left: Math.max(0, Math.min(100, 50 + (state?.imu.roll_deg ?? 0) / 60 * 100)) + '%' }"></i></div></div><div><span>PITCH <small>大小臂同向协同</small></span><strong>{{ fmt(state?.imu.pitch_deg) }}<em>°</em></strong><div class="angle-track"><i :style="{ left: Math.max(0, Math.min(100, 50 + (state?.imu.pitch_deg ?? 0) / 60 * 100)) + '%' }"></i></div></div></div>
              <div class="button-readouts"><div :class="{ pressed: buttonsFresh && state?.buttons.a }"><kbd>A</kbd><span>A · GPIO25 · 短按开始/暂停<small>{{ buttonsFresh ? state?.buttons.a ? '已按下' : '已释放' : '不可用' }}</small></span><i class="status-dot" :class="{ good: buttonsFresh && state?.buttons.a }"></i></div><div :class="{ pressed: buttonsFresh && state?.buttons.b }"><kbd>B</kbd><span>B · GPIO26 · 短按夹爪开合<small>{{ buttonsFresh ? state?.buttons.b ? '已按下' : '已释放' : '不可用' }}</small></span><i class="status-dot" :class="{ good: buttonsFresh && state?.buttons.b }"></i></div></div>
              <div class="toggle-state" :class="{ running: device.fresh && state?.control.enabled && state?.control.armed }"><strong>{{ runLabel }}</strong><small>A 短按切换运行状态，松手不暂停。</small></div>
              <div class="mode-strip"><span>PITCH 同向协同</span><strong>大臂 + 小臂 · 1 : 1</strong></div>
              <div class="mapping-guide"><strong>{{ history ? '历史目标回放' : relativeMapping ? '大小臂协同跟随' : '等待协同跟随模式' }}<span>1 : 1</span></strong><p>Roll 控制底座，Pitch 同向控制大小臂。手势参考仅在启用时捕获；固定倾角会停稳。</p></div>
              <dl class="mini-diagnostics"><div><dt>WHO_AM_I</dt><dd>{{ state?.imu.who == null ? '—' : '0x' + state.imu.who.toString(16).toUpperCase() }}</dd></div><div><dt>温度</dt><dd>{{ fmt(state?.imu.temp_c) }} °C</dd></div><div><dt>加速度模长</dt><dd :class="{ amber: diagnosticWarning }">{{ fmt(state?.imu.norm_g, 3) }} g</dd></div><div><dt>I²C 错误</dt><dd>{{ state?.imu.errors ?? '—' }}</dd></div></dl>
              <p class="inline-warning diagnostic-warning-slot" :class="{ active: diagnosticWarning }" :aria-hidden="!diagnosticWarning">加速度模长偏离 1g，连接正常不代表测量精度已验证。</p><p v-if="state?.input.detail" class="input-detail">{{ state.input.detail }}</p>
            </section>

            <section class="panel model-panel">
              <ArmScene :key="JSON.stringify(geometry)" :pose="pose" :requested="requested" :geometry="geometry" :muted="!history && !device.realReady" :history="history" :ghost="ghost" :axes="axes" :dimensions="dimensions" :reference="!state?.input.frames" />
              <div class="scene-options"><label><input v-model="ghost" type="checkbox">请求轮廓</label><label><input v-model="axes" type="checkbox">关节坐标</label><label><input v-model="dimensions" type="checkbox">尺寸参考</label><span>无实体关节位置反馈</span></div>
              <div v-if="previewResult.gap && preview" class="history-gap">该时间段存在数据缺口，保持前一帧，不进行插值。</div>
              <div v-if="preview" class="preview-timeline"><div><strong>历史预览 · {{ detail?.name }}</strong><button class="text-button" @click="exitPreview">退出预览 ↗</button></div><input v-model.number="cursor" type="range" min="0" :max="detail?.duration_ms || 1" step="20" aria-label="历史录制时间轴"><span>{{ (cursor / 1000).toFixed(2) }}s / {{ ((detail?.duration_ms ?? 0) / 1000).toFixed(2) }}s</span></div>
              <div v-else-if="state?.replay.active" class="preview-timeline"><div><strong>历史回放 · 1×</strong><button class="text-button" :disabled="busy" @click="stop">停止回放</button></div><progress :value="state.replay.progress" max="1"></progress><span>{{ (state.replay.progress * 100).toFixed(0) }}%</span></div>
            </section>

            <section class="panel output-panel">
              <div class="panel-heading"><h2>关节目标</h2><span class="tiny-tag">VIRTUAL</span></div>
              <div class="target-table-head"><span>关节</span><span>请求</span><span>模拟输出</span></div>
              <div v-for="(label, i) in axisLabels" :key="label" class="joint-row"><div class="joint-label"><span class="axis-dot" :class="'joint-' + i"></span><strong>{{ label }}</strong><small>{{ ['BASE / ψ', 'SHOULDER / α', 'ELBOW / β'][i] }}</small></div><div class="joint-numbers"><span>{{ fmt(requested[axisKeys[i]!]) }}<small>°</small></span><b>{{ fmt(pose[axisKeys[i]!]) }}<small>°</small></b></div><div class="joint-progress"><i :style="{ width: Math.max(0, Math.min(100, (pose[axisKeys[i]!] - activeLimits[i]![0]) / (activeLimits[i]![1] - activeLimits[i]![0]) * 100)) + '%' }"></i></div><div class="limit-label"><span>{{ activeLimits[i]![0] }}° — {{ activeLimits[i]![1] }}°</span><b v-if="!history && state?.joint_state.limits[i]">限位</b><span v-else>仿真参数</span></div></div>
              <div class="gripper-readout"><span>夹爪开度<small>GRIPPER · 暂停时保持</small></span><strong>{{ fmt(pose.gripper_open_ratio * 100, 0) }}<em>%</em></strong></div>
              <div class="control-state"><span>控制权</span><strong :class="{ cyan: state?.control.owner !== 'NONE' }">{{ owner }}</strong><small>{{ runLabel }}</small></div>
              <div class="neutral-guide" role="status">
                <div class="gate-heading"><strong>{{ gateLabel }}</strong><span>A 短按切换</span></div>
                <p class="follow-anchor-hint">参考只在启用时捕获；A 仅切换开始/暂停。</p>
                <div class="neutral-angle-label"><span>相对启用参考</span><b>R {{ fmt(state?.imu.roll_deg) }}° / P {{ fmt(state?.imu.pitch_deg) }}°</b></div>
                <div class="neutral-angle-label"><span>传感旋转分量</span><b>R {{ fmt(state?.imu.raw_roll_deg) }}° / P {{ fmt(state?.imu.raw_pitch_deg) }}°</b></div>
                <div class="neutral-angle-label"><span>本次启用参考</span><b>R {{ fmt(neutral[0]) }}° / P {{ fmt(neutral[1]) }}°</b></div>
                <ul class="gate-blockers"><li v-for="blocker in blockers.slice(0, 2)" :key="blocker">{{ blocker }}</li></ul>
              </div>
              <button class="button primary full-width prepare-button" :disabled="busy || !canInitialize" @click="initialize">启用并回初始<span>↗</span></button>
              <p class="prepare-hint">松开 A/B 放稳 0.5 秒，启用回初始；短按 A 开始，B 控制夹爪。<span v-if="device.realReady && !clickCountersReady">需要支持按钮点击计数的腕端固件。</span></p>
              <button class="button secondary full-width reset-button" :disabled="busy || configLocked || preview" @click="reset">复位至 45° 启动目标</button>
              <p class="control-hint">{{ reason }}</p>
              <p class="follow-limit-hint" :class="{ active: !history && state?.joint_state.limits.some(Boolean) }">超出范围已截住；退回有效角区继续跟随，回到启用手势回初始。</p>
              <p v-if="state?.control.fault" class="inline-warning">{{ statusLabels[state.control.fault] ?? state.control.fault }}</p>
            </section>
          </div>

          <div class="lower-grid">
            <section class="panel telemetry-panel"><div class="panel-heading"><h2>信号与目标曲线<span class="live-pill">{{ preview ? '历史' : paused ? '已暂停查看' : 'LIVE' }}</span></h2><div class="segmented"><button :class="{ active: chartType === 'imu' }" @click="chartType = 'imu'">IMU 姿态</button><button :class="{ active: chartType === 'joints' }" @click="chartType = 'joints'">关节目标</button><button v-if="!preview" :aria-pressed="paused" @click="toggleChart">{{ paused ? '继续' : '暂停' }}</button></div></div><TelemetryChart :points="chartPoints" :type="chartType" :live="!preview" /></section>
            <section class="panel activity-panel"><div class="panel-heading"><h2>事件记录</h2><span class="tiny-tag">{{ state?.events.length ?? 0 }} EVENTS</span></div><ul class="event-list"><li v-for="(event, i) in (state?.events ?? []).slice(-5).reverse()" :key="event.time_ms + '-' + i"><time>{{ new Date(event.time_ms).toLocaleTimeString('zh-CN', { hour12: false }) }}</time><span>{{ event.message }}</span></li><li v-if="!state?.events.length" class="empty-events">连接腕端后，事件将在这里显示。</li></ul><div class="transport-stats"><span>接收 <b>{{ state?.input.frames ?? 0 }}</b></span><span>解析丢弃 <b>{{ state?.input.parse_errors ?? 0 }}</b></span><span>USB 目标 <b>25 Hz</b></span></div></section>
          </div>

          <section class="panel recording-panel"><div class="recording-overview"><span class="record-icon" :class="{ recording: state?.recording.active }">●</span><div><h2>{{ state?.recording.active ? '正在录制' : '示教录制' }}</h2><p>{{ state?.recording.active ? state.recording.frames + ' 帧 · 松开 A 继续记录保持状态' : '记录真实 IMU、按钮与虚拟关节目标' }}</p></div></div><div class="recording-actions"><input v-model="recordName" aria-label="录制名称" placeholder="为这次动作命名" maxlength="120" :disabled="!!state?.recording.active || busy"><button v-if="!state?.recording.active" class="button secondary" :disabled="busy || !canRecord" @click="startRecording">● 开始录制</button><button v-else class="button record-stop" :disabled="busy || !device.fresh" @click="stopRecording">■ 结束并保存</button><RouterLink v-if="!library" to="/recordings" class="text-button">查看动作库 →</RouterLink></div></section>

          <section v-if="library" class="panel library-panel"><div class="panel-heading"><div><h2>动作档案</h2><p>历史回放与实时控制互斥，允许在 IMU 离线时查看。</p></div><button class="button secondary" :disabled="busy" @click="act(loadRecords)">刷新列表</button></div><div v-if="!recordings.length" class="empty-library"><span>◷</span><strong>还没有动作记录</strong><p>连接腕端并启用虚拟控制后，即可录制第一段示教。</p></div><div v-else class="recording-list"><article v-for="record in recordings" :key="record.id" :class="{ selected: detail?.id === record.id }"><span class="recording-symbol">▥</span><div class="recording-name"><strong>{{ record.name }}</strong><small>{{ date(record.created_ms) }} · {{ (record.duration_ms / 1000).toFixed(1) }}s · {{ record.frame_count }} 帧 · {{ record.source }}</small><p v-if="record.note">{{ record.note }}</p></div><span class="record-status" :class="record.status">{{ statusName(record.status) }}</span><button class="button secondary" :disabled="busy || record.status === 'recording' || !device.fresh" @click="selectRecord(record)">查看</button><a class="text-button" :href="'/api/recordings/' + record.id + '/export?format=json'" download>JSON ↓</a><a class="text-button" :href="'/api/recordings/' + record.id + '/export?format=csv'" download>CSV ↓</a></article></div><div v-if="detail" class="selected-record"><div><strong>{{ detail.name }}</strong><p v-if="!historicalFrames.length" class="amber">旧数据的角度约定不明确，仅支持查看元数据和导出。</p><p v-else-if="detail.status !== 'completed'" class="amber">录制不完整，可逐帧查看；不允许连续自动回放。</p><p v-else>虚拟执行端 · 历史目标数据 · 1× 回放</p></div><button class="button primary" :disabled="busy || !device.fresh || !historicalFrames.length || detail.status !== 'completed' || !!state?.replay.active || !!state?.recording.active || !!state?.control.enabled" @click="replayRecord">▶ 开始虚拟回放</button></div></section>
        </template>

        <template v-else>
          <div class="settings-grid"><section class="panel settings-panel"><div class="panel-heading"><div><h2>运动与输入参数</h2><p>停止控制后可编辑，保存后需要重新启用。</p></div><span class="tiny-tag">仿真参数</span></div><form @submit.prevent="saveConfig"><fieldset :disabled="configLocked || busy"><div class="settings-fields"><label>角度软死区 <span><input v-model.number="form.deadzone_deg" type="number" min="0" max="20" step="0.5" required> °</span></label><div class="mapping-reference"><span>大小臂同向协同</span><strong>1 : 1</strong><small>输出目标保留限速和加速度约束。</small></div><label>Roll 方向<select v-model.number="form.roll_sign"><option :value="1">正向 +1</option><option :value="-1">反向 −1</option></select></label><label>Pitch 协同方向<select v-model.number="form.pitch_sign"><option :value="1">正向 +1</option><option :value="-1">反向 −1</option></select></label></div><div class="settings-axis" v-for="(name, i) in axisLabels" :key="name"><strong>{{ name }}</strong><label>最小角 °<input v-model.number="form.limits_deg[i]![0]" type="number" :min="i === 0 ? -90 : 0" :max="i === 0 ? 89 : 89" step="1" required></label><label>最大角 °<input v-model.number="form.limits_deg[i]![1]" type="number" min="1" max="90" step="1" required></label><label>速度 °/s<input v-model.number="form.max_velocity_deg_s[i]" type="number" min="1" max="90" step="1" required></label><label>加速度 °/s²<input v-model.number="form.max_acceleration_deg_s2[i]" type="number" min="1" max="360" step="1" required></label></div><div class="settings-footer"><button type="submit" class="button primary">保存仿真参数</button><span>{{ configLocked ? '请先停止控制和录制' : '仅影响虚拟执行端' }}</span></div></fieldset></form></section><section class="panel model-reference"><div class="panel-heading"><h2>装配与手势参考</h2></div><div class="reference-glyph">⊥</div><h3>竖直零姿态</h3><p>大臂与小臂均竖直，底座偏航 0°。模型定义与 45° 启动目标分开保存。</p><dl><div v-for="(value, key) in device.model?.geometry ?? fallbackGeometry" :key="key"><dt>{{ ({ base_width: '底板宽', table_to_shoulder_axis: '肩轴高度', shoulder_to_elbow_axis: '肩肘轴距', elbow_to_gripper_mount: '肘至安装面', gripper_mount_to_tip: '安装面至爪尖' })[key] }}</dt><dd>{{ value }} mm</dd></div></dl><div class="neutral-reference"><span>姿态本次启用参考</span><strong>Roll {{ fmt(device.model?.config.neutral_deg[0]) }}° / Pitch {{ fmt(device.model?.config.neutral_deg[1]) }}°</strong><small>手势参考只在“启用并回初始”时捕获；A 仅切换开始/暂停，不重新设参考。</small></div></section></div><section class="panel profile-info"><span class="eyebrow">PROFILE IDENTITY</span><p><strong>仿真配置</strong><code>{{ device.model?.config_id ?? '等待配置' }}</code></p><p><strong>几何基线</strong><code>{{ device.model?.baseline_id ?? '等待配置' }}</code></p><p><strong>真实轴角</strong><span>未配备外部位置反馈 · measured = null</span></p></section>
        </template>
        <footer class="page-footer"><span>MotionArm Lab <i>/</i> 虚拟关节目标／估算预览</span><span>仿真参数不代表实机安全标定</span></footer>
      </main>
    </div>
  </div>
</template>
