<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useRoute, useRouter } from 'vue-router'
import { request } from './api'
import { isPose, useDevice } from './device'
import ArmScene from './ArmScene.vue'
import TelemetryChart from './TelemetryChart.vue'
import { fallbackGeometry, poseAt, referencePose } from './geometry'
import { nextAccelerationWarning } from './displayStability'
import { buttonTelemetryFresh, coordinatedFollowLabel } from './controlPresentation'
import type { Geometry, Point, Preflight, Recording, RecordingDetail, SimulationConfig } from './types'

const device = useDevice(), route = useRoute(), router = useRouter()
const settings = computed(() => route.path === '/settings')
const library = computed(() => route.path === '/recordings')
const state = computed(() => device.snapshot)
const executorKind = ref<'virtual' | 'robot_no_power'>('virtual')
const robot = computed(() => state.value?.executor.kind === 'robot_no_power')
const executorLabel = computed(() => state.value?.executor.label ?? '虚拟执行端')
const hasTargets = computed(() => history.value || !!state.value?.joint_state.output_target)
const displayRequested = computed(() => history.value ? requested.value : state.value?.joint_state.requested)
const displayOutput = computed(() => history.value ? pose.value : state.value?.joint_state.output_target)
const gripper = computed(() => state.value?.gripper)
const gripperReasons: Record<string, string> = { TIME_LIMIT: '达到 5 秒期限', TIMEOUT: '达到 5 秒期限', CONTROL_DISABLED: '控制已禁用', INITIALIZE: '重新初始化', RESET: '软件复位', STOP: '软件停止', INPUT_INVALID: '输入无效', INPUT_STALE: '输入过期', LINK_LOST: '无线断线', MANAGER_TIMEOUT: '管理会话超时', OWNER_CHANGED: '控制权变化', RESTART: '设备重启' }
const ports = ref<Array<{ device: string; description: string }>>([])
const port = ref(''), busy = ref(false), error = ref(''), message = ref('')
const ghost = ref(false), axes = ref(false), dimensions = ref(false)
const chartType = ref<'imu' | 'joints'>('imu'), paused = ref(false), frozen = ref<Point[]>([])
const recordings = ref<Recording[]>([]), detail = ref<RecordingDetail | null>(null)
const recordName = ref(''), preview = ref(false), cursor = ref(0), preflight = ref<Preflight | null>(null)
const defaultConfig: SimulationConfig = { deadzone_deg: 5, limits_deg: [[-45, 45], [0, 60], [0, 60]], max_velocity_deg_s: [20, 15, 15], max_acceleration_deg_s2: [60, 60, 60] }
const form = ref<SimulationConfig>(structuredClone(defaultConfig))
const axisLabels = ['底座', '大臂', '小臂']
const axisKeys = ['base_yaw_deg', 'shoulder_down_deg', 'elbow_down_deg'] as const
const statusLabels: Record<string, string> = { disconnected: '未连接', connecting: '连接中', calibrating: '静置校准中', valid: '数据有效', stale: '数据已过期', invalid: '数据无效', incompatible: '固件协议不兼容' }
const reasonLabels: Record<string, string> = {
  DATA_STALE: 'IMU 数据已过期，实时目标已保持',
  INPUT_DISCONNECTED: '请连接真实腕端 IMU',
  IMU_INVALID: 'IMU 姿态无效，请静置并松开两个按钮',
  OPERATOR_STOP: '软件已停止，请重新启用虚拟控制',
  ANGLE_AMBIGUOUS: '大角度姿态有歧义，回到启用参考附近恢复',
}
const inputStatus = computed(() => !device.fresh ? 'offline' : state.value?.input.status ?? 'disconnected')
const inputLabel = computed(() => inputStatus.value === 'offline' ? '服务未连接' : statusLabels[inputStatus.value] ?? inputStatus.value)
const buttonsFresh = computed(() => buttonTelemetryFresh(device.fresh, state.value?.input.kind ?? '',
  inputStatus.value, state.value?.input.age_ms, state.value?.input.wireless_fresh))
const relativeMapping = computed(() => state.value?.control.mapping === 'COORDINATED_RELATIVE_ANGLE')
const runLabel = computed(() => device.fresh ? coordinatedFollowLabel(!!state.value?.control.enabled, !!state.value?.control.armed) : '等待有效遥测')
const history = computed(() => preview.value || !!state.value?.replay.active)
const historicalFrames = computed(() => ['virtual', 'robot_no_power'].includes(String(detail.value?.metadata?.executor)) &&
  detail.value?.metadata?.angle_convention === 'base_yaw_shoulder_down_elbow_relative_down_gripper_open_ratio_v1'
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
const chartLabel = computed(() => preview.value ? '历史记录' : paused.value ? '已暂停查看' : chartType.value === 'imu'
  ? device.realReady ? '实时腕端' : '无有效实时输入' : state.value?.replay.active ? '历史目标回放' : device.fresh ? '实时目标' : '历史接收值')
const canStop = computed(() => device.fresh && (!!state.value?.control.enabled || !!state.value?.replay.active || !!state.value?.recording.active))
const canRecord = computed(() => device.realReady && state.value?.control.enabled && !state.value.replay.active && !preview.value)
const configLocked = computed(() => robot.value || !device.fresh || !!state.value?.control.enabled || !!state.value?.replay.active || !!state.value?.recording.active)
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
const referenceQuaternion = computed(() => state.value?.control.follow.reference_quaternion)
const readoutLabel = computed(() => !state.value?.input.frames ? '尚无输入' : !device.realReady ? '历史值 · 当前不可用于控制' : !referenceQuaternion.value ? '未初始化 · 相对角待建立' : '相对本次启用参考')
const blockers = computed(() => state.value?.control.blockers ?? [])
const gateLabels: Record<string, string> = { REPLAY: '历史回放中', INPUT_INVALID: '等待有效输入', DISABLED: '请启用并回初始', ACTIVE: '跟随中 · 短按 A 暂停', READY: '已暂停 · 短按 A 开始', PROTOCOL_INCOMPATIBLE: '请升级腕端四元数固件', ANGLE_AMBIGUOUS: '大角度有歧义 · 靠近启用参考' }
const gateLabel = computed(() => !device.fresh ? '等待本地服务' : gateLabels[state.value?.control.gate_status ?? ''] ?? runLabel.value)
const canInitialize = computed(() => device.fresh && !!state.value?.control.can_enable && !history.value)
const fmt = (n: number | null | undefined, digits = 1) => typeof n === 'number' && Number.isFinite(n) ? n.toFixed(digits) : '—'
const vectorLabel = (values: number[] | null | undefined, digits = 3) => values ? values.map((n) => fmt(n, digits)).join(' / ') : '—'
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
function connectSelected() { if (!robot.value) return connect(); void act(async () => { await request('/api/executor/connect', { port: port.value }) }, '已发起执行端连接，等待身份和完整无线遥测。') }
function disconnect() { device.releaseManagement(); void act(async () => { await request(robot.value ? '/api/executor/disconnect' : '/api/input/disconnect', {}) }, '串口已断开。') }
function selectExecutor() {
  device.releaseManagement()
  void act(async () => { await request('/api/executor', { kind: executorKind.value }); preview.value = false; await device.loadConfig() }, '执行端已切换，需显式初始化。')
}
function initialize() {
  preview.value = false
  void act(async () => {
    const response = await request<{ manager_token: string }>('/api/executor/initialize', {})
    device.grantManagement(response.manager_token)
    await device.loadConfig()
  }, '已请求捕获稳定参考并设置 0° / 45° / 45°、夹爪打开目标；以执行端遥测为准。短按 A 开始。')
}
function stop() { device.releaseManagement(); void act(async () => { await request('/api/control/stop', {}) }, '已请求停止跟随并打开夹爪。') }
function reset() { void act(async () => { await request('/api/virtual/reset', {}) }, '已复位到 45° 仿真启动目标。') }
function saveConfig() {
  void act(async () => { await request('/api/virtual/config', { ...form.value }); await device.loadConfig() }, '仿真参数已保存，需重新启用控制。')
}
function toggleChart() { if (!paused.value) frozen.value = [...device.points]; paused.value = !paused.value }
function startRecording() {
  void act(async () => { await request('/api/recordings/start', { name: recordName.value.trim() || '腕端示教 ' + new Date().toLocaleTimeString('zh-CN') }); await loadRecords() }, '正在录制 IMU、按钮、执行端软件目标和保护状态。')
}
function stopRecording() { void act(async () => { await request('/api/recordings/stop', {}); await loadRecords() }, '录制已归档。') }
function selectRecord(record: Recording) {
  void act(async () => {
    if (state.value?.recording.active) throw new Error('请先结束当前录制，再进入历史预览。')
    if (state.value?.replay.active) throw new Error('请先停止正在进行的历史回放。')
    if (state.value?.control.enabled) { device.releaseManagement(); await request('/api/control/stop', {}) }
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
watch(() => state.value?.executor.kind, (kind) => { if (kind) executorKind.value = kind })
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
      <div class="sidebar-foot"><span class="connected-line"><i class="status-dot" :class="{ good: device.fresh }"></i>{{ device.fresh ? '本地服务已连接' : '等待本地服务' }}</span><small>FastAPI · Vue 3 · Three.js</small><p>真实传感输入<br>执行端目标与估算预览</p><span class="build-label">EXECUTOR CONSOLE / V4</span></div>
    </aside>
    <div class="main-layout">
      <header class="topbar">
        <div class="breadcrumb">实验工作台 <span>/</span><strong>{{ settings ? '仿真设置' : library ? '录制与回放' : '实时控制台' }}</strong></div>
        <div class="top-status"><span class="chip"><i class="status-dot" :class="{ good: device.realReady }"></i>真实 IMU 输入</span><span class="chip virtual-chip">◈ {{ executorLabel }}</span><button class="stop-button" :disabled="busy || !canStop" @click="stop">■ 停止</button></div>
      </header>
      <main>
        <div class="page-heading"><div><span class="eyebrow">MOTIONARM / {{ settings ? 'SIMULATION PROFILE' : library ? 'MOTION ARCHIVE' : 'LIVE TELEMETRY' }}</span><h1>{{ settings ? '仿真参数' : library ? '动作记录与回放' : '机械臂三维控制台' }}<span v-if="history" class="history-pill">历史回放</span></h1><p>{{ settings ? '独立仿真配置，不作为实体机械臂的安全标定。' : '真实手腕输入与执行端软件目标；无实体位置反馈。' }}</p></div><span class="session-indicator"><i class="status-dot" :class="{ good: device.realReady, warning: inputStatus === 'stale' }"></i>{{ inputLabel }}<small>{{ state?.input.port || 'USB / 未选择端口' }}</small></span></div>
        <div v-if="device.incompatible" class="notice danger" role="alert">{{ device.incompatible }} <b>本界面实时控制已锁定。</b></div>
        <div v-if="!device.fresh && !device.incompatible" class="notice warning" role="status">遥测服务{{ device.connection === 'connecting' ? '连接中' : '已断开或过期' }}，实时入口已锁定。请检查本机 FastAPI 服务。</div>
        <div v-if="device.model?.warning" class="notice warning" role="status">{{ device.model.warning }}</div>
        <div v-if="error" class="notice danger" role="alert">{{ error }}<button aria-label="关闭错误提示" @click="error = ''">×</button></div>
        <div v-if="message" class="notice success" role="status">{{ message }}<button aria-label="关闭提示" @click="message = ''">×</button></div>

        <section v-if="!settings" class="panel executor-panel">
          <label>执行端 <select v-model="executorKind" :disabled="busy || !device.fresh || !!state?.control.enabled || !!state?.recording.active || !!state?.replay.active" @change="selectExecutor"><option value="virtual">虚拟执行端</option><option value="robot_no_power">实体执行端 · 无动力联调</option><option disabled>有动力执行 · 本轮不可用</option></select></label>
          <span>{{ executorLabel }} · {{ state?.executor.identity || '身份待确认' }}</span>
          <strong v-if="robot">硬件输出{{ state?.executor.output_locked ? '已锁定' : '状态异常' }} · 舵机动力电源保持关闭</strong>
          <span v-if="robot">ESP-NOW {{ state?.input.wireless_fresh ? '数据新鲜' : '未确认/过期' }} · PCA9685 {{ state?.health?.pca_readable ? '逻辑可读' : '未确认' }}</span>
          <span v-if="robot">无线接收 {{ state?.health?.wireless?.received ?? '—' }} · 拒绝 {{ state?.health?.wireless?.rejected ?? '—' }} · 重复 {{ state?.health?.wireless?.duplicates ?? '—' }} · 倒序 {{ state?.health?.wireless?.out_of_order ?? '—' }}</span>
          <span>输入 boot {{ state?.input.boot_id ?? '—' }} · {{ robot ? '输入年龄（执行端上报）' : '输入年龄' }} {{ state?.input.age_ms == null ? '未知' : state.input.age_ms + ' ms' }}</span>
          <span v-if="robot">执行端遥测缓存 {{ state?.executor.age_ms == null ? '未知' : state.executor.age_ms + ' ms' }} · 无线传输年龄未知</span>
          <span v-if="robot">管理会话 {{ state?.health?.manager_active ? '有效' : '未取得或已停止' }}</span>
          <small v-if="robot">仅检查逻辑请求与目标，不能据此确认舵机方向、行程或实体到位。</small>
        </section>
        <template v-if="!settings">
          <div class="workspace-grid">
            <section class="panel input-panel">
              <div class="panel-heading"><h2>实时腕端输入</h2><span class="tiny-tag">{{ robot ? 'ESP-NOW → ROBOT USB' : 'WEARABLE USB' }}</span></div>
              <div class="port-picker"><label for="port">设备端口</label><div><select id="port" v-model="port" :disabled="busy || !!state?.control.enabled || !!state?.replay.active"><option value="">选择串口</option><option v-for="p in ports" :key="p.device" :value="p.device">{{ p.device }} · {{ p.description }}</option></select><button class="icon-button" :disabled="busy" aria-label="刷新串口" @click="act(refreshPorts)">↻</button></div><div class="connect-row"><button class="button secondary" :disabled="busy || !device.fresh || !port || !!state?.control.enabled || !!state?.replay.active" @click="connectSelected">{{ robot ? '连接执行端' : '连接腕端' }}</button><button class="text-button" :disabled="busy || !device.fresh || !state?.input.port" @click="disconnect">断开</button></div></div>
              <div class="sensor-state"><i class="status-dot" :class="{ good: device.realReady, warning: inputStatus === 'calibrating' || inputStatus === 'stale' }"></i><strong>{{ inputLabel }}</strong><span>{{ state?.imu.age_ms == null ? '—' : state.imu.age_ms + ' ms' }}</span></div>
              <div v-if="inputStatus === 'calibrating'" class="calibration-progress"><progress :value="state?.imu.calibration_samples ?? 0" max="250"></progress><small>请静置并松开 A / B · {{ state?.imu.calibration_samples ?? 0 }} / 250</small></div>
              <p class="readout-status" :class="{ amber: !device.realReady && !!state?.input.frames }">{{ readoutLabel }}</p>
              <div class="orientation-readout" :class="{ historic: !device.realReady }"><div><span>ROLL <small>侧倾 / 底座</small></span><strong>{{ fmt(state?.imu.roll_deg) }}<em>°</em></strong><div class="angle-track"><i :style="{ left: Math.max(0, Math.min(100, 50 + (state?.imu.roll_deg ?? 0) / 60 * 100)) + '%' }"></i></div></div><div><span>PITCH <small>抬手 / 双臂</small></span><strong>{{ fmt(state?.imu.pitch_deg) }}<em>°</em></strong><div class="angle-track"><i :style="{ left: Math.max(0, Math.min(100, 50 + (state?.imu.pitch_deg ?? 0) / 60 * 100)) + '%' }"></i></div></div></div>
              <div class="button-readouts"><div :class="{ pressed: buttonsFresh && state?.buttons.a }"><kbd>A</kbd><span>A · GPIO25 · 短按开始/暂停<small>{{ buttonsFresh ? state?.buttons.a ? '已按下' : '已释放' : '不可用' }}</small></span><i class="status-dot" :class="{ good: buttonsFresh && state?.buttons.a }"></i></div><div :class="{ pressed: buttonsFresh && state?.buttons.b }"><kbd>B</kbd><span>B · GPIO26 · 短按夹爪开合<small>{{ buttonsFresh ? state?.buttons.b ? '已按下' : '已释放' : '不可用' }}</small></span><i class="status-dot" :class="{ good: buttonsFresh && state?.buttons.b }"></i></div></div>
              <div class="toggle-state" :class="{ running: device.fresh && state?.control.enabled && state?.control.armed }"><strong>{{ runLabel }}</strong><small>A 短按切换运行状态，松手不暂停。</small></div>
              <div class="mode-strip"><span>PITCH 同向协同</span><strong>大臂 + 小臂 · 1 : 1</strong></div>
              <div class="mapping-guide"><strong>{{ history ? '历史目标回放' : relativeMapping ? '大小臂协同跟随' : '等待协同跟随模式' }}<span>1 : 1</span></strong><p>{{ history ? '按录制关节目标回放；下方腕端诊断仍属于独立的当前输入。' : '固定手背安装：芯片正面向下，X→右，Y→肘，Z→下。右倾底座顺时针，抬手双臂抬起。' }}</p></div>
              <dl class="mini-diagnostics"><div><dt>WHO_AM_I</dt><dd>{{ state?.imu.who == null ? '—' : '0x' + state.imu.who.toString(16).toUpperCase() }}</dd></div><div><dt>温度</dt><dd>{{ fmt(state?.imu.temp_c) }} °C</dd></div><div><dt>加速度模长</dt><dd :class="{ amber: diagnosticWarning }">{{ fmt(state?.imu.norm_g, 3) }} g</dd></div><div><dt>I²C 错误</dt><dd>{{ state?.imu.errors ?? '—' }}</dd></div></dl>
              <details class="attitude-diagnostics">
                <summary>完整姿态与三轴诊断 <span>{{ device.realReady ? '当前输入' : '历史接收值' }}</span></summary>
                <p>四元数顺序 w / x / y / z；角速度、加速度按传感器 X / Y / Z 原轴。旋转向量按手腕指尖 / 右侧 / 下方坐标。</p>
                <dl>
                  <div><dt>当前四元数 · 传感器→世界</dt><dd>{{ vectorLabel(state?.imu.quaternion, 6) }}</dd></div>
                  <div><dt>参考四元数 · 当前会话</dt><dd>{{ vectorLabel(referenceQuaternion, 6) }}</dd></div>
                  <div><dt>相对旋转向量 · °</dt><dd>{{ vectorLabel(state?.control.follow.delta_rotation_vector_deg, 2) }}</dd></div>
                  <div><dt>角速度 · °/s · 已扣零偏</dt><dd>{{ vectorLabel(state?.imu.gyro_dps, 2) }}</dd></div>
                  <div><dt>加速度 · g · 原始量</dt><dd>{{ vectorLabel(state?.imu.accel_g, 3) }}</dd></div>
                  <div><dt>陀螺量程 · °/s</dt><dd>±{{ fmt(state?.imu.gyro_range_dps, 0) }}</dd></div>
                  <div><dt>拒绝原因 / 样本数</dt><dd>{{ state?.imu.reject_reason ?? '—' }} / {{ state?.imu.rejected_samples ?? '—' }}</dd></div>
                </dl>
                <p>Z 旋转仅作诊断，不直接驱动底座。MPU6050 无绝对航向，航向仍会漂移；不测量手部平移。</p>
              </details>
              <p class="inline-warning diagnostic-warning-slot" :class="{ active: diagnosticWarning }" :aria-hidden="!diagnosticWarning">加速度模长偏离 1g，连接正常不代表测量精度已验证。</p><p v-if="state?.input.detail" class="input-detail">{{ state.input.detail }}</p>
            </section>

            <section class="panel model-panel">
              <ArmScene :key="JSON.stringify(geometry)" :pose="pose" :requested="requested" :executor-label="executorLabel" :target-available="hasTargets" :physical="robot" :geometry="geometry" :muted="!history && !device.realReady" :history="history" :ghost="ghost" :axes="axes" :dimensions="dimensions" :reference="!state?.input.frames" />
              <div class="scene-options"><label><input v-model="ghost" type="checkbox">请求轮廓</label><label><input v-model="axes" type="checkbox">关节坐标</label><label><input v-model="dimensions" type="checkbox">尺寸参考</label><span>无实体关节位置反馈</span></div>
              <div v-if="previewResult.gap && preview" class="history-gap">该时间段存在数据缺口，保持前一帧，不进行插值。</div>
              <div v-if="preview" class="preview-timeline"><div><strong>历史预览 · {{ detail?.name }}</strong><button class="text-button" @click="exitPreview">退出预览 ↗</button></div><input v-model.number="cursor" type="range" min="0" :max="detail?.duration_ms || 1" step="20" aria-label="历史录制时间轴"><span>{{ (cursor / 1000).toFixed(2) }}s / {{ ((detail?.duration_ms ?? 0) / 1000).toFixed(2) }}s</span></div>
              <div v-else-if="state?.replay.active" class="preview-timeline"><div><strong>历史回放 · 1×</strong><button class="text-button" :disabled="busy" @click="stop">停止回放</button></div><progress :value="state.replay.progress" max="1"></progress><span>{{ (state.replay.progress * 100).toFixed(0) }}%</span></div>
            </section>

            <section class="panel output-panel">
              <div class="panel-heading"><h2>关节目标</h2><span class="tiny-tag">{{ robot ? 'PWM LOCKED' : 'VIRTUAL' }}</span></div>
              <div class="target-table-head"><span>关节</span><span>请求</span><span>软件输出目标</span></div>
              <div v-for="(label, i) in axisLabels" :key="label" class="joint-row"><div class="joint-label"><span class="axis-dot" :class="'joint-' + i"></span><strong>{{ label }}</strong><small>{{ ['BASE / ψ', 'SHOULDER / α', 'ELBOW / β'][i] }}</small></div><div class="joint-numbers"><span>{{ fmt(displayRequested?.[axisKeys[i]!]) }}<small>°</small></span><b>{{ fmt(displayOutput?.[axisKeys[i]!]) }}<small>°</small></b></div><div class="joint-progress"><i :style="{ width: Math.max(0, Math.min(100, (pose[axisKeys[i]!] - activeLimits[i]![0]) / (activeLimits[i]![1] - activeLimits[i]![0]) * 100)) + '%' }"></i></div><div class="limit-label"><span>{{ activeLimits[i]![0] }}° — {{ activeLimits[i]![1] }}°</span><b v-if="!history && state?.joint_state.limits[i]">限位</b><span v-else>{{ robot ? '逻辑限位' : '仿真参数' }}</span></div></div>
              <div class="gripper-protection" role="status"><div class="gripper-readout"><span>夹爪软件开度<small>请求 / 输出目标 · 无实测反馈</small></span><strong>{{ fmt(displayRequested == null ? null : displayRequested.gripper_open_ratio * 100, 0) }} / {{ fmt(displayOutput == null ? null : displayOutput.gripper_open_ratio * 100, 0) }}<em>%</em></strong></div><dl v-if="gripper && !history"><div><dt>保护状态</dt><dd>{{ gripper.state }} · 周期 {{ gripper.cycle_id }}</dd></div><div><dt>已用 / 剩余</dt><dd>{{ fmt(gripper.elapsed_ms / 1000, 2) }} / {{ fmt(gripper.remaining_ms == null ? null : gripper.remaining_ms / 1000, 2) }} s</dd></div><div><dt>自动打开原因</dt><dd>{{ gripper.auto_open_reason ? gripperReasons[gripper.auto_open_reason] ?? gripper.auto_open_reason : '—' }}</dd></div><div v-if="gripper.reclose_blocked"><dt>再次闭合</dt><dd>等待软件打开目标与新的 B 点击</dd></div></dl><p>每次有效闭合最多 5 秒，包含闭合推进；暂停跟随不暂停保护。</p></div>
              <div class="control-state"><span>控制权</span><strong :class="{ cyan: state?.control.owner !== 'NONE' }">{{ owner }}</strong><small>{{ runLabel }}</small></div>
              <div class="neutral-guide" role="status">
                <div class="gate-heading"><strong>{{ gateLabel }}</strong><span>A 短按切换</span></div>
                <p class="follow-anchor-hint">{{ history ? '此处为实时腕端参考；历史模型只使用录制目标。' : '参考只在启用时捕获；A 仅切换开始/暂停。' }}</p>
                <div class="neutral-angle-label"><span>相对启用参考</span><b>R {{ fmt(state?.imu.roll_deg) }}° / P {{ fmt(state?.imu.pitch_deg) }}°</b></div>
                <div class="neutral-angle-label"><span>会话参考</span><b>{{ referenceQuaternion ? '四元数已捕获' : '尚未初始化' }}</b></div>
                <div class="neutral-angle-label"><span>关节映射符号</span><b>底座 +1 / 双臂 −1</b></div>
                <ul class="gate-blockers"><li v-for="blocker in blockers.slice(0, 2)" :key="blocker">{{ blocker }}</li></ul>
              </div>
              <button class="button primary full-width prepare-button" :disabled="busy || !canInitialize" @click="initialize">启用并回初始<span>↗</span></button>
              <p class="prepare-hint">松开 A/B 放稳 0.5 秒，启用回初始；短按 A 开始，B 控制夹爪。</p>
              <button class="button secondary full-width reset-button" :disabled="busy || configLocked || preview || robot" @click="reset">复位至 45° 启动目标</button>
              <p class="control-hint">{{ reason }}</p>
              <p class="follow-limit-hint" :class="{ active: !history && state?.joint_state.limits.some(Boolean) }">超出范围已截住；退回有效角区继续跟随，回到启用手势回初始。</p>
              <p v-if="state?.control.fault" class="inline-warning">{{ statusLabels[state.control.fault] ?? state.control.fault }}</p>
            </section>
          </div>

          <div class="lower-grid">
            <section class="panel telemetry-panel"><div class="panel-heading"><h2>信号与目标曲线<span class="live-pill" :class="{ inactive: !preview && (paused || (chartType === 'imu' ? !device.realReady : !device.fresh)) }">{{ chartLabel }}</span></h2><div class="segmented"><button :class="{ active: chartType === 'imu' }" @click="chartType = 'imu'">IMU 姿态</button><button :class="{ active: chartType === 'joints' }" @click="chartType = 'joints'">关节目标</button><button v-if="!preview" :aria-pressed="paused" @click="toggleChart">{{ paused ? '继续' : '暂停' }}</button></div></div><TelemetryChart :points="chartPoints" :type="chartType" :live="!preview" /></section>
            <section class="panel activity-panel"><div class="panel-heading"><h2>事件记录</h2><span class="tiny-tag">{{ state?.events.length ?? 0 }} EVENTS</span></div><ul class="event-list"><li v-for="(event, i) in (state?.events ?? []).slice(-5).reverse()" :key="event.time_ms + '-' + i"><time>{{ new Date(event.time_ms).toLocaleTimeString('zh-CN', { hour12: false }) }}</time><span>{{ event.message }}</span></li><li v-if="!state?.events.length" class="empty-events">连接腕端后，事件将在这里显示。</li></ul><div class="transport-stats"><span>接收 <b>{{ state?.input.frames ?? 0 }}</b></span><span>解析丢弃 <b>{{ state?.input.parse_errors ?? 0 }}</b></span><span>遥测 <b>{{ robot ? '5 Hz' : '25 Hz' }}</b></span></div></section>
          </div>

          <section class="panel recording-panel"><div class="recording-overview"><span class="record-icon" :class="{ recording: state?.recording.active }">●</span><div><h2>{{ state?.recording.active ? '正在录制' : '示教录制' }}</h2><p>{{ state?.recording.active ? state.recording.frames + ' 帧 · 松开 A 继续记录保持状态' : '记录真实 IMU、按钮、软件目标和夹爪保护事件' }}</p></div></div><div class="recording-actions"><input v-model="recordName" aria-label="录制名称" placeholder="为这次动作命名" maxlength="120" :disabled="!!state?.recording.active || busy"><button v-if="!state?.recording.active" class="button secondary" :disabled="busy || !canRecord" @click="startRecording">● 开始录制</button><button v-else class="button record-stop" :disabled="busy || !device.fresh" @click="stopRecording">■ 结束并保存</button><RouterLink v-if="!library" to="/recordings" class="text-button">查看动作库 →</RouterLink></div></section>

          <section v-if="library" class="panel library-panel"><div class="panel-heading"><div><h2>动作档案</h2><p>历史回放与实时控制互斥，允许在 IMU 离线时查看。</p></div><button class="button secondary" :disabled="busy" @click="act(loadRecords)">刷新列表</button></div><div v-if="!recordings.length" class="empty-library"><span>◷</span><strong>还没有动作记录</strong><p>连接腕端并启用虚拟控制后，即可录制第一段示教。</p></div><div v-else class="recording-list"><article v-for="record in recordings" :key="record.id" :class="{ selected: detail?.id === record.id }"><span class="recording-symbol">▥</span><div class="recording-name"><strong>{{ record.name }}</strong><small>{{ date(record.created_ms) }} · {{ (record.duration_ms / 1000).toFixed(1) }}s · {{ record.frame_count }} 帧 · {{ record.source }}</small><p v-if="record.note">{{ record.note }}</p></div><span class="record-status" :class="record.status">{{ statusName(record.status) }}</span><button class="button secondary" :disabled="busy || record.status === 'recording' || !device.fresh" @click="selectRecord(record)">查看</button><a class="text-button" :href="'/api/recordings/' + record.id + '/export?format=json'" download>JSON ↓</a><a class="text-button" :href="'/api/recordings/' + record.id + '/export?format=csv'" download>CSV ↓</a></article></div><div v-if="detail" class="selected-record"><div><strong>{{ detail.name }}</strong><p v-if="!historicalFrames.length" class="amber">旧数据的角度约定不明确，仅支持查看元数据和导出。</p><p v-else-if="detail.status !== 'completed'" class="amber">录制不完整，可逐帧查看；不允许连续自动回放。</p><p v-else>虚拟执行端 · 历史目标数据 · 1× 回放</p></div><button class="button primary" :disabled="busy || !device.fresh || !historicalFrames.length || detail.status !== 'completed' || !!state?.replay.active || !!state?.recording.active || !!state?.control.enabled" @click="replayRecord">▶ 开始虚拟回放</button></div></section>
        </template>

        <template v-else>
          <div class="settings-grid"><section class="panel settings-panel"><div class="panel-heading"><div><h2>运动与输入参数</h2><p>停止控制后可编辑，保存后需要重新启用。</p></div><span class="tiny-tag">仿真参数</span></div><form @submit.prevent="saveConfig"><fieldset :disabled="configLocked || busy"><div class="settings-fields"><label>角度软死区 <span><input v-model.number="form.deadzone_deg" type="number" min="0.5" max="20" step="0.5" required> °</span></label><div class="mapping-reference"><span>大小臂同向协同</span><strong>1 : 1</strong><small>输出目标保留限速和加速度约束。</small></div><div class="fixed-mounting"><span>固定手背安装</span><strong>X 向右 · Y 指向手肘 · Z 向下</strong><small>芯片正面向下。控制坐标：Roll = −Y，Pitch = +X；关节映射符号：底座 +1、双臂 −1。</small></div></div><div class="settings-axis" v-for="(name, i) in axisLabels" :key="name"><strong>{{ name }}</strong><label>最小角 °<input v-model.number="form.limits_deg[i]![0]" type="number" :min="i === 0 ? -90 : 0" :max="i === 0 ? 89 : 89" step="1" required></label><label>最大角 °<input v-model.number="form.limits_deg[i]![1]" type="number" min="1" max="90" step="1" required></label><label>速度 °/s<input v-model.number="form.max_velocity_deg_s[i]" type="number" min="1" max="90" step="1" required></label><label>加速度 °/s²<input v-model.number="form.max_acceleration_deg_s2[i]" type="number" min="1" max="360" step="1" required></label></div><div class="settings-footer"><button type="submit" class="button primary">保存仿真参数</button><span>{{ configLocked ? '请先停止控制和录制' : '仅影响虚拟执行端' }}</span></div></fieldset></form></section><section class="panel model-reference"><div class="panel-heading"><h2>装配与手势参考</h2></div><div class="reference-glyph">⊥</div><h3>竖直零姿态</h3><p>大臂与小臂均竖直，底座偏航 0°。模型定义与 45° 启动目标分开保存。</p><dl><div v-for="(value, key) in device.model?.geometry ?? fallbackGeometry" :key="key"><dt>{{ ({ base_width: '底板宽', table_to_shoulder_axis: '肩轴高度', shoulder_to_elbow_axis: '肩肘轴距', elbow_to_gripper_mount: '肘至安装面', gripper_mount_to_tip: '安装面至爪尖' })[key] }}</dt><dd>{{ value }} mm</dd></div></dl><div class="neutral-reference"><span>当前会话参考四元数（w / x / y / z）</span><strong>{{ vectorLabel(referenceQuaternion, 4) }}</strong><small>只在“启用并回初始”时捕获，不写入配置、不跨重启恢复；A 仅切换开始/暂停。</small></div></section></div><section class="panel profile-info"><span class="eyebrow">PROFILE IDENTITY</span><p><strong>仿真配置</strong><code>{{ device.model?.config_id ?? '等待配置' }}</code></p><p><strong>几何基线</strong><code>{{ device.model?.baseline_id ?? '等待配置' }}</code></p><p><strong>真实轴角</strong><span>未配备外部位置反馈 · measured = null</span></p></section>
        </template>
        <footer class="page-footer"><span>MotionArm Lab <i>/</i> {{ executorLabel }} · 软件目标／估算预览</span><span>仿真参数不代表实机安全标定</span></footer>
      </main>
    </div>
  </div>
</template>
