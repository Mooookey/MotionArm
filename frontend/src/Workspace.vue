<script setup lang="ts">
import { computed, onBeforeUnmount, ref, watch } from 'vue'
import { useRoute } from 'vue-router'
import { request } from './api'
import { useDevice } from './device'
import TelemetryChart from './TelemetryChart.vue'
import type { Point, Preflight, Recording, RecordingDetail } from './types'

const route = useRoute()
const device = useDevice()
const page = computed(() => route.name)
const telemetry = computed(() => device.fresh ? device.snapshot : null)
const names = ['底座', '大臂', '小臂', '夹爪']
const ids = ['base', 'shoulder', 'elbow', 'gripper']
const channels = [14, 15, 13, 12]
const stateNames: Record<string, string> = {
  SAFE_BOOT: '上电锁定', READY: '就绪', TELEOP: '体感控制', RECORDING_TELEOP: '示教录制',
  PC_MANUAL: '电脑控制', REPLAY: '轨迹回放', HOLD: '保持', FAULT: '故障', ESTOP_LATCHED: '急停锁定',
}
const ownerNames: Record<string, string> = { NONE: '无', WEARABLE: '手腕端', REPLAY: '回放', PC_MANUAL: '电脑端' }
const replayMessages: Record<string, string> = {
  RECORD_INCOMPLETE: '录制不完整或采样时间不连续。',
  CALIBRATION_MISMATCH: '录制的标定版本或工位与当前配置不同。',
  INITIAL_TARGET_MISMATCH: '当前软件目标与录制起点超出允许偏差。',
  JOINT_LIMITS_INVALID: '轨迹超出关节行程或标定限位不可用。',
  SPEED_OR_ACCELERATION_LIMIT: '轨迹超出速度或加速度限值。',
  GRIPPER_HOLD_TIME: '夹爪闭合超时，或录制结束时夹爪仍未打开。',
  WORKCELL_NOT_CONFIRMED: '请核对工位、物体位置和起始姿态。',
  ROBOT_OFFLINE: '执行端离线。',
  CONTROL_NOT_READY: '执行端需要处于无占用的就绪状态。',
  MOTION_NOT_VERIFIED: '当前执行端未通过回放许可检查。',
  JOINT_POSITION_NOT_EXTERNALLY_MEASURED: '请现场核对机械臂起始姿态，关节角度无外部测量。',
}
const stateLabel = (state: string | undefined) => state ? stateNames[state] ?? state : '未知'
const degrees = (value?: number | null) => typeof value === 'number' && Number.isFinite(value) ? `${value.toFixed(1)}°` : '—'
const duration = (value?: number | null) => typeof value === 'number' ? `${(value / 1000).toFixed(1)} 秒` : '—'
const date = (ms: number) => Number.isFinite(ms) ? new Date(ms).toLocaleString('zh-CN') : '—'
const inspection = ref(false)
const teachingInspection = ref(false)
const replayInspection = ref(false)
const busy = ref(false)
const success = ref('')
const problem = ref('')
const recordName = ref('')
const recordings = ref<Recording[]>([])
const selected = ref('')
const detail = ref<RecordingDetail | null>(null)
const report = ref<Preflight | null>(null)
const saved = ref<Recording | null>(null)
const chartType = ref<'imu' | 'joints'>('imu')
const chartPaused = ref(false)
const frozen = ref<Point[]>([])
const fallbackTarget = computed(() => telemetry.value?.joints.applied_target_deg ?? telemetry.value?.joints.requested_deg)
const target = ref<Array<number | ''> | null>(null)
const leaseId = ref<string | null>(null)
let heartbeatTimer: ReturnType<typeof setInterval> | undefined
const limits = computed(() => ids.map((id) => device.calibration?.profile?.joints?.find((j) => j.id === id)))
const validLimits = computed(() => limits.value.every((j) =>
  typeof j?.logical_min_deg === 'number' && typeof j.logical_max_deg === 'number' &&
  j.logical_max_deg > j.logical_min_deg))
const sessionLease = computed(() => Boolean(leaseId.value && (telemetry.value?.source === 'serial'
  ? telemetry.value.control.has_lease === true
  : telemetry.value?.control.lease_id === leaseId.value)))
const ownedHere = computed(() => device.controlsAllowed && sessionLease.value &&
  telemetry.value?.control.owner === 'PC_MANUAL' && telemetry.value.control.can_move === true)
const canReleaseWearable = computed(() => telemetry.value?.control.owner === 'WEARABLE' &&
  !telemetry.value.recording.active && (sessionLease.value || telemetry.value.source === 'mock'))
const canRequestOwner = computed(() => {
  const control = telemetry.value?.control
  return device.controlsAllowed && control?.owner === 'NONE' && !control.fault &&
    ['SAFE_BOOT', 'READY', 'HOLD'].includes(control.state)
})
const archivable = computed(() => device.controlsAllowed &&
  telemetry.value?.control.owner === 'WEARABLE' &&
  ['TELEOP', 'RECORDING_TELEOP'].includes(telemetry.value.control.state) &&
  telemetry.value.imu.valid && !telemetry.value.recording.active)
const preview = computed<Point[]>(() => detail.value?.frames.map((frame) => ({
  time: frame.t_ms,
  roll: frame.roll_deg,
  pitch: frame.pitch_deg,
  targets: frame.q_out,
})) ?? [])
const canLaunchReplay = computed(() => Boolean(
  device.controlsAllowed && report.value?.ok && report.value.preflight_id && replayInspection.value &&
  !telemetry.value?.recording.active && !telemetry.value?.replay.active &&
  ['READY', 'HOLD'].includes(telemetry.value?.control.state ?? '') && telemetry.value?.control.owner === 'NONE',
))

function pauseChart() {
  if (!chartPaused.value) frozen.value = device.points.slice()
  chartPaused.value = !chartPaused.value
}

async function execute(action: () => Promise<void>) {
  if (busy.value) return
  busy.value = true
  success.value = ''
  problem.value = ''
  try {
    await action()
  } catch (error) {
    problem.value = error instanceof Error ? error.message : '无法完成请求'
  } finally {
    busy.value = false
  }
}

async function loadRecordings() {
  recordings.value = await request<Recording[]>('/api/recordings')
  if (selected.value && !recordings.value.some((r) => r.id === selected.value)) selected.value = ''
}

function startRecording() {
  if (!archivable.value || !recordName.value.trim()) return
  void execute(async () => {
    await request('/api/recordings/start', { name: recordName.value.trim() })
    saved.value = null
    success.value = '服务端已开始示教录制。'
  })
}

function stopRecording() {
  void execute(async () => {
    const result = await request<Recording>('/api/recordings/stop', {})
    saved.value = result
    success.value = result.status === 'completed' ? '示教录制已保存。' : '已保存录制，请核对完整性状态。'
    await loadRecordings()
  })
}

function arm() {
  if (!inspection.value || !validLimits.value || !canRequestOwner.value) return
  void execute(async () => {
    const response = await request<{ lease_id: string }>('/api/control/arm', { operator_confirmed_safe: true })
    leaseId.value = response.lease_id
    target.value = fallbackTarget.value ? [...fallbackTarget.value] : null
    success.value = '控制权已申请，等待执行端状态确认。'
  })
}

function restoreWearable() {
  if (!teachingInspection.value || !canRequestOwner.value) return
  void execute(async () => {
    const response = await request<{ lease_id?: string }>('/api/control/arm', {
      requested_owner: 'WEARABLE',
      operator_confirmed_safe: true,
    })
    leaseId.value = response.lease_id ?? null
    success.value = '体感控制权已申请，等待执行端状态确认。'
  })
}

function release() {
  void execute(async () => {
    await request('/api/control/release', leaseId.value ? { lease_id: leaseId.value } : {})
    leaseId.value = null
    target.value = null
    inspection.value = false
    report.value = null
    success.value = '控制权已释放。'
  })
}

function sendTarget() {
  if (!ownedHere.value || !target.value || !validLimits.value) return
  const values = [...target.value]
  if (!values.every((value, i) => typeof value === 'number' && Number.isFinite(value) &&
    value >= (limits.value[i]?.logical_min_deg ?? Infinity) &&
    value <= (limits.value[i]?.logical_max_deg ?? -Infinity))) {
    problem.value = '四路目标均需处于当前标定范围内。'
    return
  }
  void execute(async () => {
    await request('/api/control/manual-target', { q_deg: values, lease_id: leaseId.value })
    success.value = '服务端已接纳四路软件目标。'
  })
}

function preflight() {
  if (!selected.value || !device.fresh) return
  void execute(async () => {
    const id = selected.value
    report.value = null
    const response = await request<Preflight>('/api/replays/preflight', {
      record_id: id,
      operator_workcell_confirmed: replayInspection.value,
    })
    if (selected.value === id) report.value = response
  })
}

function startReplay() {
  if (!canLaunchReplay.value) return
  const token = report.value?.preflight_id
  report.value = null
  void execute(async () => {
    await request('/api/replays/start', { preflight_id: token, operator_confirmed_safe: true })
    success.value = '服务端已接纳回放请求。'
  })
}

function stopReplay() {
  void execute(async () => {
    await request('/api/replays/stop', {})
    success.value = '服务端已接纳停止回放请求。'
  })
}

function issueText(issue: string | { code?: string; message?: string }) {
  const code = typeof issue === 'string' ? issue : issue.code
  return (typeof issue === 'string' ? undefined : issue.message) ?? (code && replayMessages[code]) ?? code ?? '未知原因'
}

watch(page, (next) => {
  success.value = ''
  problem.value = ''
  inspection.value = false
  teachingInspection.value = false
  replayInspection.value = false
  report.value = null
  if (next === 'teaching' || next === 'replay') void loadRecordings().catch((error: Error) => { problem.value = error.message })
  if (next === 'settings') void device.loadCalibration()
}, { immediate: true })
watch(selected, async (id) => {
  detail.value = null
  report.value = null
  replayInspection.value = false
  if (!id) return
  try {
    const row = await request<RecordingDetail>(`/api/recordings/${encodeURIComponent(id)}`)
    if (id === selected.value) detail.value = row
  } catch (error) {
    if (id === selected.value) problem.value = error instanceof Error ? error.message : '无法读取轨迹'
  }
})
watch([ownedHere, fallbackTarget], ([owned, currentTarget]) => {
  if (owned && currentTarget && !target.value) target.value = [...currentTarget]
})
watch([page, leaseId, () => telemetry.value?.source], ([currentPage, lease, source]) => {
  clearInterval(heartbeatTimer)
  if (currentPage !== 'manual' || !lease || source !== 'serial') return
  let pending = false
  const keepAlive = async () => {
    if (pending || leaseId.value !== lease || !device.controlsAllowed || document.hidden) return
    pending = true
    try {
      await request('/api/control/heartbeat', { lease_id: lease })
    } catch {
      if (leaseId.value === lease) {
        leaseId.value = null
        target.value = null
        inspection.value = false
        problem.value = '手动控制租约已失效，需重新确认现场状态。'
      }
    } finally {
      pending = false
    }
  }
  void keepAlive()
  heartbeatTimer = setInterval(keepAlive, 100)
})
onBeforeUnmount(() => clearInterval(heartbeatTimer))
watch([() => device.fresh, () => device.snapshot?.source, () => device.snapshot?.calibration_id], () => {
  report.value = null
  leaseId.value = null
  target.value = null
  inspection.value = false
  teachingInspection.value = false
  replayInspection.value = false
  if (device.fresh && (page.value === 'teaching' || page.value === 'replay')) {
    void loadRecordings().catch((error: Error) => { problem.value = error.message })
  }
})
</script>

<template>
  <p v-if="problem" class="notice warning" role="alert">{{ problem }}</p>
  <p v-if="success" class="notice" aria-live="polite">{{ success }}</p>

  <section v-if="page === 'dashboard'">
    <div class="page-head">
      <div><h1>设备总览</h1><p>查看手腕姿态、执行端状态与关节指令。</p></div>
      <div class="state-label" :class="{ muted: !telemetry }">{{ stateLabel(telemetry?.control.state) }}</div>
    </div>
    <div class="readout-bar">
      <span>控制来源 <strong>{{ ownerNames[telemetry?.control.owner ?? ''] ?? '—' }}</strong></span>
      <span>手腕模式 <strong>{{ telemetry?.control.mode === 'SHOULDER' ? '大臂' : telemetry?.control.mode === 'ELBOW' ? '小臂' : '—' }}</strong></span>
      <span>使能按钮 A <strong>{{ !telemetry ? '—' : telemetry.control.deadman ? '按住' : '松开' }}</strong></span>
      <span>末帧年龄 <strong>{{ telemetry?.imu.age_ms == null ? '—' : `${telemetry.imu.age_ms} ms` }}</strong></span>
    </div>
    <div class="dashboard-grid">
      <article class="surface">
        <div class="section-head">
          <div><h2>遥测曲线</h2><p>最近 30 秒</p></div>
          <div class="toolbar">
            <button class="quiet" :aria-pressed="chartType === 'imu'" @click="chartType = 'imu'">手腕</button>
            <button class="quiet" :aria-pressed="chartType === 'joints'" @click="chartType = 'joints'">关节目标</button>
            <button class="quiet" :aria-pressed="chartPaused" @click="pauseChart">{{ chartPaused ? '恢复图表' : '暂停图表' }}</button>
          </div>
        </div>
        <div class="imu-readouts">
          <div><span>Roll</span><strong>{{ degrees(telemetry?.imu.valid ? telemetry.imu.roll_deg : null) }}</strong></div>
          <div><span>Pitch</span><strong>{{ degrees(telemetry?.imu.valid ? telemetry.imu.pitch_deg : null) }}</strong></div>
          <div><span>姿态</span><strong class="text-reading">{{ telemetry?.imu.valid ? '有效' : '等待估计' }}</strong></div>
        </div>
        <TelemetryChart :points="chartPaused ? frozen : device.points" :type="chartType" live />
      </article>
      <article class="surface">
        <div class="section-head"><div><h2>关节软件目标</h2><p>实际关节角度未配备反馈</p></div></div>
        <table class="targets">
          <thead><tr><th scope="col">关节</th><th scope="col">请求</th><th scope="col">已应用</th></tr></thead>
          <tbody><tr v-for="(name, i) in names" :key="name">
            <th scope="row">{{ name }} <small>CH{{ channels[i] }}</small></th>
            <td>{{ degrees(telemetry?.joints.requested_deg?.[i]) }}</td>
            <td>{{ degrees(telemetry?.joints.applied_target_deg?.[i]) }}</td>
          </tr></tbody>
        </table>
        <p v-if="telemetry?.control.fault" class="fault">执行端故障 {{ telemetry.control.fault }}</p>
      </article>
    </div>
    <div class="dashboard-grid secondary">
      <article class="surface workflow">
        <h2>示教与回放</h2>
        <dl class="workflow-list">
          <div><dt>示教录制</dt><dd>{{ telemetry?.recording.active ? `录制中 / ${telemetry.recording.frames} 帧` : '空闲' }}</dd></div>
          <div><dt>轨迹回放</dt><dd>{{ telemetry?.replay.active ? `回放中 / ${Math.round(telemetry.replay.progress * 100)}%` : '空闲' }}</dd></div>
        </dl>
        <div class="toolbar"><RouterLink class="button secondary-button" to="/teaching">查看录制</RouterLink><RouterLink class="button secondary-button" to="/replay">查看轨迹</RouterLink></div>
      </article>
      <article class="surface workflow">
        <h2>会话事件</h2>
        <ul v-if="device.events.length" class="event-list">
          <li v-for="(event, i) in device.events.slice(0, 5)" :key="i"><time>{{ new Date(event.time).toLocaleTimeString('zh-CN') }}</time>{{ event.label }}</li>
        </ul>
        <p v-else class="soft">等待首条遥测。</p>
      </article>
    </div>
  </section>

  <section v-else-if="page === 'manual'">
    <div class="page-head"><div><h1>手动调试</h1><p>申请独占控制权后可提交完整四关节软件目标。</p></div></div>
    <article class="surface panel">
      <div class="section-head"><div><h2>四路目标</h2><p>限位来自当前标定版本。实际关节角度未配备反馈。</p></div><b>{{ stateLabel(telemetry?.control.state) }}</b></div>
      <p v-if="!validLimits" class="soft">标定限位不可用，不能提交动作。</p>
      <div class="manual-targets">
        <div v-for="(name, i) in names" :key="name">
          <label :for="`joint-${i}`">{{ name }} <small>CH{{ channels[i] }}</small></label>
          <input v-if="target" :id="`joint-${i}`" v-model.number="target[i]" type="number" step="0.1"
            :min="limits[i]?.logical_min_deg ?? undefined" :max="limits[i]?.logical_max_deg ?? undefined" :disabled="!ownedHere" />
          <output v-else>{{ degrees(fallbackTarget?.[i]) }}</output>
        </div>
      </div>
      <label v-if="!ownedHere" class="checkbox"><input v-model="inspection" type="checkbox" :disabled="!device.controlsAllowed" />已核对工作台净空、承重支撑与机械臂起始位置。</label>
      <div class="toolbar">
        <button v-if="!ownedHere" class="primary" :disabled="busy || !canRequestOwner || !inspection || !validLimits" @click="arm">申请电脑控制权</button>
        <button v-else class="primary" :disabled="busy || !target" @click="sendTarget">提交四路目标</button>
        <button v-if="ownedHere || canReleaseWearable" class="secondary-button" :disabled="busy" @click="release">释放控制权</button>
      </div>
    </article>
  </section>

  <section v-else-if="page === 'teaching'">
    <div class="page-head"><div><h1>示教录制</h1><p>保存执行端已应用的软件目标和手腕姿态。</p></div></div>
    <article class="surface panel">
      <div class="section-head"><div><h2>当前会话</h2><p>{{ telemetry?.recording.active ? `已录制 ${telemetry.recording.frames} 帧` : '等待开始' }}</p></div><b>{{ stateLabel(telemetry?.control.state) }}</b></div>
      <label class="field">录制名称<input v-model="recordName" maxlength="100" placeholder="例如 泡沫块搬运-01" :disabled="telemetry?.recording.active" /></label>
      <div class="toolbar">
        <button class="primary" :disabled="busy || !archivable || !recordName.trim()" @click="startRecording">开始录制</button>
        <button class="secondary-button" :disabled="busy || !telemetry?.recording.active" @click="stopRecording">结束并保存</button>
        <button v-if="canReleaseWearable" class="secondary-button" :disabled="busy" @click="release">释放体感控制权</button>
      </div>
      <p v-if="!telemetry?.recording.active && !archivable" class="soft">开始录制需要体感控制权、有效手腕姿态和在线执行端。</p>
      <template v-if="canRequestOwner">
        <label class="checkbox"><input v-model="teachingInspection" type="checkbox" />已确认工位净空和机械臂起始位置。</label>
        <button class="secondary-button" :disabled="busy || !teachingInspection" @click="restoreWearable">申请体感控制权</button>
      </template>
      <p v-if="saved" class="soft">录制 {{ saved.id }} / {{ saved.status }} / {{ saved.frame_count }} 帧 / {{ duration(saved.duration_ms) }}</p>
    </article>
    <article class="surface panel">
      <div class="section-head"><div><h2>已保存录制</h2></div><button class="quiet" :disabled="busy" @click="execute(loadRecordings)">刷新</button></div>
      <p v-if="!recordings.length" class="soft">暂无录制。</p>
      <ul v-else class="record-list">
        <li v-for="row in recordings" :key="row.id"><span><strong>{{ row.name }}</strong><small>{{ date(row.created_ms) }} / {{ row.frame_count }} 帧</small></span><span>{{ row.status }}</span></li>
      </ul>
    </article>
  </section>

  <section v-else-if="page === 'replay'">
    <div class="page-head"><div><h1>轨迹回放</h1><p>选择一段示教记录，然后核对校准版本、初态和轨迹完整性。</p></div></div>
    <div class="replay-grid">
      <article class="surface panel">
        <div class="section-head"><h2>轨迹库</h2><button class="quiet" :disabled="busy" @click="execute(loadRecordings)">刷新</button></div>
        <p v-if="!recordings.length" class="soft">完成一段示教录制后，轨迹会出现在这里。</p>
        <div v-else class="record-picker">
          <label v-for="row in recordings" :key="row.id" :class="{ picked: selected === row.id }">
            <input v-model="selected" type="radio" name="recording" :value="row.id" :disabled="busy || telemetry?.replay.active" />
            <span><strong>{{ row.name }}</strong><small>{{ date(row.created_ms) }} / {{ row.frame_count }} 帧 / {{ row.status }}</small></span>
          </label>
        </div>
      </article>
      <article class="surface panel">
        <div class="section-head"><div><h2>{{ detail?.name ?? '软件目标预览' }}</h2><p>{{ detail ? `${duration(detail.duration_ms)} / ${detail.frame_count} 帧 / ${detail.status}` : '请选择轨迹' }}</p></div></div>
        <template v-if="detail">
          <TelemetryChart :points="preview" type="joints" />
          <div class="toolbar">
            <a class="button quiet" :href="`/api/recordings/${encodeURIComponent(detail.id)}/export?format=csv`" download>导出 CSV</a>
            <a class="button quiet" :href="`/api/recordings/${encodeURIComponent(detail.id)}/export?format=json`" download>导出 JSON</a>
          </div>
        </template>
        <label class="checkbox"><input v-model="replayInspection" type="checkbox" :disabled="busy || !detail || !device.controlsAllowed" @change="report = null" />已核对工位、物体位置、起始姿态和工作区。</label>
        <div class="toolbar">
          <button class="secondary-button" :disabled="busy || !detail || !device.fresh" @click="preflight">预检回放</button>
          <button class="primary" :disabled="busy || !canLaunchReplay" @click="startReplay">开始回放</button>
          <button class="secondary-button" :disabled="busy || !telemetry?.replay.active" @click="stopReplay">停止回放</button>
          <button v-if="canReleaseWearable" class="quiet" :disabled="busy" @click="release">释放体感控制权</button>
        </div>
        <div v-if="report" class="notice" :class="{ warning: !report.ok }" aria-live="polite">
          <strong>{{ report.ok ? '预检通过' : '预检未通过' }}</strong>
          <p v-if="report.speed_scale != null">时间缩放 {{ report.speed_scale.toFixed(2) }}×</p>
          <ul v-if="report.violations?.length || report.errors?.length || report.warnings?.length">
            <li v-for="(issue, i) in [...(report.violations ?? []), ...(report.errors ?? []), ...(report.warnings ?? [])]" :key="i">{{ issueText(issue) }}</li>
          </ul>
          <ul v-if="report.operator_checklist?.length">
            <li v-for="entry in report.operator_checklist" :key="entry">{{ entry }}</li>
          </ul>
        </div>
        <label v-if="telemetry?.replay.active" class="progress">命令回放进度 <progress max="1" :value="telemetry.replay.progress"></progress></label>
      </article>
    </div>
  </section>

  <section v-else-if="page === 'settings'">
    <div class="page-head"><div><h1>设备标定</h1><p>查看通道对应关系、脉宽限位与部署校准版本。</p></div><button class="secondary-button" @click="device.loadCalibration">重新读取</button></div>
    <article class="surface panel">
      <h2>{{ device.calibration?.allow_arm ? '控制配置已启用' : '控制配置锁定' }}</h2>
      <p class="soft">校准版本 {{ device.calibration?.calibration_id ?? '待验收' }} / 工位 {{ device.calibration?.profile?.workcell?.workcell_id ?? '待配置' }}</p>
      <p v-if="telemetry?.source === 'serial'" class="soft">PCA9685 {{ telemetry.connection.pca_ok == null ? '等待报告' : telemetry.connection.pca_ok ? '已响应' : '未响应' }}</p>
      <div class="table-scroll"><table class="targets settings-table">
        <thead><tr><th scope="col">关节</th><th scope="col">通道</th><th scope="col">逻辑角度</th><th scope="col">输出脉宽</th><th scope="col">最大速度</th></tr></thead>
        <tbody><tr v-for="(name, i) in names" :key="name">
          <th scope="row">{{ name }}</th><td>CH{{ limits[i]?.channel ?? channels[i] }}</td>
          <td>{{ degrees(limits[i]?.logical_min_deg) }} 至 {{ degrees(limits[i]?.logical_max_deg) }}</td>
          <td>{{ limits[i]?.pulse_min_us ?? '—' }} 至 {{ limits[i]?.pulse_max_us ?? '—' }} µs</td>
          <td>{{ limits[i]?.max_velocity_deg_s ?? '—' }} °/s</td>
        </tr></tbody>
      </table></div>
    </article>
  </section>
</template>
