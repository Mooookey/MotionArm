import { computed, ref, shallowRef } from 'vue'
import { defineStore } from 'pinia'
import { request } from './api'
import type { ModelConfig, Point, Pose, Telemetry } from './types'

const poseKeys = ['base_yaw_deg', 'shoulder_down_deg', 'elbow_down_deg', 'gripper_open_ratio'] as const
export function isPose(value: unknown): value is Pose {
  return typeof value === 'object' && value !== null && poseKeys.every((key) =>
    typeof (value as Pose)[key] === 'number' && Number.isFinite((value as Pose)[key]))
}
function validFrame(value: unknown): value is Telemetry {
  if (typeof value !== 'object' || !value) return false
  const t = value as Telemetry
  return t.schema_version === 2 && t.source === 'wearable' && Number.isFinite(t.server_time_ms) &&
    typeof t.input?.status === 'string' && t.executor?.kind === 'virtual' &&
    typeof t.imu?.valid === 'boolean' && typeof t.buttons?.a === 'boolean' &&
    typeof t.control?.enabled === 'boolean' && isPose(t.joint_state?.requested) &&
    isPose(t.joint_state?.output_target) && t.joint_state.measured === null &&
    typeof t.recording?.active === 'boolean' && typeof t.replay?.active === 'boolean' && Array.isArray(t.events)
}
export const useDevice = defineStore('device', () => {
  const snapshot = shallowRef<Telemetry | null>(null)
  const model = shallowRef<ModelConfig | null>(null)
  const connection = ref<'connecting' | 'connected' | 'offline' | 'stale'>('connecting')
  const incompatible = ref('')
  const points = shallowRef<Point[]>([])
  const fresh = computed(() => connection.value === 'connected' && snapshot.value !== null && !incompatible.value)
  const realReady = computed(() => fresh.value && snapshot.value?.input.status === 'valid' && snapshot.value.imu.valid)
  let socket: WebSocket | undefined
  let reconnect: ReturnType<typeof setTimeout> | undefined
  let watchdog: ReturnType<typeof setInterval> | undefined
  let generation = 0, seenAt = 0, lastPoint = 0, attempts = 0
  let configRequest: Promise<void> | undefined
  let retryConfigAt = 0
  async function loadConfig() {
    if (configRequest) return configRequest
    configRequest = request<ModelConfig>('/api/virtual/config').then((value) => {
      model.value = value
      retryConfigAt = 0
    }).catch((error) => {
      retryConfigAt = performance.now() + 3000
      throw error
    }).finally(() => { configRequest = undefined })
    return configRequest
  }
  function accept(value: unknown) {
    if (!validFrame(value)) {
      if (typeof value === 'object' && value !== null && 'source' in value) {
        incompatible.value = '当前后端输入为 ' + String((value as { source: unknown }).source) + '。请使用 wearable 模式启动服务。'
      }
      return
    }
    incompatible.value = ''
    snapshot.value = value
    connection.value = 'connected'
    seenAt = performance.now()
    attempts = 0
    if ((!model.value || (value.config_id && value.config_id !== model.value.config_id)) && performance.now() >= retryConfigAt) {
      void loadConfig().catch(() => {})
    }
    if (value.server_time_ms - lastPoint >= 95) {
      lastPoint = value.server_time_ms
      points.value = [...points.value.slice(-599), {
        time: value.server_time_ms, roll: value.imu.valid ? value.imu.roll_deg : null,
        pitch: value.imu.valid ? value.imu.pitch_deg : null,
        requested: value.joint_state.requested, output: value.joint_state.output_target,
      }]
    }
  }
  function start() {
    const current = ++generation
    void loadConfig().catch(() => {})
    void request('/api/status').then((value) => { if (current === generation) accept(value) }).catch(() => {})
    function connect() {
      if (current !== generation) return
      socket = new WebSocket((location.protocol === 'https:' ? 'wss:' : 'ws:') + '//' + location.host + '/ws/telemetry')
      socket.onmessage = ({ data }) => {
        if (current !== generation) return
        try { accept(JSON.parse(data)) } catch { /* malformed frames never refresh the watchdog */ }
      }
      socket.onerror = () => socket?.close()
      socket.onclose = () => {
        if (current !== generation) return
        connection.value = 'offline'
        reconnect = setTimeout(connect, Math.min(500 * 2 ** attempts++, 8000))
      }
    }
    connect()
    watchdog = setInterval(() => {
      if (connection.value === 'connected' && performance.now() - seenAt > 500) {
        connection.value = 'stale'
        socket?.close()
      }
    }, 100)
  }
  function stop() {
    ++generation
    clearTimeout(reconnect)
    clearInterval(watchdog)
    socket?.close()
  }
  return { snapshot, model, connection, incompatible, fresh, realReady, points, start, stop, loadConfig }
})
