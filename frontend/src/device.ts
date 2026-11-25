import { computed, ref, shallowRef } from 'vue'
import { defineStore } from 'pinia'
import { request } from './api'
import type { ModelConfig, Point, Telemetry } from './types'
import { validFrame, validModelConfig } from './telemetry'
import { ManagementLease } from './management'
export { isPose } from './telemetry'

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
  let heartbeatTimer: ReturnType<typeof setInterval> | undefined
  const management = new ManagementLease()
  let generation = 0, seenAt = 0, lastPoint = 0, attempts = 0
  let configRequest: Promise<void> | undefined
  let retryConfigAt = 0
  async function loadConfig() {
    if (configRequest) return configRequest
    configRequest = request<ModelConfig>('/api/executor/config').then((value) => {
      if (!validModelConfig(value)) throw new Error('后端仿真配置格式不兼容，请同步更新服务。')
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
      management.clear()
      if (typeof value === 'object' && value !== null && 'source' in value) {
        const source = (value as { source: unknown }).source
        incompatible.value = source === 'wearable'
          ? '后端遥测格式不兼容，需要执行端 V5 服务。请同步更新前后端。'
          : '当前后端输入为 ' + String(source) + '。请使用 wearable 模式启动服务。'
      }
      return
    }
    incompatible.value = ''
    if (snapshot.value && (snapshot.value.executor.kind !== value.executor.kind ||
        snapshot.value.executor.boot_id !== value.executor.boot_id || snapshot.value.executor.profile_hash !== value.executor.profile_hash)) management.clear()
    if (snapshot.value && snapshot.value.executor.kind !== value.executor.kind) points.value = []
    snapshot.value = value
    management.observe(value.executor.kind !== 'robot_powered' || value.executor.connected,
      value.executor.kind === 'robot_powered' ? value.health?.manager_active === true : value.control.enabled)
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
        management.clear()
        reconnect = setTimeout(connect, Math.min(500 * 2 ** attempts++, 8000))
      }
    }
    connect()
    watchdog = setInterval(() => {
      if (connection.value === 'connected' && performance.now() - seenAt > 500) {
        connection.value = 'stale'
        management.clear()
        socket?.close()
      }
    }, 100)
    heartbeatTimer = setInterval(() => {
      const message = management.message()
      if (message && fresh.value && socket?.readyState === WebSocket.OPEN) {
        try { socket.send(JSON.stringify(message)) } catch { management.clear() }
      }
    }, 250)
  }
  function stop() {
    ++generation
    management.clear()
    clearTimeout(reconnect)
    clearInterval(watchdog)
    clearInterval(heartbeatTimer)
    socket?.close()
  }
  function grantManagement(token: string) { management.grant(token) }
  function releaseManagement() { management.clear() }
  function managementPayload() { return management.payload() }
  return { snapshot, model, connection, incompatible, fresh, realReady, points, start, stop, loadConfig, grantManagement, releaseManagement, managementPayload }
})
