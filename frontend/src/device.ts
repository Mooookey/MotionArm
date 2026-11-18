import { computed, ref, shallowRef } from 'vue'
import { defineStore } from 'pinia'
import { request } from './api'
import type { Calibration, Point, Telemetry } from './types'

const MAX_POINTS = 7500
const STALE_MS = 2000

function validFrame(value: unknown): value is Telemetry {
  if (typeof value !== 'object' || value === null) return false
  const t = value as Partial<Telemetry>
  return t.event === 'telemetry' && Number.isFinite(t.server_time_ms) &&
    (t.source === 'serial' || t.source === 'mock') && typeof t.read_only === 'boolean' &&
    typeof t.connection?.robot === 'string' && typeof t.control?.state === 'string' &&
    typeof t.imu?.valid === 'boolean' && typeof t.recording?.active === 'boolean' &&
    typeof t.replay?.active === 'boolean' && t.joints?.measured_deg === null
}

export const useDevice = defineStore('device', () => {
  const snapshot = shallowRef<Telemetry | null>(null)
  const calibration = shallowRef<Calibration | null>(null)
  const connection = ref<'connecting' | 'connected' | 'offline' | 'stale'>('connecting')
  const points = shallowRef<Point[]>([])
  const events = ref<{ time: number; label: string }[]>([])
  const fresh = computed(() => connection.value === 'connected' && snapshot.value !== null)
  const controlsAllowed = computed(() => fresh.value && snapshot.value?.read_only === false &&
    snapshot.value?.connection.robot === 'online')

  let socket: WebSocket | undefined
  let reconnect: ReturnType<typeof setTimeout> | undefined
  let watchdog: ReturnType<typeof setInterval> | undefined
  let generation = 0
  let seenAt = 0
  let lastPointTime = 0
  let attempts = 0
  const buffer: Point[] = []

  function log(label: string) {
    events.value = [{ time: Date.now(), label }, ...events.value].slice(0, 30)
  }

  async function loadCalibration() {
    try {
      calibration.value = await request<Calibration>('/api/config/calibration')
    } catch {
      calibration.value = null
    }
  }

  function accept(value: unknown, fromSocket: boolean) {
    if (!validFrame(value)) return
    const previous = snapshot.value
    if (previous && fromSocket && connection.value === 'connected' && value.server_time_ms < previous.server_time_ms) return
    snapshot.value = value
    if (fromSocket) {
      if (connection.value !== 'connected') log('遥测已连接')
      connection.value = 'connected'
      seenAt = performance.now()
      attempts = 0
    }
    if (previous?.calibration_id !== value.calibration_id) void loadCalibration()
    if (previous && previous.control.state !== value.control.state) log(`控制状态变更为 ${value.control.state}`)
    if (previous && previous.control.mode !== value.control.mode) log(`手腕模式变更为 ${value.control.mode}`)
    if (value.control.fault && previous?.control.fault !== value.control.fault) log(`故障 ${value.control.fault}`)
    if (value.server_time_ms - lastPointTime < 90) return
    lastPointTime = value.server_time_ms
    buffer.push({
      time: value.server_time_ms,
      roll: value.imu.valid ? value.imu.roll_deg : null,
      pitch: value.imu.valid ? value.imu.pitch_deg : null,
      targets: value.joints.applied_target_deg,
    })
    if (buffer.length > MAX_POINTS) buffer.shift()
    points.value = buffer.slice()
  }

  function start() {
    const currentGeneration = ++generation
    void request<Telemetry>('/api/status').then((state) => {
      if (currentGeneration === generation && !fresh.value) accept(state, false)
    }).catch(() => {})
    void loadCalibration()

    const connect = () => {
      if (currentGeneration !== generation) return
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:'
      socket = new WebSocket(`${protocol}//${location.host}/ws/telemetry`)
      socket.onmessage = ({ data }) => {
        if (currentGeneration !== generation) return
        try { accept(JSON.parse(data), true) } catch { /* Ignore malformed telemetry. */ }
      }
      socket.onerror = () => socket?.close()
      socket.onclose = () => {
        if (currentGeneration !== generation) return
        if (connection.value !== 'offline') log('遥测断开，已锁定动作入口')
        connection.value = 'offline'
        reconnect = setTimeout(connect, Math.min(500 * 2 ** attempts++, 8000))
      }
    }
    connect()
    watchdog = setInterval(() => {
      if (connection.value === 'connected' && performance.now() - seenAt > STALE_MS) {
        connection.value = 'stale'
        log('遥测超过 2 秒未更新')
        socket?.close()
      }
    }, 250)
  }

  function stop() {
    ++generation
    clearTimeout(reconnect)
    clearInterval(watchdog)
    socket?.close()
    socket = undefined
  }

  return { snapshot, calibration, connection, points, events, fresh, controlsAllowed, loadCalibration, start, stop }
})
