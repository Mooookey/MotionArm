import type { ModelConfig, Telemetry } from './types'

/** These labels describe software output; none imply a measured physical pose. */
export function outputStageLabel(stage: string | undefined): string {
  return ({ locked: 'PWM 已锁定', holding: 'PWM 输出保持中', disabled: 'PWM 已释放', unknown: '输出状态未知 · 请人工处置' })[stage ?? ''] ?? '等待输出遥测'
}

export function poweredBinding(state: Telemetry | null, manager: { manager_token: string } | null) {
  const output = state?.output ?? state?.executor.output
  if (state?.executor.kind !== 'robot_powered' || !state.executor.connected ||
      state.executor.boot_id == null || !output?.profile_hash) throw new Error('等待当前实体执行端 boot 与配置身份。')
  return { boot_id: state.executor.boot_id, profile_hash: output.profile_hash, ...manager }
}

export function directObservationRequest(state: Telemetry | null, manager: { manager_token: string } | null) {
  const output = state?.output ?? state?.executor.output
  if (!state?.control.observation_ready || state.control.enabled || state.control.armed ||
      !state.imu.valid || state.input.status !== 'valid' || state.recording.active || state.replay.active ||
      state.control.fault || output?.stage !== 'holding' || output.pwm_enabled !== true || !output.committed ||
      !output.prepared || output.diagnostic_active || !manager?.manager_token) {
    throw new Error('直接观察仍需当前管理会话、有效稳定输入、PWM保持和准备完成，并停止录制与回放。')
  }
  return { ...poweredBinding(state, manager), direct_observation: true,
    note: '按本次用户选择省略剩余逐轴诊断，直接进行批准范围内腕端跟随观察；不记录任何关节验收通过。' }
}

export function canViewHistory(state: Telemetry | null): boolean {
  return !state || state.executor.kind === 'virtual' ||
    (!state.executor.connected && !state.control.enabled)
}

/** Recorded ranges remain presentation metadata, independent of today's profile. */
export function recordedLimits(metadata: Record<string, unknown> | null | undefined): [number, number][] | null {
  const limits = (metadata?.config as { limits_deg?: unknown } | undefined)?.limits_deg
  return Array.isArray(limits) && limits.length === 3 && limits.every((row) => Array.isArray(row) && row.length === 2 && row.every((n) => typeof n === 'number' && Number.isFinite(n)) && row[0] < row[1]) ? limits as [number, number][] : null
}

export interface DisplayMapping { signs: [number, number] | null; version: string | null }
function readSigns(value: unknown): [number, number] | null {
  return Array.isArray(value) && value.length === 2 && value.every((v) => v === 1 || v === -1) ? [value[0], value[1]] : null
}
export function modelMapping(model: ModelConfig | null, powered = false): DisplayMapping {
  if (!model || powered !== !!model.profile_hash) return { signs: null, version: null }
  return { signs: readSigns(model.mounting.joint_signs), version: model.mapping_version ?? (model.profile_hash ? null : 'coordinated-angle-v3') }
}
export function recordedMapping(metadata: Record<string, unknown> | null | undefined): DisplayMapping {
  const mounting = metadata?.mounting as { joint_signs?: unknown } | undefined
  const version = metadata?.controller_version ?? metadata?.mapping_version
  return { signs: readSigns(mounting?.joint_signs ?? metadata?.joint_signs), version: typeof version === 'string' ? version : null }
}
export function mappingSignsLabel(signs: [number, number] | null): string {
  const label = (sign: number) => sign > 0 ? '+1' : '−1'
  return signs ? `底座 ${label(signs[0])} / 双臂 ${label(signs[1])}` : '映射符号未记录或尚未核对'
}
export function rightTiltLabel(signs: [number, number] | null): string {
  return signs ? `右倾底座${signs[0] > 0 ? '顺时针' : '逆时针'}` : '底座方向等待配置核对'
}
