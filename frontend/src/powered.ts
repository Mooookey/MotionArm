import type { Telemetry } from './types'

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

export function canViewHistory(state: Telemetry | null): boolean {
  return !state || state.executor.kind === 'virtual' ||
    (!state.executor.connected && !state.control.enabled)
}
