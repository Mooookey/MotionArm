/** Running is server control state, independent of the raw physical button level. */
export function coordinatedFollowLabel(enabled: boolean, running: boolean): string {
  if (!enabled) return '尚未启用'
  return running ? '跟随中' : '已暂停，短按 A 继续'
}

/** Radio receipt freshness is separate from whether IMU samples are usable. */
export function buttonTelemetryFresh(fresh: boolean, kind: string, status: string, age: number | null | undefined, wirelessFresh?: boolean): boolean {
  if (!fresh) return false
  if (kind === 'esp_now') return wirelessFresh === true
  return ['valid', 'invalid', 'calibrating'].includes(status) && age != null && age < 250
}
