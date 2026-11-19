/** Running is server control state, independent of the raw physical button level. */
export function coordinatedFollowLabel(enabled: boolean, running: boolean): string {
  if (!enabled) return '尚未启用'
  return running ? '跟随中' : '已暂停，短按 A 继续'
}
