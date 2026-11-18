/** Older firmware lacks these counters and cannot safely implement click toggles. */
export function hasButtonClickCounters(buttons: { ac?: unknown; bc?: unknown } | null | undefined): boolean {
  return !!buttons && [buttons.ac, buttons.bc].every((value) =>
    typeof value === 'number' && Number.isInteger(value) && value >= 0 && value <= 0xffffffff)
}

/** Running is server control state, independent of the raw physical button level. */
export function coordinatedFollowLabel(enabled: boolean, running: boolean): string {
  if (!enabled) return '尚未启用'
  return running ? '跟随中' : '已暂停，短按 A 继续'
}
