/** Diagnostic display only. This never changes IMU validity or control inputs. */
export function nextAccelerationWarning(previous: boolean, norm: number | null | undefined, fresh: boolean): boolean {
  if (!fresh || typeof norm !== 'number' || !Number.isFinite(norm)) return false
  const deviation = Math.abs(norm - 1)
  if (previous) return deviation >= 0.12
  return deviation > 0.18
}

export interface SceneSize {
  width: number
  height: number
  viewportWidth: number
  viewportHeight: number
}

/** A content-driven resize must never reset a user's camera. */
export function sceneResizeAction(previous: SceneSize | null, next: SceneSize): 'none' | 'render' | 'fit' {
  if (next.width <= 0 || next.height <= 0) return 'none'
  if (!previous) return 'fit'
  const changed = Math.abs(next.width - previous.width) >= 0.5 || Math.abs(next.height - previous.height) >= 0.5
  if (!changed) return 'none'
  const viewportChanged = next.viewportWidth !== previous.viewportWidth || next.viewportHeight !== previous.viewportHeight
  return viewportChanged ? 'fit' : 'render'
}
