import type { Geometry, Pose, RecordingFrame } from './types'
export const referencePose: Pose = { base_yaw_deg: 0, shoulder_down_deg: 45, elbow_down_deg: 45, gripper_open_ratio: 1 }
export const fallbackGeometry: Geometry = { base_width: 90, table_to_shoulder_axis: 90, shoulder_to_elbow_axis: 115, elbow_to_gripper_mount: 65, gripper_mount_to_tip: 90 }
export function tipPosition(pose: Pose, geometry: Geometry) {
  const yaw = pose.base_yaw_deg * Math.PI / 180
  const theta = (90 - pose.shoulder_down_deg) * Math.PI / 180
  const phi = -pose.elbow_down_deg * Math.PI / 180
  const forearm = geometry.elbow_to_gripper_mount + geometry.gripper_mount_to_tip
  const radius = geometry.shoulder_to_elbow_axis * Math.cos(theta) + forearm * Math.cos(theta + phi)
  return { x: radius * Math.cos(yaw), y: radius * Math.sin(yaw), z: geometry.table_to_shoulder_axis + geometry.shoulder_to_elbow_axis * Math.sin(theta) + forearm * Math.sin(theta + phi) }
}
export function poseAt(frames: RecordingFrame[], time: number): { pose: Pose | null; gap: boolean; frame: RecordingFrame | null } {
  if (!frames.length) return { pose: null, gap: false, frame: null }
  let lo = 0, hi = frames.length
  while (lo < hi) { const mid = (lo + hi) >> 1; if (frames[mid]!.t_ms <= time) lo = mid + 1; else hi = mid }
  const left = frames[Math.max(0, lo - 1)]!
  const right = frames[Math.min(lo, frames.length - 1)]!
  if (right.t_ms - left.t_ms > 250 && time > left.t_ms && time < right.t_ms) return { pose: left.output_target, gap: true, frame: left }
  const ratio = right.t_ms === left.t_ms ? 0 : Math.max(0, Math.min(1, (time - left.t_ms) / (right.t_ms - left.t_ms)))
  const pose = { ...left.output_target }
  for (const key of Object.keys(pose) as (keyof Pose)[]) pose[key] += (right.output_target[key] - pose[key]) * ratio
  return { pose, gap: false, frame: left }
}
