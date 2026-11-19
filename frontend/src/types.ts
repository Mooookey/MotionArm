export interface Pose {
  base_yaw_deg: number
  shoulder_down_deg: number
  elbow_down_deg: number
  gripper_open_ratio: number
}
export interface Geometry {
  base_width: number
  table_to_shoulder_axis: number
  shoulder_to_elbow_axis: number
  elbow_to_gripper_mount: number
  gripper_mount_to_tip: number
}
export interface SimulationConfig {
  deadzone_deg: number
  limits_deg: [number, number][]
  max_velocity_deg_s: number[]
  max_acceleration_deg_s2: number[]
}
export type Quaternion = [number, number, number, number]
export type Vector3 = [number, number, number]
export interface Mounting {
  profile: 'hand_back_x_right_y_forearm_z_down'
  label: string
  control_from_sensor: { Roll: '-Y'; Pitch: '+X'; Z: '+Z' }
  joint_signs: [1, -1]
}
export interface ModelConfig {
  config: SimulationConfig
  config_id: string
  geometry: Geometry
  baseline_id: string
  mounting: Mounting
  warning: string | null
}
export interface ImuState {
  quaternion: Quaternion | null
  gyro_dps: Vector3 | null
  accel_g: Vector3 | null
  roll_deg: number | null; pitch_deg: number | null; valid: boolean
  age_ms: number | null; calibration_samples: number; who: number | null
  norm_g: number | null; temp_c: number | null; errors: number
  gyro_range_dps: number | null; reject_reason: number; rejected_samples: number
}
export interface ButtonState { a: boolean; b: boolean; ac: number | null; bc: number | null }
export interface FollowState {
  version: 'coordinated-angle-v3'
  reference_quaternion: Quaternion | null
  delta_rotation_vector_deg: Vector3 | null
  deadzone_deg: number
  limit_policy: 'FIXED_REFERENCE_HARD_CLAMP'
  angle_ambiguous: boolean
  pitch_weights: [number, number]
  pitch_limits_deg: [number, number]
}
export interface Telemetry {
  schema_version: 3
  source: 'wearable'
  config_id?: string
  server_time_ms: number
  input: {
    kind: string; status: string; port: string | null; boot_id: string | number | null
    age_ms: number | null; frames: number; parse_errors: number; detail: string | null
    attitude_representation: 'quaternion_sensor_to_world_v3'
  }
  executor: { kind: string; label: string }
  imu: ImuState
  buttons: ButtonState
  control: {
    state: string; owner: string; enabled: boolean; armed: boolean
    can_enable: boolean; reason: string | null; fault: string | null
    blockers: string[]; gate_status: string
    mapping: string
    limit_policy: string
    follow: FollowState
  }
  joint_state: {
    requested: Pose; output_target: Pose; measured: null
    limits: boolean[]; velocity_deg_s: number[]
  }
  recording: { active: boolean; record_id: string | null; frames: number }
  replay: { active: boolean; record_id: string | null; progress: number; geometry?: Geometry }
  events: Array<{ time_ms: number; code: string; message: string }>
}
export interface RecordingFrame {
  t_ms: number
  requested: Pose
  output_target: Pose
  // Only stored history may have an older, partial IMU/button payload.
  imu: Partial<ImuState>
  buttons: Partial<ButtonState>
  owner: string
}
export interface Recording {
  id: string; name: string; status: string; created_ms: number
  frame_count: number; duration_ms: number; note: string | null
  source: string; metadata?: Record<string, unknown> | null
}
export interface RecordingDetail extends Recording {
  frames: RecordingFrame[]
  events: Array<Record<string, unknown>>
}
export interface Point {
  time: number; roll: number | null; pitch: number | null
  requested: Pose | null; output: Pose | null
}
export interface Preflight { ok: boolean; violations: string[]; preflight_id: string | null }
