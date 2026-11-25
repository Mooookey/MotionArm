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
export interface PoweredOutput {
  profile_hash: string
  stage: 'locked' | 'holding' | 'unknown' | 'disabled'
  pwm_ticks: [number, number, number, number] | null
  last_write_ms: number | null
  write_errors: number
  pwm_enabled: boolean | null
  committed: boolean
  trial_limits_deg: [number, number][]
  max_velocity_deg_s: number
  max_acceleration_deg_s2: number
  power_confirmed: boolean
  prepared: boolean
  diagnostic_active: boolean
}
export interface AxisAcceptance { axis: string; passed: boolean; note: string; [key: string]: unknown }
export interface Telemetry {
  schema_version: 5
  source: 'wearable'
  config_id?: string
  server_time_ms: number
  input: {
    kind: string; status: string; port: string | null; boot_id: string | number | null
    age_ms: number | null; frames: number; parse_errors: number; detail: string | null
    transport_age_ms?: number | null
    wireless_fresh?: boolean
    attitude_representation: 'quaternion_sensor_to_world_v3'
  }
  executor: { kind: 'virtual' | 'robot_no_power' | 'robot_powered'; label: string; mode: string; identity: string | null; output_locked: boolean; connected: boolean; port?: string | null; age_ms?: number | null; boot_id?: string | number | null; profile_hash?: string | null; protocol_version?: number; output?: PoweredOutput | null; acceptance?: AxisAcceptance[] }
  output?: PoweredOutput | null
  health?: { wireless?: { status: string; received: number; rejected: number; duplicates: number; out_of_order: number }; pca_readable?: boolean; [key: string]: unknown } | null
  gripper: GripperState | null
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
    requested: Pose | null; output_target: Pose | null; measured: null
    last_committed_target?: Pose | null
    limits: boolean[]; velocity_deg_s: number[]
  }
  recording: { active: boolean; record_id: string | null; frames: number }
  replay: { active: boolean; record_id: string | null; progress: number; geometry?: Geometry }
  events: Array<{ time_ms: number; code: string; message: string }>
}
export interface GripperState {
  state: string
  requested_open_ratio: number
  output_open_ratio: number | null
  last_committed_open_ratio?: number | null
  limit_ms: 5000
  elapsed_ms: number
  remaining_ms: number | null
  cycle_id: number
  auto_open_reason: string | null
  reclose_blocked: boolean
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
