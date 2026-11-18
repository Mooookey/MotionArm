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
  full_scale_deg: number
  limits_deg: [number, number][]
  max_velocity_deg_s: number[]
  max_acceleration_deg_s2: number[]
  roll_sign: number
  pitch_sign: number
  neutral_deg: number[]
}
export interface ModelConfig {
  config: SimulationConfig
  config_id: string
  geometry: Geometry
  baseline_id: string
}
export interface Telemetry {
  schema_version: 2
  source: 'wearable'
  config_id?: string
  server_time_ms: number
  input: {
    kind: string; status: string; port: string | null; boot_id: string | number | null
    age_ms: number | null; frames: number; parse_errors: number; detail: string | null
  }
  executor: { kind: string; label: string }
  imu: {
    roll_deg: number | null; pitch_deg: number | null; valid: boolean
    age_ms: number | null; calibration_samples: number; who: number | null
    norm_g: number | null; temp_c: number | null; errors: number
    raw_roll_deg?: number | null; raw_pitch_deg?: number | null; neutral_deg?: number[]
  }
  buttons: { a: boolean; b: boolean; short: number; long: number; ac?: number; bc?: number }
  control: {
    state: string; owner: string; enabled: boolean; armed: boolean
    can_enable: boolean; reason: string | null; mode: string; fault: string | null
    neutral_ready?: boolean; neutral_progress?: number; blockers?: string[]; gate_status?: string
    mapping?: string
    limit_policy?: string
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
  imu: { roll_deg: number | null; pitch_deg: number | null; valid?: boolean }
  buttons: { a: boolean; b: boolean; short?: number; long?: number; ac?: number; bc?: number }
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
