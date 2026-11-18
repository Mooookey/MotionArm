export type JointVector = [number, number, number, number]

export interface Telemetry {
  event: 'telemetry'
  server_time_ms: number
  source: 'mock' | 'serial'
  read_only: boolean
  calibration_id: string | null
  connection: {
    robot: 'online' | 'diagnostic' | 'offline'
    wearable: 'online' | 'offline'
    serial_reconnecting: boolean
    pca_ok?: boolean | null
    detail?: string
  }
  control: {
    state: string
    owner: string
    deadman: boolean
    mode: string
    fault: string | null
    lease_id?: string | null
    has_lease?: boolean
    can_move?: boolean
  }
  imu: { roll_deg: number | null; pitch_deg: number | null; valid: boolean; age_ms: number | null }
  joints: {
    names: string[]
    requested_deg: JointVector | null
    applied_target_deg: JointVector | null
    measured_deg: null
  }
  recording: { active: boolean; record_id: string | null; frames: number }
  replay: { active: boolean; record_id: string | null; progress: number }
}

export interface Calibration {
  allow_arm: boolean
  calibration_id: string | null
  blocked: unknown
  source: 'mock' | 'serial'
  profile: {
    profile_name?: string
    verified_on_hardware?: boolean
    joints?: Array<{
      id: string
      channel: number
      logical_min_deg: number | null
      logical_max_deg: number | null
      pulse_min_us: number | null
      pulse_max_us: number | null
      max_velocity_deg_s: number | null
    }>
    workcell?: { workcell_id?: string | null }
  } | null
}

export interface Recording {
  id: string
  name: string
  status: string
  frame_count: number
  calibration_id: string | null
  created_ms: number
  duration_ms: number
}

export interface RecordingDetail extends Recording {
  frames: Array<{
    t_ms: number
    q_req: JointVector | null
    q_out: JointVector | null
    roll_deg: number | null
    pitch_deg: number | null
  }>
}

export interface Preflight {
  ok: boolean
  preflight_id?: string
  operator_checklist?: string[]
  speed_scale?: number
  violations?: string[]
  errors?: Array<string | { code?: string; message?: string }>
  warnings?: Array<string | { code?: string; message?: string }>
}

export interface Point {
  time: number
  roll: number | null
  pitch: number | null
  targets: JointVector | null
}
