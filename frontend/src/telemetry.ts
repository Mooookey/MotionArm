import type { ModelConfig, Pose, Telemetry } from './types'

const finite = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value)
const nullableNumber = (value: unknown) => value === null || finite(value)
const counter = (value: unknown) => value === null || (finite(value) && Number.isInteger(value) && value >= 0 && value <= 0xffffffff)
const vector = (value: unknown, length: number): value is number[] => Array.isArray(value) && value.length === length && value.every(finite)
const nullableVector = (value: unknown, length: number) => value === null || vector(value, length)
const quaternion = (value: unknown) => value === null || (vector(value, 4) && Math.abs(Math.hypot(...value) - 1) <= 0.01)
const poseKeys = ['base_yaw_deg', 'shoulder_down_deg', 'elbow_down_deg', 'gripper_open_ratio'] as const

export function isPose(value: unknown): value is Pose {
  return typeof value === 'object' && value !== null && poseKeys.every((key) => finite((value as Pose)[key])) &&
    (value as Pose).gripper_open_ratio >= 0 && (value as Pose).gripper_open_ratio <= 1
}

/** Live telemetry only accepts the current contract; history has its own boundary. */
export function validFrame(value: unknown): value is Telemetry {
  if (typeof value !== 'object' || !value) return false
  const t = value as Telemetry
  return t.schema_version === 3 && t.source === 'wearable' && finite(t.server_time_ms) &&
    t.input?.kind === 'imu_usb' && typeof t.input.status === 'string' &&
    t.input.attitude_representation === 'quaternion_sensor_to_world_v3' && t.executor?.kind === 'virtual' &&
    typeof t.imu?.valid === 'boolean' && quaternion(t.imu.quaternion) &&
    nullableVector(t.imu.gyro_dps, 3) && nullableVector(t.imu.accel_g, 3) &&
    nullableNumber(t.imu.roll_deg) && nullableNumber(t.imu.pitch_deg) &&
    typeof t.buttons?.a === 'boolean' && typeof t.buttons.b === 'boolean' && counter(t.buttons.ac) && counter(t.buttons.bc) &&
    typeof t.control?.enabled === 'boolean' && typeof t.control.armed === 'boolean' &&
    typeof t.control.can_enable === 'boolean' && typeof t.control.gate_status === 'string' &&
    Array.isArray(t.control.blockers) && t.control.blockers.every((item) => typeof item === 'string') &&
    t.control.follow?.version === 'coordinated-angle-v3' && quaternion(t.control.follow.reference_quaternion) &&
    nullableVector(t.control.follow.delta_rotation_vector_deg, 3) &&
    isPose(t.joint_state?.requested) && isPose(t.joint_state?.output_target) && t.joint_state.measured === null &&
    typeof t.recording?.active === 'boolean' && typeof t.replay?.active === 'boolean' && Array.isArray(t.events)
}

export function validModelConfig(value: unknown): value is ModelConfig {
  if (typeof value !== 'object' || !value) return false
  const m = value as ModelConfig
  const c = m.config
  return !!c && Object.keys(c).sort().join(',') === 'deadzone_deg,limits_deg,max_acceleration_deg_s2,max_velocity_deg_s' &&
    finite(c.deadzone_deg) && c.deadzone_deg > 0 && c.deadzone_deg <= 20 &&
    Array.isArray(c.limits_deg) && c.limits_deg.length === 3 && c.limits_deg.every((row) => vector(row, 2) && row[0]! < row[1]!) &&
    vector(c.max_velocity_deg_s, 3) && vector(c.max_acceleration_deg_s2, 3) &&
    typeof m.config_id === 'string' && typeof m.baseline_id === 'string' &&
    !!m.geometry && Object.values(m.geometry).length === 5 && Object.values(m.geometry).every((n) => finite(n) && n > 0) &&
    m.mounting?.profile === 'hand_back_x_right_y_forearm_z_down' &&
    m.mounting.control_from_sensor?.Roll === '-Y' && m.mounting.control_from_sensor.Pitch === '+X' &&
    m.mounting.control_from_sensor.Z === '+Z' && vector(m.mounting.joint_signs, 2) &&
    m.mounting.joint_signs[0] === 1 && m.mounting.joint_signs[1] === -1
}
