import type { ModelConfig, Pose, Telemetry } from './types'

const finite = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value)
const nullableNumber = (value: unknown) => value === null || finite(value)
const optionalAge = (value: unknown) => value === undefined || value === null || finite(value) && value >= 0
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
  const virtual = t.executor?.kind === 'virtual' && t.input?.kind === 'imu_usb'
  const noPower = t.executor?.kind === 'robot_no_power' && t.input?.kind === 'esp_now' && t.executor.output_locked === true
  const powered = t.executor?.kind === 'robot_powered' && t.input?.kind === 'esp_now'
  const output = t.output ?? t.executor?.output
  const currentOutputAvailable = powered && t.executor.connected && output?.stage === 'holding'
  const outputValid = !powered || (output == null ? !t.executor.connected :
    t.executor.protocol_version === 3 && typeof t.executor.boot_id === 'string' && t.executor.boot_id.length > 0 &&
    typeof output.profile_hash === 'string' && output.profile_hash.length > 0 &&
    t.executor.profile_hash === output.profile_hash &&
    ['locked', 'holding', 'unknown', 'disabled'].includes(output.stage) &&
    (output.stage === 'unknown' ? output.pwm_enabled === null : typeof output.pwm_enabled === 'boolean') && typeof output.committed === 'boolean' &&
    !(output.pwm_enabled && t.executor.output_locked) &&
    !(output.stage === 'holding' && (!output.pwm_enabled || !output.committed)) &&
    !(['locked', 'disabled'].includes(output.stage) && output.pwm_enabled !== false) &&
    (output.pwm_ticks === null || vector(output.pwm_ticks, 4) && output.pwm_ticks.every((n) => Number.isInteger(n) && n >= 0 && n < 4096)) &&
    optionalAge(output.last_write_ms) && finite(output.write_errors) && output.write_errors >= 0 &&
    JSON.stringify(output.trial_limits_deg) === '[[-3,3],[42,48],[42,48]]' &&
    output.max_velocity_deg_s === 2 && output.max_acceleration_deg_s2 === 5 &&
    typeof output.power_confirmed === 'boolean' && typeof output.prepared === 'boolean' && typeof output.diagnostic_active === 'boolean')
  const robot = noPower || powered
  const nullablePose = (p: unknown) => isPose(p) || robot && p === null
  const g = t.gripper
  const gripperValid = g === null ? robot : !!g && typeof g.state === 'string' && g.limit_ms === 5000 &&
    finite(g.requested_open_ratio) && g.requested_open_ratio >= 0 && g.requested_open_ratio <= 1 &&
    (powered && !currentOutputAvailable ? g.output_open_ratio === null : finite(g.output_open_ratio) && g.output_open_ratio >= 0 && g.output_open_ratio <= 1) &&
    (g.last_committed_open_ratio === undefined || g.last_committed_open_ratio === null || finite(g.last_committed_open_ratio) && g.last_committed_open_ratio >= 0 && g.last_committed_open_ratio <= 1) &&
    finite(g.elapsed_ms) && g.elapsed_ms >= 0 && nullableNumber(g.remaining_ms) &&
    (g.remaining_ms === null || g.remaining_ms >= 0 && g.remaining_ms <= 5000) &&
    counter(g.cycle_id) && g.cycle_id !== null && typeof g.reclose_blocked === 'boolean' &&
    (g.auto_open_reason === null || typeof g.auto_open_reason === 'string')
  return t.schema_version === 5 && t.source === 'wearable' && finite(t.server_time_ms) &&
    (virtual || robot) && outputValid && typeof t.input.status === 'string' &&
    optionalAge(t.input.transport_age_ms) && optionalAge(t.executor.age_ms) &&
    (t.input.wireless_fresh === undefined || typeof t.input.wireless_fresh === 'boolean') &&
    t.input.attitude_representation === 'quaternion_sensor_to_world_v3' &&
    typeof t.executor.connected === 'boolean' && typeof t.executor.output_locked === 'boolean' && gripperValid &&
    typeof t.imu?.valid === 'boolean' && quaternion(t.imu.quaternion) &&
    nullableVector(t.imu.gyro_dps, 3) && nullableVector(t.imu.accel_g, 3) &&
    nullableNumber(t.imu.roll_deg) && nullableNumber(t.imu.pitch_deg) &&
    typeof t.buttons?.a === 'boolean' && typeof t.buttons.b === 'boolean' && counter(t.buttons.ac) && counter(t.buttons.bc) &&
    typeof t.control?.enabled === 'boolean' && typeof t.control.armed === 'boolean' &&
    typeof t.control.can_enable === 'boolean' && typeof t.control.gate_status === 'string' &&
    Array.isArray(t.control.blockers) && t.control.blockers.every((item) => typeof item === 'string') &&
    t.control.follow?.version === 'coordinated-angle-v3' && quaternion(t.control.follow.reference_quaternion) &&
    nullableVector(t.control.follow.delta_rotation_vector_deg, 3) &&
    !!t.joint_state && nullablePose(t.joint_state.requested) && nullablePose(t.joint_state.output_target) && t.joint_state.measured === null &&
    (!powered || currentOutputAvailable || t.joint_state.output_target === null) &&
    (t.joint_state.last_committed_target === undefined || t.joint_state.last_committed_target === null || isPose(t.joint_state.last_committed_target)) &&
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
