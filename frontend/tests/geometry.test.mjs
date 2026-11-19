import assert from 'node:assert/strict'
import { test } from 'node:test'
import { fallbackGeometry, referencePose, tipPosition, poseAt } from '../src/geometry.ts'
import { nextAccelerationWarning, sceneResizeAction } from '../src/displayStability.ts'
import { coordinatedFollowLabel } from '../src/controlPresentation.ts'
import { validFrame, validModelConfig } from '../src/telemetry.ts'

test('upright assembly reference reaches 360mm without forward displacement', () => {
  const p = tipPosition({ ...referencePose, shoulder_down_deg: 0, elbow_down_deg: 0 }, fallbackGeometry)
  assert.ok(Math.abs(p.x) < 1e-9)
  assert.ok(Math.abs(p.y) < 1e-9)
  assert.equal(p.z, 360)
})
test('45 degree startup uses a relative elbow, with horizontal forearm', () => {
  const p = tipPosition(referencePose, fallbackGeometry)
  assert.ok(Math.abs(p.x - 236.3172798) < 1e-5)
  assert.ok(Math.abs(p.z - 171.3172798) < 1e-5)
})
test('base yaw rotates about Z without changing height', () => {
  const p = tipPosition({ ...referencePose, base_yaw_deg: 90 }, fallbackGeometry)
  assert.ok(Math.abs(p.x) < 1e-9)
  assert.ok(p.y > 236)
  assert.ok(Math.abs(p.z - tipPosition(referencePose, fallbackGeometry).z) < 1e-9)
})
const frame = (time, angle) => ({ t_ms: time, requested: { ...referencePose, base_yaw_deg: angle }, output_target: { ...referencePose, base_yaw_deg: angle }, imu: {}, buttons: {}, owner: 'WEARABLE' })
test('history interpolates valid sampling intervals without mutating recorded targets', () => {
  const frames = [frame(0, 0), frame(100, 10)]
  assert.equal(poseAt(frames, 50).pose.base_yaw_deg, 5)
  assert.equal(frames[0].output_target.base_yaw_deg, 0)
  assert.equal(poseAt(frames, 50).gap, false)
})
test('history never interpolates across a missing-data interval', () => {
  const result = poseAt([frame(0, 0), frame(500, 20)], 300)
  assert.equal(result.gap, true)
  assert.equal(result.pose.base_yaw_deg, 0)
})
test('history clamps before and after available data and handles empty input', () => {
  const frames = [frame(0, 0), frame(40, 8)]
  assert.equal(poseAt(frames, -20).pose.base_yaw_deg, 0)
  assert.equal(poseAt(frames, 100).pose.base_yaw_deg, 8)
  assert.equal(poseAt([], 10).pose, null)
})

test('diagnostic display does not flicker around the old 1.15g boundary', () => {
  let warning = false
  for (const norm of [1.15, 1.158, 1.149, 1.151, 1.16]) {
    warning = nextAccelerationWarning(warning, norm, true)
    assert.equal(warning, false)
  }
  warning = nextAccelerationWarning(warning, 1.19, true)
  assert.equal(warning, true)
  for (const norm of [1.16, 1.15, 1.121]) {
    warning = nextAccelerationWarning(warning, norm, true)
    assert.equal(warning, true)
  }
  assert.equal(nextAccelerationWarning(warning, 1.119, true), false)
})
test('diagnostic hysteresis applies below gravity and rejects missing data', () => {
  assert.equal(nextAccelerationWarning(false, 0.81, true), true)
  assert.equal(nextAccelerationWarning(true, 0.85, true), true)
  assert.equal(nextAccelerationWarning(true, 0.89, true), false)
  assert.equal(nextAccelerationWarning(true, NaN, true), false)
  assert.equal(nextAccelerationWarning(true, 0.7, false), false)
  assert.equal(nextAccelerationWarning(true, null, true), false)
})
const sceneSize = { width: 500, height: 600, viewportWidth: 1440, viewportHeight: 1000 }
test('initial scene fits once; repeated telemetry and no-op observations do not touch the camera', () => {
  assert.equal(sceneResizeAction(null, sceneSize), 'fit')
  assert.equal(sceneResizeAction(sceneSize, { ...sceneSize }), 'none')
  assert.equal(sceneResizeAction(sceneSize, { ...sceneSize, width: 500.2 }), 'none')
})
test('local warning/status layout changes resize rendering without refitting the camera', () => {
  const expanded = { ...sceneSize, height: 660 }
  assert.equal(sceneResizeAction(sceneSize, expanded), 'render')
  assert.equal(sceneResizeAction(expanded, sceneSize), 'render')
})
test('deliberate viewport resize can fit the model and zero-size observations are ignored', () => {
  assert.equal(sceneResizeAction(sceneSize, { ...sceneSize, width: 330, viewportWidth: 390 }), 'fit')
  assert.equal(sceneResizeAction(sceneSize, { ...sceneSize, height: 420, viewportHeight: 744 }), 'fit')
  assert.equal(sceneResizeAction(sceneSize, { ...sceneSize, width: 0 }), 'none')
})

test('running and paused labels remain latched when the physical A button is released', () => {
  const sequence = [
    { a: false, enabled: false, armed: false, label: '尚未启用' },
    { a: true, enabled: true, armed: true, label: '跟随中' },
    { a: false, enabled: true, armed: true, label: '跟随中' },
    { a: true, enabled: true, armed: false, label: '已暂停，短按 A 继续' },
    { a: false, enabled: true, armed: false, label: '已暂停，短按 A 继续' },
    { a: false, enabled: false, armed: true, label: '尚未启用' },
  ]
  for (const state of sequence) assert.equal(coordinatedFollowLabel(state.enabled, state.armed), state.label)
})

const liveFrame = () => ({
  schema_version: 3, source: 'wearable', server_time_ms: 1000,
  input: { kind: 'imu_usb', status: 'valid', attitude_representation: 'quaternion_sensor_to_world_v3' },
  executor: { kind: 'virtual' },
  imu: { valid: true, quaternion: [1, 0, 0, 0], gyro_dps: [0, 0, 0], accel_g: [0, 0, 1], roll_deg: null, pitch_deg: null },
  buttons: { a: false, b: false, ac: 0, bc: 0 },
  control: { enabled: false, armed: false, can_enable: false, gate_status: 'DISABLED', blockers: [],
    follow: { version: 'coordinated-angle-v3', reference_quaternion: null, delta_rotation_vector_deg: null } },
  joint_state: { requested: { ...referencePose }, output_target: { ...referencePose }, measured: null },
  recording: { active: false }, replay: { active: false }, events: [],
})
test('live telemetry requires V3 and rejects old or incomplete quaternion diagnostics', () => {
  assert.equal(validFrame(liveFrame()), true)
  for (const change of [
    (f) => { f.schema_version = 2 },
    (f) => { delete f.imu.gyro_dps },
    (f) => { f.imu.accel_g = [0, 1] },
    (f) => { f.imu.quaternion = [0, 0, 0, 0] },
    (f) => { f.imu.quaternion = [Infinity, 0, 0, 0] },
    (f) => { delete f.control.can_enable },
    (f) => { delete f.buttons.bc },
  ]) {
    const value = liveFrame(); change(value)
    assert.equal(validFrame(value), false)
  }
})
test('disconnected telemetry uses explicit null diagnostics and no fabricated relative angles', () => {
  const value = liveFrame()
  value.input.status = 'disconnected'; value.imu.valid = false
  value.imu.quaternion = value.imu.gyro_dps = value.imu.accel_g = null
  value.buttons.ac = value.buttons.bc = null
  assert.equal(validFrame(value), true)
  assert.equal(value.imu.roll_deg, null)
  assert.equal(value.control.follow.reference_quaternion, null)
})
test('current config rejects removed mounting/sign/neutral fields at the live boundary', () => {
  const model = {
    config: { deadzone_deg: 5, limits_deg: [[-45, 45], [0, 60], [0, 60]], max_velocity_deg_s: [20, 15, 15], max_acceleration_deg_s2: [60, 60, 60] },
    config_id: 'virtual:test', baseline_id: 'baseline', geometry: fallbackGeometry,
    mounting: { profile: 'hand_back_x_right_y_forearm_z_down', control_from_sensor: { Roll: '-Y', Pitch: '+X', Z: '+Z' }, joint_signs: [1, -1] }, warning: null,
  }
  assert.equal(validModelConfig(model), true)
  assert.equal(validModelConfig({ ...model, mounting: { ...model.mounting, joint_signs: [-1, -1] } }), false)
  for (const key of ['full_scale_deg', 'neutral_deg', 'mounting_profile', 'roll_sign', 'pitch_sign']) {
    assert.equal(validModelConfig({ ...model, config: { ...model.config, [key]: 1 } }), false)
  }
})
