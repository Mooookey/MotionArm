import assert from 'node:assert/strict'
import { test } from 'node:test'
import { fallbackGeometry, referencePose, tipPosition, poseAt } from '../src/geometry.ts'
import { nextAccelerationWarning, sceneResizeAction } from '../src/displayStability.ts'
import { buttonTelemetryFresh, coordinatedFollowLabel } from '../src/controlPresentation.ts'
import { diagnosticOffsets, poweredConfigMatches, validFrame, validModelConfig } from '../src/telemetry.ts'
import { ManagementLease } from '../src/management.ts'
import { poweredBinding, directObservationRequest, outputStageLabel, canViewHistory, recordedLimits, recordedMapping, modelMapping, mappingSignsLabel, rightTiltLabel } from '../src/powered.ts'

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

test('robot button levels depend on fresh wireless receipt rather than IMU validity', () => {
  assert.equal(buttonTelemetryFresh(true, 'esp_now', 'invalid', 10, true), true)
  assert.equal(buttonTelemetryFresh(true, 'esp_now', 'calibrating', 10, true), true)
  assert.equal(buttonTelemetryFresh(true, 'esp_now', 'valid', 10, false), false)
  assert.equal(buttonTelemetryFresh(true, 'esp_now', 'valid', 10), false)
  assert.equal(buttonTelemetryFresh(false, 'esp_now', 'invalid', 10, true), false)
  assert.equal(buttonTelemetryFresh(true, 'imu_usb', 'invalid', 10), true)
  assert.equal(buttonTelemetryFresh(true, 'imu_usb', 'valid', 250), false)
})

const liveFrame = () => ({
  schema_version: 5, source: 'wearable', server_time_ms: 1000,
  input: { kind: 'imu_usb', status: 'valid', attitude_representation: 'quaternion_sensor_to_world_v3' },
  executor: { kind: 'virtual', connected: true, output_locked: true },
  gripper: { state: 'OPEN_READY', requested_open_ratio: 1, output_open_ratio: 1, limit_ms: 5000, elapsed_ms: 0, remaining_ms: null, cycle_id: 0, auto_open_reason: null, reclose_blocked: false },
  imu: { valid: true, quaternion: [1, 0, 0, 0], gyro_dps: [0, 0, 0], accel_g: [0, 0, 1], roll_deg: null, pitch_deg: null },
  buttons: { a: false, b: false, ac: 0, bc: 0 },
  control: { enabled: false, armed: false, can_enable: false, gate_status: 'DISABLED', blockers: [],
    follow: { version: 'coordinated-angle-v3', reference_quaternion: null, delta_rotation_vector_deg: null } },
  joint_state: { requested: { ...referencePose }, output_target: { ...referencePose }, measured: null },
  recording: { active: false }, replay: { active: false }, events: [],
})
test('live telemetry requires V5 and rejects old or incomplete quaternion diagnostics', () => {
  assert.equal(validFrame(liveFrame()), true)
  for (const change of [
    (f) => { f.schema_version = 3 },
    (f) => { f.schema_version = 4 },
    (f) => { delete f.imu.gyro_dps },
    (f) => { f.imu.accel_g = [0, 1] },
    (f) => { f.imu.quaternion = [0, 0, 0, 0] },
    (f) => { f.imu.quaternion = [Infinity, 0, 0, 0] },
    (f) => { delete f.control.can_enable },
    (f) => { delete f.buttons.bc },
    (f) => { delete f.gripper },
    (f) => { f.gripper.limit_ms = 700 },
    (f) => { f.gripper.remaining_ms = 5001 },
    (f) => { f.input.wireless_fresh = 'true' },
  ]) {
    const value = liveFrame(); change(value)
    assert.equal(validFrame(value), false)
  }
})

test('robot targets only accept ESP-NOW source and a locked hardware output', () => {
  const value = liveFrame()
  value.executor.kind = 'robot_no_power'
  value.input.kind = 'esp_now'
  value.input.age_ms = 12
  value.input.transport_age_ms = null
  value.input.wireless_fresh = true
  value.executor.age_ms = 180
  assert.equal(validFrame(value), true)
  value.executor.age_ms = -1
  assert.equal(validFrame(value), false)
  value.executor.age_ms = 180
  value.input.transport_age_ms = NaN
  assert.equal(validFrame(value), false)
  value.input.transport_age_ms = null
  value.joint_state.requested = value.joint_state.output_target = null
  value.gripper = null
  assert.equal(validFrame(value), true)
  value.executor.output_locked = false
  assert.equal(validFrame(value), false)
  value.executor.output_locked = true
  value.input.kind = 'imu_usb'
  assert.equal(validFrame(value), false)
  value.input.kind = 'esp_now'
  value.joint_state.measured = { ...referencePose }
  assert.equal(validFrame(value), false)
})

test('page management lease requires explicit grant and never survives reload or loss', () => {
  const lease = new ManagementLease()
  lease.observe(true, true)
  assert.equal(lease.payload(), null)
  assert.equal(lease.message(), null)
  lease.grant('explicit-token')
  lease.observe(true, false) // Previous telemetry can arrive before initialization state.
  assert.deepEqual(lease.payload(), { manager_token: 'explicit-token' })
  assert.deepEqual(lease.message(), { type: 'heartbeat', manager_token: 'explicit-token' })
  lease.observe(true, true)
  lease.observe(true, false)
  assert.equal(lease.payload(), null)
  lease.grant('second-token')
  lease.observe(false, true)
  assert.equal(lease.payload(), null)
  assert.equal(lease.message(), null)
  assert.equal(new ManagementLease().payload(), null)
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

const poweredFrame = () => {
  const value = liveFrame()
  value.input.kind = 'esp_now'
  value.executor = { kind: 'robot_powered', connected: true, output_locked: false, boot_id: 'robot-boot', protocol_version: 3, profile_hash: 'sha256:' + 'a'.repeat(64) }
  value.output = { profile_hash: 'sha256:' + 'a'.repeat(64), stage: 'holding', pwm_ticks: [330, 211, 215, 232], last_write_ms: 100, write_errors: 0, pwm_enabled: true, committed: true, trial_limits_deg: [[-45, 45], [20, 60], [20, 60]], max_velocity_deg_s: 15, max_acceleration_deg_s2: 30, power_confirmed: true, prepared: true, diagnostic_active: false }
  value.config_id = value.output.profile_hash
  value.control.follow.deadzone_deg = 5
  value.control.follow.version = 'coordinated-angle-v4'
  value.control.follow.joint_signs = [-1, -1]
  value.control.observation_ready = false
  return value
}
test('powered telemetry keeps PWM holding distinct from locks and physical feedback', () => {
  assert.equal(validFrame(poweredFrame()), true)
  for (const change of [
    (f) => { f.executor.output_locked = true },
    (f) => { f.executor.protocol_version = 2 },
    (f) => { delete f.control.observation_ready },
    (f) => { f.control.direct_observation = { note: '观察', unaccepted_axes: null } },
    (f) => { f.executor.profile_hash = 'different' },
    (f) => { f.output.pwm_ticks[0] = 4096 },
    (f) => { f.output.stage = 'holding'; f.output.committed = false },
    (f) => { f.output.stage = 'unknown'; f.output.pwm_enabled = null }, // Unknown must not publish a current accepted output target.
    (f) => { f.joint_state.measured = { ...referencePose } },
  ]) {
    const frame = poweredFrame(); change(frame)
    assert.equal(validFrame(frame), false)
  }
  for (const committed of [true, false]) {
    const failed = poweredFrame()
    failed.output.stage = 'unknown'; failed.output.pwm_enabled = null; failed.output.committed = committed
    failed.joint_state.last_committed_target = committed ? failed.joint_state.output_target : null
    failed.joint_state.output_target = null
    failed.gripper.output_open_ratio = null
    failed.gripper.last_committed_open_ratio = committed ? 1 : null
    assert.equal(validFrame(failed), true)
    failed.gripper.output_open_ratio = 1
    assert.equal(validFrame(failed), false)
    failed.gripper.output_open_ratio = null
    failed.output.pwm_enabled = false
    assert.equal(validFrame(failed), false)
  }
  const disabled = poweredFrame()
  disabled.output.stage = 'disabled'; disabled.output.pwm_enabled = false
  disabled.executor.output_locked = true
  disabled.joint_state.last_committed_target = disabled.joint_state.output_target
  disabled.joint_state.output_target = null
  disabled.gripper.output_open_ratio = null
  assert.equal(validFrame(disabled), true)
  assert.notEqual(disabled.joint_state.last_committed_target, null)
  const offline = poweredFrame()
  offline.executor.connected = false
  offline.joint_state.last_committed_target = offline.joint_state.output_target
  offline.joint_state.output_target = null
  offline.gripper.output_open_ratio = null
  assert.equal(validFrame(offline), true)
})
test('powered management binds current board and config without granting a lease', () => {
  const state = poweredFrame()
  assert.deepEqual(poweredBinding(state, null), { boot_id: 'robot-boot', profile_hash: 'sha256:' + 'a'.repeat(64) })
  assert.deepEqual(poweredBinding(state, { manager_token: 'explicit' }), { boot_id: 'robot-boot', profile_hash: 'sha256:' + 'a'.repeat(64), manager_token: 'explicit' })
  state.executor.connected = false
  assert.throws(() => poweredBinding(state, null))
  assert.throws(() => poweredBinding(liveFrame(), null))
})
test('physical history viewer requires stop and disconnect, and labels never claim measured position', () => {
  const state = poweredFrame()
  assert.equal(canViewHistory(state), false)
  state.executor.connected = false; state.control.enabled = true
  assert.equal(canViewHistory(state), false)
  state.control.enabled = false
  assert.equal(canViewHistory(state), true)
  assert.equal(canViewHistory(liveFrame()), true)
  assert.match(outputStageLabel('holding'), /PWM 输出保持/)
  assert.match(outputStageLabel('unknown'), /未知/)
  assert.match(outputStageLabel('disabled'), /释放/)
})
const observationModel = () => ({
  config: { deadzone_deg: 5, limits_deg: [[-45, 45], [20, 60], [20, 60]], max_velocity_deg_s: [15, 15, 15], max_acceleration_deg_s2: [30, 30, 30] },
  config_id: 'sha256:' + 'a'.repeat(64), profile_hash: 'sha256:' + 'a'.repeat(64), mapping_version: 'coordinated-angle-v4', joint_signs: [-1, -1], baseline_id: 'baseline', geometry: fallbackGeometry,
  mounting: { profile: 'hand_back_x_right_y_forearm_z_down', control_from_sensor: { Roll: '-Y', Pitch: '+X', Z: '+Z' }, joint_signs: [-1, -1] }, warning: null,
  diagnostic_offsets_deg: [[-15, -10, -5, 0, 5, 10, 15], [-10, -5, 0, 5, 10], [-10, -5, 0, 5, 10]],
  trial_profile: { deadzone_deg: 5, operating_limits_deg: [[-45, 45], [20, 60], [20, 60]], speed_deg_s: 15, acceleration_deg_s2: 30, joint_signs: [-1, -1], mapping_version: 'coordinated-angle-v4',
    diagnostic_offsets_deg: [[-15, -10, -5, 0, 5, 10, 15], [-10, -5, 0, 5, 10], [-10, -5, 0, 5, 10]] },
})
test('powered control requires approved read-only API configuration and matching frame/hash', () => {
  const model = observationModel(), frame = poweredFrame()
  assert.equal(validModelConfig(model), true)
  assert.equal(poweredConfigMatches(frame, model), true)
  assert.equal(poweredConfigMatches(frame, null), false)
  for (const change of [
    (f) => { f.output.trial_limits_deg[0] = [-50, 50] },
    (f) => { f.output.trial_limits_deg[1] = [30, 60] },
    (f) => { f.output.max_velocity_deg_s = 20 },
    (f) => { f.output.max_acceleration_deg_s2 = 60 },
    (f) => { f.control.follow.deadzone_deg = 4 },
    (f) => { f.control.follow.version = 'coordinated-angle-v3' },
    (f) => { f.control.follow.joint_signs = [1, -1] },
    (f) => { f.executor.profile_hash = 'sha256:' + 'b'.repeat(64); f.output.profile_hash = f.executor.profile_hash },
    (f) => { f.config_id = 'another-config' },
  ]) {
    const value = poweredFrame(); change(value)
    assert.equal(poweredConfigMatches(value, model), false)
  }
  const widened = observationModel()
  widened.config.limits_deg[0] = widened.trial_profile.operating_limits_deg[0] = [-50, 50]
  assert.equal(validModelConfig(widened), false)
  assert.equal(poweredConfigMatches(frame, widened), false)
  const previous = observationModel()
  previous.config.limits_deg = previous.trial_profile.operating_limits_deg = [[-3, 3], [42, 48], [42, 48]]
  assert.equal(validModelConfig(previous), false) // Live profile was superseded; history stays viewable.
  const slower = observationModel()
  slower.config.max_velocity_deg_s = [10, 10, 10]; slower.config.max_acceleration_deg_s2 = [30, 30, 30]
  slower.trial_profile.speed_deg_s = 10; slower.trial_profile.acceleration_deg_s2 = 30
  assert.equal(validModelConfig(slower), false)
  const wrongSign = observationModel(); wrongSign.mounting.joint_signs = [1, -1]
  assert.equal(validModelConfig(wrongSign), false)
  const wrongVersion = observationModel(); wrongVersion.trial_profile.mapping_version = 'coordinated-angle-v3'
  assert.equal(validModelConfig(wrongVersion), false)
})
test('live virtual and no-power keep V3 while powered requires V4 reversed-base mapping', () => {
  const virtual = liveFrame()
  assert.equal(validFrame(virtual), true)
  virtual.control.follow.version = 'coordinated-angle-v4'
  assert.equal(validFrame(virtual), false)
  const noPower = liveFrame(); noPower.executor.kind = 'robot_no_power'; noPower.input.kind = 'esp_now'
  assert.equal(validFrame(noPower), true)
  noPower.control.follow.joint_signs = [-1, -1]
  assert.equal(validFrame(noPower), false)
  const powered = poweredFrame()
  assert.equal(validFrame(powered), true)
  powered.control.follow.version = 'coordinated-angle-v3'
  assert.equal(validFrame(powered), false)
})
test('direction labels follow current mode and saved history preserves its own mapping', () => {
  const physical = modelMapping(observationModel(), true)
  assert.deepEqual(physical, { signs: [-1, -1], version: 'coordinated-angle-v4' })
  assert.match(rightTiltLabel(physical.signs), /逆时针/)
  assert.equal(mappingSignsLabel(physical.signs), '底座 −1 / 双臂 −1')
  const old = { controller_version: 'coordinated-angle-v3', mounting: { joint_signs: [1, -1] }, config: { limits_deg: [[-20, 20], [35, 55], [35, 55]] } }
  const original = JSON.stringify(old)
  assert.deepEqual(recordedMapping(old), { signs: [1, -1], version: 'coordinated-angle-v3' })
  assert.match(rightTiltLabel(recordedMapping(old).signs), /顺时针/)
  assert.deepEqual(recordedLimits(old), [[-20, 20], [35, 55], [35, 55]])
  const current = { controller_version: 'coordinated-angle-v4', joint_signs: [-1, -1], mounting: { joint_signs: [-1, -1] } }
  assert.deepEqual(recordedMapping(current), physical)
  assert.equal(JSON.stringify(old), original)
  assert.deepEqual(recordedMapping({}), { signs: null, version: null })
  assert.deepEqual(modelMapping(observationModel(), false), { signs: null, version: null }) // Stale config from another mode.
  assert.match(mappingSignsLabel(null), /未记录|未核对/)
})
test('direct observation is explicit, stays paused, and never creates axis acceptance', () => {
  const state = poweredFrame()
  state.control.observation_ready = true
  state.control.can_enable = false // Only the normal axis-acceptance gate may be bypassed.
  state.executor.acceptance = []
  const request = directObservationRequest(state, { manager_token: 'current-manager' })
  assert.equal(request.direct_observation, true)
  assert.equal(request.boot_id, state.executor.boot_id)
  assert.equal(request.profile_hash, state.executor.profile_hash)
  assert.equal(request.manager_token, 'current-manager')
  assert.match(request.note, /省略剩余逐轴诊断/)
  assert.ok(request.note.length > 0 && request.note.length <= 500)
  assert.equal('passed' in request, false)
  assert.equal('armed' in request, false)
  assert.equal(state.control.enabled, false)
  assert.equal(state.control.armed, false)
  assert.deepEqual(state.executor.acceptance, [])
})
test('direct observation preserves all other local initialization blockers', () => {
  for (const change of [
    (f) => { f.control.observation_ready = false },
    (f) => { f.control.enabled = true },
    (f) => { f.control.armed = true },
    (f) => { f.executor.kind = 'virtual' },
    (f) => { f.executor.connected = false },
    (f) => { f.output.prepared = false },
    (f) => { f.output.committed = false },
    (f) => { f.output.diagnostic_active = true },
    (f) => { f.control.fault = 'PCA_OUTPUT_UNKNOWN' },
    (f) => { f.output.stage = 'unknown' },
    (f) => { f.output.pwm_enabled = false },
    (f) => { f.input.status = 'stale' },
    (f) => { f.imu.valid = false },
    (f) => { f.recording.active = true },
    (f) => { f.replay.active = true },
  ]) {
    const state = poweredFrame(); state.control.observation_ready = true; change(state)
    assert.throws(() => directObservationRequest(state, { manager_token: 'current' }))
  }
  const state = poweredFrame(); state.control.observation_ready = true
  assert.throws(() => directObservationRequest(state, null))
})
test('diagnostic steps use unchanged per-axis subsets and exclude extra range steps', () => {
  const model = observationModel()
  assert.deepEqual(diagnosticOffsets(model, 'base'), [-15, -10, -5, 0, 5, 10, 15])
  assert.equal(diagnosticOffsets(model, 'base').includes(20), false)
  assert.equal(diagnosticOffsets(model, 'base').includes(45), false)
  assert.deepEqual(diagnosticOffsets(model, 'shoulder'), [-10, -5, 0, 5, 10])
  assert.equal(diagnosticOffsets(model, 'elbow').includes(15), false)
  assert.deepEqual(diagnosticOffsets(null, 'base'), [])
  assert.deepEqual(diagnosticOffsets(model, 'unknown'), [])
})
test('old and expanded schema5 history retains its own range and stored target semantics', () => {
  const old = { schema_version: 5, config: { limits_deg: [[-3, 3], [42, 48], [42, 48]] } }
  const current = { schema_version: 5, config: observationModel().config }
  assert.deepEqual(recordedLimits(old), [[-3, 3], [42, 48], [42, 48]])
  assert.deepEqual(recordedLimits(current), [[-45, 45], [20, 60], [20, 60]])
  assert.equal(recordedLimits({ config: { limits_deg: [[-3, 3]] } }), null)
  assert.deepEqual(old.config.limits_deg, [[-3, 3], [42, 48], [42, 48]])
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
