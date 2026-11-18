import assert from 'node:assert/strict'
import { test } from 'node:test'
import { fallbackGeometry, referencePose, tipPosition, poseAt } from '../src/geometry.ts'
import { nextAccelerationWarning, sceneResizeAction } from '../src/displayStability.ts'
import { coordinatedFollowLabel, hasButtonClickCounters } from '../src/controlPresentation.ts'

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

test('initialization only accepts firmware with two valid click counters, including zero and wrap endpoints', () => {
  assert.equal(hasButtonClickCounters({ ac: 0, bc: 0 }), true)
  assert.equal(hasButtonClickCounters({ ac: 0xffffffff, bc: 1 }), true)
  for (const buttons of [undefined, null, {}, { ac: 1 }, { ac: true, bc: 0 }, { ac: -1, bc: 0 }, { ac: 1.5, bc: 0 }, { ac: 0, bc: 0x100000000 }]) {
    assert.equal(hasButtonClickCounters(buttons), false)
  }
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
