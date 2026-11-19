"""Quaternion fixed-reference control, click handling and failure latching."""
from copy import deepcopy
import unittest

from app.attitude import distance_deg, multiply
from app.virtual import DEFAULT_CONFIG, LIMIT_POLICY, MAPPING_VERSION, VirtualController, forward_kinematics, pose
from test_wearable import frame, sensor_quaternion, wrist_quaternion


class Clock:
    def __init__(self):
        self.now = 10.

    def __call__(self):
        return self.now


class VirtualControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.controller = VirtualController(self.clock)
        self.current = frame()
        self.controller.receive(self.current)

    def advance(self, seconds=.02, **changes):
        self.clock.now += seconds
        self.current = {**self.current, "seq": (self.current["seq"] + 1) % 2**32,
                        "ms": (self.current["ms"] + round(seconds * 1000)) % 2**32, **changes}
        self.controller.receive(self.current)
        self.controller.step()

    def initialize(self, **changes):
        for _ in range(36):
            self.advance(a=0, b=0, **changes)
        self.controller.enable()
        self.assertTrue(self.controller.enabled)
        self.assertFalse(self.controller.armed)

    def click_a(self):
        self.advance(ac=(self.current["ac"] + 1) % 2**32, a=1)

    def click_b(self):
        self.advance(bc=(self.current["bc"] + 1) % 2**32, b=1)

    def start_follow(self):
        self.initialize()
        self.click_a()

    def assertVectorAlmostEqual(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=7)

    def test_uninitialized_angles_are_unavailable_and_input_must_be_valid(self):
        self.assertEqual(self.controller.angles(), [None, None])
        for changes in ({"cal": 249}, {"valid": 0}, {"who": 0}, {"age_ms": 250}):
            with self.subTest(changes=changes):
                c = VirtualController(self.clock)
                c.receive(frame(**changes))
                with self.assertRaises(ValueError):
                    c.enable()
        old = frame()
        del old["ac"]
        stamp = self.controller.received_at
        with self.assertRaises(ValueError):
            self.controller.receive(old)
        self.assertEqual(self.controller.received_at, stamp)

    def test_reference_is_session_state_and_fixed_across_pause_and_resume(self):
        reference = sensor_quaternion(77, -42, 24)
        self.initialize(q=reference)
        self.assertAlmostEqual(distance_deg(reference, self.controller.reference_quaternion), 0)
        self.assertEqual(set(self.controller.config), set(DEFAULT_CONFIG))
        self.click_a()
        self.advance(q=multiply(reference, wrist_quaternion(15, -15)))
        self.assertVectorAlmostEqual(self.controller.q_req, [10, 55, 55])
        self.click_a()
        self.advance(q=reference, a=0)
        self.click_a()
        self.assertVectorAlmostEqual(self.controller.q_req, [0, 45, 45])
        self.assertAlmostEqual(distance_deg(reference, self.controller.reference_quaternion), 0)

    def test_initialize_resets_targets_gripper_and_button_baseline(self):
        self.controller.q_out = [30, 20, 55]
        self.controller.open_req = self.controller.open_out = 0
        self.initialize(ac=1001, bc=502)
        self.assertEqual(self.controller.q_req, [0, 45, 45])
        self.assertEqual(self.controller.q_out, self.controller.q_req)
        self.assertEqual(self.controller.open_out, 1)
        self.advance()
        self.assertFalse(self.controller.armed)
        self.click_a()
        with self.assertRaises(ValueError):
            self.controller.enable()

    def test_stability_checks_z_rotation_and_three_axis_gyro_norm(self):
        for i in range(28):
            self.advance(q=sensor_quaternion(z=i))
        with self.assertRaises(ValueError):
            self.controller.enable()
        for _ in range(35):
            self.advance(q=[1, 0, 0, 0], gyro_dps=[3, 3, 3])
        with self.assertRaises(ValueError):
            self.controller.enable()
        self.initialize(gyro_dps=[0, 0, 0])

    def test_stability_requires_full_window_released_buttons_and_no_gap(self):
        for _ in range(20):
            self.advance(a=1)
        with self.assertRaises(ValueError):
            self.controller.enable()
        for _ in range(24):
            self.advance(a=0)
        with self.assertRaises(ValueError):
            self.controller.enable()
        self.advance(seconds=.101)
        self.assertTrue(self.controller.enable_blockers())
        self.initialize()

    def test_reference_mean_accepts_sign_flips_and_uses_full_attitude(self):
        reference = sensor_quaternion(120, 40, -20)
        for i in range(36):
            self.advance(q=reference if i % 2 else [-v for v in reference])
        self.controller.enable()
        self.assertAlmostEqual(distance_deg(self.controller.reference_quaternion, reference), 0)
        self.assertVectorAlmostEqual(self.controller.angles(), [0, 0])

    def test_a_toggles_once_without_level_or_release_retrigger(self):
        self.initialize()
        self.click_a()
        for _ in range(30):
            self.advance(a=1, q=wrist_quaternion(-15))
        self.advance(a=0)
        self.assertTrue(self.controller.armed)
        self.click_a()
        paused = self.controller.q_out[:]
        for _ in range(30):
            self.advance(a=1, q=wrist_quaternion(30))
        self.assertEqual(self.controller.q_out, paused)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.gate_status()[0], "READY")

    def test_raw_button_levels_do_not_change_follow_or_gripper(self):
        self.initialize()
        for _ in range(20):
            self.advance(a=1, b=1, q=wrist_quaternion(-30, -30))
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.q_out, [0, 45, 45])
        self.assertEqual(self.controller.open_req, 1)

    def test_resume_uses_fixed_reference_and_limited_catchup(self):
        self.start_follow()
        self.advance(q=wrist_quaternion(-15, -15))
        for _ in range(80):
            self.advance()
        self.assertVectorAlmostEqual(self.controller.q_out, [-10, 55, 55])
        reference = self.controller.reference_quaternion[:]
        self.click_a()
        paused = self.controller.q_out[:]
        self.advance(q=wrist_quaternion(15, 15))
        for _ in range(20):
            self.advance()
        self.assertEqual(self.controller.q_out, paused)
        self.click_a()
        self.assertVectorAlmostEqual(self.controller.q_req, [10, 35, 35])
        for index, cap in enumerate((20, 15, 15)):
            self.assertLessEqual(abs(self.controller.q_out[index] - paused[index]), cap*.02 + 1e-9)
        self.assertEqual(self.controller.reference_quaternion, reference)

    def test_b_toggles_while_paused_and_gripper_persists_through_stop_or_fault(self):
        self.initialize()
        self.click_b()
        for _ in range(15):
            self.advance(b=1)
        self.assertEqual(self.controller.open_out, 0)
        self.click_a()
        self.click_a()
        self.assertEqual(self.controller.open_req, 0)
        self.controller.stop()
        self.advance(bc=self.current["bc"]+1)
        self.assertEqual(self.controller.open_out, 0)
        self.assertIsNone(self.controller.reference_quaternion)
        self.initialize()
        self.assertEqual(self.controller.open_out, 1)
        self.click_b()
        for _ in range(15):
            self.advance()
        self.advance(valid=0, rej=4)
        self.assertFalse(self.controller.enabled)
        self.assertEqual(self.controller.open_out, 0)
        self.assertIn("饱和", self.controller.reason)

    def test_soft_deadzone_fixed_tilt_settles_and_pure_z_does_not_drive(self):
        self.start_follow()
        self.advance(q=wrist_quaternion(5, -5))
        self.assertVectorAlmostEqual(self.controller.q_req, [0, 45, 45])
        self.advance(q=wrist_quaternion(-17.5, -12.5))
        self.assertVectorAlmostEqual(self.controller.q_req, [-12.5, 52.5, 52.5])
        for _ in range(250):
            self.advance()
        self.assertVectorAlmostEqual(self.controller.q_out, [-12.5, 52.5, 52.5])
        self.advance(q=wrist_quaternion(z=120))
        self.assertVectorAlmostEqual(self.controller.q_req, [0, 45, 45])
        self.assertFalse(self.controller.angle_ambiguous)

    def test_coupled_output_uses_slower_joint_speed_and_acceleration(self):
        self.controller.configure({"max_velocity_deg_s": [20, 8, 13], "max_acceleration_deg_s2": [60, 35, 12]})
        self.start_follow()
        previous = self.controller.q_out[:]
        previous_velocity = self.controller.velocity[:]
        for _ in range(240):
            self.advance(q=wrist_quaternion(-90, -90))
            self.assertEqual(self.controller.q_out[1], self.controller.q_out[2])
            self.assertEqual(self.controller.velocity[1], self.controller.velocity[2])
            for index, cap in enumerate((20, 8, 8)):
                self.assertLessEqual(abs(self.controller.q_out[index]-previous[index]), cap*.02+1e-8)
                lo, hi = self.controller.config["limits_deg"][index]
                self.assertGreaterEqual(self.controller.q_out[index], lo)
                self.assertLessEqual(self.controller.q_out[index], hi)
            if self.controller.q_out[1] < 60:
                self.assertLessEqual(abs(self.controller.velocity[1]-previous_velocity[1]), 12*.02+1e-8)
            previous, previous_velocity = self.controller.q_out[:], self.controller.velocity[:]
        self.assertVectorAlmostEqual(self.controller.q_out, [-45, 60, 60])

    def test_six_limits_keep_reference_and_return_home_without_drift(self):
        for axis, limits in ((0, DEFAULT_CONFIG["limits_deg"]),
                             (1, [[-45, 45], [10, 55], [0, 60]]),
                             (1, [[-45, 45], [0, 60], [20, 57]])):
            for direction in (-1, 1):
                with self.subTest(axis=axis, direction=direction, limits=limits):
                    self.setUp()
                    self.controller.configure({"limits_deg": deepcopy(limits)})
                    self.start_follow()
                    expected = (limits[0][direction > 0] if axis == 0 else
                                min(limits[1][1], limits[2][1]) if direction > 0 else max(limits[1][0], limits[2][0]))
                    reference = self.controller.reference_quaternion[:]
                    control = [direction*110, 0] if axis == 0 else [0, -direction*110]
                    for _ in range(230):
                        self.advance(q=wrist_quaternion(*control))
                    self.assertAlmostEqual(self.controller.q_out[axis], expected)
                    control[axis] = direction*140 if axis == 0 else -direction*140
                    self.advance(q=wrist_quaternion(*control))
                    self.assertAlmostEqual(self.controller.q_req[axis], expected)
                    control[axis] = direction*139 if axis == 0 else -direction*139
                    self.advance(q=wrist_quaternion(*control))
                    self.assertAlmostEqual(self.controller.q_req[axis], expected)
                    for _ in range(300):
                        self.advance(q=wrist_quaternion())
                    self.assertVectorAlmostEqual(self.controller.q_out, [0, 45, 45])
                    self.assertEqual(self.controller.reference_quaternion, reference)

    def test_count_wrap_parity_reboot_and_reverse_counter(self):
        self.initialize(ac=2**32-1, bc=2**32-1)
        self.advance(ac=0, bc=0)
        self.assertTrue(self.controller.armed)
        self.assertEqual(self.controller.open_req, 0)
        self.advance(ac=2, bc=2)
        self.assertTrue(self.controller.armed)
        self.advance(ac=1)
        self.assertFalse(self.controller.enabled)
        self.initialize()
        self.advance(boot=13, seq=0, ac=99, bc=99)
        self.assertFalse(self.controller.enabled)
        self.assertIsNone(self.controller.reference_quaternion)
        self.initialize(ac=101, bc=103)
        self.advance()
        self.assertFalse(self.controller.armed)

    def test_duplicate_frames_cannot_refresh_age_or_trigger_clicks(self):
        self.start_follow()
        old, stamp = deepcopy(self.current), self.controller.received_at
        self.clock.now += .1
        self.assertFalse(self.controller.receive({**old, "ac": old["ac"]+1, "bc": 1}))
        self.assertEqual(self.controller.received_at, stamp)
        self.assertTrue(self.controller.armed)
        self.assertEqual(self.controller.open_req, 1)

    def test_transient_invalid_and_stale_recovery_latch_disabled(self):
        for kind in ("invalid", "stale"):
            with self.subTest(kind=kind):
                self.setUp()
                self.start_follow()
                before = self.controller.q_out[:]
                self.clock.now += .02 if kind == "invalid" else .26
                current = {**self.current, "seq": self.current["seq"]+1}
                if kind == "invalid":
                    self.controller.receive({**current, "valid": 0})
                    current["seq"] += 1
                self.controller.receive({**current, "valid": 1, "q": wrist_quaternion(-30)})
                self.controller.step()
                self.assertTrue(self.controller.valid())
                self.assertFalse(self.controller.enabled)
                self.assertEqual(self.controller.q_out, before)

    def test_scheduler_pause_preserves_reference_and_can_resume(self):
        self.start_follow()
        self.advance(q=wrist_quaternion(-30))
        before, reference = self.controller.q_out[:], self.controller.reference_quaternion[:]
        self.advance(seconds=.12)
        self.assertFalse(self.controller.armed)
        self.assertTrue(self.controller.enabled)
        self.assertEqual(self.controller.q_out, before)
        self.assertEqual(self.controller.reference_quaternion, reference)
        self.click_a()
        self.assertAlmostEqual(self.controller.q_req[0], -25)

    def test_three_dimensional_175_degree_guard_and_90_degree_recovery(self):
        self.start_follow()
        self.advance(q=wrist_quaternion(-20))
        before, reference = self.controller.q_out[:], self.controller.reference_quaternion[:]
        self.advance(q=wrist_quaternion(z=175))
        self.assertTrue(self.controller.angle_ambiguous)
        self.assertEqual(self.controller.q_req, before)
        self.assertEqual(self.controller.q_out, before)
        self.assertEqual(self.controller.gate_status()[0], "ANGLE_AMBIGUOUS")
        self.advance(q=wrist_quaternion(z=179))
        self.advance(q=wrist_quaternion(z=-179))
        self.advance(q=wrist_quaternion(z=100))
        self.assertTrue(self.controller.angle_ambiguous)
        self.assertEqual(sum(e["code"] == "ANGLE_AMBIGUOUS" for e in self.controller.events), 1)
        self.click_b()
        self.assertEqual(self.controller.open_req, 0)
        self.advance(q=wrist_quaternion(z=90))
        self.assertFalse(self.controller.angle_ambiguous)
        self.assertVectorAlmostEqual(self.controller.q_req, [0, 45, 45])
        self.assertEqual(self.controller.reference_quaternion, reference)

    def test_absolute_sign_wrap_near_180_has_small_relative_motion(self):
        self.initialize(q=sensor_quaternion(179))
        self.click_a()
        self.advance(q=sensor_quaternion(-173))
        self.assertVectorAlmostEqual(self.controller.rotation_vector(), [0, 8, 0])
        self.assertFalse(self.controller.angle_ambiguous)

    def test_sparse_telemetry_crossing_180_holds_even_when_both_samples_are_below_175(self):
        reference = sensor_quaternion(65, -40, 20)
        self.initialize(q=reference)
        self.click_a()
        self.advance(q=multiply(reference, wrist_quaternion(-170)))
        self.assertFalse(self.controller.angle_ambiguous)
        self.assertAlmostEqual(self.controller.q_req[0], -45)
        held = self.controller.q_out[:]
        self.advance(q=multiply(reference, wrist_quaternion(-190)))
        self.assertTrue(self.controller.angle_ambiguous)
        self.assertEqual(self.controller.q_req, held)
        self.assertEqual(self.controller.q_out, held)
        self.assertEqual(self.controller.velocity, [0, 0, 0])
        self.assertTrue(self.controller.armed)
        self.assertAlmostEqual(distance_deg(self.controller.reference_quaternion, reference), 0)
        for _ in range(5):
            self.advance()
        self.assertEqual(self.controller.q_out, held)
        self.assertEqual(sum(e["code"] == "ANGLE_AMBIGUOUS" for e in self.controller.events), 1)

    def test_quaternion_sign_flip_at_170_is_not_a_principal_cut_crossing(self):
        reference = sensor_quaternion(65, -40, 20)
        self.initialize(q=reference)
        self.click_a()
        current = multiply(reference, wrist_quaternion(-170))
        self.advance(q=current)
        self.assertFalse(self.controller.angle_ambiguous)
        for index in range(8):
            self.advance(q=[-v for v in current] if index % 2 == 0 else current)
            self.assertFalse(self.controller.angle_ambiguous)
            self.assertAlmostEqual(self.controller.q_req[0], -45)
        self.assertTrue(self.controller.armed)
        self.assertFalse(any(e["code"] == "ANGLE_AMBIGUOUS" for e in self.controller.events))

    def test_configuration_and_replay_are_explicit_and_exclusive(self):
        self.assertEqual(MAPPING_VERSION, "coordinated-angle-v3")
        self.assertEqual(LIMIT_POLICY, "FIXED_REFERENCE_HARD_CLAMP")
        old = self.controller.config_id
        self.controller.configure({"deadzone_deg": 6})
        self.assertNotEqual(self.controller.config_id, old)
        for key in ("roll_sign", "pitch_sign", "mounting_profile", "neutral_deg", "full_scale_deg"):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.controller.configure({key: 1})
        for value in (0, 21, True, float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.controller.configure({"deadzone_deg": value})
        self.start_follow()
        with self.assertRaises(ValueError):
            self.controller.configure({"deadzone_deg": 7})
        self.controller.stop()
        self.controller.owner = "REPLAY"
        self.controller.q_out = [12, 34, 56]
        self.advance(boot=99, ac=100, bc=100, q=wrist_quaternion(80, 80))
        self.assertEqual(self.controller.q_out, [12, 34, 56])
        with self.assertRaises(ValueError):
            self.controller.enable()


class GeometryTests(unittest.TestCase):
    geometry = {"table_to_shoulder_axis": 90, "shoulder_to_elbow_axis": 115,
                "elbow_to_gripper_mount": 65, "gripper_mount_to_tip": 90}

    def test_upright_and_ready_match_reference(self):
        upright = forward_kinematics(pose([0, 0, 0], 1), self.geometry)
        self.assertAlmostEqual(upright[0], 0)
        self.assertAlmostEqual(upright[2], 360)
        ready = forward_kinematics(pose([0, 45, 45], 1), self.geometry)
        self.assertAlmostEqual(ready[0], 236.3172798, places=5)
        self.assertAlmostEqual(ready[2], 171.3172798, places=5)

    def test_base_rotation_changes_xy_only(self):
        front = forward_kinematics(pose([0, 45, 45], 1), self.geometry)
        left = forward_kinematics(pose([90, 45, 45], 1), self.geometry)
        self.assertAlmostEqual(left[0], 0)
        self.assertAlmostEqual(left[1], front[0])
        self.assertAlmostEqual(left[2], front[2])


if __name__ == "__main__":
    unittest.main()
