"""Coordinated angle following, click controls and deterministic safety tests."""
from copy import deepcopy
import unittest

from app.virtual import DEFAULT_CONFIG, LIMIT_POLICY, MAPPING_VERSION, VirtualController, forward_kinematics, pose
from test_wearable import frame


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


class VirtualControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.controller = VirtualController(clock=self.clock)
        self.current = frame()
        self.controller.receive(self.current)

    def advance(self, seconds=.02, **changes):
        self.clock.now += seconds
        self.current = {**self.current, "seq": (self.current["seq"] + 1) % 2**32,
                        "ms": (self.current["ms"] + round(seconds * 1000)) % 2**32, **changes}
        self.controller.receive(self.current)
        self.controller.step()

    def settle(self, **changes):
        for _ in range(28):
            self.advance(a=0, b=0, **changes)

    def initialize(self, **changes):
        self.settle(**changes)
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
        self.assertTrue(self.controller.armed)

    def test_requires_real_calibrated_fresh_input_and_updated_firmware(self):
        for changes in ({"cal": 249}, {"valid": 0}, {"who": 0}, {"age_ms": 250}):
            with self.subTest(changes=changes):
                controller = VirtualController(clock=self.clock)
                controller.receive(frame(**changes))
                with self.assertRaises(ValueError):
                    controller.enable()
        legacy = frame()
        del legacy["ac"]
        del legacy["bc"]
        self.controller.receive({**legacy, "seq": 2})
        with self.assertRaisesRegex(ValueError, "固件"):
            self.controller.enable()
        self.assertEqual(self.controller.gate_status()[0], "FIRMWARE_UPDATE")

    def test_initialization_requires_stable_released_hand_and_captures_once(self):
        with self.assertRaises(ValueError):
            self.controller.enable()
        self.initialize(roll=127, pitch=-48)
        self.assertEqual(self.controller.config["neutral_deg"], [127., -48.])
        self.assertEqual(self.controller.q_out, [0., 45., 45.])
        self.assertEqual(self.controller.open_out, 1)
        self.assertEqual(self.controller.gate_status()[0], "READY")
        self.click_a()
        reference = self.controller.anchor_wrist[:]
        self.advance(roll=137, pitch=-38)
        self.assertEqual(self.controller.q_req, [5., 50., 50.])
        self.click_a()
        self.advance(roll=127, pitch=-48, a=0)
        self.click_a()
        self.assertEqual(self.controller.anchor_wrist, reference)
        self.assertEqual(self.controller.q_req, [0., 45., 45.])

    def test_initialize_resets_previous_pose_gripper_and_counter_baseline(self):
        self.controller.q_out = [80., 30., 55.]
        self.controller.q_req = self.controller.q_out[:]
        self.controller.open_req = self.controller.open_out = 0
        self.initialize(ac=1001, bc=502)
        self.assertEqual(self.controller.q_req, [0., 45., 45.])
        self.assertEqual(self.controller.q_out, self.controller.q_req)
        self.assertEqual(self.controller.open_req, 1)
        self.assertFalse(self.controller.armed)
        self.advance()
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.open_req, 1)
        self.click_a()
        self.assertTrue(self.controller.armed)
        with self.assertRaises(ValueError):
            self.controller.enable()

    def test_a_click_toggles_once_and_holding_or_releasing_raw_button_does_not_toggle(self):
        self.initialize()
        self.click_a()
        for _ in range(30):
            self.advance(a=1, roll=15)
        self.assertTrue(self.controller.armed)
        self.advance(a=0)
        self.assertTrue(self.controller.armed)
        self.click_a()
        self.assertFalse(self.controller.armed)
        paused = self.controller.q_out[:]
        for _ in range(30):
            self.advance(a=1, roll=-30)
        self.assertEqual(self.controller.q_out, paused)
        self.assertEqual(self.controller.gate_status()[0], "READY")
        self.click_a()
        self.assertTrue(self.controller.armed)

    def test_raw_a_b_mode_and_old_event_counts_do_not_control_model(self):
        self.initialize()
        for _ in range(20):
            self.advance(a=1, b=1, mode=1, short=100, long=100, roll=30, pitch=30)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.q_out, [0., 45., 45.])
        self.assertEqual(self.controller.open_req, 1)
        self.click_a()
        self.assertTrue(self.controller.armed)
        self.assertEqual(self.controller.q_req, [25., 60., 60.])

    def test_pause_tracks_hand_but_resume_uses_initial_reference_and_smooth_catchup(self):
        self.start_follow()
        self.advance(roll=15, pitch=15)
        for _ in range(80):
            self.advance()
        self.assertEqual(self.controller.q_out, [10., 55., 55.])
        self.click_a()
        paused = self.controller.q_out[:]
        self.advance(roll=-15, pitch=-15)
        for _ in range(20):
            self.advance()
        self.assertEqual(self.controller.q_out, paused)
        self.click_a()
        self.assertEqual(self.controller.q_req, [-10., 35., 35.])
        self.assertLessEqual(abs(self.controller.q_out[0] - paused[0]), 20 * .02 + 1e-9)
        self.assertLessEqual(abs(self.controller.q_out[1] - paused[1]), 15 * .02 + 1e-9)
        self.assertEqual(self.controller.anchor_wrist, [0., 0.])

    def test_b_click_toggles_while_paused_and_gripper_persists_through_a_pause(self):
        self.initialize()
        self.click_b()
        for _ in range(15):
            self.advance(b=1)
        self.assertEqual(self.controller.open_req, 0)
        self.assertEqual(self.controller.open_out, 0)
        self.click_a()
        self.click_a()
        for _ in range(15):
            self.advance(a=0, b=0)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.open_req, 0)
        self.assertEqual(self.controller.open_out, 0)
        self.click_b()
        for _ in range(15):
            self.advance()
        self.assertEqual(self.controller.open_out, 1)

    def test_stop_and_fault_preserve_virtual_gripper_state_until_new_initialization(self):
        self.initialize()
        self.click_b()
        for _ in range(15):
            self.advance(b=0)
        self.assertEqual(self.controller.open_out, 0)
        self.controller.stop()
        self.advance(bc=self.current["bc"] + 1)
        self.assertEqual(self.controller.open_req, 0)
        self.assertEqual(self.controller.open_out, 0)
        self.initialize()
        self.assertEqual(self.controller.open_out, 1)
        self.click_b()
        for _ in range(15):
            self.advance(b=0)
        self.advance(valid=0)
        self.assertFalse(self.controller.enabled)
        self.assertEqual(self.controller.open_req, 0)
        self.assertEqual(self.controller.open_out, 0)

    def test_soft_deadzone_angle_mapping_and_fixed_tilt_settles(self):
        self.start_follow()
        for _ in range(10):
            self.advance(roll=5, pitch=-5)
        self.assertEqual(self.controller.q_out, [0., 45., 45.])
        self.advance(roll=17.5, pitch=12.5)
        self.assertEqual(self.controller.q_req, [12.5, 52.5, 52.5])
        for _ in range(250):
            self.advance()
        self.assertEqual(self.controller.q_req, [12.5, 52.5, 52.5])
        self.assertEqual(self.controller.q_out, self.controller.q_req)
        self.assertEqual(self.controller.velocity, [0., 0., 0.])

    def test_coupled_output_uses_shared_slower_joint_speed_and_acceleration(self):
        self.controller.configure({"max_velocity_deg_s": [20., 8., 13.],
                                   "max_acceleration_deg_s2": [60., 35., 12.]})
        self.start_follow()
        previous = self.controller.q_out[:]
        previous_velocity = self.controller.velocity[:]
        for _ in range(240):
            self.advance(roll=90, pitch=90)
            self.assertEqual(self.controller.q_req[1], self.controller.q_req[2])
            self.assertEqual(self.controller.q_out[1], self.controller.q_out[2])
            self.assertEqual(self.controller.velocity[1], self.controller.velocity[2])
            for axis, cap in ((0, 20), (1, 8), (2, 8)):
                self.assertLessEqual(abs(self.controller.q_out[axis] - previous[axis]), cap * .02 + 1e-8)
                lo, hi = self.controller.config["limits_deg"][axis]
                self.assertGreaterEqual(self.controller.q_out[axis], lo)
                self.assertLessEqual(self.controller.q_out[axis], hi)
            if self.controller.q_out[1] < 60:
                self.assertLessEqual(abs(self.controller.velocity[1] - previous_velocity[1]), 12 * .02 + 1e-8)
            previous, previous_velocity = self.controller.q_out[:], self.controller.velocity[:]
        self.assertEqual(self.controller.q_out, [45., 60., 60.])

    def test_six_limit_cases_keep_fixed_reference_and_return_home_without_drift(self):
        cases = [(0, "roll", deepcopy(DEFAULT_CONFIG["limits_deg"]), -1),
                 (0, "roll", deepcopy(DEFAULT_CONFIG["limits_deg"]), 1)]
        for limits in ([[-45, 45], [10, 55], [0, 60]], [[-45, 45], [0, 60], [20, 57]]):
            cases.extend((1, "pitch", limits, direction) for direction in (-1, 1))
        for axis, wrist, limits, direction in cases:
            with self.subTest(axis=axis, limits=limits, direction=direction):
                self.setUp()
                self.controller.configure({"limits_deg": limits})
                self.start_follow()
                expected = (limits[0][1 if direction > 0 else 0] if axis == 0 else
                            min(limits[1][1], limits[2][1]) if direction > 0 else max(limits[1][0], limits[2][0]))
                reference = self.controller.anchor_wrist[:]
                for _cycle in range(3):
                    self.advance(**{wrist: direction * 110})
                    for _ in range(230):
                        self.advance()
                    self.assertEqual(self.controller.q_out[axis], expected)
                    self.advance(**{wrist: direction * 140})
                    self.assertEqual(self.controller.q_req[axis], expected)
                    self.advance(**{wrist: direction * 139})
                    # Returning one degree while still outside the mapped
                    # interval remains clamped: no hidden anchor adjustment.
                    self.assertEqual(self.controller.q_req[axis], expected)
                    inside_offset = expected - direction - (45 if axis else 0)
                    inside_wrist = inside_offset + direction * self.controller.config["deadzone_deg"]
                    self.advance(**{wrist: inside_wrist})
                    self.assertAlmostEqual(self.controller.q_req[axis], expected - direction)
                    self.assertLess(direction * (self.controller.q_out[axis] - expected), 0)
                    self.advance(**{wrist: 0})
                    self.assertEqual(self.controller.q_req, [0., 45., 45.])
                    for _ in range(300):
                        self.advance()
                    self.assertEqual(self.controller.q_out, [0., 45., 45.])
                    self.assertEqual(self.controller.anchor_wrist, reference)
                    self.assertTrue(self.controller.armed)
                    if axis:
                        self.assertEqual(self.controller.q_req[1], self.controller.q_req[2])
                        self.assertEqual(self.controller.q_out[1], self.controller.q_out[2])

    def test_count_wrap_parity_and_reboot_never_replay_old_clicks(self):
        self.initialize(ac=2**32 - 1, bc=2**32 - 1)
        self.advance(ac=0, bc=0)
        self.assertTrue(self.controller.armed)
        self.assertEqual(self.controller.open_req, 0)
        self.advance(ac=2, bc=2)
        self.assertTrue(self.controller.armed)
        self.assertEqual(self.controller.open_req, 0)
        self.advance(boot=13, seq=0, ac=99, bc=99)
        self.assertFalse(self.controller.enabled)
        self.assertEqual(self.controller.open_req, 0)
        self.initialize(ac=101, bc=103)
        self.advance()
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.open_req, 1)

    def test_old_frames_cannot_refresh_age_or_trigger_clicks(self):
        self.start_follow()
        old = deepcopy(self.current)
        stamp = self.controller.received_at
        self.clock.now += .1
        self.assertFalse(self.controller.receive({**old, "ac": old["ac"] + 1, "bc": 1}))
        self.assertEqual(self.controller.received_at, stamp)
        self.assertTrue(self.controller.armed)
        self.assertEqual(self.controller.open_req, 1)

    def test_invalid_then_valid_before_tick_latches_disabled_session(self):
        self.start_follow()
        before = self.controller.q_out[:]
        self.clock.now += .02
        invalid = {**self.current, "seq": self.current["seq"] + 1, "valid": 0}
        self.controller.receive(invalid)
        self.controller.receive({**invalid, "seq": invalid["seq"] + 1, "valid": 1, "ac": 2, "roll": 30})
        self.controller.step()
        self.assertTrue(self.controller.valid())
        self.assertFalse(self.controller.enabled)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.q_out, before)

    def test_stale_gap_is_latched_when_new_frame_precedes_next_tick(self):
        self.start_follow()
        before = self.controller.q_out[:]
        for _ in range(12):
            self.clock.now += .02
            self.controller.step()
        self.clock.now += .02
        self.controller.receive({**self.current, "seq": self.current["seq"] + 1, "roll": 30})
        self.controller.step()
        self.assertFalse(self.controller.enabled)
        self.assertFalse(self.controller.armed)
        self.assertEqual(self.controller.q_out, before)

    def test_scheduler_pause_does_not_move_or_reset_reference(self):
        self.start_follow()
        self.advance(roll=30)
        before = self.controller.q_out[:]
        reference = self.controller.anchor_wrist[:]
        self.advance(seconds=.12, roll=30)
        self.assertFalse(self.controller.armed)
        self.assertTrue(self.controller.enabled)
        self.assertEqual(self.controller.q_out, before)
        self.assertEqual(self.controller.anchor_wrist, reference)
        self.click_a()
        self.assertTrue(self.controller.armed)
        self.assertEqual(self.controller.q_req[0], 25)

    def test_expiry_and_disconnect_preserve_targets_and_need_reinitialization(self):
        self.start_follow()
        self.advance(roll=30)
        for _ in range(14):
            self.clock.now += .02
            self.controller.step()
        before = self.controller.q_out[:]
        self.assertFalse(self.controller.enabled)
        self.advance(ac=20, roll=-30)
        self.assertEqual(self.controller.q_out, before)
        self.controller.received_at = None
        self.controller.step()
        self.assertFalse(self.controller.valid())

    def test_replay_excludes_input_and_initialization(self):
        self.controller.owner = "REPLAY"
        self.controller.q_out = [12., 34., 56.]
        self.advance(boot=99, ac=100, bc=100, roll=90, pitch=90)
        self.assertEqual(self.controller.q_out, [12., 34., 56.])
        with self.assertRaises(ValueError):
            self.controller.enable()

    def test_configuration_identity_names_mapping_and_only_changes_stopped(self):
        self.assertEqual(MAPPING_VERSION, "coordinated-angle-v2")
        self.assertEqual(LIMIT_POLICY, "FIXED_REFERENCE_HARD_CLAMP")
        self.assertEqual(self.controller.mapping_state()["limit_policy"], LIMIT_POLICY)
        old = self.controller.config_id
        self.controller.configure({"roll_sign": -1})
        self.assertNotEqual(old, self.controller.config_id)
        self.start_follow()
        with self.assertRaises(ValueError):
            self.controller.configure({"pitch_sign": -1})
        self.advance(roll=30)
        self.assertEqual(self.controller.q_req[0], -25)

    def test_so3_ambiguity_holds_without_changing_reference_then_returning_reference_returns_home(self):
        self.initialize(att=2, roll=100, pitch=140)
        self.click_a()
        self.advance(roll=110, pitch=145)
        for _ in range(4):
            self.advance()
        held = self.controller.q_out[:]
        reference = self.controller.anchor_wrist[:]
        self.advance(roll=-100, pitch=-140)
        self.assertEqual(self.controller.q_req, held)
        self.assertEqual(self.controller.q_out, held)
        self.assertEqual(self.controller.anchor_wrist, reference)
        self.assertTrue(self.controller.armed)
        self.assertEqual(self.controller.gate_status()[0], "ANGLE_AMBIGUOUS")
        self.assertEqual(self.controller.reason, "大角度姿态有歧义，回到启用参考附近恢复")
        for _ in range(10):
            self.advance()
        self.assertEqual(sum(event["code"] == "ANGLE_AMBIGUOUS" for event in self.controller.events), 1)
        self.click_b()
        self.assertEqual(self.controller.open_req, 0)
        self.advance(roll=100, pitch=140)
        self.assertFalse(self.controller.angle_ambiguous)
        self.assertEqual(self.controller.q_req, [0., 45., 45.])
        for _ in range(100):
            self.advance()
        self.assertEqual(self.controller.q_out, [0., 45., 45.])
        self.assertEqual(self.controller.anchor_wrist, reference)

    def test_so3_normal_wrap_relative_to_enabled_hand_remains_small(self):
        self.setUp()
        self.initialize(att=2, roll=179, pitch=0)
        self.click_a()
        self.advance(roll=-179)
        self.advance(roll=-173)
        self.assertEqual(self.controller.q_req[0], 3)
        self.assertFalse(self.controller.angle_ambiguous)

    def test_so3_crossing_relative_principal_cut_holds_instead_of_flipping_limit(self):
        self.initialize(att=2)
        self.click_a()
        for roll in range(5, 180, 5):
            self.advance(roll=roll)
        self.advance(roll=-179)
        self.assertTrue(self.controller.angle_ambiguous)
        self.assertEqual(self.controller.q_req, self.controller.q_out)
        self.advance(roll=0)
        self.assertFalse(self.controller.angle_ambiguous)
        self.assertEqual(self.controller.q_req, [0., 45., 45.])

    def test_pausing_and_b_clicks_at_limit_never_change_enable_reference(self):
        self.start_follow()
        reference = self.controller.anchor_wrist[:]
        self.advance(roll=100, pitch=100)
        self.assertEqual(self.controller.q_req, [45., 60., 60.])
        self.click_a()
        self.click_b()
        self.advance(roll=0, pitch=0)
        self.click_a()
        self.assertEqual(self.controller.q_req, [0., 45., 45.])
        self.assertEqual(self.controller.anchor_wrist, reference)
        self.assertEqual(self.controller.open_req, 0)


class GeometryTests(unittest.TestCase):
    geometry = {"table_to_shoulder_axis": 90, "shoulder_to_elbow_axis": 115,
                "elbow_to_gripper_mount": 65, "gripper_mount_to_tip": 90}

    def test_upright_and_ready_match_reference(self):
        upright = forward_kinematics(pose([0, 0, 0], 1), self.geometry)
        self.assertAlmostEqual(upright[0], 0)
        self.assertAlmostEqual(upright[1], 0)
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
