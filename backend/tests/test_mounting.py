"""Focused mounting-axis checks for immediate hand-back hardware trials."""
from copy import deepcopy
import json
import unittest

from app.virtual import DEFAULT_CONFIG, MAPPING_VERSION, VirtualController
from test_virtual import Clock
from test_wearable import frame


PROFILE = "hand_back_x_right_y_forearm_z_down"


class MountingTests(unittest.TestCase):
    def controller(self, profile=PROFILE, reference=(25., -30.), signs=(-1, -1)):
        clock = Clock()
        config = deepcopy(DEFAULT_CONFIG)
        config.update(mounting_profile=profile, roll_sign=signs[0], pitch_sign=signs[1])
        controller = VirtualController(clock=clock, config=config)
        current = frame(roll=reference[0], pitch=reference[1], att=2)

        def receive(**changes):
            nonlocal current
            clock.now += .04
            current = {**current, "seq": current["seq"] + 1, "ms": current["ms"] + 40, **changes}
            controller.receive(current)
            controller.step()

        for _ in range(17):
            receive()
        controller.enable()
        receive(ac=1)
        return controller, receive

    def test_hand_back_four_sensor_directions_use_minus_y_roll_and_x_pitch(self):
        cases = [(15, 0, [0., 15.], [0., 35., 35.]),
                 (-15, 0, [0., -15.], [0., 55., 55.]),
                 (0, 15, [-15., 0.], [10., 45., 45.]),
                 (0, -15, [15., 0.], [-10., 45., 45.])]
        for x, y, angles, targets in cases:
            with self.subTest(x=x, y=y):
                controller, receive = self.controller()
                receive(roll=25 + x, pitch=-30 + y)
                self.assertEqual(controller.angles(), angles)
                self.assertEqual(controller.wrist_delta, angles)
                self.assertEqual(controller.last_wrist, angles)
                self.assertEqual(controller.q_req, targets)
                self.assertFalse(controller.angle_ambiguous)

    def test_nonzero_sensor_reference_stays_sensor_xy_and_returns_home(self):
        controller, receive = self.controller()
        receive(roll=40, pitch=-45)
        self.assertEqual(controller.config["neutral_deg"], [25., -30.])
        self.assertEqual(controller.anchor_wrist, [25., -30.])
        receive(roll=25, pitch=-30)
        self.assertEqual(controller.angles(), [0., 0.])
        self.assertEqual(controller.q_req, [0., 45., 45.])
        for _ in range(60):
            receive()
        self.assertEqual(controller.q_out, [0., 45., 45.])

    def test_old_configuration_without_mounting_field_keeps_sensor_xy_semantics(self):
        legacy = deepcopy(DEFAULT_CONFIG)
        legacy.pop("mounting_profile")
        legacy["neutral_deg"] = [25., -30.]
        controller = VirtualController(config=legacy)
        controller.receive(frame(roll=40, pitch=-45))
        self.assertEqual(controller.config["mounting_profile"], "sensor_xy")
        self.assertEqual(controller.angles(), [15., -15.])

    def test_mounting_identity_survives_config_roundtrip_and_is_versioned(self):
        controller, _receive = self.controller()
        restored = VirtualController(config=json.loads(json.dumps(controller.config)))
        self.assertEqual(MAPPING_VERSION, "coordinated-angle-v2")
        self.assertEqual(restored.config_id, controller.config_id)
        self.assertEqual(restored.mounting_state()["profile"], PROFILE)
        self.assertEqual(restored.mounting_state()["control_from_sensor"], {"Roll": "-Y", "Pitch": "+X"})
        self.assertEqual(restored.mounting_state()["sensor_reference_axes"],
                         ["sensor_x_rotation_deg", "sensor_y_rotation_deg"])
        sensor_xy = deepcopy(controller.config)
        sensor_xy["mounting_profile"] = "sensor_xy"
        self.assertNotEqual(VirtualController(config=sensor_xy).config_id, controller.config_id)


if __name__ == "__main__":
    unittest.main()
