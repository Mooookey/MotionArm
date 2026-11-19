"""Fixed wrist axes and true SO(3) reference cancellation."""
import math
import unittest

from app.attitude import distance_deg, mean, multiply, wrist_rotation_vector
from app.virtual import MOUNTING, VirtualController
from test_virtual import Clock
from test_wearable import frame, sensor_quaternion, wrist_quaternion


class MountingTests(unittest.TestCase):
    def assertVectorAlmostEqual(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=7)

    def controller(self, reference):
        clock = Clock()
        controller = VirtualController(clock)
        for seq in range(15):
            clock.now += .04
            controller.receive(frame(seq=seq, ms=seq*40, q=reference))
            controller.step()
        controller.enable()
        clock.now += .04
        controller.receive(frame(seq=15, ms=600, q=reference, ac=1))
        return controller, clock

    def test_four_sensor_directions_and_fixed_joint_signs(self):
        self.assertEqual(MOUNTING["control_from_sensor"], {"Roll": "-Y", "Pitch": "+X", "Z": "+Z"})
        self.assertEqual(MOUNTING["joint_signs"], [1, -1])
        cases = [(15, 0, [0, 15, 0], [0, 35, 35]),
                 (-15, 0, [0, -15, 0], [0, 55, 55]),
                 (0, 15, [-15, 0, 0], [-10, 45, 45]),
                 (0, -15, [15, 0, 0], [10, 45, 45])]
        for x, y, vector, targets in cases:
            with self.subTest(x=x, y=y):
                c, clock = self.controller([1, 0, 0, 0])
                clock.now += .04
                c.receive(frame(seq=16, ms=640, q=sensor_quaternion(x, y), ac=1))
                self.assertVectorAlmostEqual(c.rotation_vector(), vector)
                self.assertVectorAlmostEqual(c.q_req, targets)

    def test_local_compound_motion_is_independent_of_nonzero_reference(self):
        delta = wrist_quaternion(17, -23, 31)
        references = [[1, 0, 0, 0], sensor_quaternion(77, -42, 24),
                      multiply(sensor_quaternion(0, 115, 0), sensor_quaternion(65, 0, 40))]
        for reference in references:
            with self.subTest(reference=reference):
                c, clock = self.controller(reference)
                clock.now += .04
                c.receive(frame(seq=16, ms=640, q=multiply(reference, delta), ac=1))
                self.assertVectorAlmostEqual(c.rotation_vector(), [17, -23, 31])
                self.assertVectorAlmostEqual(c.q_req, [12, 60, 60])
                clock.now += .04
                c.receive(frame(seq=17, ms=680, q=reference, ac=1))
                self.assertVectorAlmostEqual(c.q_req, [0, 45, 45])

    def test_sign_flip_preserves_orientation_and_reference_average(self):
        q = sensor_quaternion(73, -52, 24)
        negative = [-v for v in q]
        self.assertAlmostEqual(distance_deg(q, negative), 0)
        averaged = mean([q, negative, q, negative])
        self.assertAlmostEqual(distance_deg(q, averaged), 0)
        current = multiply(q, wrist_quaternion(20, 10, -8))
        self.assertVectorAlmostEqual(wrist_rotation_vector(q, [-v for v in current]), [20, 10, -8])
        self.assertAlmostEqual(math.hypot(*averaged), 1)

    def test_observed_right_tilt_and_fingers_up_follow_field_directions(self):
        # Field observations: right tilt gives negative control Roll; fingers
        # up gives positive Pitch. Negative yaw is clockwise from above, while
        # smaller shoulder/elbow down angles raise both links.
        c, clock = self.controller([1, 0, 0, 0])
        clock.now += .04
        c.receive(frame(seq=16, ms=640, q=wrist_quaternion(-34.58, 19.13), ac=1))
        self.assertVectorAlmostEqual(c.angles(), [-34.58, 19.13])
        self.assertVectorAlmostEqual(c.q_req, [-29.58, 30.87, 30.87])

    def test_pure_reference_z_rotation_has_no_control_components(self):
        reference = sensor_quaternion(60, -35, 18)
        self.assertVectorAlmostEqual(wrist_rotation_vector(reference, multiply(reference, sensor_quaternion(z=120))), [0, 0, 120])


if __name__ == "__main__":
    unittest.main()
