"""Python and C++ controllers consume the same independently constructed cases.

Run from any directory: py -3.13 -B firmware/tests/test_controller_vectors.py
The matching host regression is robot_controller_test.cpp / sharedVectors().
"""
import csv
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend"))
from app.attitude import wrist_rotation_vector
from app.virtual import VirtualController


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


def frame(sequence, quaternion, ac=0):
    return dict(v=1, att=3, role="wearable", boot=7, seq=sequence,
                ms=sequence * 40, age_ms=0, who=0x68, valid=1, cal=250,
                q=quaternion, gyro_dps=[0., 0., 0.], accel_g=[0., 0., 1.],
                a=0, b=0, ac=ac, bc=0, norm_g=1., temp_c=24., errors=0,
                rej=0, gr=2000, rc=0)


class SharedControllerVectors(unittest.TestCase):
    def test_cpp_and_python_share_full_quaternion_expected_targets(self):
        path = ROOT / "firmware" / "tests" / "control_vectors.csv"
        with path.open(newline="", encoding="ascii") as source:
            cases = list(csv.DictReader(source))
        self.assertEqual(len(cases), 6)
        for case in cases:
            with self.subTest(case=case["name"]):
                reference = [float(case["ref_" + a]) for a in "wxyz"]
                current = [float(case["q_" + a]) for a in "wxyz"]
                rotation = [float(case[a]) for a in ("roll", "pitch", "z")]
                target = [float(case[a]) for a in ("base", "shoulder", "elbow")]
                for got, expected in zip(wrist_rotation_vector(reference, current), rotation):
                    self.assertAlmostEqual(got, expected, places=5)
                clock = Clock()
                controller = VirtualController(clock)
                for seq in range(1, 17):
                    clock.now = seq * .04
                    controller.receive(frame(seq, reference))
                    controller.step()
                controller.enable()
                clock.now += .04
                controller.receive(frame(17, reference, ac=1))
                clock.now += .04
                controller.receive(frame(18, current, ac=1))
                for got, expected in zip(controller.rotation_vector(), rotation):
                    self.assertAlmostEqual(got, expected, places=5)
                for got, expected in zip(controller.q_req, target):
                    self.assertAlmostEqual(got, expected, places=5)


if __name__ == "__main__":
    unittest.main()
