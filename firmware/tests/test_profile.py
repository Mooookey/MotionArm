"""Synthetic profile validation only; never commissions the local arm."""
import importlib.util
import json
from copy import deepcopy
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("profile_generator", ROOT / "firmware/robot/generate_profile.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


def fixture():
    p = json.loads((ROOT / "config/robot_calibration.example.json").read_text(encoding="utf-8"))
    p.update(verified_on_hardware=True, allow_arm=True, operator_confirmed_initial_pose=True)
    p["board"].update(physical_cutoff_tested=True, gravity_support_tested=True,
                      pca9685_verified_i2c_address_hex="0x40", actual_servo_power_v=5,
                      pulse_command_timebase="calibrator_nominal_us")
    for axis, home, pulse in zip(p["joints"], (0, 0, 0, 0), (1610, 1630, 1570, 1130)):
        axis.update(logical_min_deg=-45 if axis["id"] == "base" else 0,
                    logical_max_deg=60, home_deg=home, home_us=pulse,
                    pulse_min_us=1000, pulse_max_us=2000, inverted=False,
                    max_velocity_deg_s=15, max_acceleration_deg_s2=60)
        if axis["id"] != "base":
            axis["pulse_min_us"] = pulse
    p["joints"][3].update(startup_deg=0, open_us=1130, close_us=1630)
    p["workcell"].update(base_fixed=True, joint_space_validated=True,
                         safe_joint_regions=[{"min_deg": [-45, 0, 0, 0], "max_deg": [60, 60, 60, 60]}])
    return p


class ProfileTests(unittest.TestCase):
    def test_uncommissioned_local_and_example_refuse_generation(self):
        for name in ("local", "example"):
            p = json.loads((ROOT / f"config/robot_calibration.{name}.json").read_text(encoding="utf-8"))
            with self.assertRaises(ValueError):
                generator.generate(p)

    def test_separate_startup_and_anchor_are_hashed(self):
        p = fixture()
        header, identity = generator.generate(p)
        self.assertIn("1630", header)
        self.assertIn("45.0f", header)
        self.assertTrue(identity.startswith("sha256:"))
        p["joints"][1].pop("startup_deg")
        with self.assertRaises(ValueError):
            generator.generate(p)

    def test_software_deadline_not_legacy_policy(self):
        for deadline in (700, 3000, 5001):
            p = fixture()
            p["joints"][3]["max_closed_hold_ms"] = deadline
            with self.assertRaises(ValueError):
                generator.generate(p)

    def test_startup_must_be_in_verified_region(self):
        p = fixture()
        p["workcell"]["safe_joint_regions"][0]["max_deg"][1] = 30
        with self.assertRaises(ValueError):
            generator.generate(p)

    def test_initial_pose_and_safety_flags_still_required(self):
        for flag in ("allow_arm", "verified_on_hardware", "operator_confirmed_initial_pose"):
            p = deepcopy(fixture()); p[flag] = False
            with self.assertRaises(ValueError):
                generator.generate(p)


if __name__ == "__main__":
    unittest.main()
