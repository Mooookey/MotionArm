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


class TrialProfileTests(unittest.TestCase):
    def trial(self):
        return json.loads((ROOT / "config/robot_trial.json").read_text(encoding="utf-8"))

    def test_mapping_anchor_outside_operating_limits_is_preserved(self):
        profile = self.trial()
        header, identity = generator.generate(profile)
        self.assertIn("{1630,1030}", header)
        self.assertIn("{1570,1050}", header)
        self.assertIn("{-45,20,20}", header)
        self.assertIn("{45,60,60}", header)
        self.assertIn('{-1,-1}, "coordinated-angle-v4"', header)
        self.assertIn("{-15,-10,-5,0,5,10,15}", header)
        self.assertIn("sha256:", identity)
        self.assertEqual(header, (ROOT / "firmware/robot/PoweredTrialProfile.h").read_text(encoding="ascii"))
        self.assertEqual(profile["shoulder_mapping"][0][0], 0)
        self.assertGreater(profile["operating_limits_deg"][1][0], 0)

    def test_trial_is_not_full_workspace_or_thermal_certification(self):
        for key in ("full_workspace_verified", "load_verified", "thermal_safety_verified"):
            p = self.trial();p[key] = True
            with self.assertRaises(ValueError): generator.generate(p)

    def test_widening_and_renewable_deadline_refused(self):
        for key, value in (("operating_limits_deg", [[-45,45],[0,60],[0,60]]),
                           ("operating_limits_deg", [[-20,20],[35,55],[35,55]]),
                           ("operating_limits_deg", [[-15,15],[35,55],[35,55]]),
                           ("operating_limits_deg", [[-3,3],[42,48],[42,48]]),
                           ("diagnostic_offsets_deg", [[-20,0,20],[-10,0,10],[-10,0,10]]),
                           ("diagnostic_offsets_deg", [[-15,0,1.5,15],[-10,0,10],[-10,0,10]]),
                           ("deadzone_deg", 0),
                           ("joint_signs", [1,-1]), ("joint_signs", [-1,1]),
                           ("joint_signs", [-1.0,-1]), ("mapping_version", "coordinated-angle-v3"),
                           ("speed_deg_s", 2), ("speed_deg_s", 10), ("speed_deg_s", 20),
                           ("acceleration_deg_s2", 5), ("acceleration_deg_s2", 60),
                           ("gripper_limit_ms", 700),
                           ("shoulder_mapping", [[45,1030],[60,830]]),
                           ("onsite_confirmation_required", False)):
            p = self.trial();p[key] = value
            with self.assertRaises(ValueError): generator.generate(p)


if __name__ == "__main__":
    unittest.main()
