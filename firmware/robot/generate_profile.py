"""Convert verified MotionArm calibration JSON into the firmware profile."""

import argparse
import hashlib
import json
import math
from pathlib import Path

JOINTS = ("base", "shoulder", "elbow", "gripper")
CHANNELS = (14, 15, 13, 12)
CALIBRATOR_TICK_US = 122 / 25


def require(condition, field):
    if not condition:
        raise ValueError(f"Missing or invalid verified value: {field}")


def number(value, field):
    require(type(value) in (int, float) and math.isfinite(value), field)
    return value


def integer(value, field):
    require(type(value) is int, field)
    return value


def literal(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    return f"{float(value)!r}f"


def generate(profile):
    for key in ("verified_on_hardware", "allow_arm", "operator_confirmed_initial_pose"):
        require(profile.get(key) is True, key)
    require(integer(profile.get("schema_version"), "schema_version") == 2, "schema_version (explicit startup angles required)")
    board = profile["board"]
    require(board.get("physical_cutoff_tested") is True and board.get("gravity_support_tested") is True,
            "board.physical_cutoff_tested / gravity_support_tested")
    require(board.get("pca9685_verified_i2c_address_hex") == "0x40", "PCA9685 address")
    require(board.get("servo_pwm_freq_hz_starting_point") == 50, "PWM nominal frequency")
    require(number(board.get("actual_servo_power_v"), "servo voltage") > 0, "servo voltage")
    timebase = board.get("pulse_command_timebase", "measured_us")
    if timebase == "calibrator_nominal_us":
        period = CALIBRATOR_TICK_US * 4096
    else:
        require(timebase == "measured_us", "board.pulse_command_timebase")
        period = number(board.get("servo_pwm_period_us"), "measured PWM period")
        require(17_000 <= period <= 23_000, "measured PWM period at nominal 50 Hz")
    entries = profile["joints"]
    require(isinstance(entries, list) and len(entries) == 4, "joints")
    rows = []
    endpoints = []
    for (name, channel, item) in zip(JOINTS, CHANNELS, entries):
        require(item.get("id") == name and item.get("channel") == channel, f"{name} mapping")
        low, high, home = (float(number(item.get(k), f"{name}.{k}"))
                           for k in ("logical_min_deg", "logical_max_deg", "home_deg"))
        require(-360 <= low <= home <= high <= 360 and low < high, f"{name} logical limits")
        pmin, pmax, phome = (integer(item.get(k), f"{name}.{k}")
                             for k in ("pulse_min_us", "pulse_max_us", "home_us"))
        require(0 < pmin <= phome <= pmax < period and pmin < pmax, f"{name} pulse limits")
        inverted = item.get("inverted")
        require(type(inverted) is bool, f"{name}.inverted")
        require((home != low or phome == (pmax if inverted else pmin)) and
                (home != high or phome == (pmin if inverted else pmax)), f"{name} endpoint mapping")
        require(home in (low, high) or pmin < phome < pmax, f"{name} monotonic home mapping")
        speed, accel = (float(number(item.get(k), f"{name}.{k}"))
                        for k in ("max_velocity_deg_s", "max_acceleration_deg_s2"))
        require(0 < speed <= 180 and 0 < accel <= 360, f"{name} bounded speed/acceleration")
        startup = float(number(item.get("startup_deg"), f"{name}.startup_deg"))
        require(low <= startup <= high, f"{name}.startup_deg within measured limits")
        if name != "gripper":
            require(startup == (0 if name == "base" else 45), f"{name} startup must match 0/45/45")
        else:
            require(startup == home, "gripper startup must be open")
        rows.append((low, high, home, pmin, pmax, phome, inverted, speed, accel, startup))
        endpoints.append((low, high, home))

    item = entries[3]
    opened, closed, hold = (integer(item.get(k), f"gripper.{k}")
                            for k in ("open_us", "close_us", "max_closed_hold_ms"))
    require(opened == rows[3][5] and rows[3][3] <= closed <= rows[3][4] and opened != closed,
            "gripper measured open/close and open home")
    require(hold == 5000, "gripper software deadline must be 5000 ms (not a thermal safety certification)")
    tick = period / 4096
    require(all(math.ceil(row[3] / tick) < math.floor(row[4] / tick) for row in rows) and
            math.ceil(min(opened, closed) / tick) < math.floor(max(opened, closed) / tick),
            "PWM range must contain at least two usable ticks")

    wearable = profile["wearable"]
    roll, pitch = (integer(wearable.get(k), k) for k in ("roll_axis_sign", "pitch_axis_sign"))
    require(roll in (-1, 1) and pitch in (-1, 1), "wearable axis sign")
    require((roll, pitch) == (1, -1), "fixed verified wrist mapping signs +1/-1")
    deadzone = float(number(wearable.get("deadzone_deg_starting_point"), "deadzone"))
    require(0 < deadzone <= 20, "deadzone")
    workcell = profile["workcell"]
    require(workcell.get("base_fixed") is True and workcell.get("joint_space_validated") is True,
            "workcell.base_fixed / joint_space_validated")
    capture_verified = workcell.get("gripper_capture_surface_verified", False)
    require(type(capture_verified) is bool, "workcell.gripper_capture_surface_verified")
    regions = workcell.get("safe_joint_regions")
    require(isinstance(regions, list) and 1 <= len(regions) <= 8, "safe_joint_regions")
    boxes = []
    for region in regions:
        a, b = region["min_deg"], region["max_deg"]
        require(len(a) == len(b) == 4, "four coordinates per region bound")
        mins = [float(number(v, "region min")) for v in a]
        maxs = [float(number(v, "region max")) for v in b]
        require(all(low <= v <= w <= high for (low, high, _), v, w in zip(endpoints, mins, maxs)),
                "region exceeds measured joint limits")
        require(mins[3] <= endpoints[3][2] <= maxs[3],
                "every workcell region must permit safe gripper opening")
        boxes.append((mins, maxs))
    require(any(all(v <= h <= w for (_, _, h), v, w in zip(endpoints, a, b))
                for a, b in boxes), "home outside all verified regions")
    require(any(all(v <= row[-1] <= w for row, v, w in zip(rows, a, b))
                for a, b in boxes), "startup outside all verified regions")

    source = dict(profile)
    source.pop("calibration_id", None)
    canonical = json.dumps(source, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
                           allow_nan=False).encode("utf-8")
    profile_id = "sha256:" + hashlib.sha256(canonical).hexdigest()
    require(profile.get("calibration_id") in (None, profile_id), "stale calibration_id")
    cpp = lambda values: "{" + ", ".join(map(literal, values)) + "}"
    joint_text = ",\n    ".join(cpp(r) for r in rows)
    region_text = ",\n    ".join("{" + cpp(a) + ", " + cpp(b) + "}" for a, b in boxes)
    return ('#pragma once\n#include "ArmCalibration.h"\n\n'
            '// Generated from an explicitly verified calibration. Do not hand-edit.\n'
            f'inline constexpr arm::Calibration kArmProfile{{\n'
            f'  true, "{profile_id}",\n  {{\n    {joint_text}\n  }},\n'
            f'  {opened}, {closed}, {hold}, {literal(capture_verified)}, {literal(tick)}, '
            f'{roll}, {pitch}, {literal(deadzone)}, {len(boxes)},\n'
            f'  {{\n    {region_text}\n  }}\n}};\n'), profile_id


def unique_fields(pairs):
    values = {}
    for key, value in pairs:
        require(key not in values, f"duplicate JSON field {key}")
        values[key] = value
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration", type=Path)
    parser.add_argument("header", type=Path)
    args = parser.parse_args()
    try:
        profile = json.loads(args.calibration.read_text(encoding="utf-8-sig"), object_pairs_hook=unique_fields)
        header, profile_id = generate(profile)
        args.header.write_text(header, encoding="ascii")
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        parser.error(str(error))
    print(profile_id)


if __name__ == "__main__":
    main()
