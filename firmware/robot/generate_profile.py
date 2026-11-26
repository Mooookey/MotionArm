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
    if profile.get("schema_version") == 3:
        return generate_trial(profile)
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


def generate_trial(profile):
    """Scoped commissioning profile: does not assert full mechanical certification."""
    require(profile.get("purpose") == "empty_load_small_range_trial", "scoped trial purpose")
    require(profile.get("source_commit") == "ad5644f", "historical calibration source")
    require(profile.get("supply_rating_v") == 5 and profile.get("supply_rating_a") == 6,
            "operator reported 5V/6A rating")
    require(profile.get("full_workspace_verified") is False and profile.get("load_verified") is False and
            profile.get("thermal_safety_verified") is False, "unverified certification fields remain false")
    require(profile.get("onsite_confirmation_required") is True, "runtime confirmation gate")
    require(integer(profile.get("gripper_limit_ms"), "gripper_limit_ms") == 5000, "5000ms non-renewable deadline")
    require(profile.get("tick_nominal_us") == CALIBRATOR_TICK_US, "historical nominal timebase")
    require(profile.get("base_mapping") == [[-45, 1010], [0, 1610], [45, 2000]], "piecewise base anchors")
    require(profile.get("shoulder_mapping") == [[0, 1630], [45, 1030]], "upright shoulder anchor")
    require(profile.get("elbow_mapping") == [[0, 1570], [45, 1050]], "upright elbow anchor")
    require(profile.get("startup_deg") == [0, 45, 45], "startup angles")
    limits = profile.get("operating_limits_deg")
    require(limits == [[-45, 45], [20, 60], [20, 60]], "approved observation trial limits")
    signs = profile.get("joint_signs")
    require(signs == [-1, -1] and all(type(v) is int for v in signs), "powered base/pitch signs -1/-1")
    require(profile.get("mapping_version") == "coordinated-angle-v4", "explicit powered mapping version")
    diagnostics = profile.get("diagnostic_offsets_deg")
    require(diagnostics == [[-15, -10, -5, 0, 5, 10, 15], [-10, -5, 0, 5, 10], [-10, -5, 0, 5, 10]],
            "approved discrete diagnostic offsets (expanded range is follow-only)")
    for group in (*limits, *diagnostics, *profile["base_mapping"], *profile["shoulder_mapping"], *profile["elbow_mapping"], profile["startup_deg"]):
        for value in group:
            number(value, "finite numeric trial bounds / diagnostic offsets")
    require(number(profile.get("deadzone_deg"), "deadzone_deg") == 5, "unchanged 5 degree deadzone")
    require(number(profile.get("speed_deg_s"), "speed_deg_s") == 15 and
            number(profile.get("acceleration_deg_s2"), "acceleration_deg_s2") == 30 and
            number(profile.get("gripper_rate_s"), "gripper_rate_s") == 1, "approved trial rates")
    require(integer(profile.get("gripper_open_us"), "gripper_open_us") == 1130 and
            integer(profile.get("gripper_close_us"), "gripper_close_us") == 1630, "gripper ratio anchors")
    canonical = json.dumps(profile, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    identity = "sha256:" + hashlib.sha256(canonical).hexdigest()
    cpp = lambda values: "{" + ",".join(literal(v) for v in values) + "}"
    low, high = ([row[i] for row in limits] for i in (0, 1))
    diagnostic_rows = ",".join(cpp(row + [0] * (7-len(row))) for row in diagnostics)
    counts = [len(row) for row in diagnostics]
    return ('#pragma once\n#include "PoweredOutput.h"\n'
            '// Generated scoped trial; full travel, load and thermal safety remain unverified.\n'
            'inline constexpr robot::TrialProfile kPoweredTrialProfile{\n'
            f'  "{identity}",\n'
            f'  {cpp(low)}, {cpp(high)}, {cpp(profile["startup_deg"])}, '
            f'{literal(profile["speed_deg_s"])},{literal(profile["acceleration_deg_s2"])},{literal(profile["gripper_rate_s"])},\n'
            f'  {cpp([v[1] for v in profile["base_mapping"]])}, '
            f'{cpp([v[1] for v in profile["shoulder_mapping"]])}, '
            f'{cpp([v[1] for v in profile["elbow_mapping"]])}, '
            f'{profile["gripper_open_us"]},{profile["gripper_close_us"]},{literal(profile["tick_nominal_us"])},\n'
            f'  {literal(profile["deadzone_deg"])}, {{{diagnostic_rows}}}, {cpp(counts)},\n'
            f'  {cpp(signs)}, "{profile["mapping_version"]}"\n}};\n'), identity


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
    parser.add_argument("--trial", action="store_true", help="explicitly generate the bounded empty-load trial, not a verified full profile")
    args = parser.parse_args()
    try:
        profile = json.loads(args.calibration.read_text(encoding="utf-8-sig"), object_pairs_hook=unique_fields)
        if args.trial:
            require(profile.get("schema_version") == 3, "--trial requires scoped schema3")
            header, profile_id = generate_trial(profile)
        else:
            require(profile.get("schema_version") != 3, "scoped trial requires explicit --trial")
            header, profile_id = generate(profile)
        args.header.write_text(header, encoding="ascii")
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        parser.error(str(error))
    print(profile_id)


if __name__ == "__main__":
    main()
