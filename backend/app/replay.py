"""Preflight and interpolation for recorded software targets."""
from bisect import bisect_right
from .protocol import JOINTS, finite_number, joints

START_TOLERANCE = 2.0


def regions_valid(regions):
    return isinstance(regions, list) and bool(regions) and all(
        isinstance(region, dict) and joints(region.get("min_deg")) and joints(region.get("max_deg"))
        for region in regions
    )


def interpolate(frames, elapsed_ms, timestamps=None):
    if timestamps is None:
        timestamps = [row["t_ms"] for row in frames]
    index = bisect_right(timestamps, elapsed_ms)
    if index == 0:
        return frames[0]["q_out"][:]
    if index == len(frames):
        return frames[-1]["q_out"][:]
    a, b = frames[index - 1:index + 1]
    ratio = (elapsed_ms - a["t_ms"]) / (b["t_ms"] - a["t_ms"])
    return [x + ratio * (y - x) for x, y in zip(a["q_out"], b["q_out"])]


def normalize_limits(profile):
    configured = profile.get("joints")
    if not isinstance(configured, list) or len(configured) != 4:
        return None
    if [axis.get("id") for axis in configured if isinstance(axis, dict)] != JOINTS:
        return None
    for axis in configured:
        numbers = [axis.get(x) for x in ("logical_min_deg", "logical_max_deg",
                                        "max_velocity_deg_s", "max_acceleration_deg_s2")]
        if not all(map(finite_number, numbers)):
            return None
        low, high, velocity, acceleration = numbers
        if low >= high or velocity <= 0 or acceleration <= 0:
            return None
    hold = configured[-1].get("max_closed_hold_ms")
    return configured if finite_number(hold) and hold > 0 else None


def preflight(record, status, profile, operator_confirmed, executables):
    frames = record["frames"]
    angles_valid = bool(frames) and all(joints(row.get("q_out")) for row in frames)
    times_valid = bool(frames) and all(type(row.get("t_ms")) is int for row in frames)
    complete = (record["status"] == "completed" and len(frames) >= 2 and
                record.get("protocol_version") == 1 and angles_valid and times_valid and
                frames[0]["t_ms"] == 0 and
                all(0 < b["t_ms"] - a["t_ms"] <= 250 for a, b in zip(frames, frames[1:])))
    workcell = profile.get("workcell")
    if not isinstance(workcell, dict):
        workcell = {}
    calibration_id = profile.get("calibration_id")
    compatible = (bool(calibration_id) and calibration_id == record.get("calibration_id") ==
                  status.get("calibration_id") and record.get("source") == status["source"] and
                  record.get("workcell_id") == workcell.get("workcell_id"))
    q = status["joints"]["applied_target_deg"]
    begins_at_pose = (angles_valid and joints(q) and
                      all(abs(a - b) <= START_TOLERANCE for a, b in zip(q, frames[0]["q_out"])))
    limits = normalize_limits(profile)
    within_limits = bool(complete and limits) and all(
        axis["logical_min_deg"] <= angle <= axis["logical_max_deg"]
        for sample in frames for axis, angle in zip(limits, sample["q_out"])
    )
    speed_ok = within_limits
    if speed_ok:
        previous, last_dt = None, None
        for left, right in zip(frames, frames[1:]):
            dt = (right["t_ms"] - left["t_ms"]) / 1000
            velocities = [(b - a) / dt for a, b in zip(left["q_out"], right["q_out"])]
            if any(abs(v) > axis["max_velocity_deg_s"] + 1e-6 for v, axis in zip(velocities, limits)):
                speed_ok = False
                break
            midpoint_dt = (dt + last_dt) / 2 if last_dt is not None else dt
            if previous is not None and any(
                abs(new - old) / midpoint_dt > axis["max_acceleration_deg_s2"] + 1e-6
                for new, old, axis in zip(velocities, previous, limits)
            ):
                speed_ok = False
                break
            previous, last_dt = velocities, dt
    hold_ok = bool(complete and limits)
    regions = workcell.get("safe_joint_regions") or []
    workspace_ok = status["source"] == "mock" or bool(
        complete and regions_valid(regions) and all(
            any(all(low <= min(a, b) <= max(a, b) <= high
                    for a, b, low, high in zip(left["q_out"], right["q_out"],
                                                region["min_deg"], region["max_deg"]))
                for region in regions)
            for left, right in zip(frames, frames[1:])
        )
    )
    if hold_ok:
        gripper = limits[-1]
        # open_deg or home_deg is the recorded open reference. Any departure from it
        # counts toward hold time, including an unreported close or intermediate targets.
        open_deg = gripper.get("open_deg", gripper.get("home_deg"))
        closed_since = None
        hold_ok = finite_number(open_deg)
        if hold_ok:
            for sample in frames:
                when = sample["t_ms"]
                if closed_since is not None and when - closed_since > gripper["max_closed_hold_ms"]:
                    hold_ok = False
                    break
                if abs(sample["q_out"][3] - open_deg) < 0.5:
                    closed_since = None
                elif closed_since is None:
                    closed_since = when
            hold_ok &= closed_since is None
            if status["source"] == "serial" and workcell.get("gripper_capture_surface_verified") is not True:
                hold_ok &= all(abs(sample["q_out"][3] - open_deg) < 0.5 for sample in frames)
    checks = (
        (complete, "RECORD_INCOMPLETE"),
        (compatible, "CALIBRATION_MISMATCH"),
        (begins_at_pose, "INITIAL_TARGET_MISMATCH"),
        (within_limits, "JOINT_LIMITS_INVALID"),
        (workspace_ok, "TRAJECTORY_OUTSIDE_VERIFIED_REGIONS"),
        (speed_ok, "SPEED_OR_ACCELERATION_LIMIT"),
        (hold_ok, "GRIPPER_HOLD_TIME"),
        (operator_confirmed, "WORKCELL_NOT_CONFIRMED"),
        (status["connection"]["robot"] == "online", "ROBOT_OFFLINE"),
        (status["control"]["owner"] == "NONE" and not status["control"]["fault"]
         and status["control"]["state"] in ("READY", "HOLD"),
         "CONTROL_NOT_READY"),
        (executables, "MOTION_NOT_VERIFIED"),
    )
    violations = [reason for ok, reason in checks if not ok]
    return {
        "preflight_id": None, "record_id": record["id"], "ok": not violations, "may_start": not violations,
        "calibration_id": calibration_id, "calibration_compatible": compatible,
        "first_q_within_tolerance": begins_at_pose, "record_complete": complete,
        "speed_limits_ok": speed_ok, "gripper_hold_time_ok": hold_ok,
        "operator_workcell_confirmed": operator_confirmed, "violations": violations,
        "warnings": ["JOINT_POSITION_NOT_EXTERNALLY_MEASURED"],
        "initial_q_tolerance_deg": START_TOLERANCE, "speed_scale": 1.0,
        "operator_checklist": ["确认机械初态与示教起点一致", "确认工作区和端点无障碍", "确认物理断电开关可用"],
    }
