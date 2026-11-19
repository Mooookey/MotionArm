"""CSV presentation; legacy semantics are confined to this history boundary."""
import csv
import io
import json

from .virtual import POSE_KEYS


def vector(value, size):
    return value if isinstance(value, list) and len(value) == size else [None] * size


def recording_csv(record):
    output = io.StringIO()
    writer = csv.writer(output)
    metadata = record.get("metadata") or {}
    targets = [f"{kind}_{key}" for kind in ("requested", "output")
               for key in (*POSE_KEYS, "gripper_open_ratio")]
    common = ["t_ms", "source", "executor", "angle_convention", *targets]
    modern = metadata.get("schema_version") == 3
    if modern:
        writer.writerow([*common, "controller_version", "attitude_representation", "mounting_profile",
                         "roll_deg", "pitch_deg", *[f"q_{a}" for a in "wxyz"],
                         *[f"reference_q_{a}" for a in "wxyz"],
                         "relative_roll_deg", "relative_pitch_deg", "relative_z_deg",
                         *[f"gyro_{a}_dps" for a in "xyz"], *[f"accel_{a}_g" for a in "xyz"],
                         "norm_g", "temp_c", "gyro_range_dps", "reject_reason", "rejected_samples",
                         "errors", "who", "calibration_samples", "valid", "imu_seq", "device_ms",
                         "a", "b", "ac", "bc", "owner"])
    else:
        writer.writerow([*common, "roll_deg", "pitch_deg", "a", "b", "ac", "bc",
                         "short", "long", "mode", "owner", "legacy_payload"])
    for frame in record["frames"]:
        imu, buttons = frame.get("imu", {}), frame.get("buttons", {})
        row = [frame["t_ms"], record["source"], metadata.get("executor"), metadata.get("angle_convention"),
               *[frame.get(kind, {}).get(key) for kind in ("requested", "output_target")
                 for key in (*POSE_KEYS, "gripper_open_ratio")]]
        if modern:
            follow = frame.get("follow", {})
            row += [metadata.get("controller_version"), metadata.get("attitude_representation"),
                    metadata.get("mounting", {}).get("profile"), imu.get("roll_deg"), imu.get("pitch_deg"),
                    *vector(imu.get("quaternion"), 4), *vector(metadata.get("reference_quaternion"), 4),
                    *vector(follow.get("delta_rotation_vector_deg"), 3),
                    *vector(imu.get("gyro_dps"), 3), *vector(imu.get("accel_g"), 3),
                    *[imu.get(k) for k in ("norm_g", "temp_c", "gyro_range_dps", "reject_reason",
                                          "rejected_samples", "errors", "who", "calibration_samples", "valid")],
                    frame.get("imu_seq"), frame.get("device_ms"),
                    *[buttons.get(k) for k in ("a", "b", "ac", "bc")], frame.get("owner")]
        else:
            row += [imu.get("roll_deg"), imu.get("pitch_deg"),
                    *[buttons.get(k) for k in ("a", "b", "ac", "bc", "short", "long")],
                    frame.get("mode"), frame.get("owner"), json.dumps(frame, ensure_ascii=False)]
        writer.writerow(row)
    return "\ufeff" + output.getvalue()
