"""Bounded v1 NDJSON and read-only compatibility with legacy diagnostic telemetry."""
import json
from math import isfinite

LIMIT = 512
JOINTS = ["base", "shoulder", "elbow", "gripper"]


def joints(value):
    return isinstance(value, list) and len(value) == 4 and all(
        finite_number(x) for x in value
    )


def finite_number(value):
    return ((type(value) is int and -1e308 <= value <= 1e308) or
            (type(value) is float and isfinite(value)))


def integer(value):
    return type(value) is int and 0 <= value <= 0xFFFFFFFF


def _reject_constant(value):
    raise ValueError(f"Nonfinite JSON: {value}")

def _parse_float(value):
    result = float(value)
    if not isfinite(result):
        raise ValueError("Nonfinite JSON float")
    return result



def decode(raw: bytes):
    if len(raw) > LIMIT or not raw.endswith(b"\n"):
        raise ValueError("Invalid NDJSON line length or terminator")
    data = json.loads(raw, parse_constant=_reject_constant, parse_float=_parse_float)
    if not isinstance(data, dict) or type(data.get("v")) is not int or data["v"] != 1:
        raise ValueError("Invalid protocol version")
    if not integer(data.get("seq")) or data.get("type") not in (
        "hello_ack", "ack", "nack", "state", "imu", "event"
    ):
        raise ValueError("Invalid sequence or message type")
    if data["type"] == "state":
        for key in ("q_req", "q_out"):
            if not joints(data.get(key)) and data.get(key) is not None:
                raise ValueError("Invalid target vector")
        if not integer(data.get("t_ms")):
            raise ValueError("Invalid state time")
    if data["type"] == "imu":
        for key in ("roll_deg", "pitch_deg"):
            if not finite_number(data.get(key)):
                raise ValueError("Invalid IMU angle")
    return data


def encode(data):
    raw = json.dumps(data, separators=(",", ":"), allow_nan=False).encode() + b"\n"
    if len(raw) > LIMIT:
        raise ValueError("Message exceeds 512-byte limit")
    return raw


def newer(seq, previous):
    return previous is None or 0 < (seq - previous) % (2**32) < (2**31)


def diagnostic(raw: bytes):
    if len(raw) > LIMIT or not raw.startswith(b"role=robot "):
        return None
    try:
        pairs = dict(token.split("=", 1) for token in raw.decode("ascii").split())
        if pairs.get("output") != "READ_ONLY":
            return None
        return {
            "type": "diagnostic",
            "roll_deg": int(pairs["roll"]) / 100,
            "pitch_deg": int(pairs["pitch"]) / 100,
            "age_ms": int(pairs["age_ms"]),
            "valid": pairs["link"] == "FRESH" and int(pairs["flags"]) & 5 == 5,
            "deadman": bool(int(pairs["flags"]) & 2),
            "mode": int(pairs["mode"]),
            "pca_readable": pairs.get("readable") == "1",
        }
    except (ValueError, KeyError):
        return None
