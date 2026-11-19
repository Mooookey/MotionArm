"""Bounded robot USB v2 management and atomic, read-only ESP-NOW snapshots.

No USB target commands exist here. Motion authority stays on the robot.
"""
import asyncio
from concurrent.futures import Future
from copy import deepcopy
import json
from queue import Empty, Full, Queue
import threading
import time

from .protocol import finite_number, integer, newer
from .wearable import validate_frame

ROBOT_MAC = "C0:CD:D6:D0:9B:54"
GROUPS = frozenset(("control", "imu", "targets", "gripper", "health"))
POSE_KEYS = ("base_yaw_deg", "shoulder_down_deg", "elbow_down_deg", "gripper_open_ratio")


def pose_valid(value):
    return (isinstance(value, dict) and set(value) == set(POSE_KEYS)
            and all(finite_number(value[k]) for k in POSE_KEYS)
            and 0 <= value["gripper_open_ratio"] <= 1)


def decode(raw):
    if len(raw) > 512 or not raw.endswith(b"\n"):
        raise ValueError("robot frame length")
    def reject(value):
        raise ValueError("nonfinite JSON")
    item = json.loads(raw, parse_constant=reject)
    if not isinstance(item, dict) or item.get("v") != 2 or not integer(item.get("seq")):
        raise ValueError("robot USB v2 required")
    if item.get("type") not in GROUPS | {"hello_ack", "ack", "nack", "event"}:
        raise ValueError("robot frame type")
    if not isinstance(item.get("boot_id"), str) or not 1 <= len(item["boot_id"]) <= 64:
        raise ValueError("robot boot identity")
    return item


def wearable_frame(item):
    keys = "ms age_ms who valid cal q gyro_dps accel_g a b ac bc norm_g temp_c errors rej gr rc".split()
    try:
        return validate_frame({"v": 1, "att": 3, "role": "wearable", "boot": item["wearable_boot"],
                               "seq": item["wearable_seq"], **{key: item[key] for key in keys}})
    except (KeyError, TypeError) as exc:
        raise ValueError("incomplete robot IMU diagnostics") from exc


class SnapshotAssembler:
    """Only a complete five-part snapshot may change the public target state."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.clear()

    def clear(self):
        self.boot = None
        self.key = None
        self.parts = {}
        self.bytes = 0
        self.started = None
        self.published = None

    def feed(self, item, size=0):
        if item["type"] not in GROUPS:
            return None
        if not integer(item.get("snapshot_id")) or not integer(item.get("t_ms")):
            raise ValueError("robot snapshot identity")
        if self.boot != item["boot_id"]:
            self.clear()
            self.boot = item["boot_id"]
        key = (item["snapshot_id"], item["t_ms"])
        if self.published is not None and not newer(key[0], self.published):
            return None
        if self.key != key:
            if self.key is not None and not newer(key[0], self.key[0]):
                return None
            self.key, self.parts, self.bytes, self.started = key, {}, 0, self.clock()
        if self.clock() - self.started > .5:
            self.parts.clear()
            return None
        if item["type"] in self.parts:
            return None
        self.bytes += size
        if self.bytes > 2048:
            self.parts.clear()
            raise ValueError("robot snapshot budget")
        self.parts[item["type"]] = deepcopy(item)
        if set(self.parts) != GROUPS:
            return None
        result = self.parts
        self.validate(result)
        self.published = key[0]
        self.parts = {}
        return result

    @staticmethod
    def validate(parts):
        c, t, g, h = (parts[k] for k in ("control", "targets", "gripper", "health"))
        if h.get("mode") != "robot_no_power" or h.get("output_locked") is not True or h.get("mac", "").upper() != ROBOT_MAC:
            raise ValueError("robot no-power identity/output lock mismatch")
        if type(h.get("pca_readable")) is not bool or not isinstance(h.get("wireless"), dict):
            raise ValueError("robot health")
        if "ev_drop" in h and not integer(h["ev_drop"]):
            raise ValueError("robot event drop counter")
        for key in ("enabled", "armed", "angle_ambiguous", "can_enable"):
            if type(c.get(key)) is not bool:
                raise ValueError("robot control flags")
        if c.get("owner") not in ("NONE", "WEARABLE"):
            raise ValueError("robot control owner")
        if not isinstance(c.get("reason"), str) or len(c["reason"]) > 96 or (c.get("fault") is not None and not isinstance(c["fault"], str)):
            raise ValueError("robot control reason")
        q = c.get("reference_quaternion")
        vector = c.get("delta_rotation_vector_deg")
        if q is not None and (not isinstance(q, list) or len(q) != 4 or not all(finite_number(v) for v in q)
                              or abs(sum(v*v for v in q) ** .5 - 1) > .01):
            raise ValueError("robot reference quaternion")
        if vector is not None and (not isinstance(vector, list) or len(vector) != 3 or not all(finite_number(v) for v in vector)):
            raise ValueError("robot relative rotation")
        if c["enabled"] and (q is None or vector is None):
            raise ValueError("initialized robot has no reference")
        if not all(pose_valid(t.get(k)) for k in ("requested", "output_target")) or t.get("measured") is not None:
            raise ValueError("robot software targets")
        if (not isinstance(t.get("velocity_deg_s"), list) or len(t["velocity_deg_s"]) != 3
                or not all(finite_number(v) for v in t["velocity_deg_s"]) or not isinstance(t.get("limits"), list)
                or len(t["limits"]) != 3 or not all(type(v) is bool for v in t["limits"])):
            raise ValueError("robot target diagnostics")
        if g.get("state") not in ("OPEN_READY", "CLOSING", "CLOSED", "OPENING") or g.get("limit_ms") != 5000:
            raise ValueError("robot gripper policy")
        if not all(integer(g.get(k)) for k in ("elapsed_ms", "remaining_ms", "cycle_id")) or g["remaining_ms"] > 5000:
            raise ValueError("robot gripper timer")
        if type(g.get("reclose_blocked")) is not bool:
            raise ValueError("robot gripper gate")
        if g.get("auto_open_reason") is not None and not isinstance(g["auto_open_reason"], str):
            raise ValueError("robot gripper reason")
        # An uninitialized/no-input robot may have no wearable boot yet. Its
        # diagnostic placeholder is never treated as valid control input.
        imu = parts["imu"]
        if imu.get("wearable_boot"):
            wearable_frame(imu)
        elif imu.get("valid"):
            raise ValueError("robot wearable identity")


class RobotLink:
    def __init__(self, receiver, device, serial_factory=None):
        self.receiver, self.device = receiver, device
        self.serial_factory = serial_factory
        self.halt = threading.Event()
        self.commands = Queue(maxsize=4)
        self.stops = Queue(maxsize=1)
        self.thread = None
        self.heartbeat_until = 0.

    def start(self):
        loop = asyncio.get_running_loop()
        def emit(item):
            if not self.halt.is_set():
                loop.call_soon_threadsafe(self.receiver, item)
        self.thread = threading.Thread(target=self._run, args=(emit,), daemon=True)
        self.thread.start()

    def renew(self, seconds=.75):
        self.heartbeat_until = time.monotonic() + seconds

    def disarm(self):
        self.heartbeat_until = 0.

    async def request(self, kind, **payload):
        if kind not in ("init", "stop"):
            raise ValueError("USB motion targets are unavailable")
        future = Future()
        try:
            (self.stops if kind == "stop" else self.commands).put_nowait((kind, payload, future, time.monotonic() + .7))
        except Full as exc:
            raise ValueError("robot command queue busy") from exc
        return await asyncio.wait_for(asyncio.wrap_future(future), .9)

    async def close(self):
        self.disarm()
        self.halt.set()
        if self.thread:
            await asyncio.to_thread(self.thread.join, 2)

    @staticmethod
    def resolve(future, value=None, error=None):
        if future.set_running_or_notify_cancel():
            if error:
                future.set_exception(ValueError(error))
            else:
                future.set_result(value)

    def _run(self, emit):
        import serial
        factory = self.serial_factory or serial.Serial
        while not self.halt.is_set():
            emit({"type": "connecting"})
            try:
                port = factory(baudrate=115200, timeout=.02, write_timeout=.1)
                port.port, port.dtr, port.rts = self.device, False, False
                with port:
                    self._listen(port, emit)
            except (OSError, ValueError, TimeoutError, serial.SerialException) as exc:
                self.disarm()
                emit({"type": "offline", "detail": str(exc)})
            self._reject_queued()
            self.halt.wait(.5)

    def _reject_queued(self):
        for queue in (self.commands, self.stops):
            while True:
                try:
                    _, _, future, _ = queue.get_nowait()
                except Empty:
                    break
                self.resolve(future, error="robot session closed")

    def _listen(self, port, emit):
        boot = session = None
        previous = None
        seq = 0
        pending = bytearray()
        overflow = False
        inflight = None
        next_hello = next_heartbeat = 0.
        deadline = time.monotonic() + 3
        assembler = SnapshotAssembler()
        def send(kind, **payload):
            nonlocal seq
            seq = (seq + 1) & 0xffffffff
            item = {"v": 2, "type": kind, "seq": seq, **payload}
            raw = json.dumps(item, separators=(",", ":"), allow_nan=False).encode() + b"\n"
            if len(raw) > 512:
                raise ValueError("robot command length")
            if port.write(raw) != len(raw):
                raise ValueError("partial robot command")
            return seq
        try:
            while not self.halt.is_set():
                now = time.monotonic()
                if now >= deadline:
                    raise TimeoutError("robot USB handshake/telemetry timeout")
                if boot is None and now >= next_hello:
                    send("hello", role="pc")
                    next_hello = now + .5
                if session and now < self.heartbeat_until and now >= next_heartbeat:
                    send("heartbeat", session_id=session)
                    next_heartbeat = now + .1
                try:
                    request = self.stops.get_nowait()
                except Empty:
                    request = None
                if request:
                    self.disarm()
                    if inflight:
                        self.resolve(inflight[2], error="preempted by stop")
                        inflight = None
                    self._reject_queued()
                if request is None and inflight is None:
                    try:
                        request = self.commands.get_nowait()
                    except Empty:
                        pass
                if request:
                    kind, payload, future, expires = request
                    if not session or expires <= now or future.cancelled():
                        self.resolve(future, error="no current robot session")
                    else:
                        sent = send(kind, session_id=session, **payload)
                        inflight = (sent, kind, future, expires)
                if inflight and now >= inflight[3]:
                    self.resolve(inflight[2], error="robot ACK timeout; outcome unknown")
                    self.disarm()
                    inflight = None
                chunk = port.read(max(1, min(port.in_waiting, 512)))
                for byte in chunk:
                    if not overflow:
                        pending.append(byte)
                        overflow = len(pending) > 512
                    if byte != 10:
                        continue
                    raw = bytes(pending) if not overflow else b""
                    pending.clear()
                    overflow = False
                    try:
                        item = decode(raw)
                    except (ValueError, UnicodeError):
                        continue
                    if item["type"] == "hello_ack":
                        if boot is not None:
                            if item["boot_id"] != boot:
                                raise ValueError("robot restarted")
                            continue
                        if (item.get("role") != "robot" or item.get("protocol_version") != 2
                                or item.get("mac", "").upper() != ROBOT_MAC or item.get("mode") != "robot_no_power"
                                or item.get("output_locked") is not True or not isinstance(item.get("session_id"), str)):
                            raise ValueError("robot identity or output lock rejected")
                        boot, session = item["boot_id"], item["session_id"]
                        self.disarm()  # HELLO/reconnect can never restore control.
                        emit(item)
                        deadline = time.monotonic() + 1
                        continue
                    if boot is None:
                        continue
                    if item["boot_id"] != boot:
                        raise ValueError("robot restarted")
                    if item["type"] in ("ack", "nack"):
                        if inflight and item["seq"] == inflight[0] and item.get("request_type") == inflight[1]:
                            self.resolve(inflight[2], item)
                            inflight = None
                        continue
                    if not newer(item["seq"], previous):
                        continue
                    previous = item["seq"]
                    if item["type"] == "event":
                        if (integer(item.get("event_seq")) and integer(item.get("t_ms"))
                                and isinstance(item.get("code"), str) and len(item["code"]) <= 96):
                            emit(item)
                    else:
                        snapshot = assembler.feed(item, len(raw))
                        if snapshot:
                            emit({"type": "snapshot", "parts": snapshot})
                            deadline = time.monotonic() + 1
        finally:
            self.disarm()
            if inflight:
                self.resolve(inflight[2], error="robot disconnected; outcome unknown")
