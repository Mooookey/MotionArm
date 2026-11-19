"""Local-only MotionArm API, recording and guarded controller access."""
import asyncio
import csv
import io
import json
import os
import time
from contextlib import asynccontextmanager, suppress
from hashlib import sha256
from pathlib import Path
from typing import Literal
from uuid import uuid4

from fastapi import Body, FastAPI, Header, HTTPException, WebSocket
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, Field, StrictBool

from .protocol import JOINTS, joints
from .recordings import Recordings
from .replay import interpolate, normalize_limits, preflight, regions_valid
from .serial_bridge import SerialBridge

ROOT = Path(__file__).resolve().parents[2]
ANGLE_LABEL = "applied_servo_targets_not_encoder_readings"
LOOPBACK_HOSTS = {"localhost", "127.0.0.1", "[::1]", "testserver"}
DEFAULT_MOCK_ANGLES = [90.0, 65.0, 105.0, 25.0]


def fail(status, code, message):
    raise HTTPException(status, {"error_code": code, "message": message})


def mock_calibration():
    axes = [
        {"id": name, "channel": channel, "home_deg": default, "logical_min_deg": 0,
         "logical_max_deg": 180, "max_velocity_deg_s": 25, "max_acceleration_deg_s2": 100}
        for name, channel, default in zip(JOINTS, (14, 15, 13, 12), DEFAULT_MOCK_ANGLES)
    ]
    axes[-1].update(open_deg=25, max_closed_hold_ms=5000)
    return {"allow_arm": True, "calibration_id": "mock-v1", "joints": axes,
            "verified_on_hardware": False, "workcell": {"workcell_id": "mock"}}


def read_calibration(path):
    try:
        profile = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if isinstance(profile, dict):
            return profile
    except (OSError, ValueError):
        pass
    return {"allow_arm": False, "calibration_id": None}


def verified_profile_id(profile):
    board, workcell = profile.get("board"), profile.get("workcell")
    if not isinstance(board, dict) or not isinstance(workcell, dict):
        return None
    switches = [profile.get(key) for key in ("allow_arm", "verified_on_hardware", "operator_confirmed_initial_pose")]
    switches.extend(board.get(key) for key in ("physical_cutoff_tested", "gravity_support_tested"))
    switches.extend(workcell.get(key) for key in ("base_fixed", "joint_space_validated"))
    if (profile.get("schema_version") != 1 or not all(flag is True for flag in switches)
            or not normalize_limits(profile) or not regions_valid(workcell.get("safe_joint_regions"))):
        return None
    fields = {key: value for key, value in profile.items() if key != "calibration_id"}
    try:
        canonical = json.dumps(fields, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
    except (ValueError, TypeError):
        return None
    identity = "sha256:" + sha256(canonical.encode("utf-8")).hexdigest()
    return identity if profile.get("calibration_id") in (None, identity) else None


class Request(BaseModel):
    request_id: str | None = Field(default=None, max_length=128)


class Confirm(Request):
    operator_confirmed_safe: StrictBool = False


class Arm(Confirm):
    requested_owner: Literal["PC_MANUAL", "WEARABLE"] = "PC_MANUAL"


class Lease(Request):
    lease_id: str | None = Field(default=None, max_length=96)


class Target(Lease):
    q_deg: list[object]


class StartRecording(Request):
    name: str = Field(default="未命名示教", min_length=1, max_length=120)


class Preflight(Request):
    record_id: str
    operator_workcell_confirmed: StrictBool = False


class StartReplay(Confirm):
    preflight_id: str


class Runtime:
    def __init__(self, source, database, calibration):
        self.source = source
        self.mock = source == "mock"
        self.profile = mock_calibration() if self.mock else read_calibration(calibration)
        self.verified_profile_id = None if self.mock else verified_profile_id(self.profile)
        if self.verified_profile_id:
            self.profile["calibration_id"] = self.verified_profile_id
        self.records = Recordings(database)
        self.boot_id = "mock-" + uuid4().hex if self.mock else None
        self.calibration_id = "mock-v1" if self.mock else None
        self.fw_version = "mock-v1" if self.mock else None
        self.connection = {"robot": "online" if self.mock else "offline",
                           "wearable": "online" if self.mock else "offline",
                           "serial_reconnecting": not self.mock}
        self.state = "TELEOP" if self.mock else "SAFE_BOOT"
        self.owner = "WEARABLE" if self.mock else "NONE"
        self.lease = None
        self.manual_target = None
        self.serial = None
        self.session = None
        self.session_channels = None
        self.device_calibration = None
        self.pca_ok = False
        self.pose_confirmed = False
        self.control_lock = asyncio.Lock()
        self.control_generation = 0
        self.stopping = False
        self.motion_enabled = os.environ.get("MOTIONARM_ENABLE_MOTION") == "1"
        self.fault = None
        self.q_out = DEFAULT_MOCK_ANGLES[:] if self.mock else None
        self.q_req = self.q_out
        self.imu = {}
        self.last_recording = None
        self.replay = {"active": False, "record_id": None, "progress": 0.0}
        self.replay_task = None
        self.run_id = None
        self.tokens = {}
        self.memo = {}
        self.started = time.monotonic()
        self.last_robot = self.last_imu = self.last_state = None
        self.mock_seq = 0

    def receive(self, frame):
        kind = frame["type"]
        if kind == "offline":
            self.connection.update(robot="offline", wearable="offline", serial_reconnecting=True,
                                   detail=frame.get("detail"))
            if self.records.active:
                self.last_recording = self.records.finish("interrupted", "serial disconnected")
            self.state, self.owner = "DISCONNECTED", "NONE"
            self.imu, self.q_out, self.q_req = {}, None, None
            self.calibration_id = self.boot_id = None
            self.session = self.lease = self.last_state = self.device_calibration = None
            self.manual_target = None
            self.pca_ok = False
            self.pose_confirmed = False
            self.control_generation += 1
            return
        self.last_robot = time.monotonic()
        if kind == "diagnostic":
            self.pca_ok = frame["pca_readable"]
            self.connection.update(robot="diagnostic", serial_reconnecting=False,
                                   detail="PCA9685 在线，执行端运行只读诊断" if frame["pca_readable"]
                                   else "执行端运行只读诊断，PCA9685 未应答")
            self.state = "READ_ONLY"
        else:
            self.connection.update(robot="online", serial_reconnecting=False, detail=None)
        if kind == "hello_ack":
            if self.boot_id != frame["boot_id"]:
                self.records.finish("interrupted", "robot boot id changed")
                self.imu, self.q_out, self.q_req = {}, None, None
                self.last_imu = None
            self.boot_id = frame["boot_id"]
            self.session = frame["session_id"]
            self.session_channels = frame.get("channels")
            self.calibration_id = frame.get("calibration_id")
            self.fw_version = frame.get("fw_version")
            self.state, self.owner = frame.get("state", "SAFE_BOOT"), "NONE"
        elif kind == "state":
            self.state, self.owner, self.fault = frame.get("state"), frame.get("owner"), frame.get("fault")
            self.last_state = time.monotonic()
            self.device_calibration = frame.get("calib")
            self.pca_ok = frame.get("pca_ok") is True
            self.pose_confirmed = frame.get("operator_initial_pose_confirmed") is True
            if self.owner == "NONE" or frame.get("lease_id") != self.lease:
                self.lease = None
                self.manual_target = None
            self.q_req, self.q_out = frame.get("q_req"), frame.get("q_out")
            self.sample(frame["seq"], frame["t_ms"])
        elif kind in ("imu", "diagnostic"):
            self.imu = {k: frame.get(k) for k in ("roll_deg", "pitch_deg", "mode", "deadman", "valid", "age_ms")}
            self.last_imu = time.monotonic()
        elif kind == "event":
            self.records.note_event(frame)

    def sample(self, seq, t_ms):
        if self.records.active is None:
            return
        if self.owner != "WEARABLE" or self.state not in ("TELEOP", "RECORDING_TELEOP"):
            self.last_recording = self.records.finish("interrupted", "teleoperation stopped")
            return
        if not joints(self.q_out):
            self.last_recording = self.records.finish("interrupted", "missing software target")
            return
        self.records.append(self.boot_id, self.calibration_id if self.mock else self.device_calibration, t_ms, {
            "seq": seq, "pc_received_ns": time.monotonic_ns(), "q_req": self.q_req,
            "q_out": self.q_out, "roll_deg": self.imu.get("roll_deg"),
            "pitch_deg": self.imu.get("pitch_deg"), "mode": self.imu.get("mode"),
            "deadman": self.imu.get("deadman"), "owner": self.owner,
        })

    def snapshot(self):
        imu = self.imu
        age = imu.get("age_ms")
        if not self.mock and type(age) is int and self.last_imu is not None:
            age += int((time.monotonic() - self.last_imu) * 1000)
        # Diagnostic probe only reports once per second. Keep the reported age in
        # the UI, but use report cadence to decide if that diagnostic display is stale.
        valid_age = 1600 if self.connection["robot"] == "diagnostic" else 250
        valid = bool(imu.get("valid")) and (self.mock or (age is not None and age < valid_age))
        mode = {0: "SHOULDER", 1: "ELBOW"}.get(imu.get("mode"), imu.get("mode"))
        return {
            "event": "telemetry", "server_time_ms": int(time.time() * 1000),
            "source": self.source, "read_only": not self.can_control(), "calibration_id": self.calibration_id,
            "connection": {**self.connection, "wearable": "online" if valid else "offline",
                           "pca_ok": self.pca_ok if self.last_state is not None or self.connection["robot"] == "diagnostic" else None},
            "control": {"state": self.state, "owner": self.owner, "lease_id": self.lease if self.mock else None,
                        "has_lease": self.lease is not None,
                        "deadman": valid and bool(imu.get("deadman")), "mode": mode, "fault": self.fault,
                        "can_move": self.can_control() and self.owner == "PC_MANUAL"},
            "imu": {"roll_deg": imu.get("roll_deg"), "pitch_deg": imu.get("pitch_deg"),
                    "valid": valid, "age_ms": age},
            "joints": {"names": JOINTS, "requested_deg": self.q_req, "applied_target_deg": self.q_out,
                       "measured_deg": None},
            "recording": {"active": self.records.active is not None,
                          "record_id": self.records.active, "frames": self.records.frames},
            "replay": self.replay.copy(),
        }

    async def generate(self):
        while True:
            await asyncio.sleep(0.040)
            if not self.mock:
                continue
            self.mock_seq += 1
            elapsed = round((time.monotonic() - self.started) * 1000)
            self.imu = {"roll_deg": 0.0, "pitch_deg": 0.0, "valid": True, "age_ms": 0,
                        "mode": 0, "deadman": self.owner == "WEARABLE"}
            self.sample(self.mock_seq, elapsed)

    def require_mock_motion(self):
        if not self.mock:
            fail(409, "HARDWARE_MOTION_LOCKED", "实体动作待完成实物标定和固件联调")

    def can_control(self):
        if self.mock:
            return True
        return bool(self.motion_enabled and not self.stopping and self.session and self.fw_version == "0.1.0"
                    and self.session_channels == [14, 15, 13, 12] and self.pca_ok
                    and self.last_state is not None and time.monotonic() - self.last_state < 0.25
                    and self.verified_profile_id == self.calibration_id == self.device_calibration
                    and self.verified_profile_id is not None)

    def can_replay(self):
        if self.mock:
            return True
        workcell_id = self.profile.get("workcell", {}).get("workcell_id")
        return bool(self.can_control() and self.pose_confirmed and workcell_id)

    async def send(self, command, **fields):
        try:
            answer = await self.serial.request(command, expected_session=self.session, **fields)
        except (ValueError, TimeoutError):
            fail(503, "SERIAL_ACK_UNAVAILABLE", "未收到执行端确认，请查看实物状态")
        if answer.get("result") not in ("OK", "ACCEPTED"):
            fail(409, str(answer.get("reason") or "COMMAND_REJECTED"), "执行端拒绝了该指令")
        return answer

    async def run_replay(self, run_id, record):
        frames = record["frames"]
        times = [row["t_ms"] for row in frames]
        duration = times[-1]
        started = time.monotonic()
        status, reason = "completed", None
        try:
            while True:
                elapsed = (time.monotonic() - started) * 1000
                self.q_req = self.q_out = interpolate(frames, elapsed, times)
                self.replay["progress"] = min(1.0, elapsed / duration)
                if elapsed >= duration:
                    return
                await asyncio.sleep(0.040)
        except asyncio.CancelledError:
            status, reason = "aborted", "operator stop or shutdown"
            raise
        finally:
            self.records.finish_run(run_id, status, reason)
            self.run_id = None
            self.state, self.owner, self.lease = "HOLD", "NONE", None
            self.replay["active"] = False

    async def run_serial_replay(self, run_id, record, lease, generation, session):
        frames = record["frames"]
        times = [row["t_ms"] for row in frames]
        duration = times[-1]
        begin = next_tick = time.monotonic()
        status, reason = "aborted", "serial session or control prerequisites lost"
        sent_end = False
        try:
            while generation == self.control_generation and self.can_replay() and not self.fault:
                now = time.monotonic()
                elapsed = (now - begin) * 1000
                if now - next_tick > 0.15 or elapsed > duration + 700:
                    reason = "replay timing window exceeded"
                    break
                if sent_end and self.owner == "REPLAY" and joints(self.q_out) and all(
                    abs(out - final) <= (0.25 if index == 3 else 0.75)
                    for index, (out, final) in enumerate(zip(self.q_out, frames[-1]["q_out"]))
                ):
                    await self.send("release_owner", lease_id=lease)
                    status, reason = "completed", None
                    return
                target = interpolate(frames, elapsed, times)
                # Each acknowledged setpoint renews the same bounded controller lease.
                await self.send("joint_target", lease_id=lease, q_deg=target, ttl_ms=180, segment_ms=80)
                self.replay["progress"] = min(1.0, elapsed / duration)
                sent_end = elapsed >= duration
                next_tick += 0.04
                await asyncio.sleep(max(0, next_tick - time.monotonic()))
        except asyncio.CancelledError:
            reason = "operator stop or shutdown"
            raise
        except HTTPException as error:
            reason = error.detail["error_code"]
        finally:
            if status == "aborted" and generation == self.control_generation and self.session == session:
                with suppress(HTTPException):
                    await self.send("stop")
            self.records.finish_run(run_id, status, reason)
            self.run_id = None
            self.lease = None
            self.replay["active"] = False

    async def stop(self):
        if self.records.active:
            self.last_recording = self.records.finish("interrupted", "operator stop")
        if self.replay_task and not self.replay_task.done():
            self.replay_task.cancel()
            with suppress(asyncio.CancelledError):
                await self.replay_task
        if self.run_id:
            self.records.finish_run(self.run_id, "aborted", "operator stop or shutdown")
            self.run_id = None
        self.state, self.owner, self.lease = "HOLD", "NONE", None
        self.replay["active"] = False
        self.tokens.clear()


def create_app(source=None, db_path=None, calibration_path=None):
    source = source or os.environ.get("MOTIONARM_SOURCE", "wearable")
    if source in ("wearable", "serial", "robot_no_power"):
        from .virtual_api import create_virtual_app
        return create_virtual_app(db_path=db_path, executor_kind="virtual" if source == "wearable" else "robot_no_power")
    if source not in ("mock", "serial"):
        raise ValueError("MOTIONARM_SOURCE must be wearable, mock or serial")
    database = db_path or os.environ.get("MOTIONARM_DB_PATH", str(ROOT / "data" / f"{source}.sqlite3"))
    calibration = calibration_path or os.environ.get("MOTIONARM_CALIBRATION", str(ROOT / "config" / "robot_calibration.json"))

    @asynccontextmanager
    async def lifespan(application):
        runtime = Runtime(source, database, calibration)
        application.state.motionarm = runtime
        serial = SerialBridge(runtime.receive) if not runtime.mock else None
        if serial:
            runtime.serial = serial
            serial.start()
        task = asyncio.create_task(runtime.generate())
        try:
            yield
        finally:
            if serial and runtime.session and (runtime.lease or runtime.replay["active"]):
                runtime.control_generation += 1
                with suppress(HTTPException):
                    await runtime.send("stop")
            await runtime.stop()
            if serial:
                await serial.close()
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            runtime.records.db.close()

    application = FastAPI(title="MotionArm", lifespan=lifespan)

    def rt():
        return application.state.motionarm

    @application.middleware("http")
    async def local_requests(request, next_handler):
        origin = request.headers.get("origin")
        if request.method == "POST" and origin:
            from urllib.parse import urlparse
            parsed = urlparse(origin)
            if parsed.scheme not in ("http", "https") or parsed.hostname not in LOOPBACK_HOSTS:
                return JSONResponse({"detail": {"error_code": "ORIGIN_NOT_ALLOWED", "message": "仅本机网页可提交指令"}}, 403)
        return await next_handler(request)

    def dedupe(action, body, header, execute):
        key = body.request_id or header
        if not key:
            return execute()
        if len(key) > 128:
            fail(422, "REQUEST_ID_INVALID", "请求 ID 超长")
        fingerprint = sha256(body.model_dump_json(exclude={"request_id"}).encode()).hexdigest()
        previous = rt().memo.get((action, key))
        if previous:
            old_fingerprint, result = previous
            if old_fingerprint != fingerprint:
                fail(409, "REQUEST_ID_CONFLICT", "同一个请求 ID 不能用于不同内容")
            return result
        result = execute()
        if len(rt().memo) >= 256:
            rt().memo.pop(next(iter(rt().memo)))
        rt().memo[action, key] = (fingerprint, result)
        return result

    @application.get("/api/status")
    async def status():
        return rt().snapshot()

    @application.get("/api/config/calibration")
    async def calibration_status():
        return {"allow_arm": rt().can_control(), "blocked": not rt().can_control(),
                "calibration_id": rt().profile.get("calibration_id"), "profile": rt().profile,
                "source": rt().source}

    @application.websocket("/ws/telemetry")
    async def telemetry(websocket: WebSocket):
        from urllib.parse import urlparse
        origin = websocket.headers.get("origin")
        if origin and urlparse(origin).hostname not in LOOPBACK_HOSTS:
            await websocket.close(code=1008)
            return
        await websocket.accept()
        from starlette.websockets import WebSocketDisconnect
        try:
            while True:
                await websocket.send_json(rt().snapshot())
                try:
                    event = await asyncio.wait_for(websocket.receive(), timeout=0.1)
                except asyncio.TimeoutError:
                    continue
                if event["type"] == "websocket.disconnect":
                    return
        except (WebSocketDisconnect, RuntimeError):
            return

    @application.post("/api/control/arm")
    async def arm(body: Arm, idempotency_key: str | None = Header(default=None)):
        runtime = rt()
        if not runtime.mock:
            async with runtime.control_lock:
                if not body.operator_confirmed_safe:
                    fail(422, "INITIAL_POSE_NOT_CONFIRMED", "请确认现场姿态与工作区")
                if not runtime.can_control() or runtime.fault:
                    fail(409, "HARDWARE_MOTION_LOCKED", "需匹配已验证的标定、固件及实时遥测")
                if runtime.owner != "NONE" or runtime.lease:
                    fail(409, "OWNER_CONFLICT", "已有控制者")
                generation = runtime.control_generation
                answer = await runtime.send("request_owner", requested_owner=body.requested_owner,
                                            operator_confirmed_safe=True, operator_confirmed_initial_pose=True,
                                            calibration_id=runtime.calibration_id)
                if generation != runtime.control_generation:
                    fail(409, "CONTROL_INTERRUPTED", "请求被停止或重连中断")
                lease = answer.get("lease_id")
                if not isinstance(lease, str) or not lease or len(lease) > 96:
                    fail(503, "LEASE_INVALID", "执行端未返回有效租约")
                runtime.lease = lease
                runtime.manual_target = None
                runtime.owner = body.requested_owner
                runtime.state = answer.get("robot_state", runtime.state)
                return {"ok": True, "lease_id": lease}
        def execute():
            if not body.operator_confirmed_safe:
                fail(422, "INITIAL_POSE_NOT_CONFIRMED", "请确认模拟起始姿态")
            if rt().owner != "NONE":
                fail(409, "OWNER_CONFLICT", "已有控制者")
            rt().state = "TELEOP" if body.requested_owner == "WEARABLE" else "PC_MANUAL"
            rt().owner, rt().lease = body.requested_owner, uuid4().hex
            return {"ok": True, "lease_id": rt().lease}
        return dedupe("arm", body, idempotency_key, execute)

    @application.post("/api/control/manual-target")
    async def manual(body: Target, idempotency_key: str | None = Header(default=None)):
        runtime = rt()
        if not runtime.mock:
            async with runtime.control_lock:
                if not runtime.can_control() or runtime.fault:
                    fail(409, "HARDWARE_MOTION_LOCKED", "实物标定或实时遥测不可用")
                if runtime.owner != "PC_MANUAL" or not runtime.lease or body.lease_id != runtime.lease:
                    fail(409, "LEASE_INVALID", "需要有效的手动控制租约")
                limits = normalize_limits(runtime.profile)
                if not joints(body.q_deg) or any(not axis["logical_min_deg"] <= angle <= axis["logical_max_deg"]
                                                 for axis, angle in zip(limits, body.q_deg)):
                    fail(422, "TARGET_INVALID", "目标超出已标定关节范围")
                await runtime.send("joint_target", q_deg=body.q_deg, ttl_ms=200, segment_ms=200,
                                   lease_id=runtime.lease)
                runtime.manual_target = list(body.q_deg)
                runtime.tokens.clear()
                return {"ok": True}
        def execute():
            if rt().owner != "PC_MANUAL" or not rt().lease or body.lease_id != rt().lease:
                fail(409, "LEASE_INVALID", "需要本客户端的手动控制租约")
            if not joints(body.q_deg) or any(not 0 <= angle <= 180 for angle in body.q_deg):
                fail(422, "TARGET_INVALID", "需要四个有效模拟目标角度")
            rt().q_req = rt().q_out = body.q_deg[:]
            rt().tokens.clear()
            return {"ok": True}
        return dedupe("manual", body, idempotency_key, execute)

    @application.post("/api/control/heartbeat")
    async def heartbeat(body: Lease):
        runtime = rt()
        async with runtime.control_lock:
            if (not runtime.can_control() or runtime.fault or runtime.owner != "PC_MANUAL"
                    or not runtime.lease or body.lease_id != runtime.lease):
                fail(409, "LEASE_INVALID", "手动控制租约已失效")
            if not runtime.mock:
                if runtime.manual_target is None:
                    await runtime.send("heartbeat", lease_id=runtime.lease)
                else:
                    await runtime.send("joint_target", lease_id=runtime.lease, q_deg=runtime.manual_target,
                                       ttl_ms=200, segment_ms=200)
            return {"ok": True}

    @application.post("/api/control/release")
    async def release(body: Lease = Body(default=Lease())):
        runtime = rt()
        if not runtime.mock:
            async with runtime.control_lock:
                if not runtime.session or not runtime.lease or runtime.lease != body.lease_id:
                    fail(409, "LEASE_INVALID", "需要有效租约。也可使用停止指令")
                if runtime.records.active or runtime.replay["active"]:
                    fail(409, "TASK_ACTIVE", "请先结束当前任务")
                answer = await runtime.send("release_owner", lease_id=runtime.lease)
                runtime.state, runtime.owner, runtime.lease = answer.get("robot_state", "HOLD"), "NONE", None
                runtime.manual_target = None
                return {"ok": True}
        if rt().records.active or rt().replay["active"]:
            fail(409, "TASK_ACTIVE", "请先结束当前任务")
        rt().state, rt().owner, rt().lease = "READY", "NONE", None
        return {"ok": True}

    @application.post("/api/control/stop")
    @application.post("/api/replays/stop")
    async def stop():
        runtime = rt()
        runtime.control_generation += 1
        runtime.lease = None
        runtime.manual_target = None
        if not runtime.mock:
            if not runtime.session or runtime.fw_version != "0.1.0":
                fail(503, "PHYSICAL_STOP_UNAVAILABLE", "执行端未提供停止接口，请使用舵机独立电源开关")
            if runtime.stopping:
                fail(409, "STOP_PENDING", "正在等待执行端确认停止")
            runtime.stopping = True
            try:
                answer = await runtime.send("stop")
            finally:
                await runtime.stop()
                runtime.stopping = False
            runtime.state = answer.get("robot_state", runtime.state)
        else:
            await runtime.stop()
        return {"ok": True}

    @application.post("/api/control/mock-teleop")
    async def simulate_teleop():
        rt().require_mock_motion()
        if rt().owner not in ("NONE", "WEARABLE") or rt().replay["active"]:
            fail(409, "OWNER_CONFLICT", "已有控制者")
        rt().state, rt().owner, rt().lease = "TELEOP", "WEARABLE", None
        return {"ok": True}

    @application.post("/api/recordings/start")
    async def record(body: StartRecording, idempotency_key: str | None = Header(default=None)):
        def execute():
            if rt().records.active:
                fail(409, "RECORDING_ACTIVE", "已有录制")
            if (rt().owner != "WEARABLE" or rt().state not in ("TELEOP", "RECORDING_TELEOP")
                    or not joints(rt().q_out) or not rt().calibration_id
                    or not rt().can_control() or rt().profile.get("calibration_id") != rt().calibration_id):
                fail(409, "TELEOP_REQUIRED", "录制需要带匹配标定的实时遥操作")
            rt().tokens.clear()
            return rt().records.start(body.name, rt().source, rt().boot_id, rt().calibration_id,
                                      rt().profile.get("workcell", {}).get("workcell_id"),
                                      rt().fw_version)
        return dedupe("record", body, idempotency_key, execute)

    @application.post("/api/recordings/stop")
    async def finish_record(body: Request = Body(default=Request()), idempotency_key: str | None = Header(default=None)):
        def execute():
            if rt().records.active:
                rt().last_recording = rt().records.finish()
            return rt().last_recording
        return dedupe("finish_record", body, idempotency_key, execute)

    @application.get("/api/recordings")
    async def recordings():
        return rt().records.list()

    def load_record(identifier):
        data = rt().records.get(identifier, detail=True)
        if data is None:
            fail(404, "RECORD_NOT_FOUND", "轨迹不存在")
        return data

    @application.get("/api/recordings/{identifier}")
    async def recording(identifier: str):
        return load_record(identifier)

    @application.get("/api/recordings/{identifier}/export")
    async def export(identifier: str, format: str = "csv"):
        data = load_record(identifier)
        if format == "json":
            return JSONResponse(data, headers={"Content-Disposition": f"attachment; filename={identifier}.json"})
        if format != "csv":
            fail(422, "EXPORT_FORMAT", "选择 csv 或 json")
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow(("t_ms", "robot_t_ms", "seq", *(f"{axis}_target_deg" for axis in JOINTS),
                         "imu_roll_deg", "imu_pitch_deg", "mode", "deadman"))
        for frame in data["frames"]:
            writer.writerow((frame["t_ms"], frame["robot_t_ms"], frame["seq"], *frame["q_out"],
                             frame["roll_deg"], frame["pitch_deg"], frame["mode"], frame["deadman"]))
        return Response(output.getvalue(), media_type="text/csv",
                        headers={"Content-Disposition": f"attachment; filename={identifier}.csv"})

    @application.post("/api/replays/preflight")
    async def check_replay(body: Preflight):
        report = preflight(load_record(body.record_id), rt().snapshot(), rt().profile,
                           body.operator_workcell_confirmed, rt().can_replay())
        if report["ok"]:
            now = time.monotonic()
            rt().tokens = {key: info for key, info in rt().tokens.items() if info[1] > now}
            if len(rt().tokens) >= 32:
                rt().tokens.pop(next(iter(rt().tokens)))
            token = uuid4().hex
            report["preflight_id"] = token
            rt().tokens[token] = (report, now + 30)
        return report

    @application.post("/api/replays/start")
    async def replay(body: StartReplay, idempotency_key: str | None = Header(default=None)):
        def validated():
            if not body.operator_confirmed_safe:
                fail(422, "INITIAL_POSE_NOT_CONFIRMED", "需要二次确认")
            token = rt().tokens.pop(body.preflight_id, None)
            if token is None or token[1] < time.monotonic():
                fail(409, "PREFLIGHT_EXPIRED", "请重新预检")
            record = load_record(token[0]["record_id"])
            result = preflight(record, rt().snapshot(), rt().profile, True, rt().can_replay())
            if not result["ok"]:
                fail(409, "PREFLIGHT_CHANGED", "当前状态变化，请重新预检")
            return record, result

        runtime = rt()
        if not runtime.mock:
            async with runtime.control_lock:
                record, result = validated()
                generation, session = runtime.control_generation, runtime.session
                answer = await runtime.send("request_owner", requested_owner="REPLAY",
                                            operator_confirmed_safe=True, operator_confirmed_initial_pose=True,
                                            calibration_id=runtime.calibration_id)
                if generation != runtime.control_generation:
                    fail(409, "CONTROL_INTERRUPTED", "请求被停止或重连中断")
                lease = answer.get("lease_id")
                if not isinstance(lease, str) or not lease or len(lease) > 96:
                    fail(503, "LEASE_INVALID", "执行端未返回有效租约")
                run = runtime.records.start_run(record["id"], result)
                runtime.run_id = run
                runtime.state, runtime.owner, runtime.lease = "REPLAY", "REPLAY", lease
                runtime.replay = {"active": True, "record_id": record["id"], "progress": 0.0}
                runtime.replay_task = asyncio.create_task(runtime.run_serial_replay(run, record, lease, generation, session))
                return {"ok": True, "replay_id": run}
        def execute():
            record, result = validated()
            run = rt().records.start_run(record["id"], result)
            rt().run_id = run
            rt().state, rt().owner, rt().lease = "REPLAY", "REPLAY", uuid4().hex
            rt().replay = {"active": True, "record_id": record["id"], "progress": 0.0}
            rt().replay_task = asyncio.create_task(rt().run_replay(run, record))
            return {"ok": True, "replay_id": run}
        return dedupe("replay", body, idempotency_key, execute)

    return application


app = create_app()
