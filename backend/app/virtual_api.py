"""Local IMU -> virtual executor application, isolated from physical command APIs."""
import asyncio
from bisect import bisect_right
from contextlib import asynccontextmanager, suppress
from copy import deepcopy
import json
import os
from pathlib import Path
import time
from uuid import uuid4

from fastapi import Body, FastAPI, HTTPException, WebSocket
from fastapi.responses import JSONResponse, Response
from starlette.websockets import WebSocketDisconnect

from .recordings import Recordings
from .recording_export import recording_csv
from .protocol import finite_number
from .virtual import DEFAULT_CONFIG, LIMIT_POLICY, MAPPING_VERSION, MOUNTING, POSE_KEYS, VirtualController, forward_kinematics, pose
from .wearable import ATTITUDE_REPRESENTATION, FirmwareIncompatible, WearableReader, validate_frame

ROOT = Path(__file__).resolve().parents[2]
CONVENTION = "base_yaw_shoulder_down_elbow_relative_down_gripper_open_ratio_v1"
LOCAL_HOSTS = {"localhost", "127.0.0.1", "::1", "testserver"}
REMOVED_CONFIG_FIELDS = {"full_scale_deg", "mounting_profile", "roll_sign", "pitch_sign", "neutral_deg"}


def error(message, code="VIRTUAL_CONTROL_REJECTED", status=409):
    raise HTTPException(status, {"error_code": code, "message": message})


def pose_valid(value):
    return isinstance(value, dict) and all(finite_number(value.get(k))
        for k in (*POSE_KEYS, "gripper_open_ratio")) and 0 <= value["gripper_open_ratio"] <= 1


class VirtualRuntime:
    def __init__(self, database, config_path=None, clock=time.monotonic):
        self.clock = clock
        self.config_path = Path(config_path or ROOT / "data" / "virtual-config.json")
        config = None
        self.config_warning = None
        migrated = False
        if self.config_path.exists():
            try:
                config = json.loads(self.config_path.read_text(encoding="utf-8"))
                if isinstance(config, dict) and set(config) & REMOVED_CONFIG_FIELDS:
                    if set(config) - set(DEFAULT_CONFIG) - REMOVED_CONFIG_FIELDS:
                        raise ValueError("未知配置字段")
                    config = {key: value for key, value in config.items() if key not in REMOVED_CONFIG_FIELDS}
                    migrated = True
            except (OSError, ValueError):
                self.config_warning = "仿真配置无法读取，已加载默认参数"
        try:
            self.controller = VirtualController(clock, config)
        except (ValueError, TypeError):
            self.controller = VirtualController(clock)
            self.config_warning = "仿真配置无效，已加载默认参数"
            migrated = False
        if migrated:
            self.save_config()
            self.config_warning = "旧仿真配置已收缩，安装方向已固定；请重新初始化四元数参考"
        self.records = Recordings(database)
        self.baseline = json.loads((ROOT / "config" / "robot_baseline.json").read_text(encoding="utf-8"))
        self.reader = None
        self.port = None
        self.link = "disconnected"
        self.detail = None
        self.frames = self.parse_errors = 0
        self.started = clock()
        self.next_sample = self.started
        self.record_seq = 0
        self.replay = {"active": False, "record_id": None, "progress": 0.}
        self.replay_frames = []
        self.replay_times = []
        self.replay_started = None
        self.run_id = None
        self.tokens = {}
        self.lock = asyncio.Lock()
        self.last_recording = None
        self.recorded_events = set()

    def save_config(self):
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.config_path.with_suffix(".tmp")
        try:
            temporary.write_text(json.dumps(self.controller.config, indent=2, allow_nan=False), encoding="utf-8")
            temporary.replace(self.config_path)
        finally:
            temporary.unlink(missing_ok=True)

    @property
    def geometry(self):
        if self.replay["active"]:
            return self.replay.get("geometry", self.baseline["geometry_mm_estimated"])
        return self.baseline["geometry_mm_estimated"]

    def interrupt_record(self, reason):
        if self.records.active:
            self.last_recording = self.records.finish("interrupted", reason)

    def receive(self, message):
        c = self.controller
        if message["type"] == "frame":
            try:
                frame = validate_frame(message["frame"])
            except FirmwareIncompatible as exc:
                self.receive({"type": "incompatible", "detail": str(exc)})
                return
            was_valid = c.valid()
            restart = c.frame is not None and frame["boot"] != c.frame["boot"]
            if not c.receive(frame):
                return
            # Preserve even sub-tick failures in the recording provenance.
            if self.records.active and (not was_valid or not c.valid()):
                self.interrupt_record("IMU 无效或数据过期")
            if restart:
                self.interrupt_record("腕端重启")
            self.frames += 1
            self.link = "connected"
            self.detail = None
            self.parse_errors = message.get("parse_errors", self.parse_errors)
        elif message["type"] == "connecting":
            if self.link != "incompatible":
                self.link = "connecting"
        else:
            self.link = "incompatible" if message["type"] == "incompatible" else "disconnected"
            self.detail = message.get("detail")
            self.parse_errors = message.get("parse_errors", self.parse_errors)
            self.interrupt_record(self.detail or "串口断开")
            if c.owner != "REPLAY":
                c.stop(self.detail or "串口断开，目标保持")
            c.received_at = None
            c.settled_samples.clear()

    async def connect(self, port):
        import re
        from serial.tools import list_ports
        if not isinstance(port, str) or not re.fullmatch(r"COM[1-9][0-9]*", port):
            error("请选择有效串口", status=422)
        if port not in {item.device for item in list_ports.comports()}:
            error("串口当前不可用，请刷新端口", status=422)
        await self.disconnect()
        self.port, self.link, self.detail = port, "connecting", None
        self.controller.frame = None
        self.reader = WearableReader(self.receive)
        self.reader.start(port)

    async def disconnect(self):
        if self.reader:
            await self.reader.close()
            self.reader = None
        self.receive({"type": "offline", "detail": "腕端未连接"})

    def input_status(self):
        c = self.controller
        if self.link != "connected":
            return self.link
        age = c.age_ms()
        if age is None or age >= 250:
            return "stale"
        if c.frame["cal"] < 250:
            return "calibrating"
        return "valid" if c.valid() else "invalid"

    def config_snapshot(self):
        return {"config": deepcopy(self.controller.config), "config_id": self.controller.config_id,
                "mounting": deepcopy(MOUNTING),
                "geometry": deepcopy(self.geometry), "baseline_id": self.baseline["baseline_id"],
                "warning": self.config_warning}

    def metadata(self):
        return {**self.config_snapshot(), "input": "imu_usb", "executor": "virtual",
                "controller_version": MAPPING_VERSION, "mapping": "COORDINATED_RELATIVE_ANGLE",
                "limit_policy": LIMIT_POLICY,
                "button_semantics": "ac_toggle_follow_bc_toggle_gripper_v1", "pitch_weights": [1., 1.],
                "attitude_representation": ATTITUDE_REPRESENTATION,
                "reference_quaternion": deepcopy(self.controller.reference_quaternion),
                "gyro_range_dps": self.controller.frame["gr"] if self.controller.frame else None,
                "angle_convention": CONVENTION, "schema_version": 3,
                "label": "虚拟关节目标／估算预览", "measured": None}

    def enable_blockers(self):
        if self.controller.owner == "REPLAY" or self.controller.armed:
            return self.controller.enable_blockers()
        if self.records.active:
            return ["请先结束录制，再启用并回初始"]
        if self.input_status() != "valid":
            return [self.detail or "需要新鲜、有效且完成校准的 IMU 数据"]
        return self.controller.enable_blockers()

    def snapshot(self):
        c, status = self.controller, self.input_status()
        f = c.frame or {}
        valid = status == "valid"
        state = "REPLAY" if self.replay["active"] else "TELEOP" if c.armed else "HOLD" if c.enabled else "READY"
        roll, pitch = c.angles()
        age = c.age_ms()
        gate_status, blockers = c.gate_status()
        enable_blockers = self.enable_blockers()
        if status == "incompatible":
            gate_status, blockers = "PROTOCOL_INCOMPATIBLE", enable_blockers
        elif self.records.active or not c.enabled:
            blockers = enable_blockers or blockers
        return {
            "schema_version": 3, "event": "telemetry", "server_time_ms": int(time.time() * 1000),
            "source": "wearable", "read_only": False,
            "input": {"kind": "imu_usb", "status": status, "port": self.port, "boot_id": f.get("boot"),
                      "attitude_representation": ATTITUDE_REPRESENTATION,
                      "age_ms": age, "frames": self.frames, "parse_errors": self.parse_errors, "detail": self.detail},
            "executor": {"kind": "virtual", "label": "虚拟执行端"},
            "imu": {"roll_deg": roll, "pitch_deg": pitch, "valid": valid, "age_ms": age,
                    "quaternion": f.get("q"), "gyro_dps": f.get("gyro_dps"), "accel_g": f.get("accel_g"),
                    "calibration_samples": f.get("cal", 0), "who": f.get("who"),
                    "norm_g": f.get("norm_g"), "temp_c": f.get("temp_c"), "errors": f.get("errors", 0),
                    "gyro_range_dps": f.get("gr"), "reject_reason": f.get("rej", 0),
                    "rejected_samples": f.get("rc", 0)},
            "buttons": {"a": bool(f.get("a")) if age is not None and age < 250 else False,
                        "b": bool(f.get("b")) if age is not None and age < 250 else False,
                        "ac": f.get("ac"), "bc": f.get("bc")},
            "control": {"state": state, "owner": c.owner, "enabled": c.enabled, "armed": c.armed,
                        "mapping": "COORDINATED_RELATIVE_ANGLE",
                        "limit_policy": LIMIT_POLICY,
                        "follow": c.mapping_state(),
                        "can_enable": not enable_blockers,
                        "reason": c.reason,
                        "gate_status": gate_status, "blockers": blockers,
                        "fault": None if valid else status},
            "joint_state": {**c.joint_state(), "tip_mm": forward_kinematics(pose(c.q_out, c.open_out), self.geometry)},
            "recording": {"active": self.records.active is not None, "record_id": self.records.active,
                          "frames": self.records.frames},
            "replay": self.replay.copy(), "events": c.events[:], "config_id": c.config_id,
        }

    def start_record(self, name):
        c = self.controller
        if self.records.active:
            error("已有录制正在进行")
        if not c.enabled or c.owner != "WEARABLE" or not c.valid():
            error("录制需要已启用的真实 IMU 控制")
        self.next_sample = self.clock()
        return self.records.start(name, "wearable", str(c.frame["boot"]), c.config_id,
                                  "virtual", "wearable-usb-click-att3", metadata=self.metadata())

    def load_record(self, identifier):
        if not isinstance(identifier, str) or not 1 <= len(identifier) <= 128:
            error("无效录制标识", status=422)
        record = self.records.get(identifier, detail=True)
        if record is None:
            error("录制不存在", "RECORD_NOT_FOUND", 404)
        return record

    def check_record(self, record):
        violations = []
        metadata, frames = record.get("metadata") or {}, record["frames"]
        if metadata.get("angle_convention") != CONVENTION or metadata.get("executor") != "virtual":
            violations.append("旧记录角度约定不明确，仅可查看和导出")
        if record["status"] != "completed" or len(frames) < 2:
            violations.append("录制不完整，不能连续回放")
        if not frames or frames[0].get("t_ms") != 0 or any(
                type(f.get("t_ms")) is not int or not pose_valid(f.get("requested"))
                or not pose_valid(f.get("output_target")) for f in frames):
            violations.append("轨迹数据无效")
        elif any(not 0 < b["t_ms"] - a["t_ms"] <= 250 for a, b in zip(frames, frames[1:])):
            violations.append("轨迹包含采样缺口")
        # Use the recorded configuration: history never borrows hardware calibration.
        limits = metadata.get("config", {}).get("limits_deg", DEFAULT_CONFIG["limits_deg"])
        if any(pose_valid(f.get("output_target")) and any(not lo <= f["output_target"][key] <= hi
                for key, (lo, hi) in zip(POSE_KEYS, limits)) for f in frames):
            violations.append("轨迹超出录制时的仿真限位")
        if self.controller.enabled or self.replay["active"] or self.records.active:
            violations.append("请先停止实时控制、录制或回放")
        return violations

    def stop_replay(self, completed=False):
        if not self.replay["active"]:
            return
        if self.run_id:
            self.records.finish_run(self.run_id, "completed" if completed else "aborted", None if completed else "软件停止")
        self.run_id = None
        self.replay["active"] = False
        self.controller.stop("历史回放结束，请重新启用" if completed else "历史回放已停止")

    def step(self, now=None):
        now = self.clock() if now is None else now
        c = self.controller
        c.step(now)
        if self.replay["active"]:
            elapsed = (now - self.replay_started) * 1000
            frames, times = self.replay_frames, self.replay_times
            index = bisect_right(times, elapsed)
            left = frames[max(0, index - 1)]
            right = frames[min(index, len(frames) - 1)]
            fraction = 0 if left is right else max(0., min(1., (elapsed - left["t_ms"]) / (right["t_ms"] - left["t_ms"])))
            for kind, dest in (("requested", "q_req"), ("output_target", "q_out")):
                setattr(c, dest, [left[kind][key] + fraction * (right[kind][key] - left[kind][key]) for key in POSE_KEYS])
            c.open_req = left["requested"]["gripper_open_ratio"]
            c.open_out = left["output_target"]["gripper_open_ratio"] + fraction * (right["output_target"]["gripper_open_ratio"] - left["output_target"]["gripper_open_ratio"])
            self.replay["progress"] = min(1., elapsed / times[-1])
            if elapsed >= times[-1]:
                self.stop_replay(completed=True)
        elif self.records.active:
            if not c.valid(now):
                self.interrupt_record("IMU 无效或数据过期")
            elif now >= self.next_sample:
                self.record_seq += 1
                state = self.snapshot()
                self.records.append(str(c.frame["boot"]), c.config_id, round((now - self.started) * 1000), {
                    "seq": self.record_seq, "pc_received_ns": time.monotonic_ns(),
                    "requested": c.joint_state()["requested"], "output_target": c.joint_state()["output_target"],
                    "imu": state["imu"], "buttons": state["buttons"], "owner": c.owner,
                    "follow": state["control"]["follow"], "imu_seq": c.frame["seq"], "device_ms": c.frame["ms"],
                })
                self.next_sample += .04
                if self.next_sample <= now:
                    self.next_sample = now + .04
        # Persist meaningful controller events using backend elapsed time.
        for item in c.events:
            marker = (item["time_ms"], item["code"], item["message"])
            if marker not in self.recorded_events:
                self.records.note_event({**item, "t_ms": round((now - self.started) * 1000)})
        self.recorded_events = {(e["time_ms"], e["code"], e["message"]) for e in c.events}

    async def generate(self):
        deadline = self.clock()
        while True:
            deadline += .02
            await asyncio.sleep(max(0., deadline - self.clock()))
            self.step()
            if self.clock() - deadline > .1:
                deadline = self.clock()


def create_virtual_app(db_path=None, config_path=None):
    database = db_path or os.environ.get("MOTIONARM_DB_PATH", str(ROOT / "data" / "wearable.sqlite3"))
    @asynccontextmanager
    async def lifespan(application):
        runtime = VirtualRuntime(database, config_path or os.environ.get("MOTIONARM_VIRTUAL_CONFIG"))
        application.state.motionarm = runtime
        task = asyncio.create_task(runtime.generate())
        try:
            yield
        finally:
            await runtime.disconnect()
            runtime.stop_replay()
            runtime.interrupt_record("服务关闭")
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            runtime.records.db.close()

    app = FastAPI(title="MotionArm Virtual Console", lifespan=lifespan)
    def rt():
        return app.state.motionarm

    @app.middleware("http")
    async def local_requests(request, next_handler):
        from urllib.parse import urlparse
        if request.url.hostname not in LOCAL_HOSTS or (request.method == "POST" and request.headers.get("origin")
                and urlparse(request.headers["origin"]).hostname not in LOCAL_HOSTS):
            return JSONResponse({"detail": "仅允许本机访问"}, status_code=403)
        return await next_handler(request)

    @app.exception_handler(ValueError)
    async def invalid_operation(request, exc):
        return JSONResponse({"detail": {"error_code": "VIRTUAL_CONTROL_REJECTED", "message": str(exc)}}, status_code=409)

    @app.get("/api/status")
    async def status():
        return rt().snapshot()

    @app.websocket("/ws/telemetry")
    async def telemetry(socket: WebSocket):
        from urllib.parse import urlparse
        origin = socket.headers.get("origin")
        if socket.url.hostname not in LOCAL_HOSTS or (origin and urlparse(origin).hostname not in LOCAL_HOSTS):
            await socket.close(code=1008)
            return
        await socket.accept()
        try:
            while True:
                await socket.send_json(rt().snapshot())
                await asyncio.sleep(.04)
        except (WebSocketDisconnect, RuntimeError):
            pass

    @app.get("/api/input/ports")
    async def ports():
        from serial.tools import list_ports
        return [{"device": p.device, "description": p.description} for p in list_ports.comports()]

    @app.post("/api/input/connect")
    async def connect(body: dict = Body(...)):
        async with rt().lock:
            await rt().connect(body.get("port"))
        return {"ok": True}

    @app.post("/api/input/disconnect")
    async def disconnect():
        async with rt().lock:
            await rt().disconnect()
        return {"ok": True}

    @app.get("/api/virtual/config")
    async def config():
        return rt().config_snapshot()

    @app.post("/api/virtual/config")
    async def configure(body: dict = Body(...)):
        changes = {key: value for key, value in body.items() if key != "request_id"}
        rt().controller.configure(changes)
        rt().interrupt_record("仿真配置变化")
        rt().save_config()
        rt().tokens.clear()
        return rt().config_snapshot()

    @app.post("/api/virtual/enable")
    async def enable():
        runtime, c = rt(), rt().controller
        blockers = runtime.enable_blockers()
        if blockers:
            error("；".join(blockers))
        c.enable()
        runtime.tokens.clear()
        return {"ok": True, **runtime.config_snapshot()}

    @app.post("/api/control/stop")
    @app.post("/api/virtual/stop")
    @app.post("/api/replays/stop")
    async def stop():
        rt().stop_replay()
        rt().interrupt_record("软件停止")
        rt().controller.stop()
        rt().tokens.clear()
        return {"ok": True}

    @app.post("/api/virtual/reset")
    async def reset():
        rt().controller.reset()
        return {"ok": True}

    @app.get("/api/recordings")
    async def recordings():
        return rt().records.list()

    @app.post("/api/recordings/start")
    async def start_record(body: dict = Body(...)):
        name = body.get("name", "未命名录制")
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 120:
            error("录制名称应为 1–120 字符", status=422)
        return rt().start_record(name.strip())

    @app.post("/api/recordings/stop")
    async def finish_record():
        if rt().records.active:
            rt().last_recording = rt().records.finish()
        return rt().last_recording

    @app.get("/api/recordings/{identifier}")
    async def record(identifier: str):
        return rt().load_record(identifier)

    @app.get("/api/recordings/{identifier}/export")
    async def export(identifier: str, format: str = "csv"):
        data = rt().load_record(identifier)
        headers = {"Content-Disposition": f'attachment; filename="{identifier}.{format}"'}
        if format == "json":
            return JSONResponse(data, headers=headers)
        if format != "csv":
            error("请选择 csv 或 json", status=422)
        return Response(recording_csv(data), media_type="text/csv; charset=utf-8", headers=headers)

    @app.post("/api/replays/preflight")
    async def preflight(body: dict = Body(...)):
        record = rt().load_record(body.get("record_id", ""))
        violations = rt().check_record(record)
        token = None
        if not violations:
            token = uuid4().hex
            now = rt().clock()
            rt().tokens = {k: v for k, v in rt().tokens.items() if v[1] > now}
            if len(rt().tokens) >= 32:
                rt().tokens.pop(next(iter(rt().tokens)))
            rt().tokens[token] = (record["id"], now + 30)
        return {"ok": not violations, "violations": violations, "preflight_id": token}

    @app.post("/api/replays/start")
    async def start_replay(body: dict = Body(...)):
        identifier = body.get("preflight_id")
        if not isinstance(identifier, str) or not 1 <= len(identifier) <= 128:
            error("无效回放预检标识", status=422)
        token = rt().tokens.pop(identifier, None)
        if token is None or token[1] < rt().clock():
            error("回放预检已失效，请重新检查")
        record = rt().load_record(token[0])
        violations = rt().check_record(record)
        if violations:
            error("；".join(violations))
        rt().replay_frames = record["frames"]
        rt().replay_times = [f["t_ms"] for f in record["frames"]]
        rt().replay_started = rt().clock()
        rt().replay = {"active": True, "record_id": record["id"], "progress": 0.,
                       "geometry": record["metadata"].get("geometry", rt().geometry),
                       "config_id": record["metadata"].get("config_id")}
        rt().run_id = rt().records.start_run(record["id"], {"executor": "virtual", "metadata": record["metadata"]})
        rt().controller.owner = "REPLAY"
        rt().controller.reason = "历史回放／虚拟关节目标"
        rt().controller.velocity = [0., 0., 0.]
        rt().step()
        return {"ok": True, "replay_id": rt().run_id}

    # Explicitly reject old physical/manual routes in this application.
    @app.post("/api/control/{operation}")
    async def unsupported(operation: str):
        error("当前仅支持真实 IMU 驱动的虚拟执行端", "PHYSICAL_CONTROL_UNAVAILABLE")

    return app
