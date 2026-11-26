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
from anyio import CancelScope

from fastapi import Body, FastAPI, HTTPException, WebSocket
from fastapi.responses import JSONResponse, Response
from starlette.websockets import WebSocketDisconnect

from .recordings import Recordings
from .recording_export import recording_csv
from .protocol import finite_number
from .virtual import DEFAULT_CONFIG, LIMIT_POLICY, MAPPING_VERSION, MOUNTING, POSE_KEYS, VirtualController, forward_kinematics, pose
from .wearable import ATTITUDE_REPRESENTATION, FirmwareIncompatible, WearableReader, validate_frame
from .robot import RobotLink, SnapshotAssembler, wearable_frame, ROBOT_MAC, profile_hash_valid, trial_profile

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
        self.executor_kind = "virtual"
        self.robot_link = None
        self.robot_port = None
        self.robot_hello = None
        self.robot_parts = None
        self.robot_received = None
        self.robot_status = "disconnected"
        self.robot_detail = None
        self.robot_events = []
        self.robot_event_ids = set()
        self.robot_record_snapshot = None
        self.robot_clock_raw = self.robot_clock_total = None
        self.robot_pc_received_ns = None
        self.robot_last_event_seq = None
        self.record_event_start = self.record_ev_drop = None
        self.manager_token = None
        self.manager_at = None
        self.manager_socket = None
        self.robot_managed_enabled = False
        self.initializations = {}
        self.direct_observation = None
        self.power_actions = {}
        self.acceptance = []
        self.trial_config, self.trial_hash = trial_profile()
        self.manager_observed = False
        self.robot_last_output = None
        self.manager_generation = 0

    @property
    def physical(self):
        return self.executor_kind in ("robot_no_power", "robot_powered")

    @property
    def powered(self):
        return self.executor_kind == "robot_powered"

    @property
    def robot_config_id(self):
        if self.powered:
            return (self.robot_hello or {}).get("profile_hash", "robot_powered:unknown")
        return "robot_no_power:" + MAPPING_VERSION

    @property
    def executor_mapping_version(self):
        return self.trial_config["mapping_version"] if self.powered else MAPPING_VERSION

    @property
    def executor_joint_signs(self):
        return deepcopy(self.trial_config["joint_signs"] if self.powered else MOUNTING["joint_signs"])

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
            identifier = self.records.active
            self.flush_events()
            terminal = self.snapshot()
            physical = self.physical
            timeline = self.robot_clock_total if physical and self.robot_clock_total is not None else round((self.clock() - self.started) * 1000)
            self.records.note_event({"code": "RECORD_INTERRUPTED", "message": reason, "origin": "backend",
                                     "clock_domain": "backend_monotonic", "pc_elapsed_ms": round((self.clock() - self.started) * 1000),
                                     "time_ms": int(time.time() * 1000), "t_ms": timeline,
                                     "final_state_confirmed": bool(not physical or self.robot_parts and terminal["executor"]["connected"] and not terminal["control"]["enabled"]
                                                                   and terminal["joint_state"]["requested"]["gripper_open_ratio"] == 1
                                                                   and (not self.powered or terminal["output"]["stage"] != "unknown")),
                                     "final_state": {"joint_state": terminal["joint_state"], "gripper": terminal["gripper"],
                                                     "output": terminal.get("output"),
                                                     "executor": terminal["executor"], "control": terminal["control"],
                                                     "health": terminal.get("health"), "imu": terminal["imu"], "buttons": terminal["buttons"]}})
            if physical:
                self.append_robot_sample(self.clock())
            self.last_recording = self.records.finish("interrupted", reason) or self.records.get(identifier)

    def flush_events(self):
        now = self.clock()
        events = self.robot_events if self.physical else self.controller.events
        for item in events:
            marker = (item.get("boot_id", "virtual"), item.get("event_seq", item["time_ms"]))
            if marker not in self.recorded_events:
                if self.physical and self.robot_clock_raw is not None:
                    raw = item["robot_device_ms"]
                    delta = ((raw - self.robot_clock_raw + 2**31) % 2**32) - 2**31
                    stamp = self.robot_clock_total + delta
                    domain = "robot_monotonic_extended"
                else:
                    stamp, domain = round((now - self.started) * 1000), "backend_monotonic"
                self.records.note_event({**item, "t_ms": stamp, "clock_domain": domain})
        self.recorded_events = {(e.get("boot_id", "virtual"), e.get("event_seq", e["time_ms"])) for e in events}

    def incomplete_robot_events(self, reason, **diagnostics):
        if not self.records.active or not self.physical:
            return
        self.flush_events()
        self.records.note_event({"code": "EVENT_STREAM_INCOMPLETE", "message": reason, "origin": "backend",
                                 "clock_domain": "backend_monotonic", "t_ms": self.robot_clock_total,
                                 "pc_elapsed_ms": round((self.clock() - self.started) * 1000), **diagnostics})
        self.interrupt_record(reason)

    def finish_record(self):
        if self.records.active:
            identifier = self.records.active
            if self.physical:
                control = self.robot_parts["control"] if self.robot_parts else {}
                if not self.robot_input_valid() or not control.get("enabled") or control.get("fault"):
                    self.interrupt_record(control.get("fault") or control.get("reason") or "执行端输入无效或控制已停止")
                    return self.last_recording
                self.flush_events()
                self.append_robot_sample(self.clock())
            else:
                self.flush_events()
            self.last_recording = self.records.finish() or self.records.get(identifier)
        return self.last_recording

    def clear_manager(self):
        self.manager_generation += 1
        self.manager_token = self.manager_at = None
        self.manager_socket = None
        self.robot_managed_enabled = False
        self.manager_observed = False
        self.initializations.clear()
        self.direct_observation = None
        self.power_actions.clear()
        if self.robot_link:
            self.robot_link.disarm()

    def start_manager(self):
        self.manager_token, self.manager_at = uuid4().hex, self.clock()
        self.manager_socket = None
        if self.physical and self.robot_link:
            self.robot_link.renew()
        return self.manager_token

    def heartbeat(self, token):
        if not token or token != self.manager_token or self.manager_at is None or self.clock() - self.manager_at >= .75:
            raise ValueError("管理会话已失效，请明确重新初始化")
        self.manager_at = self.clock()
        if self.physical and self.robot_link:
            self.robot_link.renew()

    def robot_receive(self, message):
        kind = message["type"]
        if kind == "hello_ack":
            if self.powered and (message.get("profile_hash") != self.trial_hash
                    or message.get("mac", "").upper() != ROBOT_MAC
                    or message.get("mode") != "robot_powered" or message.get("protocol_version") != 3):
                self.reject_robot("执行端身份、协议或配置哈希不匹配")
                return
            self.clear_manager()
            self.robot_parts = self.robot_received = None
            self.robot_clock_raw = self.robot_clock_total = None
            self.robot_last_event_seq = None
            self.acceptance = []
            self.robot_hello = message.copy()
            if self.powered:
                self.robot_last_output = {"stage": "locked" if message.get("output_locked") is True else "unknown"}
            self.robot_status = "connected"
        elif kind == "snapshot":
            parts = message["parts"]
            try:
                SnapshotAssembler.validate(parts, self.executor_kind)
            except (ValueError, KeyError, TypeError):
                self.reject_robot("执行端完整快照校验失败")
                return
            if not self.robot_hello or parts["control"]["boot_id"] != self.robot_hello["boot_id"]:
                return
            if self.powered and parts["output"]["profile_hash"] != self.robot_hello["profile_hash"]:
                self.reject_robot("执行端配置哈希变化")
                return
            if self.robot_parts and parts["imu"].get("wearable_boot") != self.robot_parts["imu"].get("wearable_boot"):
                self.interrupt_record("腕端重启")
                self.clear_manager()
            raw_time = parts["control"]["t_ms"]
            if self.robot_clock_raw is None:
                self.robot_clock_total = raw_time
            else:
                delta = (raw_time - self.robot_clock_raw) % 2**32
                if not 0 < delta < 2**31:
                    return
                self.robot_clock_total += delta
            self.robot_clock_raw = raw_time
            self.robot_parts, self.robot_received = deepcopy(parts), self.clock()
            if self.powered:
                self.robot_last_output = deepcopy(parts["output"])
            self.robot_pc_received_ns = time.monotonic_ns()
            self.robot_status, self.robot_detail = "connected", None
            if self.manager_token:
                if parts["health"].get("manager_active"):
                    self.manager_observed = True
                if parts["control"]["enabled"]:
                    self.robot_managed_enabled = True
                elif self.robot_managed_enabled or parts["control"].get("fault"):
                    self.clear_manager()
                elif self.powered and self.manager_observed and not parts["health"].get("manager_active"):
                    self.clear_manager()
            if (self.records.active and self.physical and self.record_ev_drop is not None
                    and parts["health"].get("ev_drop", 0) != self.record_ev_drop):
                self.incomplete_robot_events("执行端事件丢失计数变化", previous_ev_drop=self.record_ev_drop,
                                             ev_drop=parts["health"].get("ev_drop", 0))
            if self.records.active and (not parts["control"]["enabled"] or parts["control"].get("fault")):
                self.clear_manager()
                self.interrupt_record(parts["control"].get("fault") or parts["control"].get("reason") or "执行端已停止")
            if not self.robot_input_valid():
                self.clear_manager()
                self.interrupt_record("无线输入无效或过期")
            if self.physical and self.records.active:
                # Admit each complete, validated snapshot before a later
                # callback can replace the public latest-state cache.
                self.append_robot_sample(self.clock())
        elif kind == "event":
            if not self.robot_hello or message["boot_id"] != self.robot_hello["boot_id"]:
                return
            marker = (message["boot_id"], message["event_seq"])
            if marker in self.robot_event_ids:
                return
            delta = None if self.robot_last_event_seq is None else (message["event_seq"] - self.robot_last_event_seq) % 2**32
            if delta is not None and not 0 < delta < 2**31:
                return
            previous_event_seq = self.robot_last_event_seq
            self.robot_last_event_seq = message["event_seq"]
            self.robot_event_ids.add(marker)
            if len(self.robot_event_ids) > 256:
                self.robot_event_ids = {(e["boot_id"], e["event_seq"]) for e in self.robot_events}
            self.robot_events.append({**message, "origin": "robot", "robot_device_ms": message.get("t_ms"), "pc_received_ns": time.monotonic_ns(),
                                      "time_ms": int(time.time() * 1000), "message": message["code"]})
            self.robot_events = self.robot_events[-60:]
            self.flush_events()
            raw = message["t_ms"]
            stamp = (self.robot_clock_total + ((raw - self.robot_clock_raw + 2**31) % 2**32) - 2**31
                     if self.robot_clock_raw is not None else raw)
            if (delta is not None and delta > 1 and self.records.active and self.record_event_start is not None
                    and stamp > self.record_event_start):
                self.incomplete_robot_events("执行端事件序号存在缺口", previous_event_seq=previous_event_seq,
                                             event_seq=message["event_seq"], missing_events=delta - 1)
        else:
            self.robot_status = "connecting" if kind == "connecting" else "disconnected"
            self.robot_detail = message.get("detail")
            if self.powered and self.robot_last_output and self.robot_last_output["stage"] == "holding":
                self.robot_last_output = {**self.robot_last_output, "stage": "unknown", "committed": False}
            self.clear_manager()
            self.interrupt_record(self.robot_detail or "执行端连接中断")

    def reject_robot(self, detail):
        self.robot_status, self.robot_detail = "incompatible", detail
        self.clear_manager()
        self.interrupt_record(detail)

    def robot_input_valid(self):
        if not self.robot_parts or self.robot_received is None or self.robot_status != "connected":
            return False
        f = self.robot_parts["imu"]
        return bool(f.get("wearable_boot") and f.get("valid") and f.get("who") == 104 and f.get("cal") == 250
                    and f.get("rej", 0) == 0 and f.get("age_ms", 0xffffffff) < 250 and self.clock() - self.robot_received < .5)

    async def connect_robot(self, port):
        import re
        from serial.tools import list_ports
        if not self.physical:
            raise ValueError("请先选择实体执行端")
        if not isinstance(port, str) or not re.fullmatch(r"COM[1-9][0-9]*", port) or port not in {p.device for p in list_ports.comports()}:
            raise ValueError("请选择有效执行端串口")
        if self.reader and port == self.port:
            raise ValueError("执行端与腕端不能占用同一串口")
        await self.disconnect_robot()
        self.robot_port, self.robot_status = port, "connecting"
        self.robot_link = RobotLink(self.robot_receive, port, mode=self.executor_kind)
        self.robot_link.start()

    async def disconnect_robot(self):
        self.clear_manager()
        self.interrupt_record("执行端已断开")
        if self.robot_link:
            await self.robot_link.close()
            self.robot_link = None
        self.robot_status = "disconnected"
        self.robot_hello = self.robot_parts = self.robot_received = None

    async def select_executor(self, kind):
        if kind not in ("virtual", "robot_no_power", "robot_powered"):
            raise ValueError("未知执行端类型")
        if self.records.active or self.replay["active"] or self.snapshot()["control"]["enabled"]:
            raise ValueError("请先停止控制、录制和回放，再切换执行端")
        if kind != self.executor_kind:
            if self.powered and self.robot_last_output and self.robot_last_output["stage"] in ("holding", "unknown"):
                raise ValueError("请先托住并明确释放PWM输出，再切换执行端")
            self.clear_manager()
            self.controller.stop("执行端变化，请重新初始化")
            await self.disconnect_robot()
            self.executor_kind = kind
            self.tokens.clear()

    def powered_binding(self, body, require_manager=True):
        if not self.powered or not self.robot_link or not self.robot_hello or not self.robot_snapshot()["executor"]["connected"]:
            raise ValueError("需要连接当前有动力执行端并收到新鲜完整快照")
        if (body.get("boot_id") != self.robot_hello["boot_id"]
                or body.get("profile_hash") != self.robot_hello["profile_hash"]
                or body.get("profile_hash") != self.trial_hash):
            raise ValueError("设备boot或配置哈希已变化，请重新核对准备")
        if require_manager:
            self.heartbeat(body.get("manager_token"))
        return {"boot_id": body["boot_id"], "profile_hash": body["profile_hash"]}

    async def powered_action(self, action, body):
        if action not in ("prepare", "enable_output", "test_joint", "disable_output"):
            raise ValueError("未知有动力管理操作")
        binding = self.powered_binding(body, False)
        identifier = body.get("request_id")
        if not isinstance(identifier, str) or not 1 <= len(identifier) <= 64:
            raise ValueError("管理操作request_id应为1–64字符")
        marker = json.dumps({"action": action, **body}, sort_keys=True, separators=(",", ":"), allow_nan=False)
        if identifier in self.power_actions:
            previous, result = self.power_actions[identifier]
            if marker != previous:
                raise ValueError("同一request_id不可用于不同操作或参数")
            if action != "disable_output":
                self.heartbeat(self.manager_token if action == "prepare" else body.get("manager_token"))
            return result
        if action not in ("prepare", "disable_output") or action == "prepare" and self.manager_token:
            self.heartbeat(body.get("manager_token"))
        if self.records.active or self.replay["active"]:
            raise ValueError("请先停止录制和回放")
        if self.robot_parts["control"]["enabled"]:
            raise ValueError("请先停止姿态控制")
        payload = {**binding, "request_id": identifier}
        if action == "enable_output":
            if not all(body.get(k) is True for k in ("supported", "power_confirmed", "startup_aligned")):
                raise ValueError("首次使能前须明确确认托住、对齐0/45/45打开姿态及动力已接通")
            payload.update(supported=True, power_confirmed=True, startup_aligned=True)
        elif action == "disable_output":
            if body.get("supported") is not True:
                raise ValueError("释放承重轴PWM前须现场确认托住机械臂")
            payload["supported"] = True
        elif action == "test_joint":
            axis, delta = body.get("axis"), body.get("offset_deg")
            if (not isinstance(axis, int) or isinstance(axis, bool) or axis not in (0, 1, 2)
                    or not finite_number(delta) or delta not in self.trial_config["diagnostic_offsets_deg"][axis]):
                raise ValueError("单轴检查仅允许当前配置列出的离散偏移，不能以跟随边界代替诊断步长")
            payload.update(axis=axis, offset_deg=delta)
        generation = self.manager_generation
        try:
            answer = await self.robot_link.request(action, **payload)
        except (Exception, asyncio.CancelledError) as exc:
            # An ACK timeout does not prove the robot rejected the command.
            # Revoke browser renewal before attempting the bounded STOP so an
            # uncertain diagnostic/enable cannot retain authority via heartbeat.
            self.clear_manager()
            outcome = "管理操作结果未知；已撤销会话并请求停止，请核对遥测后重新准备"
            self.robot_detail = outcome
            with CancelScope(shield=True):
                with suppress(ValueError, TimeoutError, OSError):
                    await asyncio.wait_for(self.robot_link.request("stop"), 1.)
            if isinstance(exc, asyncio.CancelledError):
                raise
            self.robot_detail = outcome
            raise ValueError(outcome) from exc
        if generation != self.manager_generation:
            raise ValueError("管理状态已停止或失效，迟到ACK不能恢复控制权")
        if answer.get("type") != "ack":
            raise ValueError(answer.get("error") or answer.get("error_code") or "执行端拒绝管理操作")
        if action == "prepare" and not self.manager_token:
            self.start_manager()
        result = {"ok": True, "accepted": True, "manager_token": self.manager_token,
                  "boot_id": binding["boot_id"], "profile_hash": binding["profile_hash"], "action": action}
        self.power_actions[identifier] = (marker, result)
        if len(self.power_actions) > 32:
            self.power_actions.pop(next(iter(self.power_actions)))
        return result

    def record_acceptance(self, body):
        self.powered_binding(body)
        axis = body.get("axis")
        if axis not in ("base", "shoulder", "elbow", "coordinated", "gripper", "stop_protection") or type(body.get("passed")) is not bool:
            raise ValueError("请选择验收项目并明确现场结果")
        note = body.get("note", "")
        if not isinstance(note, str) or len(note) > 500:
            raise ValueError("现场记录须为500字符以内文本")
        item = {"axis": axis, "passed": body["passed"], "note": note, "boot_id": body["boot_id"],
                "profile_hash": body["profile_hash"], "time_ms": int(time.time()*1000), "origin": "operator",
                "code": "OPERATOR_ACCEPTANCE", "message": f"现场观察 {axis}: {body['passed']}",
                "robot_device_ms": self.robot_clock_raw, "event_seq": "operator-" + uuid4().hex}
        self.acceptance = [a for a in self.acceptance if a["axis"] != axis] + [item]
        self.robot_events = (self.robot_events + [item])[-60:]
        self.flush_events()
        return {"ok": True, "acceptance": deepcopy(self.acceptance)}

    async def initialize(self, request_id=None, direct_observation=False, note=""):
        if request_id is None:
            request_id = uuid4().hex
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
            raise ValueError("初始化 request_id 应为 1–64 字符")
        if type(direct_observation) is not bool or not isinstance(note, str) or len(note) > 500:
            raise ValueError("直接观察标记须为布尔值，说明须为500字符以内文本")
        if direct_observation and (not self.powered or not note.strip()):
            raise ValueError("直接观察仅用于当前实体有动力执行端，并须明确填写本次观察说明")
        parameters = (direct_observation, note)
        if request_id in self.initializations:
            previous, result = self.initializations[request_id]
            if parameters != previous:
                raise ValueError("同一初始化request_id不可用于不同直接观察参数或说明")
            return result
        if self.executor_kind == "virtual":
            blockers = self.enable_blockers()
            if blockers:
                raise ValueError("；".join(blockers))
            self.controller.enable()
        else:
            state = self.robot_snapshot()
            ready = state["control"].get("observation_ready") if direct_observation else state["control"]["can_enable"]
            if self.records.active or self.replay["active"] or not ready or not self.robot_link:
                raise ValueError("执行端不满足初始化条件")
            payload = {"request_id": request_id}
            if self.powered:
                if not self.manager_token or not self.robot_parts["output"]["prepared"]:
                    raise ValueError("请先准备当前有动力执行端管理会话")
                if not direct_observation and not all(any(a["axis"] == axis and a["passed"] for a in self.acceptance) for axis in ("base", "shoulder", "elbow")):
                    raise ValueError("须先逐轴完成小范围检查并记录现场通过")
                payload.update(boot_id=self.robot_hello["boot_id"], profile_hash=self.robot_hello["profile_hash"])
            try:
                generation = self.manager_generation
                answer = await self.robot_link.request("init", **payload)
            except (ValueError, TimeoutError) as exc:
                self.clear_manager()
                raise ValueError("初始化未获得确认；不恢复控制，请检查执行端状态") from exc
            if generation != self.manager_generation:
                raise ValueError("初始化期间管理状态已停止或失效，迟到ACK不能恢复控制权")
            if answer["type"] != "ack":
                raise ValueError(answer.get("error") or answer.get("error_code") or "执行端拒绝初始化")
            if direct_observation:
                self.direct_observation = {"code": "OPERATOR_DIRECT_OBSERVATION", "origin": "operator",
                    "message": note.strip(), "note": note.strip(), "request_id": request_id,
                    "boot_id": self.robot_hello["boot_id"], "profile_hash": self.robot_hello["profile_hash"],
                    "time_ms": int(time.time()*1000), "robot_device_ms": self.robot_clock_raw,
                    "event_seq": "direct-" + uuid4().hex, "management_generation": self.manager_generation,
                    "unaccepted_axes": [axis for axis in ("base", "shoulder", "elbow")
                                        if not any(a["axis"] == axis and a["passed"] for a in self.acceptance)]}
                self.robot_events = (self.robot_events + [deepcopy(self.direct_observation)])[-60:]
                self.flush_events()
        self.tokens.clear()
        result = {"ok": True, "manager_token": self.manager_token if self.powered else self.start_manager(), **self.config_snapshot()}
        self.initializations[request_id] = (parameters, result)
        return result

    async def stop_control(self, reason="软件停止"):
        self.clear_manager()
        was_replay = self.replay["active"]
        self.stop_replay()
        if self.physical and self.robot_link and self.robot_hello:
            try:
                await self.robot_link.request("stop")
                # ACK confirms command handling only. While recording, admit
                # the subsequent robot events/state before closing the record.
                if self.records.active:
                    deadline = time.monotonic() + .45
                    while time.monotonic() < deadline:
                        if self.robot_parts and not self.robot_parts["control"]["enabled"]:
                            break
                        await asyncio.sleep(.02)
            except (ValueError, TimeoutError):
                self.robot_detail = "停止确认超时；本地管理看门狗仍会停止，等待执行端遥测"
        if not was_replay:
            self.controller.stop(reason)
        self.flush_events()
        self.interrupt_record(reason)
        self.tokens.clear()

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
            if not c.enabled and self.executor_kind == "virtual":
                self.clear_manager()
            # Preserve even sub-tick failures in the recording provenance.
            if self.executor_kind == "virtual" and self.records.active and (not was_valid or not c.valid()):
                self.interrupt_record("IMU 无效或数据过期")
            if restart and self.executor_kind == "virtual":
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
            if c.owner != "REPLAY":
                c.stop(self.detail or "串口断开，目标保持")
            if self.executor_kind == "virtual":
                self.clear_manager()
                self.interrupt_record(self.detail or "串口断开")
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
        physical = self.physical
        config = deepcopy(DEFAULT_CONFIG if physical else self.controller.config)
        if self.powered:
            config.update(deadzone_deg=self.trial_config["deadzone_deg"],
                          limits_deg=deepcopy(self.trial_config["operating_limits_deg"]),
                          max_velocity_deg_s=[self.trial_config["speed_deg_s"]]*3,
                          max_acceleration_deg_s2=[self.trial_config["acceleration_deg_s2"]]*3)
        mounting = deepcopy(MOUNTING)
        mounting["joint_signs"] = self.executor_joint_signs
        return {"config": config,
                "config_id": self.trial_hash if self.powered else self.robot_config_id if physical else self.controller.config_id,
                "mounting": mounting,
                "joint_signs": self.executor_joint_signs, "mapping_version": self.executor_mapping_version,
                "geometry": deepcopy(self.geometry), "baseline_id": self.baseline["baseline_id"],
                "warning": self.config_warning,
                "trial_profile": deepcopy(self.trial_config) if self.powered else None,
                "diagnostic_offsets_deg": deepcopy(self.trial_config["diagnostic_offsets_deg"]) if self.powered else None,
                "profile_hash": self.trial_hash if self.powered else None}

    def robot_pitch_limits(self):
        limits = self.trial_config["operating_limits_deg"] if self.powered else DEFAULT_CONFIG["limits_deg"]
        startup = self.trial_config["startup_deg"] if self.powered else [0, 45, 45]
        return [max(limits[i][0] - startup[i] for i in (1, 2)),
                min(limits[i][1] - startup[i] for i in (1, 2))]

    def metadata(self):
        result = {**self.config_snapshot(), "input": "imu_usb", "executor": "virtual",
                "controller_version": self.executor_mapping_version, "mapping": "COORDINATED_RELATIVE_ANGLE",
                "limit_policy": LIMIT_POLICY,
                "button_semantics": "ac_toggle_follow_bc_toggle_gripper_v1", "pitch_weights": [1., 1.],
                "attitude_representation": ATTITUDE_REPRESENTATION,
                "reference_quaternion": deepcopy(self.controller.reference_quaternion),
                "gyro_range_dps": self.controller.frame["gr"] if self.controller.frame else None,
                "angle_convention": CONVENTION, "schema_version": 5,
                "gripper_policy": "close_cycle_5000ms_including_motion_v1",
                "label": "虚拟关节目标／估算预览", "measured": None}
        if self.physical:
            result.update(input="esp_now", executor=self.executor_kind,
                          output_locked=self.robot_parts["health"]["output_locked"],
                          robot_boot=self.robot_hello["boot_id"], robot_mac=ROBOT_MAC,
                          wearable_boot=self.robot_parts["imu"]["wearable_boot"],
                          wearable_mac=self.robot_parts["health"].get("wearable_mac"),
                          profile_hash=self.robot_hello.get("profile_hash"),
                          output=deepcopy(self.robot_parts.get("output")), acceptance=deepcopy(self.acceptance),
                          direct_observation=deepcopy(self.direct_observation),
                          event_baseline={"event_seq": self.robot_last_event_seq,
                                          "ev_drop": self.robot_parts["health"].get("ev_drop", 0),
                                          "robot_device_ms": self.robot_clock_raw},
                          reference_quaternion=self.robot_parts["control"]["reference_quaternion"],
                          label="实体执行端已提交软件目标／无位置反馈" if self.powered else "实体执行端软件目标／PWM锁定／无位置反馈")
        return result

    def enable_blockers(self):
        if self.controller.owner == "REPLAY" or self.controller.armed:
            return self.controller.enable_blockers()
        if self.records.active:
            return ["请先结束录制，再启用并回初始"]
        if self.input_status() != "valid":
            return [self.detail or "需要新鲜、有效且完成校准的 IMU 数据"]
        return self.controller.enable_blockers()

    def snapshot(self):
        return self.robot_snapshot() if self.physical else self.virtual_snapshot()

    def virtual_snapshot(self):
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
            "schema_version": 5, "event": "telemetry", "server_time_ms": int(time.time() * 1000),
            "source": "wearable", "read_only": False,
            "input": {"kind": "imu_usb", "status": status, "port": self.port, "boot_id": f.get("boot"),
                      "attitude_representation": ATTITUDE_REPRESENTATION,
                      "age_ms": age, "frames": self.frames, "parse_errors": self.parse_errors, "detail": self.detail},
            "executor": {"kind": "virtual", "mode": "virtual", "label": "虚拟执行端", "connected": True,
                         "port": None, "identity": None, "output_locked": True, "status": "connected"},
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
                        "can_enable": not enable_blockers, "observation_ready": False,
                        "reason": c.reason,
                        "gate_status": gate_status, "blockers": blockers,
                        "fault": None if valid else status},
            "joint_state": {**c.joint_state(), "tip_mm": forward_kinematics(pose(c.q_out, c.open_out), self.geometry)},
            "recording": {"active": self.records.active is not None, "record_id": self.records.active,
                          "frames": self.records.frames},
            "replay": self.replay.copy(), "events": c.events[:], "config_id": c.config_id,
            "gripper": c.gripper_status(), "health": None,
        }

    def robot_snapshot(self):
        # This starts with a diagnostic shell, then replaces ALL authoritative
        # fields. Virtual preview targets never become robot-accepted targets.
        state = self.virtual_snapshot()
        parts = self.robot_parts
        age = None if self.robot_received is None else max(0, round((self.clock() - self.robot_received) * 1000))
        connected = self.robot_status == "connected" and age is not None and age < 500
        valid = connected and self.robot_input_valid()
        f = parts["imu"] if parts else {}
        wireless_fresh = bool(connected and f.get("wearable_boot") and f.get("age_ms", 0xffffffff) < 250)
        if not connected:
            input_status = "stale" if parts else self.robot_status
        elif not f.get("wearable_boot"):
            input_status = "disconnected"
        elif not wireless_fresh:
            input_status = "stale"
        elif f.get("who") != 104 or f.get("rej", 0) != 0:
            input_status = "invalid"
        elif f.get("cal", 0) < 250:
            input_status = "calibrating"
        else:
            input_status = "valid" if valid else "invalid"
        hello = self.robot_hello or {}
        state["executor"] = {"kind": self.executor_kind, "mode": self.executor_kind,
                             "label": "实体执行端 · 空载小范围" if self.powered else "实体执行端 · 无动力联调",
                             "identity": hello.get("mac"), "port": self.robot_port,
                             "connected": connected, "output_locked": parts["health"]["output_locked"] if parts else hello.get("output_locked", True), "status": self.robot_status,
                             "boot_id": hello.get("boot_id"), "age_ms": age,
                             "protocol_version": hello.get("protocol_version"), "profile_hash": hello.get("profile_hash"),
                             "acceptance": deepcopy(self.acceptance)}
        state["output"] = deepcopy(parts.get("output")) if parts else None
        if self.powered and state["output"] and not connected:
            state["output"].update(stage="unknown", pwm_enabled=None, prepared=False)
            state["executor"]["output_locked"] = False
        state["input"] = {"kind": "esp_now", "status": input_status, "wireless_fresh": wireless_fresh,
                          "port": self.robot_port, "boot_id": parts["imu"].get("wearable_boot") if parts else None,
                          "attitude_representation": ATTITUDE_REPRESENTATION,
                          "age_ms": parts["imu"].get("age_ms") if parts else None, "transport_age_ms": None,
                          "frames": (parts["health"].get("wireless", {}).get("received", 0) if parts else 0),
                          "parse_errors": (parts["health"].get("wireless", {}).get("rejected", 0) if parts else 0),
                          "detail": self.robot_detail}
        f = parts["imu"] if parts else {}
        control = parts["control"] if parts else {}
        vector = control.get("delta_rotation_vector_deg")
        state["imu"] = {"roll_deg": vector[0] if vector else None, "pitch_deg": vector[1] if vector else None,
                        "valid": bool(valid), "age_ms": state["input"]["age_ms"], "quaternion": f.get("q") if f.get("wearable_boot") else None,
                        "gyro_dps": f.get("gyro_dps") if f.get("wearable_boot") else None,
                        "accel_g": f.get("accel_g") if f.get("wearable_boot") else None,
                        "calibration_samples": f.get("cal", 0), "who": f.get("who"), "norm_g": f.get("norm_g"),
                        "temp_c": f.get("temp_c"), "errors": f.get("errors", 0), "gyro_range_dps": f.get("gr"),
                        "reject_reason": f.get("rej", 0), "rejected_samples": f.get("rc", 0)}
        state["buttons"] = {"a": bool(f.get("a")) if wireless_fresh else False, "b": bool(f.get("b")) if wireless_fresh else False,
                            "ac": f.get("ac"), "bc": f.get("bc")}
        blockers = [] if connected and valid and control.get("can_enable") else [control.get("reason") or "等待完整执行端遥测与稳定无线输入"]
        can_enable = not blockers and not self.records.active and not self.replay["active"]
        observation_ready = False
        if self.powered:
            if (not self.manager_token or self.manager_at is None or self.clock() - self.manager_at >= .75
                    or not parts or not parts["output"].get("prepared") or not parts["health"].get("manager_active")):
                blockers.append("须明确准备当前管理会话")
            if (not parts or parts["output"].get("stage") != "holding" or not parts["output"].get("pwm_enabled")
                    or not parts["output"].get("committed") or parts["output"].get("diagnostic_active")
                    or not parts["health"].get("pca_readable")
                    or control.get("enabled") or control.get("armed") or control.get("fault")):
                blockers.append("须保持PWM输出、停止推进、完成当前诊断且无故障")
            observation_ready = bool(can_enable and not blockers)
            if not all(any(a["axis"] == axis and a["passed"] for a in self.acceptance) for axis in ("base", "shoulder", "elbow")):
                blockers.append("须逐轴记录现场动作通过后初始化跟随")
            can_enable = can_enable and not blockers
        if self.records.active:
            blockers = ["请先结束录制"]
        # Last accepted robot state stays identifiable when stale. It is never
        # projected forward by the PC and does not grant a management session.
        state["control"] = {"state": "TELEOP" if control.get("armed") else "HOLD" if control.get("enabled") else "READY",
                            "source_stale": not connected, "management_active": bool(self.manager_token),
                            "owner": control.get("owner", "NONE"), "enabled": bool(control.get("enabled")),
                            "armed": bool(control.get("armed")), "can_enable": bool(can_enable),
                            "observation_ready": observation_ready, "direct_observation": deepcopy(self.direct_observation),
                            "reason": control.get("reason") or self.robot_detail, "fault": control.get("fault") if connected else "EXECUTOR_OFFLINE",
                            "blockers": blockers, "gate_status": "INPUT_INVALID" if not valid else "ACTIVE" if control.get("armed") else "READY" if control.get("enabled") else "DISABLED",
                            "mapping": "COORDINATED_RELATIVE_ANGLE", "limit_policy": LIMIT_POLICY,
                            "follow": {"version": self.executor_mapping_version, "joint_signs": self.executor_joint_signs,
                                       "reference_quaternion": control.get("reference_quaternion"),
                                       "delta_rotation_vector_deg": vector,
                                       "deadzone_deg": self.trial_config["deadzone_deg"] if self.powered else DEFAULT_CONFIG["deadzone_deg"], "limit_policy": LIMIT_POLICY,
                                       "angle_ambiguous": bool(control.get("angle_ambiguous")), "pitch_weights": [1., 1.],
                                       "pitch_limits_deg": self.robot_pitch_limits()}}
        state["joint_state"] = ({key: deepcopy(parts["targets"].get(key)) for key in ("requested", "output_target", "measured", "velocity_deg_s", "limits")}
                                if parts else {"requested": None, "output_target": None, "measured": None, "velocity_deg_s": [0., 0., 0.], "limits": [False]*3})
        if self.powered and parts:
            known_commit = parts["output"]["committed"]
            state["joint_state"]["last_committed_target"] = deepcopy(parts["targets"]["output_target"]) if known_commit else None
            if not connected or parts["output"]["stage"] != "holding" or not parts["output"]["committed"]:
                state["joint_state"]["output_target"] = None
        state["joint_state"]["tip_mm"] = forward_kinematics(state["joint_state"]["output_target"], self.geometry) if state["joint_state"]["output_target"] else None
        state["gripper"] = ({**{k: v for k, v in parts["gripper"].items() if k not in ("v", "type", "seq", "boot_id", "snapshot_id", "t_ms")},
                              "requested_open_ratio": parts["targets"]["requested"]["gripper_open_ratio"],
                              "output_open_ratio": parts["targets"]["output_target"]["gripper_open_ratio"]} if parts else None)
        if self.powered and parts:
            state["gripper"]["last_committed_open_ratio"] = parts["targets"]["output_target"]["gripper_open_ratio"] if parts["output"]["committed"] else None
            if not connected or parts["output"]["stage"] != "holding":
                state["gripper"]["output_open_ratio"] = None
        state["health"] = deepcopy(parts["health"]) if parts else None
        state["events"] = self.robot_events[:]
        state["config_id"] = self.robot_config_id
        return state

    def start_record(self, name):
        c = self.controller
        if self.records.active:
            error("已有录制正在进行")
        if self.physical:
            if not self.robot_input_valid() or not self.robot_parts["control"]["enabled"] or not self.manager_token:
                error("录制需要已初始化且有新鲜无线输入的执行端")
            self.robot_record_snapshot = None
            self.record_event_start = self.robot_clock_total
            self.record_ev_drop = self.robot_parts["health"].get("ev_drop", 0)
            result = self.records.start(name, "wearable", self.robot_hello["boot_id"] + ":" + str(self.robot_parts["imu"]["wearable_boot"]),
                                        self.robot_config_id, self.executor_kind, "robot-usb-v3" if self.powered else "robot-usb-v2", metadata=self.metadata())
            self.append_robot_sample(self.clock())
            if self.direct_observation:
                self.records.note_event({**deepcopy(self.direct_observation), "t_ms": self.robot_clock_total,
                    "clock_domain": "robot_monotonic_extended", "authorization_preceded_recording": True})
            return self.records.get(result["id"])
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
        if self.executor_kind != "virtual":
            violations.append("历史回放仅允许虚拟执行端，请先停止并明确切换为虚拟执行端")
        metadata, frames = record.get("metadata") or {}, record["frames"]
        known_executor = (metadata.get("executor") == "virtual"
                          or metadata.get("schema_version") in (4, 5) and metadata.get("executor") == "robot_no_power"
                          and metadata.get("output_locked") is True
                          or metadata.get("schema_version") == 5 and metadata.get("executor") == "robot_powered"
                          and profile_hash_valid(metadata.get("profile_hash")))
        if self.robot_link is not None or self.robot_last_output and self.robot_last_output["stage"] in ("holding", "unknown"):
            violations.append("实体执行端须停止、明确释放PWM并断开运动控制后才能虚拟回放")
        if metadata.get("angle_convention") != CONVENTION or not known_executor:
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
        opening = (self.controller.open_req, self.controller.open_out)
        self.controller.stop("历史回放结束，请重新启用" if completed else "历史回放已停止")
        # Saved targets retain their historical meaning, including a legacy
        # closed target. Offline replay is never an actuator command.
        self.controller.open_req, self.controller.open_out = opening
        self.controller.gripper_state = "HISTORY"

    def append_robot_sample(self, now):
        if (not self.records.active or not self.robot_parts or not self.robot_hello
                or self.robot_parts["control"]["snapshot_id"] == self.robot_record_snapshot):
            return
        state = self.robot_snapshot()
        self.record_seq += 1
        finished = self.records.append(self.robot_hello["boot_id"] + ":" + str(self.robot_parts["imu"]["wearable_boot"]),
                            state["config_id"], self.robot_clock_total,
                            {"seq": self.record_seq, "requested": state["joint_state"]["requested"],
                             "output_target": state["joint_state"]["output_target"], "measured": None,
                             "imu": state["imu"], "buttons": state["buttons"], "owner": state["control"]["owner"],
                             "follow": state["control"]["follow"], "gripper": state["gripper"], "executor": state["executor"],
                             "output": state.get("output"),
                             "health": state["health"], "robot_device_ms": self.robot_parts["control"]["t_ms"],
                             "clock_domain": "robot_monotonic_extended", "pc_received_ns": self.robot_pc_received_ns,
                             "pc_elapsed_ms": round((now - self.started) * 1000),
                             "imu_seq": self.robot_parts["imu"]["wearable_seq"], "device_ms": self.robot_parts["imu"]["ms"]})
        if finished:
            self.last_recording = finished
        self.robot_record_snapshot = self.robot_parts["control"]["snapshot_id"]

    def step(self, now=None):
        now = self.clock() if now is None else now
        c = self.controller
        if self.manager_token and now - self.manager_at >= .75:
            self.clear_manager()
            if self.executor_kind == "virtual":
                c.release_gripper("MANAGER_TIMEOUT")
                c.stop("浏览器管理心跳超时，请重新初始化")
            elif self.robot_link:
                async def request_stop():
                    with suppress(ValueError, TimeoutError):
                        await self.robot_link.request("stop")
                asyncio.create_task(request_stop())
            self.interrupt_record("浏览器管理心跳超时")
        if self.physical:
            if self.records.active:
                if not self.robot_input_valid():
                    self.interrupt_record("无线输入无效或过期")
                else:
                    self.append_robot_sample(now)
            self.flush_events()
            return
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
                finished = self.records.append(str(c.frame["boot"]), c.config_id, round((now - self.started) * 1000), {
                    "seq": self.record_seq, "pc_received_ns": time.monotonic_ns(),
                    "requested": c.joint_state()["requested"], "output_target": c.joint_state()["output_target"],
                    "imu": state["imu"], "buttons": state["buttons"], "owner": c.owner,
                    "follow": state["control"]["follow"], "imu_seq": c.frame["seq"], "device_ms": c.frame["ms"],
                    "gripper": state["gripper"], "executor": state["executor"], "measured": None,
                    "clock_domain": "backend_monotonic", "backend_t_ms": round((now - self.started) * 1000),
                })
                if finished:
                    self.last_recording = finished
                self.next_sample += .04
                if self.next_sample <= now:
                    self.next_sample = now + .04
        self.flush_events()

    async def generate(self):
        deadline = self.clock()
        while True:
            deadline += .02
            await asyncio.sleep(max(0., deadline - self.clock()))
            self.step()
            if self.clock() - deadline > .1:
                deadline = self.clock()


def create_virtual_app(db_path=None, config_path=None, executor_kind="virtual"):
    database = db_path or os.environ.get("MOTIONARM_DB_PATH", str(ROOT / "data" / "wearable.sqlite3"))
    @asynccontextmanager
    async def lifespan(application):
        runtime = VirtualRuntime(database, config_path or os.environ.get("MOTIONARM_VIRTUAL_CONFIG"))
        runtime.executor_kind = executor_kind
        application.state.motionarm = runtime
        task = asyncio.create_task(runtime.generate())
        try:
            yield
        finally:
            await runtime.stop_control("服务关闭")
            await runtime.disconnect_robot()
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
        bound_token = None
        identity = object()
        async def send_snapshots():
            while True:
                await socket.send_json(rt().snapshot())
                await asyncio.sleep(.04)
        async def receive_management():
            nonlocal bound_token
            while True:
                message = await socket.receive_json()
                if not isinstance(message, dict) or message.get("type") != "heartbeat":
                    continue
                token = message.get("manager_token")
                if rt().manager_socket is not None and rt().manager_socket is not identity:
                    await socket.close(code=1008)
                    return
                try:
                    rt().heartbeat(token)
                except ValueError:
                    await socket.close(code=1008)
                    return
                rt().manager_socket, bound_token = identity, token
        tasks = {asyncio.create_task(send_snapshots()), asyncio.create_task(receive_management())}
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                with suppress(WebSocketDisconnect, RuntimeError, ValueError):
                    await task
        except asyncio.CancelledError:
            pass
        finally:
            # ASGI servers and TestClient can cancel the socket task as soon as
            # it disconnects. Shield the bounded STOP cleanup from that cancel.
            with CancelScope(shield=True):
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
                if bound_token and rt().manager_token == bound_token and rt().manager_socket is identity:
                    await rt().stop_control("管理 WebSocket 已断开，请重新初始化")

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

    @app.get("/api/executor/config")
    async def config():
        return rt().config_snapshot()

    @app.get("/api/virtual/config")
    async def virtual_config():
        if rt().executor_kind != "virtual":
            error("请使用当前执行端只读配置入口")
        return rt().config_snapshot()

    @app.post("/api/virtual/config")
    async def configure(body: dict = Body(...)):
        if rt().executor_kind != "virtual":
            error("实体执行端不使用仿真配置")
        changes = {key: value for key, value in body.items() if key != "request_id"}
        rt().controller.configure(changes)
        rt().interrupt_record("仿真配置变化")
        rt().save_config()
        rt().tokens.clear()
        return rt().config_snapshot()

    @app.post("/api/virtual/enable")
    async def enable(body: dict = Body(default={})):
        if rt().executor_kind != "virtual":
            error("请使用当前执行端初始化入口")
        return await rt().initialize(body.get("request_id"))

    @app.get("/api/executor")
    async def executor():
        return rt().snapshot()["executor"]

    @app.post("/api/executor/select")
    @app.post("/api/executor")
    async def select_executor(body: dict = Body(...)):
        async with rt().lock:
            await rt().select_executor(body.get("kind"))
        return rt().snapshot()["executor"]

    @app.post("/api/executor/connect")
    async def connect_executor(body: dict = Body(...)):
        async with rt().lock:
            await rt().connect_robot(body.get("port"))
        return {"ok": True}

    @app.post("/api/executor/disconnect")
    async def disconnect_executor():
        async with rt().lock:
            await rt().stop_control("执行端断开")
            await rt().disconnect_robot()
        return {"ok": True}

    @app.post("/api/executor/enable")
    @app.post("/api/executor/initialize")
    async def initialize_executor(body: dict = Body(default={})):
        async with rt().lock:
            if rt().powered:
                rt().powered_binding(body)
            return await rt().initialize(body.get("request_id"), body.get("direct_observation", False), body.get("note", ""))

    @app.post("/api/executor/prepare")
    async def prepare_executor(body: dict = Body(...)):
        async with rt().lock:
            return await rt().powered_action("prepare", body)

    @app.post("/api/executor/enable-output")
    async def enable_output(body: dict = Body(...)):
        async with rt().lock:
            return await rt().powered_action("enable_output", body)

    @app.post("/api/executor/test-joint")
    async def test_joint(body: dict = Body(...)):
        async with rt().lock:
            return await rt().powered_action("test_joint", body)

    @app.post("/api/executor/disable-output")
    async def disable_output(body: dict = Body(...)):
        async with rt().lock:
            return await rt().powered_action("disable_output", body)

    @app.post("/api/executor/acceptance")
    async def acceptance(body: dict = Body(...)):
        async with rt().lock:
            return rt().record_acceptance(body)

    @app.post("/api/control/heartbeat")
    async def manager_heartbeat(body: dict = Body(...)):
        rt().heartbeat(body.get("manager_token"))
        return {"ok": True}

    @app.post("/api/control/stop")
    @app.post("/api/executor/stop")
    async def stop():
        await rt().stop_control()
        return {"ok": True}

    @app.post("/api/virtual/stop")
    @app.post("/api/replays/stop")
    async def virtual_stop():
        if rt().executor_kind != "virtual":
            error("请使用当前执行端停止入口")
        await rt().stop_control()
        return {"ok": True}

    @app.post("/api/virtual/reset")
    async def reset():
        if rt().executor_kind != "virtual":
            error("实体执行端不支持虚拟复位")
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
        return rt().finish_record()

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
