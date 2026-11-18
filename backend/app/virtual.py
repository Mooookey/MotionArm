"""Deterministic virtual executor. Angles are geometric targets, never feedback."""
from copy import deepcopy
from hashlib import sha256
import json
import math
import time

from .protocol import newer, finite_number

POSE_KEYS = ("base_yaw_deg", "shoulder_down_deg", "elbow_down_deg")
MAPPING_VERSION = "coordinated-angle-v1"
LIMIT_POLICY = "FIXED_REFERENCE_HARD_CLAMP"
DEFAULT_CONFIG = {
    "deadzone_deg": 5.0, "full_scale_deg": 30.0,
    "limits_deg": [[-45.0, 45.0], [0.0, 60.0], [0.0, 60.0]],
    "max_velocity_deg_s": [20.0, 15.0, 15.0],
    "max_acceleration_deg_s2": [60.0, 60.0, 60.0],
    "roll_sign": 1, "pitch_sign": 1, "neutral_deg": [0.0, 0.0],
}


def pose(angles, opening):
    return {**dict(zip(POSE_KEYS, angles)), "gripper_open_ratio": opening}


def forward_kinematics(target, geometry):
    yaw = math.radians(target[POSE_KEYS[0]])
    theta = math.radians(90 - target[POSE_KEYS[1]])
    phi = math.radians(-target[POSE_KEYS[2]])
    upper = geometry["shoulder_to_elbow_axis"]
    lower = geometry["elbow_to_gripper_mount"] + geometry["gripper_mount_to_tip"]
    radius = upper * math.cos(theta) + lower * math.cos(theta + phi)
    return [radius * math.cos(yaw), radius * math.sin(yaw),
            geometry["table_to_shoulder_axis"] + upper * math.sin(theta) + lower * math.sin(theta + phi)]


def validate_config(config):
    if set(config) != set(DEFAULT_CONFIG):
        raise ValueError("未知或缺少仿真参数")
    def number(value):
        return finite_number(value)
    if not all(number(config[k]) for k in ("deadzone_deg", "full_scale_deg")) or not (
            0 < config["deadzone_deg"] < config["full_scale_deg"] <= 90):
        raise ValueError("死区应大于 0 且小于满速角（≤90°）")
    for key in ("max_velocity_deg_s", "max_acceleration_deg_s2"):
        values = config[key]
        if not isinstance(values, list) or len(values) != 3 or not all(number(x) and 0 < x <= 360 for x in values):
            raise ValueError("无效的速度或加速度")
    limits = config["limits_deg"]
    if not isinstance(limits, list) or len(limits) != 3 or any(
            not isinstance(row, list) or len(row) != 2 or not all(number(x) for x in row)
            or not -180 <= row[0] < row[1] <= 180 for row in limits):
        raise ValueError("无效的仿真限位")
    if any(not lo <= start <= hi for (lo, hi), start in zip(limits, (0, 45, 45))):
        raise ValueError("仿真限位必须包含启动姿态 0/45/45°")
    if any(type(config[k]) is not int or config[k] not in (-1, 1) for k in ("roll_sign", "pitch_sign")):
        raise ValueError("方向只能为 +1 或 −1")
    zero = config["neutral_deg"]
    if not isinstance(zero, list) or len(zero) != 2 or not all(number(x) and abs(x) <= 180 for x in zero):
        raise ValueError("无效中立角")


class VirtualController:
    """One initialized wrist reference, click-to-run and coordinated arm targets."""
    def __init__(self, clock=time.monotonic, config=None):
        self.clock = clock
        self.config = deepcopy(config or DEFAULT_CONFIG)
        validate_config(self.config)
        self.q_req = [0., 45., 45.]
        self.q_out = self.q_req[:]
        self.velocity = [0., 0., 0.]
        self.open_req = self.open_out = 1.
        self.frame = None
        self.received_at = None
        self.last_tick = self.clock()
        self.enabled = self.armed = False
        self.owner = "NONE"
        self.reason = "连接腕端并启用、回到虚拟初始姿态"
        self.neutral_since = None
        self.neutral_ready = False
        self.events = []
        self.settled_samples = []
        self.anchor_q = [0., 45., 45.]
        self.anchor_wrist = self.last_wrist = None
        self.wrist_delta = [0., 0.]
        self.angle_ambiguous = False
        self.click_baseline = None

    @property
    def config_id(self):
        identity = {"mapping": MAPPING_VERSION, "limit_policy": LIMIT_POLICY, "config": self.config}
        return "virtual:" + sha256(json.dumps(identity, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]

    def event(self, code, message):
        self.events.append({"time_ms": int(time.time() * 1000), "code": code, "message": message})
        self.events = self.events[-60:]

    def age_ms(self, now=None):
        if self.received_at is None:
            return None
        return max(0, int(((self.clock() if now is None else now) - self.received_at) * 1000)) + self.frame["age_ms"]

    def valid(self, now=None):
        age = self.age_ms(now)
        return bool(self.frame and self.frame["who"] == 104 and self.frame["cal"] == 250
                    and self.frame["valid"] and age is not None and age < 250)

    def click_firmware_ready(self):
        return bool(self.frame is not None and all(type(self.frame.get(key)) is int
                    and 0 <= self.frame[key] <= 0xFFFFFFFF for key in ("ac", "bc")))

    def angles(self):
        if not self.frame:
            return [0., 0.]
        return [math.remainder(self.frame[key] - zero, 360)
                for key, zero in zip(("roll", "pitch"), self.config["neutral_deg"])]

    def hold(self, reason, disable=False):
        self.q_req = self.q_out[:]
        self.velocity = [0., 0., 0.]
        self.armed = False
        self.neutral_since = None
        self.neutral_ready = self.enabled and not disable
        self.reason = reason
        # The virtual gripper is independent of A, fault and stop. Its logical
        # open/closed target persists; disabled sessions freeze its output too.
        if disable:
            self.enabled = False
            self.owner = "NONE"
            self.neutral_ready = False
            self.click_baseline = None
            self.angle_ambiguous = False

    def receive(self, frame, now=None):
        now = self.clock() if now is None else now
        old = self.frame
        if old and frame["boot"] == old["boot"] and not newer(frame["seq"], old["seq"]):
            return False
        restart = old is None or frame["boot"] != old["boot"]
        input_lost = (not frame["valid"] or frame["who"] != 104 or frame["cal"] != 250
                      or frame["age_ms"] >= 250 or (old is not None and not self.valid(now)))
        if input_lost and self.enabled and self.owner != "REPLAY":
            detail = {1: "IMU 总线读取失败", 2: "IMU 采样间隔超时", 3: "加速度数据失效",
                      4: "转动过快，陀螺仪量程饱和", 5: "姿态解算输入失效"}.get(frame.get("rej"), "IMU 无效或数据过期")
            self.hold(detail + "；静置恢复后重新初始化", disable=True)
            self.event("INPUT_INVALID", detail)
        if restart and self.owner != "REPLAY":
            self.hold("腕端已连接／重启，请重新初始化", disable=True)
        # Stable, released data is required only for explicit initialization.
        if restart or input_lost or frame["a"] or frame["b"]:
            self.settled_samples.clear()
        else:
            self.settled_samples.append((now, frame["roll"], frame["pitch"]))
            self.settled_samples = [sample for sample in self.settled_samples if now - sample[0] <= .8]
        self.frame, self.received_at = frame.copy(), now
        if self.owner == "REPLAY" or not self.enabled or restart or input_lost:
            return True
        if not self.click_firmware_ready():
            self.hold("腕端固件需更新：缺少 A/B 点击计数 ac/bc", disable=True)
            return True
        if not self._track_wrist(old):
            return True
        deltas = {key: (frame[key] - self.click_baseline[key]) % 2**32 for key in ("ac", "bc")}
        self.click_baseline = {key: frame[key] for key in ("ac", "bc")}
        if any(delta >= 2**31 for delta in deltas.values()):
            self.hold("按钮计数异常，请重新初始化", disable=True)
            return True
        if deltas["bc"] % 2:
            self.open_req = 1. - self.open_req
            self.event("GRIPPER", "虚拟夹爪" + ("打开" if self.open_req else "闭合"))
        if deltas["ac"] % 2:
            if self.armed:
                self.hold("A 点击：跟随已暂停，B 仍可切换夹爪")
                self.event("PAUSE", "A 点击，暂停姿态跟随")
            else:
                self.armed = True
                self.neutral_ready = False
                self.reason = "协同角度跟随中；点击 A 暂停，点击 B 开合夹爪"
                self.event("RUN", "A 点击，按初始化姿态参考继续跟随")
        if self.angle_ambiguous:
            self.reason = "大角度姿态有歧义，回到启用参考附近恢复"
        if self.armed and not self.angle_ambiguous:
            self._map_requests()
        return True

    def _track_wrist(self, old):
        angles = self.angles()
        relative = [math.remainder(self.frame[key] - reference, 360)
                    for key, reference in zip(("roll", "pitch"), self.anchor_wrist)]
        increments = [math.remainder(new - previous, 360) for new, previous in zip(angles, self.last_wrist)]
        interval = (self.frame["ms"] - old["ms"]) % 2**32
        near_reference = all(abs(value) < 90 for value in relative)
        principal_cut = any(abs(new - previous) > 180 for new, previous in zip(relative, self.wrist_delta))
        discontinuity = interval <= 100 and (math.hypot(*increments) > 60 or principal_cut)
        if self.frame.get("att") == 2 and discontinuity and not near_reference and not self.angle_ambiguous:
            self.angle_ambiguous = True
            self.q_req = self.q_out[:]
            self.velocity = [0., 0., 0.]
            self.reason = "大角度姿态有歧义，回到启用参考附近恢复"
            self.event("ANGLE_AMBIGUOUS", self.reason)
        elif self.angle_ambiguous and near_reference:
            self.angle_ambiguous = False
            self.reason = "已回到启用参考附近，继续按固定参考跟随" if self.armed else "已回到启用参考附近，点击 A 开始跟随"
            self.event("ANGLE_RECOVERED", "已回到启用参考附近，固定参考保持不变")
        # The enable reference is immutable across clicks, pause, limits and
        # branch handling. Returning to it must always request the home pose.
        self.wrist_delta = relative
        self.last_wrist = angles
        return True

    def _pitch_limits(self):
        return (max(self.config["limits_deg"][1][0] - 45., self.config["limits_deg"][2][0] - 45.),
                min(self.config["limits_deg"][1][1] - 45., self.config["limits_deg"][2][1] - 45.))

    def _map_requests(self):
        effective = [math.copysign(max(0., abs(delta) - self.config["deadzone_deg"]), delta)
                     for delta in self.wrist_delta]
        for index, direction, bounds in ((0, self.config["roll_sign"], self.config["limits_deg"][0]),
                                         (1, self.config["pitch_sign"], self._pitch_limits())):
            raw = direction * effective[index]
            low, high = bounds
            limited = max(low, min(high, raw))
            if index == 0:
                self.q_req[0] = limited
            else:
                self.q_req[1] = self.q_req[2] = 45. + limited

    def mapping_state(self):
        return {"version": MAPPING_VERSION, "anchor_joint_deg": self.anchor_q[:],
                "anchor_wrist_deg": self.anchor_wrist[:] if self.anchor_wrist is not None else None,
                "delta_wrist_deg": self.wrist_delta[:], "deadzone_deg": self.config["deadzone_deg"],
                "limit_policy": LIMIT_POLICY, "angle_ambiguous": self.angle_ambiguous, "pitch_weights": [1., 1.],
                "pitch_limits_deg": list(self._pitch_limits())}

    def settled_neutral(self):
        if not self.valid() or not self.settled_samples or self.frame["a"] or self.frame["b"]:
            raise ValueError("请松开 A/B，在舒适的中立手势静置约 0.5 秒")
        now = self.clock()
        recent = [sample for sample in self.settled_samples if now - sample[0] <= .65]
        if len(recent) < 10 or recent[-1][0] - recent[0][0] < .48:
            raise ValueError("请松开 A/B，在舒适的中立手势静置约 0.5 秒")
        result = []
        for index in (1, 2):
            anchor = recent[-1][index]
            offsets = [math.remainder(row[index] - anchor, 360) for row in recent]
            if max(offsets) - min(offsets) > 3:
                raise ValueError("手腕仍在移动，请保持当前手势稳定约 0.5 秒后重试")
            result.append(math.remainder(anchor + sum(offsets) / len(offsets), 360))
        return result

    def gate_status(self):
        if self.owner == "REPLAY":
            return "REPLAY", ["历史回放中，实时输入不控制模型"], 0.
        if not self.valid():
            return "INPUT_INVALID", ["需要新鲜、有效且完成校准的 IMU 数据"], 0.
        if not self.click_firmware_ready():
            return "FIRMWARE_UPDATE", ["请更新腕端固件以提供 A/B 点击计数 ac/bc"], 0.
        if not self.enabled:
            return "DISABLED", ["松开 A/B 并静置，点击启用并回初始"], 0.
        if self.angle_ambiguous:
            return "ANGLE_AMBIGUOUS", ["大角度姿态有歧义，回到启用参考附近恢复"], 0.
        if self.armed:
            return "ACTIVE", ["A 点击暂停；B 点击开合夹爪"], 1.
        return "READY", ["A 点击开始跟随；B 点击开合夹爪"], 1.

    def enable(self):
        if self.owner == "REPLAY" or self.armed:
            raise ValueError("请先暂停跟随或停止历史回放，再启用并回初始")
        if not self.valid():
            raise ValueError("需要有效、已完成校准的真实 IMU")
        if not self.click_firmware_ready():
            raise ValueError("腕端固件需更新：缺少 A/B 点击计数 ac/bc")
        reference = self.settled_neutral()
        self.config["neutral_deg"] = reference
        self.q_req = [0., 45., 45.]
        self.q_out = self.q_req[:]
        self.velocity = [0., 0., 0.]
        self.open_req = self.open_out = 1.
        self.anchor_q = self.q_out[:]
        self.anchor_wrist = reference[:]
        self.last_wrist = self.angles()
        self.wrist_delta = self.last_wrist[:]
        self.angle_ambiguous = False
        self.click_baseline = {key: self.frame[key] for key in ("ac", "bc")}
        self.enabled, self.armed, self.owner = True, False, "WEARABLE"
        self.neutral_ready = True
        self.last_tick = self.clock()
        self.reason = "已回初始并记录手势参考；A 点击开始，B 点击开合夹爪"
        self.event("INITIALIZE", "已设置初始化手势参考，虚拟机械臂回到 0/45/45°")
        return reference

    initialize = enable

    def stop(self, reason="软件停止"):
        self.hold(reason, disable=True)
        self.event("STOP", reason)

    def reset(self):
        if self.enabled or self.owner == "REPLAY":
            raise ValueError("请先停止控制或回放")
        self.q_req = [0., 45., 45.]
        self.q_out = self.q_req[:]
        self.open_req = self.open_out = 1.
        self.velocity = [0., 0., 0.]
        self.angle_ambiguous = False
        self.event("RESET", "虚拟目标已复位至 0/45/45°")

    def configure(self, changes):
        if self.enabled or self.owner == "REPLAY":
            raise ValueError("仿真参数仅可在停止状态修改")
        config = {**deepcopy(self.config), **changes}
        validate_config(config)
        if any(not lo <= value <= hi for (lo, hi), value in zip(config["limits_deg"], self.q_out)):
            raise ValueError("当前目标不在新限位内，请先复位")
        self.config = config
        self.hold("仿真参数已更新，请重新初始化", disable=True)
        self.event("CONFIG", "仿真参数版本已更新")

    @staticmethod
    def _slew(current, requested, velocity, bounds, speed, acceleration, dt):
        low, high = bounds
        error = requested - current
        desired = math.copysign(min(speed, math.sqrt(2 * acceleration * abs(error))), error)
        change = acceleration * dt
        velocity += max(-change, min(change, desired - velocity))
        movement = velocity * dt
        if error * movement >= 0 and abs(movement) >= abs(error):
            current = requested
        else:
            current = max(low, min(high, current + movement))
        if (current <= low and velocity < 0) or (current >= high and velocity > 0):
            velocity = 0.
        return current, velocity

    def step(self, now=None):
        now = self.clock() if now is None else now
        elapsed = now - self.last_tick
        self.last_tick = now
        if self.owner == "REPLAY":
            return
        dt = max(0., min(elapsed, .05))
        if not self.valid(now):
            if self.enabled:
                self.hold("IMU 无效或数据过期，请重新初始化", disable=True)
        elif elapsed > .1:
            if self.enabled:
                self.hold("控制周期超时，跟随已暂停；点击 A 继续")
        elif self.enabled:
            if self.armed and not self.angle_ambiguous:
                self.q_out[0], self.velocity[0] = self._slew(self.q_out[0], self.q_req[0], self.velocity[0],
                    self.config["limits_deg"][0], self.config["max_velocity_deg_s"][0],
                    self.config["max_acceleration_deg_s2"][0], dt)
                pitch, velocity = self._slew(self.q_out[1] - 45., self.q_req[1] - 45., self.velocity[1],
                    self._pitch_limits(), min(self.config["max_velocity_deg_s"][1:]),
                    min(self.config["max_acceleration_deg_s2"][1:]), dt)
                self.q_out[1] = self.q_out[2] = 45. + pitch
                self.velocity[1] = self.velocity[2] = velocity
            change = dt / .2
            self.open_out += max(-change, min(change, self.open_req - self.open_out))

    def joint_state(self):
        return {"requested": pose(self.q_req, self.open_req), "output_target": pose(self.q_out, self.open_out),
                "measured": None, "velocity_deg_s": self.velocity[:],
                "limits": [abs(q - lo) < 1e-6 or abs(q - hi) < 1e-6
                           for q, (lo, hi) in zip(self.q_out, self.config["limits_deg"])]}
