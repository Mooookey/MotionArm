"""Deterministic virtual executor. Angles are geometric targets, never feedback."""
from copy import deepcopy
from hashlib import sha256
import json
import math
import time

from .protocol import newer, finite_number
from .attitude import distance_deg, mean, relative, wrist_rotation_vector
from .wearable import validate_frame

POSE_KEYS = ("base_yaw_deg", "shoulder_down_deg", "elbow_down_deg")
MAPPING_VERSION = "coordinated-angle-v3"
LIMIT_POLICY = "FIXED_REFERENCE_HARD_CLAMP"
# Verified on the worn wrist: right tilt produces negative control Roll.
# Base yaw is positive counterclockwise; Pitch retains the downward joint sign.
JOINT_SIGNS = (1, -1)
MOUNTING = {
    "profile": "hand_back_x_right_y_forearm_z_down",
    "label": "芯片正面向下；X 向右、Y 从手背指向手肘、Z 向下",
    "control_from_sensor": {"Roll": "-Y", "Pitch": "+X", "Z": "+Z"},
    "joint_signs": list(JOINT_SIGNS),
}
DEFAULT_CONFIG = {
    "deadzone_deg": 5.0,
    "limits_deg": [[-45.0, 45.0], [0.0, 60.0], [0.0, 60.0]],
    "max_velocity_deg_s": [20.0, 15.0, 15.0],
    "max_acceleration_deg_s2": [60.0, 60.0, 60.0],
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
    if not isinstance(config, dict) or set(config) != set(DEFAULT_CONFIG):
        raise ValueError("未知或缺少仿真参数")
    def number(value):
        return finite_number(value)
    if not number(config["deadzone_deg"]) or not 0 < config["deadzone_deg"] <= 20:
        raise ValueError("死区应大于 0 且不超过 20°")
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


class VirtualController:
    """One initialized wrist reference, click-to-run and coordinated arm targets."""
    def __init__(self, clock=time.monotonic, config=None):
        self.clock = clock
        self.config = deepcopy(DEFAULT_CONFIG if config is None else config)
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
        self.events = []
        self.settled_samples = []
        self.reference_quaternion = None
        self.angle_ambiguous = False
        self.click_baseline = None

    @property
    def config_id(self):
        identity = {"mapping": MAPPING_VERSION, "mounting": MOUNTING,
                    "limit_policy": LIMIT_POLICY, "config": self.config}
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

    def angles(self):
        vector = self.rotation_vector()
        return vector[:2] if vector is not None else [None, None]

    def rotation_vector(self):
        if self.frame is None or self.reference_quaternion is None:
            return None
        return wrist_rotation_vector(self.reference_quaternion, self.frame["q"])

    def hold(self, reason, disable=False):
        self.q_req = self.q_out[:]
        self.velocity = [0., 0., 0.]
        self.armed = False
        self.reason = reason
        # The virtual gripper is independent of A, fault and stop. Its logical
        # open/closed target persists; disabled sessions freeze its output too.
        if disable:
            self.enabled = False
            self.owner = "NONE"
            self.click_baseline = None
            self.angle_ambiguous = False
            self.reference_quaternion = None

    def receive(self, frame, now=None):
        frame = validate_frame(frame)
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
        if (restart or input_lost or frame["a"] or frame["b"] or math.hypot(*frame["gyro_dps"]) >= 5
                or self.received_at is not None and now - self.received_at > .1 + 1e-9):
            self.settled_samples.clear()
        if not input_lost and not frame["a"] and not frame["b"] and math.hypot(*frame["gyro_dps"]) < 5:
            self.settled_samples.append((now, frame["q"]))
            self.settled_samples = [sample for sample in self.settled_samples if now - sample[0] <= .8]
        self.frame, self.received_at = frame.copy(), now
        if self.owner == "REPLAY" or not self.enabled or restart or input_lost:
            return True
        self._track_wrist(old)
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
                self.reason = "协同角度跟随中；点击 A 暂停，点击 B 开合夹爪"
                self.event("RUN", "A 点击，按初始化姿态参考继续跟随")
        if self.angle_ambiguous:
            self.reason = "大角度姿态有歧义，回到启用参考附近恢复"
        if self.armed and not self.angle_ambiguous:
            self._map_requests()
        return True

    def _track_wrist(self, old):
        angle = distance_deg(self.reference_quaternion, self.frame["q"])
        previous = relative(self.reference_quaternion, old["q"])
        current = relative(self.reference_quaternion, self.frame["q"])
        # Telemetry can step over the entire 175..185 degree guard band.
        # Align signs first: q/-q is representation noise, not a branch crossing.
        if sum(a*b for a, b in zip(previous, current)) < 0:
            current = [-v for v in current]
        crossed_cut = previous[0] * current[0] < 0
        if (angle >= 175 - 1e-9 or crossed_cut) and not self.angle_ambiguous:
            self.angle_ambiguous = True
            self.q_req = self.q_out[:]
            self.velocity = [0., 0., 0.]
            self.reason = "大角度姿态有歧义，回到启用参考附近恢复"
            self.event("ANGLE_AMBIGUOUS", self.reason)
        elif self.angle_ambiguous and angle <= 90 + 1e-9:
            self.angle_ambiguous = False
            self.reason = "已回到启用参考附近，继续按固定参考跟随" if self.armed else "已回到启用参考附近，点击 A 开始跟随"
            self.event("ANGLE_RECOVERED", "已回到启用参考附近，固定参考保持不变")

    def _pitch_limits(self):
        return (max(self.config["limits_deg"][1][0] - 45., self.config["limits_deg"][2][0] - 45.),
                min(self.config["limits_deg"][1][1] - 45., self.config["limits_deg"][2][1] - 45.))

    def _map_requests(self):
        effective = [math.copysign(max(0., abs(delta) - self.config["deadzone_deg"]), delta)
                     for delta in self.angles()]
        for index, bounds in ((0, self.config["limits_deg"][0]), (1, self._pitch_limits())):
            raw = JOINT_SIGNS[index] * effective[index]
            low, high = bounds
            limited = max(low, min(high, raw))
            if index == 0:
                self.q_req[0] = limited
            else:
                self.q_req[1] = self.q_req[2] = 45. + limited

    def mapping_state(self):
        return {"version": MAPPING_VERSION,
                "reference_quaternion": self.reference_quaternion[:] if self.reference_quaternion is not None else None,
                "delta_rotation_vector_deg": self.rotation_vector(), "deadzone_deg": self.config["deadzone_deg"],
                "limit_policy": LIMIT_POLICY, "angle_ambiguous": self.angle_ambiguous, "pitch_weights": [1., 1.],
                "pitch_limits_deg": list(self._pitch_limits())}

    def settled_reference(self):
        if not self.valid() or not self.settled_samples or self.frame["a"] or self.frame["b"]:
            raise ValueError("请松开 A/B，在舒适的中立手势静置约 0.5 秒")
        now = self.clock()
        recent = [sample for sample in self.settled_samples if now - sample[0] <= .65]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < .48 - 1e-9:
            raise ValueError("请松开 A/B，在舒适的中立手势静置约 0.5 秒")
        if any(b[0] - a[0] > .1 + 1e-9 for a, b in zip(recent, recent[1:])) or any(
                distance_deg(a[1], b[1]) > 3 + 1e-9 for i, a in enumerate(recent) for b in recent[i+1:]):
            raise ValueError("手腕仍在移动，请保持当前手势稳定约 0.5 秒后重试")
        return mean([sample[1] for sample in recent])

    def enable_blockers(self):
        if self.owner == "REPLAY" or self.armed:
            return ["请先暂停跟随或停止历史回放，再启用并回初始"]
        try:
            self.settled_reference()
        except ValueError as error:
            return [str(error)]
        return []

    def gate_status(self):
        if self.owner == "REPLAY":
            return "REPLAY", ["历史回放中，实时输入不控制模型"]
        if not self.valid():
            return "INPUT_INVALID", ["需要新鲜、有效且完成校准的 IMU 数据"]
        if not self.enabled:
            return "DISABLED", self.enable_blockers() or ["点击启用并回初始"]
        if self.angle_ambiguous:
            return "ANGLE_AMBIGUOUS", ["大角度姿态有歧义，回到启用参考附近恢复"]
        if self.armed:
            return "ACTIVE", ["A 点击暂停；B 点击开合夹爪"]
        return "READY", self.enable_blockers() or ["A 点击开始跟随；B 点击开合夹爪"]

    def enable(self):
        if self.owner == "REPLAY" or self.armed:
            raise ValueError("请先暂停跟随或停止历史回放，再启用并回初始")
        if not self.valid():
            raise ValueError("需要有效、已完成校准的真实 IMU")
        reference = self.settled_reference()
        self.q_req = [0., 45., 45.]
        self.q_out = self.q_req[:]
        self.velocity = [0., 0., 0.]
        self.open_req = self.open_out = 1.
        self.reference_quaternion = reference
        self.angle_ambiguous = False
        self.click_baseline = {key: self.frame[key] for key in ("ac", "bc")}
        self.enabled, self.armed, self.owner = True, False, "WEARABLE"
        self.last_tick = self.clock()
        self.reason = "已回初始并记录手势参考；A 点击开始，B 点击开合夹爪"
        self.event("INITIALIZE", "已设置初始化手势参考，虚拟机械臂回到 0/45/45°")
        return reference

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
