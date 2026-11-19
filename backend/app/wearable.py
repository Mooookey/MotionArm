"""Passive, bounded USB telemetry reader. This module never writes to a device."""
import asyncio
import json
import math
import threading
import time

from .protocol import newer, finite_number
from .attitude import normalize

MAX_FRAME = 384
ATTITUDE_REPRESENTATION = "quaternion_sensor_to_world_v3"
FRAME_FIELDS = frozenset("v att role boot seq ms age_ms who valid cal q gyro_dps accel_g a b ac bc norm_g temp_c errors rej gr rc".split())


class FirmwareIncompatible(ValueError):
    """Recognizable wearable data with an unsupported protocol/schema."""


def validate_frame(data):
    if not isinstance(data, dict) or data.get("role") != "wearable":
        raise ValueError("Not wearable USB telemetry")
    if type(data.get("v")) is not int or data["v"] != 1 or type(data.get("att")) is not int or data["att"] != 3:
        raise FirmwareIncompatible("腕端协议不兼容，请升级为完整四元数 att:3 固件")
    if set(data) != FRAME_FIELDS:
        raise FirmwareIncompatible("腕端字段不兼容，请升级为完整四元数 att:3 固件")
    for key in ("boot", "seq", "ms", "age_ms", "who", "cal", "errors", "rc", "ac", "bc", "rej", "gr"):
        if type(data[key]) is not int or not 0 <= data[key] <= 0xFFFFFFFF:
            raise ValueError("Invalid integer: " + key)
    if not data["boot"] or data["who"] > 255 or data["cal"] > 250 or data["rej"] > 5 or data["gr"] != 2000:
        raise ValueError("Invalid device diagnostic")
    for key in ("valid", "a", "b"):
        if type(data[key]) is not int or data[key] not in (0, 1):
            raise ValueError("Invalid bit: " + key)
    for key, size in (("q", 4), ("gyro_dps", 3), ("accel_g", 3)):
        values = data[key]
        if not isinstance(values, list) or len(values) != size or not all(finite_number(v) for v in values):
            raise ValueError("Invalid vector: " + key)
    if abs(math.hypot(*data["q"]) - 1) > .01:
        raise ValueError("Invalid quaternion norm")
    if any(abs(v) > 2031 for v in data["gyro_dps"]) or any(abs(v) > 4 for v in data["accel_g"]):
        raise ValueError("Invalid sensor range")
    if not all(finite_number(data[key]) for key in ("norm_g", "temp_c")) or not 0 <= data["norm_g"] <= 7:
        raise ValueError("Invalid sensor diagnostic")
    return {**data, "q": normalize(data["q"]), "gyro_dps": data["gyro_dps"][:], "accel_g": data["accel_g"][:]}


def decode_wearable(raw):
    if len(raw) > MAX_FRAME or not raw.endswith(b"\n"):
        raise ValueError("Invalid wearable frame length")
    def reject(value):
        raise ValueError("Nonfinite JSON")
    data = json.loads(raw, parse_constant=reject)
    return validate_frame(data)


class LineDecoder:
    def __init__(self):
        self.pending = bytearray()
        self.overflow = False
        self.errors = 0
        self.protocol_error = None

    def feed(self, chunk):
        frames = []
        for byte in chunk:
            if not self.overflow:
                self.pending.append(byte)
                self.overflow = len(self.pending) > MAX_FRAME
            if byte != 10:
                continue
            try:
                if self.overflow:
                    raise ValueError("Overflow")
                frames.append(decode_wearable(bytes(self.pending)))
                self.protocol_error = None
            except FirmwareIncompatible as error:
                self.errors += 1
                self.protocol_error = str(error)
            except (ValueError, UnicodeError, OverflowError):
                self.errors += 1
            self.pending.clear()
            self.overflow = False
        return frames


class WearableReader:
    def __init__(self, receiver, serial_factory=None):
        self.receiver = receiver
        self.serial_factory = serial_factory
        self.halt = threading.Event()
        self.thread = None

    def start(self, device):
        loop = asyncio.get_running_loop()
        def emit(message):
            if not self.halt.is_set():
                loop.call_soon_threadsafe(self.receiver, message)
        self.thread = threading.Thread(target=self._run, args=(device, emit), daemon=True)
        self.thread.start()

    async def close(self):
        self.halt.set()
        if self.thread:
            await asyncio.to_thread(self.thread.join, 2)

    def _run(self, device, emit):
        import serial
        factory = self.serial_factory or serial.Serial
        while not self.halt.is_set():
            emit({"type": "connecting", "port": device})
            try:
                port = factory(baudrate=115200, timeout=0.05)
                port.port = device
                port.dtr = port.rts = False
                with port:
                    decoder = LineDecoder()
                    boot = previous = None
                    reported_error = None
                    # Opening some CH340 boards resets them; permit boot + calibration.
                    deadline = time.monotonic() + 4
                    while not self.halt.is_set():
                        chunk = port.read(max(1, min(port.in_waiting, 512)))
                        for frame in decoder.feed(chunk):
                            if frame["boot"] != boot:
                                boot, previous = frame["boot"], None
                            if not newer(frame["seq"], previous):
                                continue
                            previous = frame["seq"]
                            deadline = time.monotonic() + 1
                            emit({"type": "frame", "frame": frame, "parse_errors": decoder.errors})
                        if decoder.protocol_error and decoder.protocol_error != reported_error:
                            emit({"type": "incompatible", "detail": decoder.protocol_error, "parse_errors": decoder.errors})
                        reported_error = decoder.protocol_error
                        if time.monotonic() >= deadline:
                            if decoder.protocol_error:
                                raise FirmwareIncompatible(decoder.protocol_error)
                            raise TimeoutError("未收到腕端 USB att:3 数据；请检查固件与端口")
            except (serial.SerialException, OSError, ValueError, TimeoutError) as error:
                emit({"type": "incompatible" if isinstance(error, FirmwareIncompatible) else "offline", "detail": str(error)})
            self.halt.wait(1)
