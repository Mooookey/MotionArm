"""Passive, bounded USB telemetry reader. This module never writes to a device."""
import asyncio
import json
import math
import threading
import time

from .protocol import newer, finite_number

MAX_FRAME = 384


def decode_wearable(raw):
    if len(raw) > MAX_FRAME or not raw.endswith(b"\n"):
        raise ValueError("Invalid wearable frame length")
    def reject(value):
        raise ValueError("Nonfinite JSON")
    data = json.loads(raw, parse_constant=reject)
    if not isinstance(data, dict) or type(data.get("v")) is not int or data["v"] != 1 or data.get("role") != "wearable":
        raise ValueError("Not wearable USB v1")
    for key in ("boot", "seq", "ms", "age_ms", "who", "cal", "short", "long", "errors"):
        if type(data.get(key)) is not int or not 0 <= data[key] <= 0xFFFFFFFF:
            raise ValueError("Invalid integer: " + key)
    if not data["boot"] or data["cal"] > 250:
        raise ValueError("Invalid boot or calibration")
    if "att" in data and (type(data["att"]) is not int or data["att"] not in (1, 2)):
        raise ValueError("Invalid attitude representation")
    for key, maximum in (("rej", 5), ("gr", 2000), ("rc", 0xFFFFFFFF),
                         ("ac", 0xFFFFFFFF), ("bc", 0xFFFFFFFF)):
        if key in data and (type(data[key]) is not int or not 0 <= data[key] <= maximum):
            raise ValueError("Invalid sensor diagnostic: " + key)
    for key in ("valid", "a", "b", "mode"):
        if type(data.get(key)) is not int or data[key] not in (0, 1):
            raise ValueError("Invalid bit: " + key)
    for key in ("roll", "pitch", "norm_g", "temp_c"):
        value = data.get(key)
        if not finite_number(value):
            raise ValueError("Invalid number: " + key)
    if abs(data["roll"]) > 180 or abs(data["pitch"]) > 180 or not 0 <= data["norm_g"] <= 16:
        raise ValueError("Invalid IMU range")
    return data


class LineDecoder:
    def __init__(self):
        self.pending = bytearray()
        self.overflow = False
        self.errors = 0

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
                        if time.monotonic() >= deadline:
                            raise TimeoutError("未收到腕端 USB v1 数据；请检查固件与端口")
            except (serial.SerialException, OSError, ValueError, TimeoutError) as error:
                emit({"type": "offline", "detail": str(error)})
            self.halt.wait(1)
