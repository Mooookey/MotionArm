"""Dedicated serial worker. The HTTP event loop never touches a port handle."""
import asyncio
import os
from concurrent.futures import Future
from queue import Empty, Full, Queue
import threading
import time

from .protocol import LIMIT, decode, diagnostic, encode, newer


class SerialBridge:
    def __init__(self, receiver):
        self.receiver = receiver
        self.halt = threading.Event()
        self.thread = None
        self.commands = Queue(maxsize=1)
        self.stops = Queue(maxsize=1)

    async def request(self, kind, *, expected_session, **payload):
        result = Future()
        queue = self.stops if kind == "stop" else self.commands
        try:
            queue.put_nowait((kind, expected_session, payload, time.monotonic() + 0.39, result))
        except Full as error:
            raise ValueError("Serial command queue busy") from error
        return await asyncio.wait_for(asyncio.wrap_future(result), 0.49)

    @staticmethod
    def complete(result, value=None, error=None):
        # Atomically resolve or skip canceled HTTP requests.
        if result.set_running_or_notify_cancel():
            if error is not None:
                result.set_exception(error)
            else:
                result.set_result(value)

    def reject_queued(self):
        for queue in (self.stops, self.commands):
            try:
                *_, future = queue.get_nowait()
            except Empty:
                continue
            self.complete(future, error=ValueError("Serial session closed"))

    def start(self):
        loop = asyncio.get_running_loop()

        def emit(message):
            if not self.halt.is_set():
                loop.call_soon_threadsafe(self.receiver, message)

        self.thread = threading.Thread(target=self._run, args=(emit,), daemon=True)
        self.thread.start()

    async def close(self):
        self.halt.set()
        if self.thread:
            await asyncio.to_thread(self.thread.join, 2.0)
        self.reject_queued()

    def _run(self, emit):
        try:
            import serial
            from serial.tools import list_ports
        except ImportError:
            emit({"type": "offline", "detail": "缺少 PySerial"})
            return
        preferred = os.environ.get("MOTIONARM_SERIAL_PORT")
        baud = int(os.environ.get("MOTIONARM_SERIAL_BAUD", "115200"))
        while not self.halt.is_set():
            ports = [preferred] if preferred else [
                p.device for p in list_ports.comports() if p.vid in (0x1A86, 0x10C4)
            ]
            failure = "未识别到机械臂串口"
            for device in ports:
                if self.halt.is_set():
                    return
                for rate in dict.fromkeys((baud, 115200)):
                    try:
                        port = serial.Serial(baudrate=rate, timeout=0.02, write_timeout=0.15)
                        port.port = device
                        port.dtr = port.rts = False
                        with port:
                            self._listen(port, emit)
                            failure = f"{device} @ {rate}: 串口握手或遥测超时"
                    except (serial.SerialException, OSError, ValueError) as error:
                        failure = f"{device}: {error}"
                    self.reject_queued()
                    emit({"type": "offline", "detail": failure})
                    if self.halt.is_set():
                        return
            emit({"type": "offline", "detail": failure})
            self.halt.wait(2)

    def _listen(self, port, emit):
        inflight = None
        boot, session, previous = None, None, None
        transmitted_seq, hello_at, legacy = 1, 0, False
        deadline = time.monotonic() + 2.4
        pending = bytearray()
        overflow = False
        try:
            while not self.halt.is_set() and time.monotonic() < deadline:
                now = time.monotonic()
                if not boot and not legacy and now >= hello_at:
                    port.write(encode({"v": 1, "type": "hello", "seq": 1, "role": "pc"}))
                    hello_at = now + 0.7
                if inflight and inflight[1] <= now:
                    self.complete(inflight[2], error=ValueError("Robot ACK timeout"))
                    inflight = None
                try:
                    request = self.stops.get_nowait()
                except Empty:
                    request = None
                if request is not None:
                    if inflight:
                        self.complete(inflight[2], error=ValueError("Preempted by STOP"))
                        inflight = None
                    try:
                        *_, abandoned = self.commands.get_nowait()
                        self.complete(abandoned, error=ValueError("Preempted by STOP"))
                    except Empty:
                        pass
                if request is None and inflight is None:
                    try:
                        request = self.commands.get_nowait()
                    except Empty:
                        pass
                if request is not None:
                    kind, expected_session, payload, expires, future = request
                    if not boot or not session or session != expected_session or expires <= now or future.cancelled():
                        self.complete(future, error=ValueError("No current robot session"))
                    else:
                        transmitted_seq = (transmitted_seq + 1) % 2**32
                        inflight = (transmitted_seq, expires, future, kind)
                        port.write(encode({
                            "v": 1, "type": kind, "seq": transmitted_seq, "session_id": session,
                            **payload,
                        }))
                chunk = port.read(max(1, min(port.in_waiting, 256)))
                for byte in chunk:
                    if not overflow:
                        pending.append(byte)
                        overflow = len(pending) > LIMIT
                    if byte != 10:
                        continue
                    line = bytes(pending) if not overflow else b""
                    pending.clear()
                    overflow = False
                    probe = diagnostic(line)
                    if probe is not None and boot is None:
                        legacy = True
                        emit(probe)
                        deadline = time.monotonic() + 2.4
                        continue
                    try:
                        frame = decode(line)
                    except (ValueError, UnicodeError):
                        continue
                    if frame["type"] == "hello_ack" and frame.get("role") == "robot" and (
                        frame.get("protocol_version") == 1
                        and isinstance(frame.get("boot_id"), str)
                        and bool(frame["boot_id"])
                        and isinstance(frame.get("session_id"), str)
                        and bool(frame["session_id"])
                    ):
                        if boot is not None:
                            continue
                        boot, session, previous = frame["boot_id"], frame["session_id"], None
                        emit(frame)
                        deadline = time.monotonic() + 2.4
                        continue
                    if boot is None or frame["type"] == "hello_ack":
                        continue
                    if frame["type"] in ("ack", "nack"):
                        if (inflight and frame["seq"] == inflight[0] and frame.get("boot_id", boot) == boot
                                and frame.get("request_type") == inflight[3]):
                            self.complete(inflight[2], frame)
                            inflight = None
                        continue
                    if frame.get("boot_id") != boot:
                        return  # reconnect performs a new HELLO
                    if newer(frame["seq"], previous):
                        previous = frame["seq"]
                        emit(frame)
                        deadline = time.monotonic() + 2.4
        finally:
            if inflight:
                self.complete(inflight[2], error=ValueError("Serial link lost"))
