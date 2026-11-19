"""Strict quaternion USB framing and passive transport, without hardware."""
import json
import math
import sys
import types
import unittest
from unittest.mock import patch

from app.wearable import FRAME_FIELDS, LineDecoder, MAX_FRAME, WearableReader, decode_wearable


def sensor_quaternion(x=0., y=0., z=0.):
    angle = math.hypot(x, y, z)
    scale = math.sin(math.radians(angle) / 2) / angle if angle else 0
    return [math.cos(math.radians(angle) / 2), x * scale, y * scale, z * scale]


def wrist_quaternion(roll=0., pitch=0., z=0.):
    return sensor_quaternion(pitch, -roll, z)


def frame(**changes):
    return {"v": 1, "att": 3, "role": "wearable", "boot": 12, "seq": 1, "ms": 3000,
            "age_ms": 0, "who": 104, "valid": 1, "cal": 250, "q": [1., 0., 0., 0.],
            "gyro_dps": [0., 0., 0.], "accel_g": [0., 0., 0.64],
            "a": 0, "b": 0, "ac": 0, "bc": 0, "norm_g": 0.64, "temp_c": 28.3,
            "errors": 0, "rej": 0, "gr": 2000, "rc": 0, **changes}


def encode(value):
    return json.dumps(value, separators=(",", ":")).encode() + b"\n"


class WearableProtocolTests(unittest.TestCase):
    def test_only_complete_current_schema_is_accepted(self):
        self.assertEqual(set(frame()), FRAME_FIELDS)
        self.assertEqual(decode_wearable(encode(frame())), frame())
        for value in (None, True, 1, 2, "3", 4):
            with self.subTest(att=value), self.assertRaises(ValueError):
                decode_wearable(encode(frame(att=value)))
        for key in FRAME_FIELDS:
            value = frame()
            del value[key]
            with self.subTest(missing=key), self.assertRaises(ValueError):
                decode_wearable(encode(value))
        for key in ("roll", "pitch", "mode", "short", "long", "unknown"):
            with self.subTest(extra=key), self.assertRaises(ValueError):
                decode_wearable(encode(frame(**{key: 0})))

    def test_raw_diagnostics_remain_unscaled_and_quaternion_is_normalized(self):
        original = frame(q=[1.005, 0, 0, 0], accel_g=[.1, -.2, 1.58], norm_g=1.596)
        decoded = decode_wearable(encode(original))
        self.assertEqual(decoded["q"], [1., 0., 0., 0.])
        self.assertEqual(decoded["accel_g"], original["accel_g"])
        self.assertEqual(decoded["norm_g"], 1.596)
        self.assertEqual(original["q"][0], 1.005)

    def test_rejects_invalid_types_ranges_and_nonfinite_numbers(self):
        cases = ({"v": True}, {"v": 2}, {"role": "robot"}, {"boot": 0}, {"seq": -1},
                 {"seq": 2**32}, {"cal": 251}, {"a": True}, {"a": 2}, {"valid": False},
                 {"who": 256}, {"q": [0, 0, 0, 0]}, {"q": [1.011, 0, 0, 0]},
                 {"q": [1, 0, 0]}, {"q": [True, 0, 0, 0]}, {"q": [1, 0, 0, float("nan")]},
                 {"gyro_dps": [0, 0]}, {"gyro_dps": [0, 0, float("inf")]},
                 {"gyro_dps": [2032, 0, 0]}, {"accel_g": [4.1, 0, 0]},
                 {"accel_g": "0,0,1"}, {"norm_g": -1}, {"norm_g": 7.1},
                 {"temp_c": float("nan")}, {"errors": "0"}, {"rej": 6}, {"gr": 500})
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                decode_wearable(encode(frame(**changes)))
        with self.assertRaises(ValueError):
            decode_wearable(encode(frame()).replace(b'"temp_c":28.3', b'"temp_c":1e999'))

    def test_stream_handles_noise_fragmentation_and_multiple_lines(self):
        decoder = LineDecoder()
        packet = encode(frame())
        self.assertEqual(decoder.feed(b"boot noise\xff\n" + packet[:25]), [])
        self.assertEqual(decoder.feed(packet[25:] + encode(frame(seq=2))), [frame(), frame(seq=2)])
        self.assertEqual(decoder.errors, 1)

    def test_old_protocol_reports_upgrade_and_valid_frame_recovers(self):
        decoder = LineDecoder()
        self.assertEqual(decoder.feed(encode(frame(att=2))), [])
        self.assertIn("att:3", decoder.protocol_error)
        self.assertEqual(decoder.feed(encode(frame())), [frame()])
        self.assertIsNone(decoder.protocol_error)

    def test_overflow_has_bounded_memory_and_recovers_at_next_line(self):
        decoder = LineDecoder()
        self.assertEqual(decoder.feed(b"x" * 100_000), [])
        self.assertLessEqual(len(decoder.pending), MAX_FRAME + 1)
        self.assertEqual(decoder.feed(b"\n" + encode(frame())), [frame()])
        self.assertEqual(decoder.errors, 1)

    def test_newline_and_length_are_mandatory(self):
        self.assertLess(len(encode(frame())), MAX_FRAME)
        with self.assertRaises(ValueError):
            decode_wearable(encode(frame()).rstrip(b"\n"))
        with self.assertRaises(ValueError):
            decode_wearable(b" " * (MAX_FRAME + 1) + b"\n")


class PassiveReaderTests(unittest.TestCase):
    def read_values(self, values):
        emitted = []
        chunks = [encode(value) for value in values]
        reader = WearableReader(emitted.append)

        class ReadOnlyPort:
            """No write method: accidental output must fail this test."""
            in_waiting = 1

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return False

            def read(self, _count):
                if chunks:
                    return chunks.pop(0)
                reader.halt.set()
                return b""

        reader.serial_factory = lambda **kwargs: ReadOnlyPort()
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(SerialException=OSError, Serial=None)}):
            reader._run("FAKE_ONLY", emitted.append)
        return emitted

    def test_sequence_wrap_reboot_and_read_only_port(self):
        values = [frame(seq=x) for x in (2**32 - 2, 2**32 - 1, 0, 0, 2**32 - 1, 1)]
        emitted = self.read_values(values + [frame(boot=13, seq=0)])
        received = [item["frame"] for item in emitted if item["type"] == "frame"]
        self.assertEqual([item["seq"] for item in received], [2**32 - 2, 2**32 - 1, 0, 1, 0])
        self.assertEqual(received[-1]["boot"], 13)
        self.assertEqual(emitted[0], {"type": "connecting", "port": "FAKE_ONLY"})

    def test_incompatible_input_is_reported_without_becoming_a_frame(self):
        emitted = self.read_values([frame(att=2), frame(att=2)])
        self.assertFalse(any(item["type"] == "frame" for item in emitted))
        failures = [item for item in emitted if item["type"] == "incompatible"]
        self.assertEqual(len(failures), 1)
        self.assertIn("升级", failures[0]["detail"])


if __name__ == "__main__":
    unittest.main()
