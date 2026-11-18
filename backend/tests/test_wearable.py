"""USB framing and passive transport behavior without opening hardware."""
import json
import sys
import types
import unittest
from unittest.mock import patch

from app.wearable import LineDecoder, MAX_FRAME, WearableReader, decode_wearable


def frame(**changes):
    return {"v": 1, "role": "wearable", "boot": 12, "seq": 1, "ms": 3000,
            "age_ms": 0, "who": 104, "valid": 1, "cal": 250, "roll": 0.0,
            "pitch": 0.0, "a": 0, "b": 0, "ac": 0, "bc": 0, "mode": 0, "short": 0, "long": 0,
            "norm_g": 0.64, "temp_c": 28.3, "errors": 0, **changes}


def encode(value):
    return json.dumps(value, separators=(",", ":")).encode() + b"\n"


class WearableProtocolTests(unittest.TestCase):
    def test_attitude_representation_is_explicit_and_backward_compatible(self):
        self.assertEqual(decode_wearable(encode(frame(att=2)))["att"], 2)
        self.assertNotIn("att", decode_wearable(encode(frame())))
        for value in (True, 3, "2"):
            with self.assertRaises(ValueError):
                decode_wearable(encode(frame(att=value)))
        self.assertEqual(decode_wearable(encode(frame(gr=2000, rej=4, rc=1)))["rej"], 4)
        with self.assertRaises(ValueError):
            decode_wearable(encode(frame(rej=6)))

    def test_valid_sensor_frame_keeps_diagnostic_anomaly(self):
        original = frame()
        self.assertLess(len(encode(original)), MAX_FRAME)
        self.assertEqual(decode_wearable(encode(original)), original)
        self.assertEqual(decode_wearable(encode(original))["norm_g"], .64)

    def test_rejects_invalid_types_ranges_and_nonfinite_numbers(self):
        cases = ({"v": True}, {"v": 2}, {"role": "robot"}, {"boot": 0}, {"seq": -1},
                 {"seq": 2**32}, {"cal": 251}, {"a": True}, {"a": 2}, {"mode": -1},
                 {"roll": 181}, {"pitch": float("inf")}, {"norm_g": -1},
                 {"temp_c": float("nan")}, {"errors": "0"}, {"long": None})
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                decode_wearable(encode(frame(**changes)))

    def test_missing_field_and_float_overflow_are_invalid(self):
        value = frame()
        del value["age_ms"]
        with self.assertRaises(ValueError):
            decode_wearable(encode(value))
        with self.assertRaises(ValueError):
            decode_wearable(encode(frame()).replace(b'"temp_c":28.3', b'"temp_c":1e999'))

    def test_stream_handles_noise_fragmentation_and_multiple_lines(self):
        decoder = LineDecoder()
        packet = encode(frame())
        self.assertEqual(decoder.feed(b"boot noise\xff\n" + packet[:25]), [])
        self.assertEqual(decoder.feed(packet[25:] + encode(frame(seq=2))), [frame(), frame(seq=2)])
        self.assertEqual(decoder.errors, 1)

    def test_overflow_has_bounded_memory_and_recovers_at_next_line(self):
        decoder = LineDecoder()
        self.assertEqual(decoder.feed(b"x" * 100_000), [])
        self.assertLessEqual(len(decoder.pending), MAX_FRAME + 1)
        self.assertEqual(decoder.feed(b"\n" + encode(frame())), [frame()])
        self.assertEqual(decoder.errors, 1)

    def test_newline_and_length_are_mandatory(self):
        with self.assertRaises(ValueError):
            decode_wearable(encode(frame()).rstrip(b"\n"))
        with self.assertRaises(ValueError):
            decode_wearable(b" " * (MAX_FRAME + 1) + b"\n")


class PassiveReaderTests(unittest.TestCase):
    def test_reader_filters_sequence_wrap_and_never_writes_to_port(self):
        emitted = []
        values = [frame(seq=x) for x in (2**32 - 2, 2**32 - 1, 0, 0, 2**32 - 1, 1)]
        values += [frame(boot=13, seq=0, short=900)]
        chunks = [encode(value) for value in values]
        reader = WearableReader(emitted.append)

        class ReadOnlyPort:
            """Deliberately has no write method: writes must fail this test."""
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

        serial_module = types.SimpleNamespace(SerialException=OSError, Serial=None)
        reader.serial_factory = lambda **kwargs: ReadOnlyPort()
        with patch.dict(sys.modules, {"serial": serial_module}):
            reader._run("FAKE_ONLY", emitted.append)
        received = [item["frame"] for item in emitted if item["type"] == "frame"]
        self.assertEqual([item["seq"] for item in received], [2**32 - 2, 2**32 - 1, 0, 1, 0])
        self.assertEqual(received[-1]["boot"], 13)
        self.assertEqual(emitted[0], {"type": "connecting", "port": "FAKE_ONLY"})


if __name__ == "__main__":
    unittest.main()
