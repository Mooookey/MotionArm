"""Exercise the public virtual API with a controlled clock and no serial hardware."""
import asyncio
import csv
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

from app.virtual_api import CONVENTION, VirtualRuntime, create_virtual_app
from app.virtual import MAPPING_VERSION, forward_kinematics
from helpers import temporary_directory
from test_virtual import Clock
from test_wearable import frame


async def no_background_ticks(_runtime):
    await asyncio.Event().wait()


class VirtualApiTests(unittest.TestCase):
    def setUp(self):
        self.directory = temporary_directory("api-")
        root = Path(self.directory.name)
        self.database = root / "records.sqlite3"
        self.config_path = root / "virtual-config.json"
        self.clock = Clock()
        self.current = frame()
        self.app = create_virtual_app(self.database, self.config_path)
        real_runtime = VirtualRuntime
        self.constructor = patch("app.virtual_api.VirtualRuntime", side_effect=lambda database, config_path:
                                 real_runtime(database, config_path, clock=self.clock))
        self.tick_patch = patch.object(real_runtime, "generate", no_background_ticks)
        self.constructor.start()
        self.tick_patch.start()
        self.client = TestClient(self.app)
        self.client.__enter__()
        self.runtime = self.app.state.motionarm

    def tearDown(self):
        self.client.__exit__(None, None, None)
        self.tick_patch.stop()
        self.constructor.stop()
        self.directory.cleanup()

    def on_server(self, action):
        return self.client.portal.call(action)

    def inject(self, seconds=.04, **changes):
        self.clock.now += seconds
        self.current = {**self.current, "seq": (self.current["seq"] + 1) % 2**32,
                        "ms": self.current["ms"] + round(seconds * 1000), **changes}
        self.on_server(lambda: self.runtime.receive({"type": "frame", "frame": self.current}))
        self.on_server(self.runtime.step)

    def enable(self):
        for _ in range(15):
            self.inject(a=0, b=0)
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 200)

    def save_record(self):
        self.enable()
        for _ in range(4):
            self.inject(a=0)
        self.inject(ac=self.current["ac"] + 1, a=1)
        result = self.client.post("/api/recordings/start", json={"name": "real wrist virtual output"})
        self.assertEqual(result.status_code, 200, result.text)
        identifier = result.json()["id"]
        for _ in range(8):
            self.inject(roll=15, pitch=20)
        self.inject(ac=self.current["ac"] + 1, a=1, roll=0, pitch=0)
        self.inject(a=0)
        for _ in range(3):
            self.inject()
        saved = self.client.post("/api/recordings/stop").json()
        self.assertEqual(saved["status"], "completed")
        self.assertGreaterEqual(saved["frame_count"], 2)
        return identifier

    def test_disconnected_status_cannot_enable_or_record_and_no_mock_routes(self):
        status = self.client.get("/api/status").json()
        self.assertEqual(status["source"], "wearable")
        self.assertEqual(status["executor"]["kind"], "virtual")
        self.assertEqual(status["input"]["status"], "disconnected")
        self.assertIsNone(status["joint_state"]["measured"])
        self.assertFalse(status["control"]["can_enable"])
        for route in ("/api/virtual/enable", "/api/recordings/start"):
            self.assertEqual(self.client.post(route, json={"name": "invalid"}).status_code, 409)
        for operation in ("arm", "manual-target", "heartbeat", "mock-teleop", "release"):
            result = self.client.post("/api/control/" + operation, json={"q_deg": [90, 90, 90, 90]})
            self.assertEqual(result.status_code, 409)
            self.assertEqual(result.json()["detail"]["error_code"], "PHYSICAL_CONTROL_UNAVAILABLE")

    def test_guided_prepare_maps_natural_hand_to_dynamic_virtual_joints(self):
        for _ in range(20):
            self.inject(roll=131, pitch=5, a=0, b=0)
        result = self.client.post("/api/virtual/prepare")
        self.assertEqual(result.status_code, 200, result.text)
        status = self.client.get("/api/status").json()
        self.assertAlmostEqual(status["imu"]["roll_deg"], 0)
        self.assertEqual(status["imu"]["raw_roll_deg"], 131)
        self.assertEqual(status["control"]["gate_status"], "READY")
        self.assertFalse(status["control"]["armed"])
        self.inject(ac=1, a=1)
        for _ in range(20):
            self.inject(roll=148.5, pitch=22.5)
        moved = self.client.get("/api/status").json()["joint_state"]["output_target"]
        self.assertGreater(moved["base_yaw_deg"], 5)
        self.assertGreater(moved["shoulder_down_deg"], 48)
        self.assertEqual(moved["elbow_down_deg"], moved["shoulder_down_deg"])
        self.inject(bc=1, b=1)
        for _ in range(6):
            self.inject()
        self.assertLess(self.client.get("/api/status").json()["joint_state"]["output_target"]["gripper_open_ratio"], .1)
        self.inject(ac=2, a=1, roll=131, pitch=5)
        paused = self.client.get("/api/status").json()["joint_state"]["output_target"]
        for _ in range(5):
            self.inject(a=0, b=0)
        self.assertEqual(self.client.get("/api/status").json()["joint_state"]["output_target"], paused)
        self.assertLess(paused["gripper_open_ratio"], .1)
        self.inject(ac=3, a=1)
        for _ in range(20):
            self.inject(pitch=22.5)
        self.assertGreater(self.client.get("/api/status").json()["joint_state"]["output_target"]["elbow_down_deg"], 48)
        self.assertEqual(self.client.get("/api/status").json()["control"]["mode"], "COUPLED")
        self.assertEqual(self.client.get("/api/status").json()["control"]["mapping"], "COORDINATED_RELATIVE_ANGLE")

    def test_guided_prepare_rejects_motion_buttons_and_absent_input(self):
        self.assertEqual(self.client.post("/api/virtual/prepare").status_code, 409)
        for i in range(20):
            self.inject(roll=130 + i, pitch=5)
        self.assertEqual(self.client.post("/api/virtual/prepare").status_code, 409)
        for _ in range(20):
            self.inject(roll=131, pitch=5, a=1)
        self.assertEqual(self.client.post("/api/virtual/prepare").status_code, 409)

    def test_guided_prepare_accepts_stable_roll_across_angle_wrap(self):
        for i in range(20):
            self.inject(roll=179.8 if i % 2 else -179.8, pitch=5)
        result = self.client.post("/api/virtual/prepare")
        self.assertEqual(result.status_code, 200, result.text)
        self.assertLess(abs(self.client.get("/api/status").json()["imu"]["roll_deg"]), .5)

    def test_disconnect_keeps_reference_pose_but_never_crashes_or_reenables(self):
        self.enable()
        before = self.client.get("/api/status").json()["joint_state"]["output_target"]
        self.assertEqual(self.client.post("/api/input/disconnect").status_code, 200)
        self.clock.now += .04
        self.on_server(self.runtime.step)
        status = self.client.get("/api/status").json()
        self.assertEqual(status["input"]["status"], "disconnected")
        self.assertFalse(status["control"]["enabled"])
        self.assertEqual(status["joint_state"]["output_target"], before)
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)

    def test_config_persists_only_while_stopped_and_neutral_changes_identity(self):
        old = self.client.get("/api/virtual/config").json()["config_id"]
        result = self.client.post("/api/virtual/config", json={"roll_sign": -1})
        self.assertEqual(result.status_code, 200)
        self.assertNotEqual(result.json()["config_id"], old)
        self.assertEqual(json.loads(self.config_path.read_text())["roll_sign"], -1)
        self.enable()
        self.assertEqual(self.client.post("/api/virtual/config", json={"pitch_sign": -1}).status_code, 409)
        self.client.post("/api/virtual/stop")
        self.inject(roll=12, pitch=-7, a=0, b=0)
        self.assertEqual(self.client.post("/api/virtual/neutral").status_code, 200)
        status = self.client.get("/api/status").json()
        self.assertAlmostEqual(status["imu"]["roll_deg"], 0)
        self.assertAlmostEqual(status["imu"]["pitch_deg"], 0)

    def test_recording_preserves_input_buttons_targets_and_export_semantics(self):
        self.current = frame(att=2)
        identifier = self.save_record()
        detail = self.client.get(f"/api/recordings/{identifier}").json()
        self.assertEqual(detail["source"], "wearable")
        self.assertEqual(detail["metadata"]["input"], "imu_usb")
        self.assertEqual(detail["metadata"]["executor"], "virtual")
        self.assertEqual(detail["metadata"]["attitude_representation"], "relative_rotation_vector_v2")
        self.assertEqual(detail["fw_version"], "wearable-usb-click-att2")
        self.assertEqual(detail["metadata"]["controller_version"], MAPPING_VERSION)
        self.assertEqual(detail["metadata"]["mapping"], "COORDINATED_RELATIVE_ANGLE")
        self.assertEqual(detail["metadata"]["limit_policy"], "FIXED_REFERENCE_HARD_CLAMP")
        self.assertEqual(detail["metadata"]["angle_convention"], CONVENTION)
        self.assertIn("geometry", detail["metadata"])
        self.assertIn("config", detail["metadata"])
        self.assertEqual(detail["frames"][0]["t_ms"], 0)
        self.assertTrue(any(item["buttons"]["a"] for item in detail["frames"]))
        self.assertTrue(any(not item["buttons"]["a"] for item in detail["frames"]))
        self.assertIn("output_target", detail["frames"][0])
        self.assertIn("requested", detail["frames"][0])
        exported_json = self.client.get(f"/api/recordings/{identifier}/export?format=json")
        self.assertEqual(exported_json.json(), detail)
        exported_csv = self.client.get(f"/api/recordings/{identifier}/export?format=csv")
        rows = list(csv.DictReader(io.StringIO(exported_csv.text.lstrip("\ufeff"))))
        self.assertEqual(len(rows), detail["frame_count"])
        self.assertEqual(rows[0]["source"], "wearable")
        self.assertEqual(rows[0]["executor"], "virtual")
        self.assertIn("output_gripper_open_ratio", rows[0])
        self.assertIn("ac", rows[0])
        self.assertIn("bc", rows[0])

    def test_offline_replay_excludes_live_control_and_finishes_at_recorded_pose(self):
        identifier = self.save_record()
        final = self.client.get(f"/api/recordings/{identifier}").json()["frames"][-1]["output_target"]
        blocked = self.client.post("/api/replays/preflight", json={"record_id": identifier}).json()
        self.assertFalse(blocked["ok"])
        self.client.post("/api/virtual/stop")
        self.client.post("/api/input/disconnect")
        check = self.client.post("/api/replays/preflight", json={"record_id": identifier}).json()
        self.assertTrue(check["ok"], check)
        self.assertEqual(self.client.post("/api/replays/start", json={"preflight_id": check["preflight_id"]}).status_code, 200)
        running = self.client.get("/api/status").json()
        self.assertEqual(running["control"]["owner"], "REPLAY")
        self.assertEqual(running["input"]["status"], "disconnected")
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)
        self.assertEqual(self.client.post("/api/recordings/start", json={"name": "conflict"}).status_code, 409)
        self.clock.now += 2
        self.on_server(self.runtime.step)
        ended = self.client.get("/api/status").json()
        self.assertFalse(ended["replay"]["active"])
        self.assertEqual(ended["control"]["owner"], "NONE")
        self.assertFalse(ended["control"]["enabled"])
        for key in ("base_yaw_deg", "shoulder_down_deg", "elbow_down_deg"):
            self.assertAlmostEqual(ended["joint_state"]["output_target"][key], final[key])

    def test_gap_and_reboot_interrupt_records_and_prevent_continuous_replay(self):
        for reason in ("gap", "reboot"):
            with self.subTest(reason=reason):
                self.client.post("/api/virtual/stop")
                self.current = frame(boot=self.current["boot"] + 1)
                self.enable()
                record = self.client.post("/api/recordings/start", json={"name": reason}).json()
                self.inject()
                self.inject()
                if reason == "gap":
                    self.clock.now += .3
                    self.on_server(self.runtime.step)
                else:
                    self.inject(boot=self.current["boot"] + 1, seq=1)
                saved = self.client.get(f"/api/recordings/{record['id']}").json()
                self.assertEqual(saved["status"], "interrupted")
                self.assertGreater(saved["frame_count"], 0)
                self.client.post("/api/virtual/stop")
                report = self.client.post("/api/replays/preflight", json={"record_id": record["id"]}).json()
                self.assertFalse(report["ok"])

    def test_legacy_record_is_exportable_but_never_replayable(self):
        def seed():
            record = self.runtime.records.start("legacy", "serial", "old", "old-profile", "bench", "0.1.0")
            for seq, stamp in ((1, 100), (2, 140)):
                self.runtime.records.append("old", "old-profile", stamp,
                                            {"seq": seq, "q_out": [90, 65, 105, 25]})
            self.runtime.records.finish()
            return record["id"]
        identifier = self.on_server(seed)
        detail = self.client.get(f"/api/recordings/{identifier}").json()
        self.assertEqual(detail["source"], "serial")
        self.assertIsNone(detail["metadata"])
        self.assertEqual(self.client.get(f"/api/recordings/{identifier}/export?format=csv").status_code, 200)
        report = self.client.post("/api/replays/preflight", json={"record_id": identifier}).json()
        self.assertFalse(report["ok"])
        self.assertIn("旧记录", "".join(report["violations"]))

    def test_nonlocal_write_requests_are_rejected(self):
        self.assertEqual(self.client.post("/api/virtual/stop", headers={"Origin": "https://example.org"}).status_code, 403)

    def test_malformed_identifiers_and_extreme_config_are_rejected(self):
        for value in ([], {}, None, 7, "x" * 129):
            self.assertEqual(self.client.post("/api/replays/start", json={"preflight_id": value}).status_code, 422)
            self.assertEqual(self.client.post("/api/replays/preflight", json={"record_id": value}).status_code, 422)
        self.assertEqual(self.client.post("/api/virtual/config", json={"deadzone_deg": 10**400}).status_code, 409)

    def test_fresh_buttons_remain_identifiable_when_imu_invalid(self):
        self.inject(valid=0, a=1, b=0)
        status = self.client.get("/api/status").json()
        self.assertEqual(status["input"]["status"], "invalid")
        self.assertTrue(status["buttons"]["a"])
        self.assertFalse(status["buttons"]["b"])
        self.assertFalse(status["control"]["can_enable"])
        self.clock.now += .3
        status = self.client.get("/api/status").json()
        self.assertFalse(status["buttons"]["a"])

    def test_history_replay_uses_recorded_geometry(self):
        identifier = self.save_record()
        self.client.post("/api/virtual/stop")
        def change_geometry():
            meta = self.runtime.records.get(identifier)["metadata"]
            meta["geometry"]["table_to_shoulder_axis"] = 190
            with self.runtime.records.db:
                self.runtime.records.db.execute("UPDATE recordings SET metadata=? WHERE id=?", (json.dumps(meta), identifier))
        self.on_server(change_geometry)
        check = self.client.post("/api/replays/preflight", json={"record_id": identifier}).json()
        self.assertTrue(check["ok"])
        self.client.post("/api/replays/start", json={"preflight_id": check["preflight_id"]})
        status = self.client.get("/api/status").json()
        self.assertEqual(status["replay"]["geometry"]["table_to_shoulder_axis"], 190)
        expected = forward_kinematics(status["joint_state"]["output_target"], status["replay"]["geometry"])
        self.assertAlmostEqual(status["joint_state"]["tip_mm"][2], expected[2])

    def test_enable_and_prepare_both_initialize_reference_home_and_new_counter_baseline(self):
        for route in ("/api/virtual/enable", "/api/virtual/prepare"):
            with self.subTest(route=route):
                self.client.post("/api/virtual/stop")
                self.on_server(lambda: setattr(self.runtime.controller, "q_out", [30., 20., 55.]))
                for _ in range(20):
                    self.inject(roll=135, pitch=12, a=0, b=0, ac=1001, bc=1003)
                result = self.client.post(route)
                self.assertEqual(result.status_code, 200, result.text)
                status = self.client.get("/api/status").json()
                self.assertEqual(status["joint_state"]["output_target"],
                                 {"base_yaw_deg": 0, "shoulder_down_deg": 45, "elbow_down_deg": 45,
                                  "gripper_open_ratio": 1})
                self.assertEqual(status["imu"]["neutral_deg"], [135., 12.])
                self.assertFalse(status["control"]["armed"])
                self.assertEqual(status["buttons"]["ac"], 1001)
                self.assertEqual(status["buttons"]["bc"], 1003)
                self.inject()
                self.assertFalse(self.client.get("/api/status").json()["control"]["armed"])

    def test_initialization_rejects_running_recording_and_old_button_firmware(self):
        self.enable()
        self.inject(ac=1)
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)
        self.inject(ac=2)
        for _ in range(20):
            self.inject(a=0, b=0)
        record = self.client.post("/api/recordings/start", json={"name": "recording blocks reset"})
        self.assertEqual(record.status_code, 200)
        self.assertEqual(self.client.post("/api/virtual/prepare").status_code, 409)
        self.client.post("/api/virtual/stop")
        def old_firmware():
            message = frame(seq=self.current["seq"] + 1, boot=self.current["boot"] + 1)
            del message["ac"]
            del message["bc"]
            self.runtime.receive({"type": "frame", "frame": message})
        self.on_server(old_firmware)
        status = self.client.get("/api/status").json()
        self.assertIsNone(status["buttons"]["ac"])
        self.assertIsNone(status["buttons"]["bc"])
        self.assertFalse(status["control"]["can_enable"])
        self.assertEqual(status["control"]["gate_status"], "FIRMWARE_UPDATE")
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)

    def test_transient_invalid_frame_interrupts_record_even_before_control_tick(self):
        self.enable()
        record = self.client.post("/api/recordings/start", json={"name": "transient invalid"}).json()
        self.inject()
        self.inject()
        self.clock.now += .02
        invalid = {**self.current, "seq": self.current["seq"] + 1, "valid": 0}
        recovered = {**invalid, "seq": invalid["seq"] + 1, "valid": 1}
        self.on_server(lambda: self.runtime.receive({"type": "frame", "frame": invalid}))
        self.on_server(lambda: self.runtime.receive({"type": "frame", "frame": recovered}))
        self.on_server(self.runtime.step)
        result = self.client.get(f"/api/recordings/{record['id']}").json()
        self.assertEqual(result["status"], "interrupted")
        self.assertGreater(result["frame_count"], 0)
        self.assertFalse(self.client.get("/api/status").json()["control"]["enabled"])


if __name__ == "__main__":
    unittest.main()
