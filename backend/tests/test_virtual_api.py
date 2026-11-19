"""Public schema3 API, recording provenance and history-only compatibility."""
import asyncio
from copy import deepcopy
import csv
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

from app.attitude import distance_deg, multiply
from app.virtual_api import CONVENTION, VirtualRuntime, create_virtual_app
from app.virtual import DEFAULT_CONFIG, MAPPING_VERSION, MOUNTING, forward_kinematics, pose
from app.wearable import ATTITUDE_REPRESENTATION
from helpers import temporary_directory
from test_virtual import Clock
from test_wearable import frame, sensor_quaternion, wrist_quaternion


async def no_background_ticks(_runtime):
    await asyncio.Event().wait()


class VirtualApiTests(unittest.TestCase):
    def setUp(self):
        self.directory = temporary_directory("api-")
        root = Path(self.directory.name)
        self.database, self.config_path = root / "records.sqlite3", root / "virtual-config.json"
        self.clock, self.current = Clock(), frame()
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

    def status(self):
        return self.client.get("/api/status").json()

    def inject(self, seconds=.04, **changes):
        self.clock.now += seconds
        if self.runtime.manager_token:
            self.on_server(lambda: self.runtime.heartbeat(self.runtime.manager_token))
        self.current = {**self.current, "seq": (self.current["seq"]+1) % 2**32,
                        "ms": (self.current["ms"]+round(seconds*1000)) % 2**32, **changes}
        self.on_server(lambda: self.runtime.receive({"type": "frame", "frame": self.current}))
        self.on_server(self.runtime.step)

    def enable(self, **changes):
        for _ in range(20):
            self.inject(a=0, b=0, **changes)
        response = self.client.post("/api/virtual/enable")
        self.assertEqual(response.status_code, 200, response.text)

    def save_record(self):
        self.enable()
        self.inject(ac=self.current["ac"]+1, a=1)
        response = self.client.post("/api/recordings/start", json={"name": "quaternion wrist"})
        self.assertEqual(response.status_code, 200, response.text)
        identifier = response.json()["id"]
        for _ in range(8):
            self.inject(a=0, q=wrist_quaternion(-15, -20, 8))
        self.inject(ac=self.current["ac"]+1, a=1)
        for _ in range(3):
            self.inject(a=0)
        saved = self.client.post("/api/recordings/stop").json()
        self.assertEqual(saved["status"], "completed")
        return identifier

    def export_csv(self, identifier):
        response = self.client.get(f"/api/recordings/{identifier}/export?format=csv")
        self.assertEqual(response.status_code, 200)
        return list(csv.DictReader(io.StringIO(response.text.lstrip("\ufeff"))))

    def start_replay(self, identifier):
        check = self.client.post("/api/replays/preflight", json={"record_id": identifier}).json()
        self.assertTrue(check["ok"], check)
        result = self.client.post("/api/replays/start", json={"preflight_id": check["preflight_id"]})
        self.assertEqual(result.status_code, 200, result.text)

    def test_disconnected_schema_has_no_fake_angles_or_hardware_feedback(self):
        state = self.status()
        self.assertEqual(state["schema_version"], 4)
        self.assertEqual(state["input"]["status"], "disconnected")
        self.assertEqual(state["input"]["attitude_representation"], ATTITUDE_REPRESENTATION)
        self.assertIsNone(state["imu"]["quaternion"])
        self.assertIsNone(state["imu"]["roll_deg"])
        self.assertIsNone(state["imu"]["pitch_deg"])
        self.assertIsNone(state["joint_state"]["measured"])
        self.assertFalse(state["control"]["can_enable"])
        self.assertIsNone(state["control"]["follow"]["reference_quaternion"])
        self.assertEqual(set(state["buttons"]), {"a", "b", "ac", "bc"})
        for removed in ("raw_roll_deg", "raw_pitch_deg", "raw_sensor_x_deg", "neutral_deg", "mounting_profile"):
            self.assertNotIn(removed, state["imu"])
        for removed in ("mode", "neutral_ready", "neutral_progress"):
            self.assertNotIn(removed, state["control"])
        for route in ("/api/virtual/enable", "/api/recordings/start"):
            self.assertEqual(self.client.post(route, json={"name": "invalid"}).status_code, 409)
        for operation in ("arm", "manual-target", "mock-teleop", "release"):
            response = self.client.post("/api/control/"+operation, json={"q_deg": [90]*4})
            self.assertEqual(response.status_code, 409)
            self.assertEqual(response.json()["detail"]["error_code"], "PHYSICAL_CONTROL_UNAVAILABLE")

    def test_unique_initialization_api_and_backend_readiness_gate(self):
        for removed in ("prepare", "neutral"):
            self.assertEqual(self.client.post("/api/virtual/"+removed).status_code, 404)
        self.inject()
        self.assertFalse(self.status()["control"]["can_enable"])
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)
        self.enable()
        self.assertFalse(self.status()["control"]["can_enable"])
        self.inject(a=1)
        self.assertFalse(self.status()["control"]["can_enable"])
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)
        for i in range(20):
            self.inject(a=0, q=sensor_quaternion(z=i))
        self.assertFalse(self.status()["control"]["can_enable"])
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)

    def test_compound_reference_maps_true_relative_rotation_with_fixed_signs(self):
        reference = sensor_quaternion(73, -42, 35)
        self.enable(q=reference)
        state = self.status()
        self.assertAlmostEqual(distance_deg(state["control"]["follow"]["reference_quaternion"], reference), 0)
        self.assertAlmostEqual(state["imu"]["roll_deg"], 0)
        self.assertEqual(state["control"]["gate_status"], "READY")
        self.assertFalse(state["control"]["armed"])
        self.inject(ac=1, a=1)
        for _ in range(25):
            self.inject(a=0, q=multiply(reference, wrist_quaternion(-17.5, -17.5)))
        state = self.status()
        requested, output = state["joint_state"]["requested"], state["joint_state"]["output_target"]
        self.assertAlmostEqual(requested["base_yaw_deg"], -12.5)
        self.assertAlmostEqual(requested["shoulder_down_deg"], 57.5)
        self.assertLess(output["base_yaw_deg"], -5)
        self.assertEqual(output["shoulder_down_deg"], output["elbow_down_deg"])
        self.inject(bc=1, b=1)
        for _ in range(6):
            self.inject()
        self.assertLess(self.status()["joint_state"]["output_target"]["gripper_open_ratio"], .1)
        self.inject(ac=2, a=1, q=reference)
        paused = self.status()["joint_state"]["output_target"]
        for _ in range(5):
            self.inject(a=0, b=0)
        self.assertEqual(self.status()["joint_state"]["output_target"], paused)
        self.inject(ac=3)
        self.assertEqual(self.status()["control"]["mapping"], "COORDINATED_RELATIVE_ANGLE")

    def test_configuration_is_reduced_persisted_and_reference_never_persisted(self):
        old = self.client.get("/api/virtual/config").json()
        self.assertEqual(set(old["config"]), set(DEFAULT_CONFIG))
        self.assertEqual(old["mounting"], MOUNTING)
        response = self.client.post("/api/virtual/config", json={"deadzone_deg": 7})
        self.assertEqual(response.status_code, 200)
        new_id = response.json()["config_id"]
        self.assertNotEqual(new_id, old["config_id"])
        self.assertEqual(set(json.loads(self.config_path.read_text())), set(DEFAULT_CONFIG))
        for key in ("full_scale_deg", "mounting_profile", "roll_sign", "pitch_sign", "neutral_deg"):
            self.assertEqual(self.client.post("/api/virtual/config", json={key: 1}).status_code, 409)
        self.enable(q=sensor_quaternion(50, -30, 10))
        self.assertEqual(self.status()["config_id"], new_id)
        self.assertNotIn("reference_quaternion", json.loads(self.config_path.read_text()))
        self.assertEqual(self.client.post("/api/virtual/config", json={"deadzone_deg": 8}).status_code, 409)
        self.client.post("/api/virtual/stop")
        self.assertIsNone(self.status()["control"]["follow"]["reference_quaternion"])

    def test_disconnect_and_stale_input_hold_outputs_and_require_reinitialization(self):
        self.enable()
        self.inject(ac=1, q=wrist_quaternion(-20))
        before = self.status()["joint_state"]["output_target"]
        self.clock.now += .3
        self.on_server(self.runtime.step)
        state = self.status()
        self.assertEqual(state["input"]["status"], "stale")
        self.assertFalse(state["control"]["enabled"])
        self.assertEqual(state["joint_state"]["output_target"], before)
        self.assertEqual(self.client.post("/api/input/disconnect").status_code, 200)
        self.assertEqual(self.status()["input"]["status"], "disconnected")
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)

    def test_incompatible_protocol_immediately_stops_and_interrupts_record(self):
        self.enable()
        record = self.client.post("/api/recordings/start", json={"name": "protocol fault"}).json()
        self.inject()
        self.inject()
        stamp = self.runtime.controller.received_at
        self.on_server(lambda: self.runtime.receive({"type": "frame", "frame": frame(att=2)}))
        state = self.status()
        self.assertEqual(state["input"]["status"], "incompatible")
        self.assertIn("att:3", state["input"]["detail"])
        self.assertEqual(state["control"]["gate_status"], "PROTOCOL_INCOMPATIBLE")
        self.assertFalse(state["control"]["enabled"])
        self.assertFalse(state["control"]["can_enable"])
        self.assertIsNone(self.runtime.controller.received_at)
        self.assertIsNotNone(stamp)
        self.assertEqual(self.client.get(f"/api/recordings/{record['id']}").json()["status"], "interrupted")
        self.inject()
        self.assertEqual(self.status()["input"]["status"], "valid")
        self.assertFalse(self.status()["control"]["enabled"])

    def test_current_recording_and_csv_preserve_complete_pose_provenance(self):
        identifier = self.save_record()
        detail = self.client.get(f"/api/recordings/{identifier}").json()
        metadata = detail["metadata"]
        self.assertEqual(metadata["schema_version"], 4)
        self.assertEqual(metadata["attitude_representation"], ATTITUDE_REPRESENTATION)
        self.assertEqual(metadata["controller_version"], MAPPING_VERSION)
        self.assertEqual(metadata["mounting"], MOUNTING)
        self.assertEqual(metadata["reference_quaternion"], [1, 0, 0, 0])
        self.assertEqual(detail["fw_version"], "wearable-usb-click-att3")
        self.assertEqual(metadata["angle_convention"], CONVENTION)
        self.assertEqual(detail["frames"][0]["t_ms"], 0)
        sample = detail["frames"][0]
        self.assertEqual(len(sample["imu"]["quaternion"]), 4)
        self.assertEqual(len(sample["imu"]["gyro_dps"]), 3)
        self.assertEqual(len(sample["imu"]["accel_g"]), 3)
        self.assertEqual(len(sample["follow"]["delta_rotation_vector_deg"]), 3)
        self.assertNotIn("mode", sample)
        self.assertEqual(set(sample["buttons"]), {"a", "b", "ac", "bc"})
        self.assertEqual(self.client.get(f"/api/recordings/{identifier}/export?format=json").json(), detail)
        rows = self.export_csv(identifier)
        self.assertEqual(len(rows), detail["frame_count"])
        for key in ("q_w", "q_x", "q_y", "q_z", "reference_q_w", "relative_z_deg", "gyro_z_dps", "accel_z_g", "reject_reason", "output_gripper_open_ratio"):
            self.assertIn(key, rows[0])
        for key in ("mode", "short", "long", "legacy_payload"):
            self.assertNotIn(key, rows[0])
        self.assertAlmostEqual(float(rows[0]["reference_q_w"]), 1)
        self.assertAlmostEqual(float(rows[0]["relative_z_deg"]), 8)

    def test_offline_replay_excludes_live_input_and_finishes_at_saved_target(self):
        identifier = self.save_record()
        final = self.client.get(f"/api/recordings/{identifier}").json()["frames"][-1]["output_target"]
        self.assertFalse(self.client.post("/api/replays/preflight", json={"record_id": identifier}).json()["ok"])
        self.client.post("/api/virtual/stop")
        self.client.post("/api/input/disconnect")
        self.start_replay(identifier)
        self.assertEqual(self.status()["control"]["owner"], "REPLAY")
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)
        self.assertEqual(self.client.post("/api/recordings/start", json={"name": "conflict"}).status_code, 409)
        self.inject(boot=99, ac=100, bc=100, q=wrist_quaternion(80, 80))
        self.assertEqual(self.status()["control"]["owner"], "REPLAY")
        self.clock.now += 2
        self.on_server(self.runtime.step)
        state = self.status()
        self.assertFalse(state["replay"]["active"])
        self.assertEqual(state["control"]["owner"], "NONE")
        self.assertEqual(state["joint_state"]["output_target"], final)

    def test_gap_reboot_and_transient_invalid_interrupt_recording(self):
        for reason in ("gap", "reboot", "transient"):
            with self.subTest(reason=reason):
                self.client.post("/api/virtual/stop")
                self.current = frame(boot=self.current["boot"]+1)
                self.enable()
                record = self.client.post("/api/recordings/start", json={"name": reason}).json()
                self.inject()
                self.inject()
                if reason == "gap":
                    self.clock.now += .3
                    self.on_server(self.runtime.step)
                elif reason == "reboot":
                    self.inject(boot=self.current["boot"]+1, seq=1)
                else:
                    invalid = {**self.current, "seq": self.current["seq"]+1, "valid": 0}
                    self.on_server(lambda: self.runtime.receive({"type": "frame", "frame": invalid}))
                    self.on_server(lambda: self.runtime.receive({"type": "frame", "frame": {**invalid, "seq": invalid["seq"]+1, "valid": 1}}))
                saved = self.client.get(f"/api/recordings/{record['id']}").json()
                self.assertEqual(saved["status"], "interrupted")
                self.assertGreater(saved["frame_count"], 0)
                self.assertFalse(self.status()["control"]["enabled"])
                self.assertFalse(self.client.post("/api/replays/preflight", json={"record_id": record["id"]}).json()["ok"])

    def seed_legacy(self, known_convention):
        def seed():
            metadata = {"executor": "virtual", "schema_version": 2, "angle_convention": CONVENTION,
                        "geometry": deepcopy(self.runtime.geometry), "config": deepcopy(DEFAULT_CONFIG)} if known_convention else None
            record = self.runtime.records.start("legacy", "wearable" if known_convention else "serial", "old", "old-profile", "bench", "0.1.0", metadata=metadata)
            for seq, stamp in ((1, 100), (2, 140)):
                sample = {"seq": seq, "imu": {"roll_deg": 131., "pitch_deg": 12.},
                          "buttons": {"a": False, "b": False, "short": 7, "long": 3}, "mode": "COUPLED", "owner": "WEARABLE"}
                if known_convention:
                    sample.update(requested=pose([seq*5, 45, 45], 1), output_target=pose([seq*5, 45, 45], 1))
                else:
                    sample["q_out"] = [90, 65, 105, 25]
                self.runtime.records.append("old", "old-profile", stamp, sample)
            self.runtime.records.finish()
            return record["id"]
        return self.on_server(seed)

    def test_known_legacy_virtual_record_replays_saved_targets_without_reinterpretation(self):
        identifier = self.seed_legacy(True)
        original = self.client.get(f"/api/recordings/{identifier}").json()
        rows = self.export_csv(identifier)
        self.assertEqual(rows[0]["roll_deg"], "131.0")
        self.assertEqual(rows[0]["short"], "7")
        self.assertEqual(rows[0]["mode"], "COUPLED")
        self.start_replay(identifier)
        self.clock.now += 1
        self.on_server(self.runtime.step)
        self.assertEqual(self.status()["joint_state"]["output_target"]["base_yaw_deg"], 10)
        self.assertEqual(self.client.get(f"/api/recordings/{identifier}").json(), original)

    def test_unknown_legacy_record_exports_original_payload_but_never_replays(self):
        identifier = self.seed_legacy(False)
        detail = self.client.get(f"/api/recordings/{identifier}").json()
        self.assertIsNone(detail["metadata"])
        rows = self.export_csv(identifier)
        self.assertEqual(json.loads(rows[0]["legacy_payload"]), detail["frames"][0])
        report = self.client.post("/api/replays/preflight", json={"record_id": identifier}).json()
        self.assertFalse(report["ok"])
        self.assertIn("旧记录", "".join(report["violations"]))

    def test_replay_uses_recorded_geometry_and_preflight_tokens_expire(self):
        identifier = self.save_record()
        self.client.post("/api/virtual/stop")
        def change_geometry():
            metadata = self.runtime.records.get(identifier)["metadata"]
            metadata["geometry"]["table_to_shoulder_axis"] = 190
            with self.runtime.records.db:
                self.runtime.records.db.execute("UPDATE recordings SET metadata=? WHERE id=?", (json.dumps(metadata), identifier))
        self.on_server(change_geometry)
        token = self.client.post("/api/replays/preflight", json={"record_id": identifier}).json()["preflight_id"]
        self.clock.now += 31
        self.assertEqual(self.client.post("/api/replays/start", json={"preflight_id": token}).status_code, 409)
        self.start_replay(identifier)
        state = self.status()
        self.assertEqual(state["replay"]["geometry"]["table_to_shoulder_axis"], 190)
        expected = forward_kinematics(state["joint_state"]["output_target"], state["replay"]["geometry"])
        self.assertEqual(state["joint_state"]["tip_mm"], expected)

    def test_buttons_remain_identifiable_on_invalid_imu_then_expire(self):
        self.inject(valid=0, a=1)
        state = self.status()
        self.assertEqual(state["input"]["status"], "invalid")
        self.assertTrue(state["buttons"]["a"])
        self.assertFalse(state["control"]["can_enable"])
        self.clock.now += .3
        self.assertFalse(self.status()["buttons"]["a"])

    def test_running_and_recording_reject_reinitialization_and_share_gate(self):
        self.enable()
        self.inject(ac=1)
        self.assertFalse(self.status()["control"]["can_enable"])
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)
        self.inject(ac=2)
        for _ in range(20):
            self.inject(a=0, b=0)
        self.assertFalse(self.status()["control"]["can_enable"])
        self.client.post("/api/recordings/start", json={"name": "gate"})
        self.assertFalse(self.status()["control"]["can_enable"])
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)

    def test_nonlocal_requests_and_malformed_identifiers_are_rejected(self):
        self.assertEqual(self.client.post("/api/virtual/stop", headers={"Origin": "https://example.org"}).status_code, 403)
        for value in ([], {}, None, 7, "x"*129):
            self.assertEqual(self.client.post("/api/replays/start", json={"preflight_id": value}).status_code, 422)
            self.assertEqual(self.client.post("/api/replays/preflight", json={"record_id": value}).status_code, 422)
        self.assertEqual(self.client.post("/api/virtual/config", json={"deadzone_deg": 10**400}).status_code, 409)


class ConfigMigrationTests(unittest.TestCase):
    def test_known_legacy_fields_are_removed_once_preserving_motion_parameters(self):
        with temporary_directory("config-migration-") as directory:
            root = Path(directory)
            config_path = root / "config.json"
            config = {**deepcopy(DEFAULT_CONFIG), "deadzone_deg": 8, "max_velocity_deg_s": [9, 8, 7],
                      "full_scale_deg": 30, "mounting_profile": "sensor_xy", "roll_sign": 1,
                      "pitch_sign": 1, "neutral_deg": [135, -42]}
            config_path.write_text(json.dumps(config), encoding="utf-8")
            runtime = VirtualRuntime(root / "records.sqlite3", config_path, Clock())
            try:
                self.assertEqual(set(runtime.controller.config), set(DEFAULT_CONFIG))
                self.assertEqual(runtime.controller.config["max_velocity_deg_s"], [9, 8, 7])
                self.assertEqual(runtime.controller.config["deadzone_deg"], 8)
                self.assertIsNone(runtime.controller.reference_quaternion)
                self.assertFalse(runtime.controller.enabled)
                self.assertIn("收缩", runtime.config_warning)
                self.assertEqual(json.loads(config_path.read_text()), runtime.controller.config)
                self.assertFalse(config_path.with_suffix(".tmp").exists())
            finally:
                runtime.records.db.close()
            reopened = VirtualRuntime(root / "records.sqlite3", config_path, Clock())
            try:
                self.assertIsNone(reopened.config_warning)
                self.assertIsNone(reopened.controller.reference_quaternion)
            finally:
                reopened.records.db.close()

    def test_invalid_or_unknown_legacy_configuration_does_not_leak_old_values(self):
        for extra in ({"unknown": 1}, {"deadzone_deg": 100}):
            with self.subTest(extra=extra), temporary_directory("config-invalid-") as directory:
                root = Path(directory)
                config_path = root / "config.json"
                config_path.write_text(json.dumps({**DEFAULT_CONFIG, "full_scale_deg": 30, **extra}), encoding="utf-8")
                runtime = VirtualRuntime(root / "records.sqlite3", config_path, Clock())
                try:
                    self.assertEqual(runtime.controller.config, DEFAULT_CONFIG)
                    self.assertIsNotNone(runtime.config_warning)
                finally:
                    runtime.records.db.close()


if __name__ == "__main__":
    unittest.main()
