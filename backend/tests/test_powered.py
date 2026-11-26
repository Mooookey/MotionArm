"""Powered USB authority, operator gates and history boundaries without hardware."""
from copy import deepcopy
import asyncio
import csv
import io
import json
import unittest

from app.robot import SnapshotAssembler, RobotLink, decode, ROBOT_MAC, trial_profile
from app.virtual import pose
from app.virtual_api import CONVENTION
from test_executor import robot_parts
import test_virtual_api as api_helpers

PROFILE, HASH = trial_profile()


def powered_parts(number=1, stage="locked", prepared=False, enabled=False):
    parts = robot_parts(number, enabled=enabled)
    for item in parts.values():
        item["v"] = 3
    parts["health"].update(mode="robot_powered", output_locked=stage in ("locked", "disabled"), manager_active=prepared)
    parts["targets"].update(requested=pose([0, 45, 45], 1), output_target=pose([0, 45, 45], 1))
    parts["output"] = {**{k: parts["control"][k] for k in ("v", "boot_id", "snapshot_id", "t_ms")},
        "type": "output", "seq": number*6+5, "profile_hash": HASH, "stage": stage,
        "pwm_ticks": [330, 211, 215, 232] if stage == "holding" else None,
        "last_write_ms": 100 if stage == "holding" else None, "write_errors": 0,
        "pwm_enabled": None if stage == "unknown" else stage == "holding", "committed": stage == "holding",
        "power_confirmed": stage == "holding", "prepared": prepared, "diagnostic_active": False,
        "trial_limits_deg": deepcopy(PROFILE["operating_limits_deg"]),
        "max_velocity_deg_s": PROFILE["speed_deg_s"], "max_acceleration_deg_s2": PROFILE["acceleration_deg_s2"]}
    return parts


class PoweredProtocolTests(unittest.TestCase):
    def test_current_limits_hash_and_target_boundaries_are_checked_together(self):
        parts = powered_parts()
        parts["targets"]["requested"] = pose([45, 20, 60], 1)
        parts["targets"]["output_target"] = pose([-45, 60, 20], 1)
        SnapshotAssembler.validate(parts, "robot_powered")
        for kind, key, value in (("requested", "base_yaw_deg", 45.001),
                                  ("output_target", "shoulder_down_deg", 19.999),
                                  ("requested", "elbow_down_deg", 60.001)):
            invalid = deepcopy(parts)
            invalid["targets"][kind][key] = value
            with self.assertRaises(ValueError):
                SnapshotAssembler.validate(invalid, "robot_powered")
        for field, old in (("trial_limits_deg", [[-3, 3], [42, 48], [42, 48]]),
                           ("trial_limits_deg", [[-20, 20], [35, 55], [35, 55]]),
                           ("profile_hash", "sha256:673bb8f6dd5b2478f5e5c0fe698a8b5c10cb1a957dcb2edbbfd2ec926604063c")):
            invalid = deepcopy(parts)
            invalid["output"][field] = old
            with self.assertRaises(ValueError):
                SnapshotAssembler.validate(invalid, "robot_powered")

    def test_versioned_atomic_snapshot_and_legacy_client_rejection(self):
        parts = powered_parts()
        assembler = SnapshotAssembler(mode="robot_powered")
        for item in list(parts.values())[:-1]:
            self.assertIsNone(assembler.feed(item, len(json.dumps(item).encode())))
        self.assertIsNotNone(assembler.feed(parts["output"], 500))
        raw = json.dumps(parts["output"], separators=(",", ":")).encode() + b"\n"
        self.assertEqual(decode(raw, 3)["profile_hash"], HASH)
        with self.assertRaises(ValueError):
            decode(raw)
        with self.assertRaises(ValueError):
            SnapshotAssembler.validate(parts)

    def test_unknown_output_is_not_claimed_held_or_measured(self):
        parts = powered_parts(stage="unknown")
        SnapshotAssembler.validate(parts, "robot_powered")
        for key, value in (("pwm_enabled", True),):
            invalid = deepcopy(parts)
            invalid["output"][key] = value
            with self.assertRaises(ValueError):
                SnapshotAssembler.validate(invalid, "robot_powered")
        parts["targets"]["measured"] = pose([0, 45, 45], 1)
        with self.assertRaises(ValueError):
            SnapshotAssembler.validate(parts, "robot_powered")


class PoweredApiTests(unittest.TestCase):
    setUp = api_helpers.VirtualApiTests.setUp
    tearDown = api_helpers.VirtualApiTests.tearDown
    on_server = api_helpers.VirtualApiTests.on_server
    status = api_helpers.VirtualApiTests.status

    def test_disconnected_powered_readonly_config_has_identity_without_claiming_board_acceptance(self):
        self.assertEqual(self.client.post("/api/executor/select", json={"kind": "robot_powered"}).status_code, 200)
        configuration = self.client.get("/api/executor/config").json()
        self.assertEqual(configuration["config_id"], HASH)
        self.assertEqual(configuration["profile_hash"], HASH)
        self.assertEqual(configuration["config"]["limits_deg"], [[-45, 45], [20, 60], [20, 60]])
        state = self.status()
        self.assertFalse(state["executor"]["connected"])
        self.assertIsNone(state["executor"]["profile_hash"])
        self.assertEqual(state["config_id"], "robot_powered:unknown")
        self.assertFalse(state["control"]["can_enable"])
        self.assertIsNone(state["joint_state"]["output_target"])
        self.assertIsNone(state["joint_state"]["measured"])

    def test_readonly_configuration_metadata_and_follow_limits_use_current_profile(self):
        self.setup_robot()
        configuration = self.client.get("/api/executor/config").json()
        self.assertEqual(configuration["config"]["limits_deg"], [[-45, 45], [20, 60], [20, 60]])
        self.assertEqual(configuration["config"]["max_velocity_deg_s"], [15, 15, 15])
        self.assertEqual(configuration["config"]["max_acceleration_deg_s2"], [30, 30, 30])
        self.assertEqual(configuration["config"]["deadzone_deg"], 5)
        self.assertEqual(configuration["diagnostic_offsets_deg"],
                         [[-15, -10, -5, 0, 5, 10, 15], [-10, -5, 0, 5, 10], [-10, -5, 0, 5, 10]])
        self.assertEqual(configuration["profile_hash"], HASH)
        self.assertEqual(configuration["config_id"], HASH)
        self.assertEqual(self.status()["control"]["follow"]["pitch_limits_deg"], [-25, 15])
        self.assertEqual(configuration["mounting"]["joint_signs"], [-1, -1])
        self.assertEqual(configuration["joint_signs"], [-1, -1])
        self.assertEqual(configuration["mapping_version"], "coordinated-angle-v4")
        self.assertEqual(self.status()["control"]["follow"]["version"], "coordinated-angle-v4")
        self.assertEqual(self.status()["control"]["follow"]["joint_signs"], [-1, -1])
        metadata = self.on_server(self.runtime.metadata)
        self.assertEqual(metadata["config"], configuration["config"])
        self.assertEqual(metadata["profile_hash"], HASH)
        self.assertEqual(metadata["controller_version"], "coordinated-angle-v4")
        self.assertEqual(metadata["joint_signs"], [-1, -1])
        self.assertIsNone(self.status()["joint_state"]["measured"])

    def test_virtual_and_no_power_mapping_defaults_stay_v3_with_original_signs(self):
        for executor in ("robot_powered", "robot_no_power", "virtual"):
            self.assertEqual(self.client.post("/api/executor/select", json={"kind": executor}).status_code, 200)
            configuration = self.client.get("/api/executor/config").json()
            self.assertEqual(configuration["mapping_version"], "coordinated-angle-v4" if executor == "robot_powered" else "coordinated-angle-v3")
            self.assertEqual(configuration["mounting"]["joint_signs"], [-1, -1] if executor == "robot_powered" else [1, -1])
        self.assertEqual(self.on_server(self.runtime.metadata)["controller_version"], "coordinated-angle-v3")
        self.assertEqual(self.status()["control"]["follow"]["version"], "coordinated-angle-v3")

    def test_diagnostics_accept_only_approved_discrete_steps_including_zero(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        for axis, allowed in enumerate(([-15, -10, -5, 0, 5, 10, 15], [-10, -5, 0, 5, 10], [-10, -5, 0, 5, 10])):
            for offset in allowed:
                response = self.action("test-joint", f"axis-{axis}-{offset}", axis=axis, offset_deg=offset)
                self.assertEqual(response.status_code, 200, response.text)
        accepted = len(self.link.calls)
        for axis, offset in ((0, -20), (0, 20), (0, 3), (0, 15.01), (1, -15), (2, 15), (1, 3), (2, True)):
            response = self.action("test-joint", f"rejected-{axis}-{offset}", axis=axis, offset_deg=offset)
            self.assertEqual(response.status_code, 409, response.text)
        self.assertEqual(len(self.link.calls), accepted)

    def test_direct_observation_is_explicit_idempotent_and_does_not_mark_axes_passed(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.assertFalse(self.status()["control"]["observation_ready"])
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        self.assertTrue(self.status()["control"]["observation_ready"])
        self.assertFalse(self.status()["control"]["can_enable"])
        self.assertEqual(self.action("initialize", "ordinary").status_code, 409)
        for invalid in ({"note": ""}, {"note": "x"*501}, {"note": 1}, {"direct_observation": 1, "note": "observe"}):
            arguments = {"direct_observation": True, **invalid}
            self.assertEqual(self.action("initialize", "bad-note", **arguments).status_code, 409)
        note = "用户明确选择省略剩余分档，直接跟随观察；未测轴保持未测。"
        response = self.action("initialize", "direct", direct_observation=True, note=note)
        self.assertEqual(response.status_code, 200, response.text)
        repeated = self.action("initialize", "direct", direct_observation=True, note=note)
        self.assertEqual(repeated.json(), response.json())
        self.assertEqual(self.action("initialize", "direct").status_code, 409)
        self.assertEqual(self.action("initialize", "direct", direct_observation=True, note="changed").status_code, 409)
        self.assertEqual(sum(kind == "init" for kind, _ in self.link.calls), 1)
        state = self.status()
        self.assertTrue(state["control"]["enabled"])
        self.assertFalse(state["control"]["armed"])
        self.assertEqual(state["executor"]["acceptance"], [])
        self.assertEqual(self.runtime.direct_observation["unaccepted_axes"], ["base", "shoulder", "elbow"])
        self.assertEqual(sum(e["code"] == "OPERATOR_DIRECT_OBSERVATION" for e in state["events"]), 1)
        self.assertEqual(self.on_server(self.runtime.metadata)["direct_observation"]["note"], note)
        previous_token = self.runtime.manager_token
        self.client.post("/api/control/stop")
        self.assertIsNone(self.runtime.direct_observation)
        self.assertIsNone(self.runtime.manager_token)
        self.assertFalse(self.status()["control"]["observation_ready"])
        self.assertEqual(self.client.post("/api/control/heartbeat", json={"manager_token": previous_token}).status_code, 409)
        self.assertEqual(self.action("initialize", "direct", direct_observation=True, note=note).status_code, 409)

    def test_direct_observation_preserves_other_initialization_gates(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        original = deepcopy(self.runtime.robot_parts)
        changes = (("output", "prepared", False), ("output", "stage", "unknown"),
                   ("output", "diagnostic_active", True), ("control", "can_enable", False),
                   ("control", "fault", "PCA_OUTPUT_UNKNOWN"), ("health", "pca_readable", False))
        before = len(self.link.calls)
        for section, key, value in changes:
            changed = deepcopy(original)
            changed[section][key] = value
            self.on_server(lambda: setattr(self.runtime, "robot_parts", changed))
            self.assertFalse(self.status()["control"]["observation_ready"], (section, key))
            result = self.action("initialize", "gated", direct_observation=True, note="direct observation requested")
            self.assertEqual(result.status_code, 409, result.text)
        self.on_server(lambda: setattr(self.runtime, "robot_parts", original))
        self.assertEqual(len(self.link.calls), before)
        self.assertTrue(self.status()["control"]["observation_ready"])
        self.clock.now += .75
        self.assertEqual(self.action("initialize", "expired", direct_observation=True, note="observe").status_code, 409)
        self.assertFalse(self.status()["control"]["observation_ready"])

    def test_direct_observation_note_and_event_are_recorded_with_unaccepted_axes(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        note = "按用户选择直接观察，并未验收大臂小臂。"
        self.assertEqual(self.action("initialize", "direct", direct_observation=True, note=note).status_code, 200)
        record = self.client.post("/api/recordings/start", json={"name": "direct observation"}).json()
        self.link.number += 1
        self.on_server(lambda: self.runtime.robot_receive({"type": "snapshot", "parts": powered_parts(self.link.number, "holding", True, True)}))
        self.assertEqual(self.client.post("/api/recordings/stop").json()["status"], "completed")
        saved = self.client.get("/api/recordings/" + record["id"]).json()
        self.assertEqual(saved["metadata"]["direct_observation"]["note"], note)
        self.assertEqual(saved["metadata"]["acceptance"], [])
        events = [e for e in saved["events"] if e["code"] == "OPERATOR_DIRECT_OBSERVATION"]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["unaccepted_axes"], ["base", "shoulder", "elbow"])

    def test_automatic_record_gap_is_returned_by_stop_without_changing_interrupted_status(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        self.assertEqual(self.action("initialize", "direct", direct_observation=True, note="direct observation").status_code, 200)
        first = self.client.post("/api/recordings/start", json={"name": "prior completed"}).json()
        self.link.number += 1
        self.on_server(lambda: self.runtime.robot_receive({"type": "snapshot", "parts": powered_parts(self.link.number, "holding", True, True)}))
        self.assertEqual(self.client.post("/api/recordings/stop").json()["status"], "completed")
        record = self.client.post("/api/recordings/start", json={"name": "gap"}).json()
        self.link.number += 1
        self.on_server(lambda: self.runtime.robot_receive({"type": "snapshot", "parts": powered_parts(self.link.number, "holding", True, True)}))
        self.on_server(self.runtime.step)
        self.link.number += 2
        self.on_server(lambda: self.runtime.robot_receive({"type": "snapshot", "parts": powered_parts(self.link.number, "holding", True, True)}))
        self.on_server(self.runtime.step)
        self.assertIsNone(self.runtime.records.active)
        result = self.client.post("/api/recordings/stop").json()
        self.assertEqual(result["id"], record["id"])
        self.assertNotEqual(result["id"], first["id"])
        self.assertEqual(result["status"], "interrupted")
        self.assertIn("telemetry gap", result["note"])
        self.assertEqual(result["frame_count"], 2)
        self.assertEqual(self.client.post("/api/recordings/stop").json(), result)
        self.assertTrue(self.status()["control"]["enabled"])

    def test_historical_schema3_4_5_keep_their_saved_profile_and_limits(self):
        old_limits = [[-3, 3], [42, 48], [42, 48]]
        old_hash = "sha256:673bb8f6dd5b2478f5e5c0fe698a8b5c10cb1a957dcb2edbbfd2ec926604063c"
        for schema, executor in ((3, "virtual"), (4, "robot_no_power"), (5, "robot_powered")):
            record = {"status": "completed", "metadata": {"executor": executor, "schema_version": schema,
                      "angle_convention": CONVENTION, "output_locked": executor != "robot_powered",
                      "controller_version": "coordinated-angle-v3", "joint_signs": [1, -1],
                      "profile_hash": old_hash, "config": {"limits_deg": old_limits}},
                      "frames": [{"t_ms": stamp, "requested": pose([2, 44, 46], 1),
                                  "output_target": pose([2, 44, 46], 1)} for stamp in (0, 200)]}
            original = deepcopy(record)
            self.assertEqual(self.runtime.check_record(record), [], schema)
            self.assertEqual(record, original)
            record["frames"][1]["output_target"]["base_yaw_deg"] = 4
            self.assertTrue(any("限位" in reason for reason in self.runtime.check_record(record)), schema)

    def test_pending_prepare_ack_cannot_restore_manager_after_stop(self):
        self.setup_robot()
        async def scenario():
            entered, release = asyncio.Event(), asyncio.Event()
            link = self.runtime.robot_link
            async def request(kind, **body):
                if kind == "prepare":
                    entered.set()
                    await release.wait()
                return {"type": "ack"}
            link.request = request
            task = asyncio.create_task(self.runtime.powered_action("prepare", {**self.binding, "request_id": "pending"}))
            await entered.wait()
            await self.runtime.stop_control()
            release.set()
            with self.assertRaises(ValueError):
                await task
        self.on_server(scenario)
        self.assertIsNone(self.runtime.manager_token)
        self.assertFalse(self.status()["control"]["enabled"])

    def test_profile_mismatch_and_uncommitted_targets_are_unavailable(self):
        self.setup_robot()
        self.assertEqual(self.client.get("/api/virtual/config").status_code, 409)
        self.assertEqual(self.client.post("/api/virtual/stop").status_code, 409)
        self.assertIsNone(self.status()["joint_state"]["output_target"])
        self.binding["profile_hash"] = "sha256:" + "b" * 64
        self.assertEqual(self.action("prepare", "bad-profile").status_code, 409)
        self.assertEqual(self.link.calls, [])

    def test_uncertain_rpc_revokes_browser_renewal_and_requests_stop_without_pwm_release(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        previous_token = self.runtime.manager_token
        request = self.link.request
        async def lose_ack(kind, **body):
            answer = await request(kind, **body)
            if kind == "enable_output":
                raise TimeoutError("ACK lost after possible write")
            return answer
        self.link.request = lose_ack
        result = self.action("enable-output", "uncertain-enable", supported=True, power_confirmed=True, startup_aligned=True)
        self.assertEqual(result.status_code, 409)
        self.assertIn("结果未知", result.json()["detail"]["message"])
        self.assertIsNone(self.runtime.manager_token)
        self.assertEqual([kind for kind, _ in self.link.calls], ["prepare", "enable_output", "stop"])
        self.assertEqual(self.status()["output"]["stage"], "holding")
        self.assertFalse(self.status()["output"]["prepared"])
        self.assertFalse(self.status()["control"]["enabled"])
        self.assertEqual(self.client.post("/api/control/heartbeat", json={"manager_token": previous_token}).status_code, 409)

    def test_supported_disable_is_available_after_manager_and_wireless_loss(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        self.client.post("/api/control/stop")
        self.assertIsNone(self.runtime.manager_token)
        self.assertEqual(self.action("disable-output", "disable", supported=True).status_code, 200)
        self.assertFalse(self.status()["output"]["pwm_enabled"])
        self.assertIsNone(self.runtime.manager_token)

    def test_stale_or_disconnected_holding_has_only_historical_target(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        self.clock.now += .5
        stale = self.status()
        self.assertEqual(stale["output"]["stage"], "unknown")
        self.assertIsNone(stale["output"]["pwm_enabled"])
        self.assertIsNone(stale["joint_state"]["output_target"])
        self.assertEqual(stale["joint_state"]["last_committed_target"], pose([0, 45, 45], 1))
        self.assertIsNone(stale["gripper"]["output_open_ratio"])
        self.on_server(lambda: self.runtime.robot_receive({"type": "offline", "detail": "USB lost"}))
        self.assertEqual(self.status()["output"]["stage"], "unknown")
        self.assertIsNone(self.runtime.manager_token)

    def test_wrong_profile_hello_or_snapshot_revokes_without_callback_exception(self):
        self.setup_robot()
        self.on_server(lambda: self.runtime.robot_receive({"type": "hello_ack", "boot_id": "robot-test", "mac": ROBOT_MAC,
            "profile_hash": "sha256:" + "b"*64, "protocol_version": 3, "mode": "robot_powered", "output_locked": True}))
        self.assertEqual(self.status()["executor"]["status"], "incompatible")
        self.assertIsNone(self.runtime.manager_token)
        self.assertFalse(self.status()["control"]["can_enable"])

    def setup_robot(self):
        self.assertEqual(self.client.post("/api/executor/select", json={"kind": "robot_powered"}).status_code, 200)
        self.on_server(lambda: self.runtime.robot_receive({"type": "hello_ack", "boot_id": "robot-test", "mac": ROBOT_MAC,
            "profile_hash": HASH, "protocol_version": 3, "mode": "robot_powered", "output_locked": True}))
        runtime = self.runtime
        class Link:
            calls = []
            number = 1
            stage = "locked"
            prepared = False
            enabled = False
            def renew(self): pass
            def disarm(self): pass
            async def close(self): pass
            async def request(self, kind, **body):
                self.calls.append((kind, body))
                if kind == "prepare": self.prepared = True
                if kind == "enable_output": self.stage = "holding"
                if kind == "disable_output": self.stage = "disabled"
                if kind == "init": self.enabled = True
                if kind == "stop": self.prepared = self.enabled = False
                self.number += 1
                runtime.robot_receive({"type": "snapshot", "parts": powered_parts(self.number, self.stage, self.prepared, self.enabled)})
                return {"type": "ack"}
        self.link = Link()
        runtime.robot_link = self.link
        self.on_server(lambda: runtime.robot_receive({"type": "snapshot", "parts": powered_parts()}))
        self.binding = {"boot_id": "robot-test", "profile_hash": HASH}

    def action(self, action, identifier, **body):
        token = {"manager_token": self.runtime.manager_token} if self.runtime.manager_token else {}
        return self.client.post("/api/executor/" + action, json={**self.binding, **token, "request_id": identifier, **body})

    def test_prepare_enable_requires_explicit_operator_confirmation_and_is_idempotent(self):
        self.setup_robot()
        self.assertEqual(self.action("enable-output", "blocked").status_code, 409)
        prepare = self.action("prepare", "prepare-1")
        self.assertEqual(prepare.status_code, 200, prepare.text)
        self.assertFalse(self.status()["control"]["enabled"])
        self.assertTrue(self.status()["executor"]["output_locked"])
        self.assertEqual(self.action("enable-output", "missing-confirmation").status_code, 409)
        enabled = self.action("enable-output", "enable-1", supported=True, power_confirmed=True, startup_aligned=True)
        self.assertEqual(enabled.status_code, 200, enabled.text)
        self.assertEqual(self.action("enable-output", "enable-1", supported=True, power_confirmed=True, startup_aligned=True).json(), enabled.json())
        self.assertEqual([c[0] for c in self.link.calls].count("enable_output"), 1)
        state = self.status()
        self.assertFalse(state["executor"]["output_locked"])
        self.assertFalse(state["control"]["enabled"])
        self.assertIsNone(state["joint_state"]["measured"])
        self.assertEqual(self.action("test-joint", "outside", axis=0, offset_deg=3.01).status_code, 409)
        self.assertEqual(self.action("test-joint", "small", axis=1, offset_deg=-5).status_code, 200)
        self.assertEqual(self.action("disable-output", "without-support").status_code, 409)
        self.assertEqual(self.action("disable-output", "supported", supported=True).status_code, 200)

    def test_boot_hash_lease_and_observations_gate_follow_and_stop_keeps_pwm(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        token = self.runtime.manager_token
        body = {**self.binding, "manager_token": token, "request_id": "init"}
        self.assertEqual(self.client.post("/api/executor/initialize", json=body).status_code, 409)
        for axis in ("base", "shoulder", "elbow"):
            result = self.client.post("/api/executor/acceptance", json={**body, "axis": axis, "passed": True, "note": "现场观察通过"})
            self.assertEqual(result.status_code, 200, result.text)
        self.assertEqual(self.client.post("/api/executor/initialize", json={**body, "boot_id": "old"}).status_code, 409)
        self.assertEqual(self.client.post("/api/executor/initialize", json=body).status_code, 200)
        self.client.post("/api/control/stop")
        state = self.status()
        self.assertFalse(state["control"]["enabled"])
        self.assertIsNone(state["control"]["follow"]["reference_quaternion"])
        self.assertEqual(state["output"]["stage"], "holding")
        self.assertFalse(state["executor"]["output_locked"])
        self.assertIsNone(self.runtime.manager_token)
        self.assertEqual(self.client.post("/api/executor/initialize", json=body).status_code, 409)
        self.assertEqual(self.client.post("/api/executor/select", json={"kind": "virtual"}).status_code, 409)

    def test_schema5_record_exports_pwm_and_only_replays_disconnected_virtual(self):
        self.setup_robot()
        self.action("prepare", "prepare")
        self.action("enable-output", "enable", supported=True, power_confirmed=True, startup_aligned=True)
        for axis in ("base", "shoulder", "elbow"):
            self.client.post("/api/executor/acceptance", json={**self.binding, "manager_token": self.runtime.manager_token,
                              "axis": axis, "passed": True, "note": "observed"})
        self.assertEqual(self.action("initialize", "init").status_code, 200)
        record = self.client.post("/api/recordings/start", json={"name": "powered trial"}).json()
        self.link.number += 1
        self.on_server(lambda: self.runtime.robot_receive({"type": "snapshot", "parts": powered_parts(self.link.number, "holding", True, True)}))
        self.assertEqual(self.client.post("/api/recordings/stop").json()["status"], "completed")
        original = self.client.get("/api/recordings/" + record["id"]).json()
        self.assertEqual(original["metadata"]["schema_version"], 5)
        self.assertEqual(original["metadata"]["profile_hash"], HASH)
        rows = list(csv.DictReader(io.StringIO(self.client.get("/api/recordings/" + record["id"] + "/export").text.lstrip("\ufeff"))))
        self.assertEqual(rows[0]["pwm_base"], "330")
        self.client.post("/api/control/stop")
        self.action("prepare", "prepare2")
        self.action("disable-output", "disable", supported=True)
        self.assertEqual(self.client.post("/api/executor/select", json={"kind": "virtual"}).status_code, 200)
        calls = len(self.link.calls)
        preflight = self.client.post("/api/replays/preflight", json={"record_id": record["id"]}).json()
        self.assertTrue(preflight["ok"], preflight)
        self.assertEqual(self.client.post("/api/replays/start", json={"preflight_id": preflight["preflight_id"]}).status_code, 200)
        self.assertEqual(len(self.link.calls), calls)
        self.assertEqual(self.client.get("/api/recordings/" + record["id"]).json(), original)


if __name__ == "__main__":
    unittest.main()
