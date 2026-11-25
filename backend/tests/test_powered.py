"""Powered USB authority, operator gates and history boundaries without hardware."""
from copy import deepcopy
import asyncio
import csv
import io
import json
import unittest

from app.robot import SnapshotAssembler, RobotLink, decode, ROBOT_MAC, trial_profile
from app.virtual import pose
from test_executor import robot_parts
import test_virtual_api as api_helpers

HASH = trial_profile()[1]


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
        "trial_limits_deg": [[-3, 3], [42, 48], [42, 48]], "max_velocity_deg_s": 2, "max_acceleration_deg_s2": 5}
    return parts


class PoweredProtocolTests(unittest.TestCase):
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
        self.assertEqual(self.action("test-joint", "small", axis=1, offset_deg=-3).status_code, 200)
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
