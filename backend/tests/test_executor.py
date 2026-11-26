"""No-hardware protection, USB snapshot isolation and browser management leases."""
from copy import deepcopy
import csv
import json
from pathlib import Path
import unittest

from app.robot import ROBOT_MAC, RobotLink, SnapshotAssembler, decode
from app.virtual import VirtualController, pose
from test_virtual import Clock
import test_virtual_api as api_helpers
from test_wearable import frame


def robot_parts(snapshot=1, boot="robot-test", enabled=False):
    common = {"v": 2, "boot_id": boot, "snapshot_id": snapshot, "t_ms": snapshot * 200}
    f = frame()
    imu = {k: v for k, v in f.items() if k not in ("v", "att", "role", "boot", "seq")}
    imu.update(wearable_boot=f["boot"], wearable_seq=f["seq"])
    items = {
        "control": {"enabled": enabled, "armed": False, "owner": "WEARABLE" if enabled else "NONE",
                    "reference_quaternion": [1., 0., 0., 0.] if enabled else None,
                    "delta_rotation_vector_deg": [0., 0., 0.] if enabled else None,
                    "angle_ambiguous": False, "can_enable": not enabled, "fault": None, "reason": "READY"},
        "imu": imu,
        "targets": {"requested": pose([12, 34, 34], 1), "output_target": pose([11, 35, 35], 1),
                    "measured": None, "velocity_deg_s": [0., 0., 0.], "limits": [False]*3},
        "gripper": {"state": "OPEN_READY", "limit_ms": 5000, "elapsed_ms": 0, "remaining_ms": 0,
                    "cycle_id": 0, "auto_open_reason": None, "reclose_blocked": False},
        "health": {"mode": "robot_no_power", "output_locked": True, "mac": ROBOT_MAC,
                   "wearable_mac": "EC:E3:34:EF:81:64", "pca_readable": True, "manager_active": enabled,
                   "wireless": {"received": snapshot, "rejected": 0, "duplicates": 0, "out_of_order": 0, "status": "valid"}},
    }
    return {kind: {**common, "seq": snapshot*5 + i, "type": kind, **values}
            for i, (kind, values) in enumerate(items.items())}


class RobotSnapshotTests(unittest.TestCase):
    def test_worker_handshakes_without_reacquiring_and_publishes_complete_batch(self):
        messages = []
        link = RobotLink(messages.append, "TEST")
        hello = {"v": 2, "type": "hello_ack", "seq": 1, "role": "robot", "protocol_version": 2,
                 "boot_id": "robot-test", "session_id": "session-test", "mac": ROBOT_MAC,
                 "mode": "robot_no_power", "output_locked": True}
        packets = [hello, *robot_parts().values()]
        class Port:
            in_waiting = 512
            def __init__(self):
                self.data = bytearray(b''.join(json.dumps(p, separators=(',', ':')).encode() + b'\n' for p in packets))
                self.writes = []
            def write(self, raw):
                self.writes.append(json.loads(raw))
                return len(raw)
            def read(self, count):
                if not self.data:
                    link.halt.set()
                    return b''
                value = bytes(self.data[:count])
                del self.data[:count]
                return value
        port = Port()
        link.renew()  # A preexisting lease must be revoked by HELLO.
        link._listen(port, messages.append)
        self.assertEqual([p['type'] for p in port.writes], ['hello'])
        self.assertEqual([p['type'] for p in messages], ['hello_ack', 'snapshot'])
        self.assertEqual(link.heartbeat_until, 0)

    def test_publish_only_complete_same_boot_time_and_snapshot(self):
        assembler = SnapshotAssembler()
        items = robot_parts()
        for kind in ("control", "imu", "targets", "gripper"):
            self.assertIsNone(assembler.feed(items[kind]))
        newer = robot_parts(2)
        self.assertIsNone(assembler.feed(newer["health"]))
        for kind in ("control", "imu", "targets"):
            self.assertIsNone(assembler.feed(newer[kind]))
        self.assertEqual(assembler.feed(newer["gripper"])["targets"]["snapshot_id"], 2)
        self.assertIsNone(assembler.feed(items["health"]))

    def test_reboot_duplicates_expiry_and_memory_budget(self):
        clock = Clock()
        assembler = SnapshotAssembler(clock)
        parts = robot_parts()
        self.assertIsNone(assembler.feed(parts["control"], 450))
        self.assertIsNone(assembler.feed(parts["control"], 450))
        self.assertEqual(assembler.bytes, 450)
        clock.now += .51
        self.assertIsNone(assembler.feed(parts["imu"], 450))
        parts = robot_parts(0, "new-boot")
        result = None
        for item in parts.values():
            result = assembler.feed(item, 399)
        self.assertIsNotNone(result)
        assembler = SnapshotAssembler()
        with self.assertRaises(ValueError):
            for item in robot_parts().values():
                assembler.feed(item, 410)

    def test_reject_powered_unlocked_feedback_and_legacy_frames(self):
        for mutate in (lambda p: p["health"].update(output_locked=False),
                       lambda p: p["health"].update(mode="robot_powered"),
                       lambda p: p["health"].update(mac="00:00:00:00:00:01"),
                       lambda p: p["targets"].update(measured=pose([1, 2, 3], 1)),
                       lambda p: p["gripper"].update(limit_ms=700)):
            parts = robot_parts()
            mutate(parts)
            with self.assertRaises(ValueError):
                SnapshotAssembler.validate(parts)
        with self.assertRaises(ValueError):
            decode(b'{"v":1,"type":"state","seq":1,"boot_id":"old"}\n')
        with self.assertRaises(ValueError):
            decode(b'x'*513 + b'\n')


class GripperProtectionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.c = VirtualController(self.clock)
        self.f = frame()
        for _ in range(36):
            self.advance()
        self.c.enable()

    def advance(self, dt=.02, **changes):
        self.clock.now += dt
        self.f = {**self.f, "seq": (self.f["seq"]+1) & 0xffffffff, **changes}
        self.c.receive(self.f)
        self.c.step()

    def click(self, delta=1):
        self.advance(bc=(self.f["bc"]+delta) & 0xffffffff, b=1)

    def test_five_seconds_includes_closing_and_pause_does_not_renew(self):
        self.click()
        start = self.c.gripper_started
        for index in range(249):
            self.advance(b=1, ac=index//20)
        self.assertEqual(self.c.open_req, 0)
        self.assertEqual(self.c.gripper_started, start)
        self.assertLessEqual(self.c.gripper_status()["remaining_ms"], 20)
        self.advance()
        self.assertEqual(self.c.open_req, 1)
        self.assertEqual(self.c.gripper_reason, "TIME_LIMIT")
        self.assertEqual(self.c.gripper_cycle, 1)

    def test_opening_consumes_clicks_and_requires_a_new_released_click(self):
        self.click()
        for _ in range(5):
            self.advance()
        self.click()  # Early opening; output has not reached open yet.
        self.click(2)
        self.assertEqual(self.c.gripper_cycle, 1)
        for _ in range(12):
            self.advance(b=1)
        self.click()  # New count while held is consumed until release observed.
        self.assertEqual(self.c.open_req, 1)
        self.advance(b=0)
        self.click()
        self.assertEqual(self.c.open_req, 0)
        self.assertEqual(self.c.gripper_cycle, 2)

    def test_batched_clicks_cannot_reclose_before_open_target(self):
        self.click(3)
        self.assertEqual(self.c.open_req, 1)
        self.assertEqual(self.c.gripper_cycle, 0)
        self.assertEqual(self.c.events[-1]["code"], "BUTTON_EVENTS_COALESCED")

    def test_initialization_does_not_reset_an_active_or_releasing_cycle(self):
        self.click()
        with self.assertRaises(ValueError):
            self.c.enable()
        for _ in range(8):
            self.advance()
        self.c.stop()
        with self.assertRaises(ValueError):
            self.c.enable()
        self.assertEqual(self.c.gripper_cycle, 1)
        for _ in range(36):
            self.advance(b=0)
        self.c.enable()
        self.assertEqual(self.c.open_out, 1)
        self.assertEqual(self.c.gripper_cycle, 1)

    def test_duplicate_input_stop_and_fault_cannot_extend_closing(self):
        for reason in ("stop", "invalid", "stale"):
            self.setUp()
            self.click()
            for _ in range(12):
                self.advance()
            before = self.c.q_out[:]
            duplicate = deepcopy(self.f)
            self.clock.now += .05
            self.assertFalse(self.c.receive(duplicate))
            if reason == "stop":
                self.c.stop()
            elif reason == "invalid":
                self.advance(valid=0)
            else:
                self.clock.now += .25
                self.c.step()
            self.assertEqual(self.c.open_req, 1)
            for _ in range(12):
                self.advance(valid=0)
            self.assertEqual(self.c.open_out, 1)
            self.assertEqual(self.c.q_out, before)


class ExecutorApiTests(unittest.TestCase):
    setUp = api_helpers.VirtualApiTests.setUp
    tearDown = api_helpers.VirtualApiTests.tearDown
    on_server = api_helpers.VirtualApiTests.on_server
    status = api_helpers.VirtualApiTests.status
    inject = api_helpers.VirtualApiTests.inject
    enable = api_helpers.VirtualApiTests.enable

    def test_robot_initialization_nack_preserves_device_reason_and_no_authority(self):
        self.client.post('/api/executor/select', json={'kind': 'robot_no_power'})
        self.on_server(lambda: self.runtime.robot_receive({'type': 'hello_ack', 'boot_id': 'robot-test', 'mac': ROBOT_MAC}))
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': robot_parts()}))
        class Link:
            def disarm(self):
                pass
            async def close(self):
                pass
            async def request(self, kind, **payload):
                return {'type': 'nack', 'error': 'INPUT_NOT_STABLE'}
        self.runtime.robot_link = Link()
        response = self.client.post('/api/executor/initialize', json={'request_id': 'rejected-init'})
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json()['detail']['message'], 'INPUT_NOT_STABLE')
        self.assertIsNone(self.runtime.manager_token)
        self.assertFalse(self.status()['control']['enabled'])

    def begin_robot_record(self, ev_drop=0, event_seq=None):
        self.client.post('/api/executor', json={'kind': 'robot_no_power'})
        self.on_server(lambda: self.runtime.robot_receive({'type': 'hello_ack', 'boot_id': 'robot-test', 'mac': ROBOT_MAC}))
        initial = robot_parts(enabled=True)
        initial['health']['ev_drop'] = ev_drop
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': initial}))
        if event_seq is not None:
            self.on_server(lambda: self.runtime.robot_receive({'type': 'event', 'boot_id': 'robot-test', 'event_seq': event_seq,
                                                             't_ms': 190, 'code': 'INITIALIZE'}))
        self.on_server(self.runtime.start_manager)
        return self.client.post('/api/recordings/start', json={'name': 'event audit'}).json()['id']

    def test_historical_event_drop_baseline_is_allowed_but_new_drops_interrupt(self):
        identifier = self.begin_robot_record(ev_drop=7, event_seq=4)
        metadata = self.client.get('/api/recordings/' + identifier).json()['metadata']
        self.assertEqual(metadata['event_baseline']['ev_drop'], 7)
        second = robot_parts(2, enabled=True)
        second['health']['ev_drop'] = 7
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': second}))
        self.on_server(self.runtime.step)
        self.assertEqual(self.runtime.records.active, identifier)
        third = robot_parts(3, enabled=True)
        third['health']['ev_drop'] = 8
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': third}))
        detail = self.client.get('/api/recordings/' + identifier).json()
        self.assertEqual(detail['status'], 'interrupted')
        self.assertEqual(detail['frames'][-1]['health']['ev_drop'], 8)
        self.assertIn('EVENT_STREAM_INCOMPLETE', [e['code'] for e in detail['events']])
        self.assertEqual(self.client.post('/api/recordings/stop').json()['status'], 'interrupted')

    def test_event_sequence_wrap_and_duplicates_are_valid_but_a_gap_interrupts(self):
        identifier = self.begin_robot_record(event_seq=0xfffffffe)
        for sequence, stamp in ((0xffffffff, 250), (0, 300), (0, 300)):
            self.on_server(lambda: self.runtime.robot_receive({'type': 'event', 'boot_id': 'robot-test', 'event_seq': sequence,
                                                             't_ms': stamp, 'code': 'GRIPPER_OPEN'}))
        self.assertEqual(self.runtime.records.active, identifier)
        self.on_server(lambda: self.runtime.robot_receive({'type': 'event', 'boot_id': 'robot-test', 'event_seq': 2,
                                                         't_ms': 330, 'code': 'GRIPPER_AUTO_OPEN'}))
        detail = self.client.get('/api/recordings/' + identifier).json()
        self.assertEqual(detail['status'], 'interrupted')
        audit = next(e for e in detail['events'] if e['code'] == 'EVENT_STREAM_INCOMPLETE')
        self.assertEqual(audit['missing_events'], 1)
        self.assertEqual(len([e for e in detail['events'] if e.get('event_seq') == 0]), 1)

    def test_known_historical_event_backlog_does_not_interrupt_a_new_recording(self):
        identifier = self.begin_robot_record(ev_drop=3, event_seq=1)
        self.on_server(lambda: self.runtime.robot_receive({'type': 'event', 'boot_id': 'robot-test', 'event_seq': 4,
                                                         't_ms': 195, 'code': 'OLD_EVENT'}))
        self.assertEqual(self.runtime.records.active, identifier)
        self.on_server(lambda: self.runtime.robot_receive({'type': 'event', 'boot_id': 'robot-test', 'event_seq': 5,
                                                         't_ms': 250, 'code': 'NEW_EVENT'}))
        second = robot_parts(2, enabled=True)
        second['health']['ev_drop'] = 3
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': second}))
        saved = self.client.post('/api/recordings/stop').json()
        self.assertEqual(saved['status'], 'completed')

    def test_controller_disable_or_fault_interrupts_despite_valid_imu(self):
        for fault in (None, 'PCA_LOCK_OR_I2C_FAULT'):
            identifier = self.begin_robot_record()
            final = robot_parts(2)
            final['control'].update(fault=fault, reason=fault or 'MANAGER_TIMEOUT')
            final['targets']['output_target']['gripper_open_ratio'] = .4
            final['gripper'].update(state='OPENING', auto_open_reason=fault or 'MANAGER_TIMEOUT')
            self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': final}))
            detail = self.client.get('/api/recordings/' + identifier).json()
            self.assertEqual(detail['status'], 'interrupted')
            self.assertEqual(detail['note'], fault or 'MANAGER_TIMEOUT')
            self.assertIsNone(self.runtime.manager_token)
            self.assertTrue(detail['frames'][-1]['imu']['valid'])
            self.assertEqual(detail['frames'][-1]['gripper']['state'], 'OPENING')
            self.assertEqual(detail['frames'][-1]['requested']['gripper_open_ratio'], 1)

    def test_normal_record_stop_flushes_a_new_snapshot_without_waiting_for_tick(self):
        identifier = self.begin_robot_record()
        second = robot_parts(2, enabled=True)
        second['targets']['output_target']['base_yaw_deg'] = 20
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': second}))
        self.assertEqual(self.runtime.records.frames, 2)
        saved = self.client.post('/api/recordings/stop').json()
        self.assertEqual(saved['status'], 'completed')
        detail = self.client.get('/api/recordings/' + identifier).json()
        self.assertEqual(detail['frame_count'], 2)
        self.assertEqual(detail['frames'][-1]['output_target']['base_yaw_deg'], 20)

    def test_complete_snapshots_are_saved_before_later_callbacks_replace_latest_cache(self):
        identifier = self.begin_robot_record()
        def deliver_back_to_back():
            second = robot_parts(2, enabled=True)
            third = robot_parts(3, enabled=True)
            second['targets']['output_target']['base_yaw_deg'] = 17
            third['targets']['output_target']['base_yaw_deg'] = 23
            self.runtime.robot_receive({'type': 'snapshot', 'parts': second})
            self.runtime.robot_receive({'type': 'snapshot', 'parts': third})
        self.on_server(deliver_back_to_back)
        self.assertEqual(self.runtime.records.frames, 3)
        self.on_server(self.runtime.step)
        self.assertEqual(self.runtime.records.frames, 3)
        saved = self.client.post('/api/recordings/stop').json()
        self.assertEqual(saved['status'], 'completed')
        detail = self.client.get('/api/recordings/' + identifier).json()
        self.assertEqual([f['t_ms'] for f in detail['frames']], [0, 200, 400])
        self.assertEqual([f['output_target']['base_yaw_deg'] for f in detail['frames']], [11, 17, 23])

    def test_no_power_record_replays_saved_targets_only_after_explicit_virtual_selection(self):
        identifier = self.begin_robot_record()
        second = robot_parts(2, enabled=True)
        second['targets']['output_target']['base_yaw_deg'] = 20
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': second}))
        self.assertEqual(self.client.post('/api/recordings/stop').json()['status'], 'completed')
        original = self.client.get('/api/recordings/' + identifier).json()
        rejected = self.client.post('/api/replays/preflight', json={'record_id': identifier}).json()
        self.assertFalse(rejected['ok'])
        self.assertIn('明确切换为虚拟执行端', '；'.join(rejected['violations']))
        self.assertEqual(self.runtime.executor_kind, 'robot_no_power')
        stopped = robot_parts(3)
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': stopped}))
        runtime = self.runtime
        class Link:
            calls = []
            def disarm(self):
                pass
            async def close(self):
                pass
            async def request(self, kind, **payload):
                self.calls.append(kind)
                raise AssertionError('Replay must never send a robot command')
        link = Link()
        self.runtime.robot_link = link
        self.assertEqual(self.client.post('/api/executor/select', json={'kind': 'virtual'}).status_code, 200)
        report = self.client.post('/api/replays/preflight', json={'record_id': identifier}).json()
        self.assertTrue(report['ok'], report)
        self.assertEqual(self.client.post('/api/replays/start', json={'preflight_id': report['preflight_id']}).status_code, 200)
        self.assertEqual(self.status()['executor']['kind'], 'virtual')
        self.assertEqual(self.status()['control']['owner'], 'REPLAY')
        self.clock.now += 1
        self.on_server(self.runtime.step)
        self.assertEqual(self.status()['joint_state']['output_target']['base_yaw_deg'], 20)
        self.assertEqual(link.calls, [])
        self.assertEqual(self.client.get('/api/recordings/' + identifier).json(), original)

    def test_physical_record_requires_current_schema_and_output_lock_to_replay(self):
        identifier = self.begin_robot_record()
        second = robot_parts(2, enabled=True)
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': second}))
        self.client.post('/api/recordings/stop')
        original = self.client.get('/api/recordings/' + identifier).json()
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': robot_parts(3)}))
        self.client.post('/api/executor/select', json={'kind': 'virtual'})
        for changes in ({'schema_version': 3}, {'output_locked': False}, {'executor': 'robot_powered'}, {'angle_convention': 'unknown'}):
            record = deepcopy(original)
            record['metadata'].update(changes)
            self.assertTrue(self.runtime.check_record(record), changes)

    def test_wireless_freshness_and_buttons_are_independent_of_imu_validity(self):
        self.client.post('/api/executor', json={'kind': 'robot_no_power'})
        self.on_server(lambda: self.runtime.robot_receive({'type': 'hello_ack', 'boot_id': 'robot-test', 'mac': ROBOT_MAC}))
        for index, (changes, expected, fresh) in enumerate((({'cal': 100, 'valid': 0}, 'calibrating', True),
                                                           ({'valid': 0, 'rej': 4}, 'invalid', True),
                                                           ({'valid': 0, 'age_ms': 250}, 'stale', False))):
            parts = robot_parts(index + 1)
            parts['imu'].update(a=1, b=1, **changes)
            parts['health']['wireless']['status'] = 'stale'  # Firmware raw field conflates IMU validity.
            self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': parts}))
            state = self.status()
            self.assertEqual(state['input']['status'], expected)
            self.assertEqual(state['input']['wireless_fresh'], fresh)
            self.assertEqual(state['buttons']['a'], fresh)
            self.assertEqual(state['buttons']['b'], fresh)

    def test_manager_token_is_not_disclosed_by_reads_and_expires_closed(self):
        self.enable()
        token = self.runtime.manager_token
        self.assertIsNotNone(token)
        self.assertNotIn("manager_token", self.status())
        self.assertEqual(self.client.post("/api/control/heartbeat", json={"manager_token": "old"}).status_code, 409)
        self.inject(bc=1)
        self.clock.now += .75
        self.on_server(self.runtime.step)
        self.assertIsNone(self.runtime.manager_token)
        self.assertFalse(self.runtime.controller.enabled)
        self.assertEqual(self.runtime.controller.open_req, 1)
        self.assertEqual(self.client.post("/api/control/heartbeat", json={"manager_token": token}).status_code, 409)
        self.inject()
        self.assertFalse(self.runtime.controller.enabled)

    def test_initialize_is_idempotent_and_heartbeat_cannot_initialize(self):
        for _ in range(20):
            self.inject()
        first = self.client.post('/api/executor/enable', json={'request_id': 'init-1'}).json()
        reference = self.runtime.controller.reference_quaternion[:]
        again = self.client.post('/api/executor/initialize', json={'request_id': 'init-1'}).json()
        self.assertEqual(first['manager_token'], again['manager_token'])
        self.assertEqual(self.runtime.controller.reference_quaternion, reference)
        self.assertEqual(len([e for e in self.runtime.controller.events if e['code'] == 'INITIALIZE']), 1)
        self.client.post('/api/control/stop')
        self.assertEqual(self.client.post('/api/control/heartbeat', json={'manager_token': first['manager_token']}).status_code, 409)
        self.assertFalse(self.runtime.controller.enabled)

    def test_only_the_bound_management_websocket_disconnect_stops_control(self):
        self.enable()
        token = self.runtime.manager_token
        with self.client.websocket_connect('/ws/telemetry') as manager:
            manager.send_json({'type': 'heartbeat', 'manager_token': token})
            for _ in range(4):
                manager.receive_json()
                if self.runtime.manager_socket is not None:
                    break
            self.assertIsNotNone(self.runtime.manager_socket)
            with self.client.websocket_connect('/ws/telemetry') as observer:
                observer.receive_json()
            self.assertTrue(self.runtime.controller.enabled)
            self.assertEqual(self.runtime.manager_token, token)
        self.assertFalse(self.runtime.controller.enabled)
        self.assertIsNone(self.runtime.manager_token)

    def test_readonly_websocket_and_http_get_never_grant_management(self):
        with self.client.websocket_connect('/ws/telemetry') as observer:
            self.assertNotIn('manager_token', observer.receive_json())
        self.assertIsNone(self.runtime.manager_token)
        self.assertFalse(self.runtime.controller.enabled)

    def test_selection_has_no_virtual_target_fallback_or_power_unlock(self):
        self.assertEqual(self.client.post("/api/executor", json={"kind": "robot_powered"}).status_code, 200)
        self.assertEqual(self.client.post("/api/executor/enable-output", json={"request_id": "no-device"}).status_code, 409)
        self.assertEqual(self.client.post("/api/executor", json={"kind": "robot_no_power"}).status_code, 200)
        state = self.status()
        self.assertIsNone(state["joint_state"]["requested"])
        self.assertIsNone(state["joint_state"]["output_target"])
        self.assertTrue(state["executor"]["output_locked"])
        self.assertEqual(state["input"]["kind"], "esp_now")
        self.assertEqual(self.client.post("/api/executor/initialize", json={}).status_code, 409)
        self.assertEqual(self.client.post("/api/virtual/enable").status_code, 409)

    def test_robot_atomic_targets_and_historical_identity_are_authoritative(self):
        self.client.post("/api/executor", json={"kind": "robot_no_power"})
        self.on_server(lambda: self.runtime.robot_receive({"type": "hello_ack", "boot_id": "robot-test", "mac": ROBOT_MAC}))
        self.on_server(lambda: self.runtime.robot_receive({"type": "snapshot", "parts": robot_parts(enabled=True)}))
        state = self.status()
        self.assertEqual(state["joint_state"]["requested"]["base_yaw_deg"], 12)
        self.assertEqual(state["joint_state"]["output_target"]["base_yaw_deg"], 11)
        self.assertIsNone(state["joint_state"]["measured"])
        self.assertEqual(state["executor"]["identity"], ROBOT_MAC)
        self.assertIsNone(self.runtime.manager_token)
        self.clock.now += .3
        state = self.status()
        self.assertTrue(state["imu"]["valid"])
        self.assertEqual(state["input"]["age_ms"], 0)
        self.assertIsNone(state["input"]["transport_age_ms"])
        self.assertEqual(state["executor"]["age_ms"], 300)
        self.clock.now += .2
        self.assertFalse(self.status()["executor"]["connected"])
        self.assertFalse(self.status()["control"]["can_enable"])

    def test_invalid_robot_snapshot_saves_terminal_open_request_and_raw_event_clock(self):
        self.client.post('/api/executor/select', json={'kind': 'robot_no_power'})
        self.on_server(lambda: self.runtime.robot_receive({'type': 'hello_ack', 'boot_id': 'robot-test', 'mac': ROBOT_MAC}))
        first = robot_parts(enabled=True)
        first['targets']['requested']['gripper_open_ratio'] = 0
        first['targets']['output_target']['gripper_open_ratio'] = 0
        first['gripper'].update(state='CLOSED', cycle_id=1, elapsed_ms=400, remaining_ms=4600)
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': first}))
        self.on_server(self.runtime.start_manager)
        record = self.client.post('/api/recordings/start', json={'name': 'robot terminal'}).json()
        event = {'type': 'event', 'boot_id': 'robot-test', 'event_seq': 7, 't_ms': 330, 'code': 'GRIPPER_AUTO_OPEN'}
        self.on_server(lambda: self.runtime.robot_receive(event))
        final = robot_parts(2)
        final['imu']['valid'] = 0
        final['targets']['requested']['gripper_open_ratio'] = 1
        final['targets']['output_target']['gripper_open_ratio'] = .6
        final['gripper'].update(state='OPENING', cycle_id=1, auto_open_reason='INPUT_INVALID', reclose_blocked=True)
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': final}))
        detail = self.client.get('/api/recordings/' + record['id']).json()
        self.assertEqual(detail['status'], 'interrupted')
        self.assertEqual([f['t_ms'] for f in detail['frames']], [0, 200])
        self.assertEqual(detail['frames'][-1]['requested']['gripper_open_ratio'], 1)
        self.assertEqual(detail['frames'][-1]['gripper']['state'], 'OPENING')
        self.assertEqual(detail['frames'][-1]['robot_device_ms'], 400)
        self.assertEqual(detail['frames'][-1]['robot_t_ms'], 400)
        saved = next(e for e in detail['events'] if e['code'] == 'GRIPPER_AUTO_OPEN')
        self.assertEqual(saved['robot_device_ms'], 330)
        self.assertEqual(saved['robot_t_ms'], 330)
        self.assertEqual(saved['t_ms'], 130)
        self.assertGreater(saved['pc_received_ns'], 0)
        terminal = next(e for e in detail['events'] if e['code'] == 'RECORD_INTERRUPTED')
        self.assertTrue(terminal['final_state_confirmed'])
        self.assertIsNone(terminal['robot_t_ms'])

    def test_robot_recording_uses_monotonic_robot_time_across_millis_wrap(self):
        self.client.post('/api/executor', json={'kind': 'robot_no_power'})
        self.on_server(lambda: self.runtime.robot_receive({'type': 'hello_ack', 'boot_id': 'robot-test', 'mac': ROBOT_MAC}))
        initial = robot_parts(enabled=True)
        for item in initial.values():
            item['t_ms'] = 0xffffffff - 99
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': initial}))
        self.on_server(self.runtime.start_manager)
        record = self.client.post('/api/recordings/start', json={'name': 'wrap'}).json()
        second = robot_parts(2, enabled=True)
        for item in second.values():
            item['t_ms'] = 100
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': second}))
        self.on_server(self.runtime.step)
        saved = self.client.post('/api/recordings/stop').json()
        self.assertEqual(saved['status'], 'completed')
        detail = self.client.get('/api/recordings/' + record['id']).json()
        self.assertEqual(detail['frames'][1]['t_ms'], 200)
        self.assertEqual(detail['frames'][1]['robot_device_ms'], 100)
        self.assertEqual(detail['frames'][1]['robot_t_ms'], 2**32 + 100)

    def test_explicit_robot_stop_archives_the_subsequent_opening_snapshot(self):
        self.client.post('/api/executor', json={'kind': 'robot_no_power'})
        self.on_server(lambda: self.runtime.robot_receive({'type': 'hello_ack', 'boot_id': 'robot-test', 'mac': ROBOT_MAC}))
        initial = robot_parts(enabled=True)
        initial['targets']['requested']['gripper_open_ratio'] = 0
        initial['targets']['output_target']['gripper_open_ratio'] = 0
        initial['gripper'].update(state='CLOSED', cycle_id=1, remaining_ms=4500, elapsed_ms=500)
        self.on_server(lambda: self.runtime.robot_receive({'type': 'snapshot', 'parts': initial}))
        runtime = self.runtime
        class Link:
            snapshot = 1
            def renew(self):
                pass
            def disarm(self):
                pass
            async def close(self):
                pass
            async def request(self, kind):
                self.snapshot += 1
                final = robot_parts(self.snapshot)
                final['targets']['output_target']['gripper_open_ratio'] = .2
                final['gripper'].update(state='OPENING', cycle_id=1, auto_open_reason='SOFTWARE_STOP', reclose_blocked=True)
                runtime.robot_receive({'type': 'event', 'boot_id': 'robot-test', 'event_seq': self.snapshot,
                                       't_ms': self.snapshot*200 - 1, 'code': 'GRIPPER_AUTO_OPEN'})
                runtime.robot_receive({'type': 'snapshot', 'parts': final})
                return {'type': 'ack'}
        self.runtime.robot_link = Link()
        self.on_server(self.runtime.start_manager)
        record = self.client.post('/api/recordings/start', json={'name': 'stop'}).json()
        self.assertEqual(self.client.post('/api/control/stop').status_code, 200)
        detail = self.client.get('/api/recordings/' + record['id']).json()
        self.assertEqual(detail['frames'][-1]['requested']['gripper_open_ratio'], 1)
        self.assertEqual(detail['frames'][-1]['output_target']['gripper_open_ratio'], .2)
        self.assertEqual(detail['frames'][-1]['gripper']['auto_open_reason'], 'SOFTWARE_STOP')
        self.assertEqual(detail['frame_count'], 2)
        self.assertIn('GRIPPER_AUTO_OPEN', [event['code'] for event in detail['events']])

    def test_stop_release_event_is_saved_before_record_is_closed(self):
        self.enable()
        record = self.client.post("/api/recordings/start", json={"name": "release"}).json()
        self.inject(bc=1)
        self.inject()
        self.client.post("/api/control/stop")
        detail = self.client.get('/api/recordings/' + record['id']).json()
        self.assertEqual(detail["status"], "interrupted")
        self.assertIn("GRIPPER_AUTO_OPEN", [event["code"] for event in detail["events"]])
        self.assertIn("STOP", [event["code"] for event in detail["events"]])


class SharedControllerVectorTests(unittest.TestCase):
    def test_python_and_cpp_share_hand_constructed_quaternion_vectors(self):
        path = Path(__file__).resolve().parents[2] / "firmware" / "tests" / "control_vectors.csv"
        with path.open(encoding="utf-8") as source:
            for row in csv.DictReader(source):
                with self.subTest(name=row["name"]):
                    clock = Clock()
                    c = VirtualController(clock)
                    ref = [float(row["ref_" + axis]) for axis in "wxyz"]
                    for seq in range(36):
                        clock.now += .02
                        c.receive(frame(seq=seq, q=ref))
                        c.step()
                    c.enable()
                    clock.now += .02
                    c.receive(frame(seq=37, ac=1, q=[float(row["q_" + axis]) for axis in "wxyz"]))
                    for actual, key in zip(c.rotation_vector(), ("roll", "pitch", "z")):
                        self.assertAlmostEqual(actual, float(row[key]), places=5)
                    for actual, key in zip(c.q_req, ("base", "shoulder", "elbow")):
                        self.assertAlmostEqual(actual, float(row[key]), places=5)


if __name__ == "__main__":
    unittest.main()
