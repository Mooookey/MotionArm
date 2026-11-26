"""SQLite persistence for robot-clock control samples and associated events."""
import json
import sqlite3
import time
from pathlib import Path
from uuid import uuid4


class Recordings:
    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.row_factory = sqlite3.Row
        self.db.executescript("""
            PRAGMA journal_mode = WAL;
            PRAGMA foreign_keys = ON;
            CREATE TABLE IF NOT EXISTS recordings (
                id TEXT PRIMARY KEY, name TEXT NOT NULL, created_ms INTEGER NOT NULL,
                status TEXT NOT NULL, source TEXT NOT NULL, boot_id TEXT NOT NULL,
                calibration_id TEXT NOT NULL, workcell_id TEXT,
                protocol_version INTEGER NOT NULL, fw_version TEXT,
                frame_count INTEGER NOT NULL DEFAULT 0,
                duration_ms INTEGER NOT NULL DEFAULT 0, note TEXT
            );
            CREATE TABLE IF NOT EXISTS samples (
                recording_id TEXT NOT NULL REFERENCES recordings(id),
                sample_index INTEGER NOT NULL, payload TEXT NOT NULL,
                PRIMARY KEY (recording_id, sample_index)
            );
            CREATE TABLE IF NOT EXISTS events (
                recording_id TEXT NOT NULL REFERENCES recordings(id),
                event_index INTEGER NOT NULL, payload TEXT NOT NULL,
                PRIMARY KEY (recording_id, event_index)
            );
            CREATE TABLE IF NOT EXISTS replay_runs (
                id TEXT PRIMARY KEY, recording_id TEXT NOT NULL REFERENCES recordings(id),
                started_ms INTEGER NOT NULL, finished_ms INTEGER, status TEXT NOT NULL,
                reason TEXT, preflight TEXT NOT NULL
            );
            PRAGMA user_version = 1;
        """)
        # Additive migration: existing sample payloads keep their original semantics.
        if "metadata" not in {row[1] for row in self.db.execute("PRAGMA table_info(recordings)")}:
            self.db.execute("ALTER TABLE recordings ADD COLUMN metadata TEXT")
        self.db.execute("PRAGMA user_version = 2")
        with self.db:
            self.db.execute("UPDATE recordings SET status='interrupted', note='backend restart' WHERE status='recording'")
            self.db.execute("UPDATE replay_runs SET status='aborted', reason='backend restart' WHERE status='running'")
        self.active = None
        self.last_seq = None
        self.first_time = self.last_time = None
        self.frames = self.event_count = 0

    def start(self, name, source, boot_id, calibration_id, workcell_id, fw_version, metadata=None):
        identifier = uuid4().hex
        with self.db:
            self.db.execute("""
                INSERT INTO recordings (id, name, created_ms, status, source, boot_id,
                    calibration_id, workcell_id, protocol_version, fw_version)
                VALUES (?, ?, ?, 'recording', ?, ?, ?, ?, 1, ?)
            """, (identifier, name, int(time.time() * 1000), source, boot_id,
                  calibration_id, workcell_id, fw_version))
            if metadata is not None:
                self.db.execute("UPDATE recordings SET metadata=? WHERE id=?",
                                (json.dumps(metadata, allow_nan=False), identifier))
        self.active = identifier
        self.last_seq = None
        self.first_time = self.last_time = None
        self.frames = self.event_count = 0
        return self.get(identifier)

    def append(self, boot_id, calibration_id, t_ms, sample):
        if self.active is None:
            return
        meta = self.get(self.active)
        if meta["boot_id"] != boot_id or meta["calibration_id"] != calibration_id:
            return self.finish("interrupted", "device or calibration changed")
        if self.last_seq == sample["seq"]:
            return
        if self.last_time is not None and not 0 < t_ms - self.last_time <= 250:
            return self.finish("interrupted", "nonmonotonic robot time or telemetry gap")
        if self.first_time is None:
            self.first_time = t_ms
        duration = t_ms - self.first_time
        if duration > 600_000:
            return self.finish("interrupted", "recording time limit")
        payload = {**sample, "t_ms": duration,
                   "robot_t_ms": None if sample.get("clock_domain") == "backend_monotonic" else t_ms}
        with self.db:
            self.db.execute("INSERT INTO samples VALUES (?, ?, ?)",
                            (self.active, self.frames, json.dumps(payload, allow_nan=False)))
            self.db.execute("UPDATE recordings SET frame_count=?, duration_ms=? WHERE id=?",
                            (self.frames + 1, duration, self.active))
        self.last_time = t_ms
        self.last_seq = sample["seq"]
        self.frames += 1

    def note_event(self, event):
        if self.active is not None:
            payload = {**event}
            payload.setdefault("pc_received_ns", time.monotonic_ns())
            if self.first_time is not None and type(event.get("t_ms")) is int:
                payload["robot_t_ms"] = None if event.get("clock_domain") == "backend_monotonic" else event["t_ms"]
                payload["t_ms"] = event["t_ms"] - self.first_time
            with self.db:
                self.db.execute("INSERT INTO events VALUES (?, ?, ?)",
                                (self.active, self.event_count, json.dumps(payload, allow_nan=False)))
            self.event_count += 1

    def finish(self, status="completed", reason=None):
        identifier = self.active
        if identifier is None:
            return None
        self.active = None
        if status == "completed" and self.frames < 2:
            status, reason = "invalid", "at least two samples required"
        with self.db:
            self.db.execute("UPDATE recordings SET status=?, note=? WHERE id=?",
                            (status, reason, identifier))
        return self.get(identifier)

    def get(self, identifier, detail=False):
        row = self.db.execute("SELECT * FROM recordings WHERE id=?", (identifier,)).fetchone()
        if row is None:
            return None
        data = dict(row)
        data["metadata"] = json.loads(data["metadata"]) if data.get("metadata") else None
        if detail:
            for key, table, index in (("frames", "samples", "sample_index"), ("events", "events", "event_index")):
                rows = self.db.execute(f"SELECT payload FROM {table} WHERE recording_id=? ORDER BY {index}",
                                       (identifier,))
                data[key] = [json.loads(row[0]) for row in rows]
            executor = (data.get("metadata") or {}).get("executor")
            data["angles_are"] = ("virtual_joint_targets_estimated_preview" if executor == "virtual"
                                  else "software_output_targets_pwm_locked" if executor == "robot_no_power"
                                  else "committed_software_targets_no_position_feedback" if executor == "robot_powered"
                                  else "applied_servo_targets_not_encoder_readings")
        return data

    def list(self):
        return [self.get(row[0]) for row in self.db.execute("SELECT id FROM recordings ORDER BY created_ms DESC")]

    def start_run(self, recording, preflight):
        identifier = uuid4().hex
        with self.db:
            self.db.execute("INSERT INTO replay_runs VALUES (?, ?, ?, NULL, 'running', NULL, ?)",
                            (identifier, recording, int(time.time() * 1000), json.dumps(preflight)))
        return identifier

    def finish_run(self, identifier, status, reason):
        with self.db:
            self.db.execute("UPDATE replay_runs SET finished_ms=?, status=?, reason=? WHERE id=?",
                            (int(time.time() * 1000), status, reason, identifier))
