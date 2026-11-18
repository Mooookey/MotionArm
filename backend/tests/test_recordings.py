"""Persistence guarantees shared by virtual and legacy recording modes."""
import sqlite3
import unittest
from pathlib import Path

from app.recordings import Recordings
from helpers import temporary_directory


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.directory = temporary_directory("recordings-")
        self.path = Path(self.directory.name) / "recordings.sqlite3"
        self.records = Recordings(self.path)

    def tearDown(self):
        self.records.db.close()
        self.directory.cleanup()

    def start(self):
        return self.records.start("sensor session", "wearable", "boot-a", "sim-a", "virtual", "usb-v1")

    def sample(self, seq=1, **extra):
        return {"seq": seq, "q_req": [0, 45, 45, 1], "q_out": [0, 45, 45, 1], **extra}

    def test_samples_use_elapsed_time_and_keep_original_timestamp(self):
        record = self.start()
        self.records.append("boot-a", "sim-a", 12340, self.sample())
        self.records.append("boot-a", "sim-a", 12380, self.sample(2))
        result = self.records.finish()
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["duration_ms"], 40)
        detail = self.records.get(record["id"], detail=True)
        self.assertEqual([row["t_ms"] for row in detail["frames"]], [0, 40])
        self.assertEqual([row["robot_t_ms"] for row in detail["frames"]], [12340, 12380])

    def test_duplicate_sample_does_not_duplicate_database_rows(self):
        record = self.start()
        self.records.append("boot-a", "sim-a", 100, self.sample())
        self.records.append("boot-a", "sim-a", 100, self.sample())
        self.assertEqual(len(self.records.get(record["id"], detail=True)["frames"]), 1)

    def test_long_gap_interrupts_and_keeps_existing_frames(self):
        record = self.start()
        self.records.append("boot-a", "sim-a", 100, self.sample())
        self.records.append("boot-a", "sim-a", 140, self.sample(2))
        self.records.append("boot-a", "sim-a", 500, self.sample(3))
        saved = self.records.get(record["id"], detail=True)
        self.assertIsNone(self.records.active)
        self.assertEqual(saved["status"], "interrupted")
        self.assertEqual(len(saved["frames"]), 2)
        self.assertIn("gap", saved["note"])

    def test_reboot_and_config_change_interrupt_recording(self):
        for boot, config in (("boot-b", "sim-a"), ("boot-a", "sim-b")):
            with self.subTest(boot=boot, config=config):
                record = self.start()
                self.records.append("boot-a", "sim-a", 100, self.sample())
                self.records.append(boot, config, 140, self.sample(2))
                saved = self.records.get(record["id"], detail=True)
                self.assertEqual(saved["status"], "interrupted")
                self.assertEqual(len(saved["frames"]), 1)

    def test_backend_restart_marks_unfinished_work_interrupted(self):
        record = self.start()
        run = self.records.start_run(record["id"], {"executor": "virtual"})
        self.records.db.close()
        self.records = Recordings(self.path)
        self.assertEqual(self.records.get(record["id"])["status"], "interrupted")
        saved_run = self.records.db.execute("SELECT status, reason FROM replay_runs WHERE id=?", (run,)).fetchone()
        self.assertEqual(saved_run["status"], "aborted")
        self.assertEqual(saved_run["reason"], "backend restart")

    def test_one_sample_is_not_a_complete_recording(self):
        self.start()
        self.records.append("boot-a", "sim-a", 100, self.sample())
        self.assertEqual(self.records.finish()["status"], "invalid")

    def test_virtual_metadata_roundtrips_and_labels_target_semantics(self):
        metadata = {"executor": "virtual", "input": "imu_usb", "geometry_version": "baseline-a",
                    "simulation_config": {"roll_sign": -1}, "angle_convention": "down_from_upright"}
        record = self.records.start("virtual", "wearable", "boot-a", "sim-a", "virtual", "usb-v1",
                                    metadata=metadata)
        self.assertEqual(record["metadata"], metadata)
        self.assertEqual(self.records.list()[0]["metadata"], metadata)
        detail = self.records.get(record["id"], detail=True)
        self.assertEqual(detail["angles_are"], "virtual_joint_targets_estimated_preview")


class LegacyDatabaseTests(unittest.TestCase):
    def test_existing_samples_survive_opening_and_schema_upgrade(self):
        with temporary_directory("legacy-") as directory:
            path = Path(directory) / "old.sqlite3"
            with sqlite3.connect(path) as db:
                db.executescript("""
                    CREATE TABLE recordings (
                        id TEXT PRIMARY KEY, name TEXT NOT NULL, created_ms INTEGER NOT NULL,
                        status TEXT NOT NULL, source TEXT NOT NULL, boot_id TEXT NOT NULL,
                        calibration_id TEXT NOT NULL, workcell_id TEXT, protocol_version INTEGER NOT NULL,
                        fw_version TEXT, frame_count INTEGER NOT NULL DEFAULT 0,
                        duration_ms INTEGER NOT NULL DEFAULT 0, note TEXT);
                    CREATE TABLE samples (
                        recording_id TEXT NOT NULL, sample_index INTEGER NOT NULL, payload TEXT NOT NULL,
                        PRIMARY KEY(recording_id,sample_index));
                    INSERT INTO recordings VALUES ('legacy','old recording',1,'completed','serial',
                        'robot-boot','hardware-profile','bench',1,'0.1.0',1,0,NULL);
                    INSERT INTO samples VALUES ('legacy',0,'{"seq":1,"t_ms":0,"q_out":[90,65,105,25]}');
                """)
            db.close()
            records = Recordings(path)
            try:
                legacy = records.get("legacy", detail=True)
                self.assertEqual(legacy["source"], "serial")
                self.assertIsNone(legacy["metadata"])
                self.assertEqual(legacy["angles_are"], "applied_servo_targets_not_encoder_readings")
                self.assertEqual(records.db.execute("PRAGMA user_version").fetchone()[0], 2)
                self.assertEqual(legacy["frames"][0]["q_out"], [90, 65, 105, 25])
                raw = records.db.execute("SELECT payload FROM samples WHERE recording_id='legacy'").fetchone()[0]
                self.assertEqual(raw, '{"seq":1,"t_ms":0,"q_out":[90,65,105,25]}')
            finally:
                records.db.close()


if __name__ == "__main__":
    unittest.main()
