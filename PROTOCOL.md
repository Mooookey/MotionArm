# IMU wire protocol v1

All device output is framed binary at 460800 baud, 8N1. Upload remains 115200.
Host commands are ASCII lines: `INFO`, `START`, `STOP`, `CALIBRATE`, `ZERO`.
Only one owner may open the serial port. START is accepted only when calibrated
and idle; CALIBRATE and ZERO are accepted only when idle. STOP is idempotent.

## Envelope

Bytes 0..1: `A5 5A`; byte 2: version 1; byte 3: type;
bytes 4..5: uint16 total frame length (header + payload + CRC, <=4096).
All multibyte values are little endian. Final two bytes are CRC16/CCITT-FALSE
(poly 0x1021, init 0xFFFF, no reflection, xorout 0), over every preceding byte,
including magic and header. Types: DATA=1, META=2, END=3, STATUS=4, ERROR=5.
META/END/STATUS/ERROR payloads are UTF-8 JSON objects.

## DATA (exactly 90 bytes)

| Offset | Type | Meaning |
|---|---|---|
| 6 | uint32 | session_id |
| 10 | uint32 | seq, starts at 0 per session |
| 14 | uint64 | t_us, ESP32 read-start timestamp |
| 22 | uint16 | flags: calibrated=1, stationary=2, integration_gap=4 |
| 24 | int16[6] | raw ax,ay,az,gx,gy,gz (ADC counts) |
| 36 | float32[6] | filtered ax,ay,az in m/s2; gx,gy,gz in deg/s |
| 60 | float32[4] | quaternion w,x,y,z, sensor/body -> world |
| 76 | float32[3] | world position x,y,z in metres |
| 88 | uint16 | CRC |

World is right handed, Z up. Body XYZ follow the sensor's printed XYZ.
Euler display uses ZYX yaw/pitch/roll. Raw is before software bias/filter but
after the shared hardware DLPF. Convert raw accel with g/8192 and gyro /65.5.

## JSON fields

META includes `session_id`, `state`, `sample_hz` (200), `baud` (460800),
`accel_lsb_per_g` (8192), `gyro_lsb_per_dps` (65.5), `g` (9.80665),
`accel_range_g` (4), `gyro_range_dps` (500), `hardware_dlpf_hz` (44),
`sensor_output_hz` (1000), `filter_hz` (20), `mahony_kp` (0.5),
`mahony_ki` (0), `gyro_bias_dps` (three numbers), `calibrated` (boolean),
`calibration_samples`, `quaternion_order` ("wxyz"), `coordinate_frame`
("right-handed body-to-world, world Z up"), `firmware_version` ("1.0.0").

STATUS and END include `session_id`, `state`, `generated`, `sent`,
`sample_fail`, `stale_samples`, `missed_deadlines`, `queue_drops`,
`integration_gaps`, `first_t_us`, `last_t_us`, `dt_count`, `dt_mean_us`,
`dt_std_us`, `dt_min_us`, `dt_max_us`, and `calibrated`.
ERROR includes `code` and `message`, optionally `session_id`.
Session counters reset on START; state/calibration updates use META.

META also records `accel_norm_scale` and `accel_calibration_method`
(`single_pose_norm_scale`). This scalar applies ONLY to processed acceleration;
raw counts and their nominal SI conversion remain unchanged. It normalizes
the stationary calibration vector magnitude to g, and cannot identify
individual axis offsets/gains or distinguish them from tilt. Independent
measurement data and further orientations are needed to assess accuracy.
460800 replaces the planned 230400 after CRC errors were observed at 230400.

## Boundaries and validity

START creates a new session, zeros its counters and sequence, then sends META
before any DATA. Processing continues while idle. Each successful fresh
read during recording increments generated and seq BEFORE enqueue; queue
drops therefore count as lost frames. Read failures/stale reads are counted
separately, and never emit a recycled sample. Missed timer slots are separate.
STOP stops generating DATA, drains the queue, and sends END. A repeated STOP
resends the latest END so a host can recover a missed summary. No debug text
may be mixed into output. INFO returns META and latest STATUS/END.

The receiver counts CRC-valid, finite, unique DATA belonging to this session.
Loss=(END.generated-unique_received)/END.generated; without END it is unknown.
Calibration and ZERO cannot change an active recording. Integration dt>20ms
sets the gap flag and increments integration_gaps; do not bridge that gap.
A 5-minute drift test with sample failures, gaps, drops, or missing data is
incomplete and must not be presented as a complete 5-minute drift result.
