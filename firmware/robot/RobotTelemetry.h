#pragma once
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include "RobotController.h"
#include "PoweredOutput.h"

namespace robot {
// Whole snapshots are admitted atomically to UART. Shared with host tests.
struct TelemetryBatch {
  static constexpr size_t budget = 3072; // v3/230400; v2 retains its original five parts.
  char data[budget + 1]{};
  size_t length = 0;
  bool valid = true;
  const char *boot = nullptr;
  uint32_t snapshot = 0, now = 0;
  uint32_t *sequence = nullptr;
  unsigned version = 2;
  static const char *jsonBool(bool value) { return value ? "true" : "false"; }
  void append(const char *type, const char *format, ...) {
    char fields[513]; va_list args; va_start(args, format);
    const int count = vsnprintf(fields, sizeof(fields), format, args); va_end(args);
    if (count < 0 || count >= int(sizeof(fields))) { valid = false; return; }
    char line[513];
    const int n = snprintf(line, sizeof(line),
      "{\"v\":%u,\"type\":\"%s\",\"seq\":%lu,\"boot_id\":\"%s\",\"snapshot_id\":%lu,\"t_ms\":%lu,%s}\n",
      version, type, (unsigned long)++*sequence, boot, (unsigned long)snapshot, (unsigned long)now, fields);
    if (n < 0 || n > 512 || length + n > budget) { valid = false; return; }
    memcpy(data + length, line, n); length += n; data[length] = 0;
  }
  static void vector(char *out, size_t size, const float *v, unsigned count, bool present) {
    if (!present) { snprintf(out, size, "null"); return; }
    if (count == 4) snprintf(out, size, "[%.6f,%.6f,%.6f,%.6f]", v[0], v[1], v[2], v[3]);
    else snprintf(out, size, "[%.3f,%.3f,%.3f]", v[0], v[1], v[2]);
  }
  void format(const Controller &c, uint32_t timestamp, const char *bootId, uint32_t id,
      uint32_t &seq, const char *mac, bool pca, bool manager,
      uint32_t received, uint32_t rejected, uint32_t droppedTelemetry, uint32_t serialErrors,
      const PoweredOutput *output = nullptr) {
    version=output?3:2;
    boot = bootId; snapshot = id; now = timestamp; sequence = &seq;
    char ref[64], rotation[64], q[64], gyro[64], accel[64];
    vector(ref, sizeof(ref), c.reference, 4, c.hasReference);
    vector(rotation, sizeof(rotation), c.rotation, 3, c.hasReference);
    vector(q, sizeof(q), c.input.q, 4, true);
    vector(gyro, sizeof(gyro), c.input.gyroDps, 3, true);
    vector(accel, sizeof(accel), c.input.accelG, 3, true);
    append("control", "\"enabled\":%s,\"armed\":%s,\"owner\":\"%s\",\"reference_quaternion\":%s,"
      "\"delta_rotation_vector_deg\":%s,\"angle_ambiguous\":%s,\"can_enable\":%s,\"fault\":%s%s%s,\"reason\":\"%s\"",
      jsonBool(c.enabled), jsonBool(c.armed), c.enabled ? "WEARABLE" : "NONE", ref, rotation, jsonBool(c.ambiguous),
      jsonBool(pca && !c.enabled && c.canEnableInput(now)), c.fault ? "\"" : "", c.fault ? c.fault : "null", c.fault ? "\"" : "", c.reason);
    const auto &f = c.input;
    append("imu", "\"wearable_boot\":%lu,\"wearable_seq\":%lu,\"ms\":%lu,\"age_ms\":%lu,\"who\":%u,\"valid\":%u,"
      "\"cal\":%u,\"q\":%s,\"gyro_dps\":%s,\"accel_g\":%s,\"a\":%u,\"b\":%u,\"ac\":%lu,\"bc\":%lu,"
      "\"norm_g\":%.3f,\"temp_c\":%.2f,\"errors\":%lu,\"rej\":%u,\"gr\":2000,\"rc\":%lu",
      (unsigned long)f.boot, (unsigned long)f.seq, (unsigned long)f.uptime,
      (unsigned long)c.ageMs(now), unsigned(f.who), unsigned(c.valid(now)), unsigned(f.calibrationSamples),
      q, gyro, accel, unsigned(bool(f.flags & motionarm::buttonA)), unsigned(bool(f.flags & motionarm::buttonB)),
      (unsigned long)f.ac, (unsigned long)f.bc, f.normG, f.tempC, (unsigned long)f.errors,
      unsigned(f.rejection), (unsigned long)f.rejected);
    append("targets", "\"requested\":{\"base_yaw_deg\":%.3f,\"shoulder_down_deg\":%.3f,\"elbow_down_deg\":%.3f,\"gripper_open_ratio\":%.3f},"
      "\"output_target\":{\"base_yaw_deg\":%.3f,\"shoulder_down_deg\":%.3f,\"elbow_down_deg\":%.3f,\"gripper_open_ratio\":%.3f},"
      "\"measured\":null,\"velocity_deg_s\":[%.3f,%.3f,%.3f],\"limits\":[%s,%s,%s]",
      c.qReq[0], c.qReq[1], c.qReq[2], c.openReq, c.qOut[0], c.qOut[1], c.qOut[2], c.openOut,
      c.velocity[0], c.velocity[1], c.velocity[2], jsonBool(c.qOut[0] <= c.low[0] || c.qOut[0] >= c.high[0]),
      jsonBool(c.qOut[1] <= c.low[1] || c.qOut[1] >= c.high[1]), jsonBool(c.qOut[2] <= c.low[2] || c.qOut[2] >= c.high[2]));
    append("gripper", "\"state\":\"%s\",\"limit_ms\":5000,\"elapsed_ms\":%lu,\"remaining_ms\":%lu,\"cycle_id\":%lu,"
      "\"auto_open_reason\":%s%s%s,\"reclose_blocked\":%s",
      c.gripStateName(), (unsigned long)c.gripElapsedMs(now), (unsigned long)c.gripRemainingMs(now),
      (unsigned long)c.gripCycle, c.gripReason ? "\"" : "", c.gripReason ? c.gripReason : "null", c.gripReason ? "\"" : "",
      jsonBool(!c.gripCanClose()));
    char serialDiagnostics[40]{};
    if(output)snprintf(serialDiagnostics,sizeof(serialDiagnostics),",\"serial_errors\":%lu",(unsigned long)serialErrors);
    append("health", "\"mode\":\"%s\",\"output_locked\":%s,\"mac\":\"%s\","
      "\"pca_readable\":%s,\"manager_active\":%s,\"wireless\":{\"received\":%lu,\"rejected\":%lu,\"duplicates\":%lu,"
      "\"out_of_order\":%lu,\"drop\":%lu,\"restarts\":%lu,\"status\":\"%s\"},\"drop_uart\":%lu,\"ev_drop\":%lu%s",
      output?"robot_powered":"robot_no_power", jsonBool(!output || output->locked()),
      mac, jsonBool(pca), jsonBool(manager), (unsigned long)received, (unsigned long)rejected, (unsigned long)c.duplicates,
      (unsigned long)c.outOfOrder, (unsigned long)c.dropped, (unsigned long)c.restarts,
      c.valid(now) ? "valid" : "stale", (unsigned long)droppedTelemetry, (unsigned long)c.eventDrops,serialDiagnostics);
    if(output) {
      char pwm[48],written[16];
      if(output->committed)snprintf(pwm,sizeof(pwm),"[%u,%u,%u,%u]",output->ticks[0],output->ticks[1],output->ticks[2],output->ticks[3]);
      else snprintf(pwm,sizeof(pwm),"null");
      if(output->committed || output->stage==PoweredOutput::Stage::Disabled)snprintf(written,sizeof(written),"%lu",(unsigned long)output->lastWriteMs);
      else snprintf(written,sizeof(written),"null");
      append("output", "\"profile_hash\":\"%s\",\"stage\":\"%s\",\"pwm_ticks\":%s,\"last_write_ms\":%s,\"write_errors\":%lu,"
        "\"pwm_enabled\":%s,\"committed\":%s,\"trial_limits_deg\":[[-3,3],[42,48],[42,48]],"
        "\"max_velocity_deg_s\":2,\"max_acceleration_deg_s2\":5,\"power_confirmed\":%s,\"prepared\":%s,\"diagnostic_active\":%s",
        output->profile.hash,output->stageName(),pwm,written,(unsigned long)output->writeErrors,
        output->stage==PoweredOutput::Stage::Unknown?"null":jsonBool(output->enabled()),
        jsonBool(output->committed),jsonBool(output->powerConfirmed),
        jsonBool(output->prepared),jsonBool(c.diagnosticActive));
    }
  }
};
} // namespace robot
