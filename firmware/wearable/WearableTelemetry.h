#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include "OrientationFilter.h"

// USB v1 / att3 is the only live wrist format. Keep serialization shared with
// the host-side regression so its worst-case UART budget is actually tested.
constexpr size_t kTelemetryMaxBytes = 384;

struct WearableTelemetry {
  uint32_t boot, seq, ms, ageMs;
  uint8_t who;
  bool valid;
  uint16_t calibrationSamples;
  OrientationFilter::Quaternion q;  // Sensor/body to filter world, [w,x,y,z].
  float gyroDps[3], accelG[3];  // Sensor XYZ; bias corrected deg/s, raw g.
  bool a, b;
  uint32_t ac, bc;
  float normG, temperatureC;
  uint32_t errors;
  uint8_t rejection;
  uint32_t rejected;
};

inline int serializeWearable(char *buffer, size_t capacity, const WearableTelemetry &t) {
  return std::snprintf(buffer, capacity,
      "{\"v\":1,\"att\":3,\"role\":\"wearable\",\"boot\":%lu,\"seq\":%lu,\"ms\":%lu,"
      "\"age_ms\":%lu,\"who\":%u,\"valid\":%u,\"cal\":%u,"
      "\"q\":[%.6f,%.6f,%.6f,%.6f],\"gyro_dps\":[%.2f,%.2f,%.2f],"
      "\"accel_g\":[%.3f,%.3f,%.3f],\"a\":%u,\"b\":%u,\"ac\":%lu,\"bc\":%lu,"
      "\"norm_g\":%.3f,\"temp_c\":%.2f,\"errors\":%lu,\"rej\":%u,\"gr\":2000,\"rc\":%lu}\n",
      (unsigned long)t.boot, (unsigned long)t.seq, (unsigned long)t.ms,
      (unsigned long)t.ageMs, unsigned(t.who), unsigned(t.valid), unsigned(t.calibrationSamples),
      t.q.w, t.q.x, t.q.y, t.q.z,
      t.gyroDps[0], t.gyroDps[1], t.gyroDps[2], t.accelG[0], t.accelG[1], t.accelG[2],
      unsigned(t.a), unsigned(t.b), (unsigned long)t.ac, (unsigned long)t.bc,
      t.normG, t.temperatureC, (unsigned long)t.errors, unsigned(t.rejection),
      (unsigned long)t.rejected);
}
