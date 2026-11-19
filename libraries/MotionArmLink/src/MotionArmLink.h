#pragma once

#include <stddef.h>
#include <stdint.h>
#include <math.h>

namespace motionarm {
constexpr uint8_t channel = 6;
constexpr uint8_t protocolVersion = 2;
constexpr size_t frameSize = 82;
constexpr uint16_t imuValid = 1;
constexpr uint16_t buttonA = 2, buttonB = 4, buttonsPrimed = 8;
constexpr uint32_t timeoutMs = 250;

struct Frame {
  uint16_t flags = 0;
  uint32_t boot = 0;
  uint32_t seq = 0;
  uint32_t uptime = 0;
  uint32_t ageMs = UINT32_MAX;
  float q[4] = {1, 0, 0, 0};
  float gyroDps[3] = {}, accelG[3] = {};
  uint16_t calibrationSamples = 0;
  uint32_t ac = 0, bc = 0, errors = 0, rejected = 0;
  float normG = 0, tempC = 0;
  uint8_t who = 0, rejection = 0;
};

inline uint16_t crc(const uint8_t *p, size_t length) {
  uint16_t value = 0xffff;
  while (length--) {
    value ^= uint16_t(*p++) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      value = (value & 0x8000) ? (value << 1) ^ 0x1021 : value << 1;
    }
  }
  return value;
}

inline void put(uint8_t *p, uint32_t value, size_t n) {
  for (size_t i = 0; i < n; ++i) p[i] = uint8_t(value >> (8 * i));
}

inline uint32_t get(const uint8_t *p, size_t n) {
  uint32_t value = 0;
  for (size_t i = 0; i < n; ++i) value |= uint32_t(p[i]) << (8 * i);
  return value;
}

inline bool scalar(float value, float low, float high) {
  return isfinite(value) && value >= low && value <= high;
}
// Explicit fixed-point little-endian layout, not the native Frame struct.
inline bool encode(const Frame &f, uint8_t (&out)[frameSize]) {
  if (!f.boot || (f.flags & ~uint16_t(15)) || f.calibrationSamples > 250 || f.rejection > 5 ||
      !scalar(f.normG, 0, 7) || !scalar(f.tempC, -327.68f, 327.67f)) return false;
  for (float v : f.q) if (!scalar(v, -1.01f, 1.01f)) return false;
  for (float v : f.gyroDps) if (!scalar(v, -2031, 2031)) return false;
  for (float v : f.accelG) if (!scalar(v, -4, 4)) return false;
  out[0] = protocolVersion; out[1] = 1;
  put(out + 2, f.flags, 2); put(out + 4, f.boot, 4);
  put(out + 8, f.seq, 4); put(out + 12, f.uptime, 4); put(out + 16, f.ageMs, 4);
  for (size_t i = 0; i < 4; ++i) put(out + 20 + 4*i, uint32_t(int32_t(lroundf(f.q[i]*1000000))), 4);
  for (size_t i = 0; i < 3; ++i) put(out + 36 + 4*i, uint32_t(int32_t(lroundf(f.gyroDps[i]*100))), 4);
  for (size_t i = 0; i < 3; ++i) put(out + 48 + 2*i, uint16_t(int16_t(lroundf(f.accelG[i]*1000))), 2);
  put(out + 54, f.calibrationSamples, 2); put(out + 56, f.ac, 4); put(out + 60, f.bc, 4);
  put(out + 64, f.errors, 4); put(out + 68, f.rejected, 4);
  put(out + 72, uint16_t(lroundf(f.normG*1000)), 2);
  put(out + 74, uint16_t(int16_t(lroundf(f.tempC*100))), 2);
  out[76] = f.who; out[77] = f.rejection; out[78] = out[79] = 0;
  put(out + 80, crc(out, 80), 2); return true;
}
// Callback-safe: bounded integer/range checks, no quaternion normalization.
inline bool decode(const uint8_t *p, size_t length, Frame &f) {
  if (!p || length != frameSize || p[0] != protocolVersion || p[1] != 1 ||
      get(p + 80, 2) != crc(p, 80) || p[78] || p[79]) return false;
  const uint16_t flags = uint16_t(get(p + 2, 2));
  if ((flags & ~uint16_t(15)) || !get(p + 4, 4) || get(p + 54, 2) > 250 || p[77] > 5 ||
      get(p + 72, 2) > 7000) return false;
  Frame next;
  next.flags = flags; next.boot = get(p + 4, 4); next.seq = get(p + 8, 4);
  next.uptime = get(p + 12, 4); next.ageMs = get(p + 16, 4);
  for (size_t i = 0; i < 4; ++i) {
    const int32_t v = int32_t(get(p + 20 + 4*i, 4));
    if (v < -1010000 || v > 1010000) return false;
    next.q[i] = v / 1000000.0f;
  }
  for (size_t i = 0; i < 3; ++i) {
    const int32_t g = int32_t(get(p + 36 + 4*i, 4));
    const int16_t a = int16_t(get(p + 48 + 2*i, 2));
    if (g < -203100 || g > 203100 || a < -4000 || a > 4000) return false;
    next.gyroDps[i] = g / 100.0f; next.accelG[i] = a / 1000.0f;
  }
  next.calibrationSamples = uint16_t(get(p + 54, 2));
  next.ac = get(p + 56, 4); next.bc = get(p + 60, 4);
  next.errors = get(p + 64, 4); next.rejected = get(p + 68, 4);
  next.normG = get(p + 72, 2) / 1000.0f; next.tempC = int16_t(get(p + 74, 2)) / 100.0f;
  next.who = p[76]; next.rejection = p[77]; f = next; return true;
}

inline bool newer(uint32_t next, uint32_t previous) {
  const uint32_t difference = next - previous;
  return difference != 0 && difference < 0x80000000UL;
}
}
