#pragma once

#include <stddef.h>
#include <stdint.h>

namespace motionarm {
constexpr uint8_t channel = 6;
constexpr size_t frameSize = 24;
constexpr uint16_t imuValid = 1;
constexpr uint16_t deadman = 2;
constexpr uint16_t neutral = 4;
constexpr uint32_t timeoutMs = 250;

struct Frame {
  uint16_t flags = 0;
  uint32_t boot = 0;
  uint32_t seq = 0;
  uint32_t uptime = 0;
  int16_t roll = 0;
  int16_t pitch = 0;
  uint8_t mode = 0;
  uint8_t gripperEvent = 0;
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

inline void encode(const Frame &frame, uint8_t (&out)[frameSize]) {
  out[0] = out[1] = 1;
  put(out + 2, frame.flags, 2);
  put(out + 4, frame.boot, 4);
  put(out + 8, frame.seq, 4);
  put(out + 12, frame.uptime, 4);
  put(out + 16, uint16_t(frame.roll), 2);
  put(out + 18, uint16_t(frame.pitch), 2);
  out[20] = frame.mode;
  out[21] = frame.gripperEvent;
  put(out + 22, crc(out, 22), 2);
}

inline bool decode(const uint8_t *p, size_t length, Frame &frame) {
  if (length != frameSize || p[0] != 1 || p[1] != 1 || get(p + 22, 2) != crc(p, 22)) {
    return false;
  }
  frame.flags = get(p + 2, 2);
  frame.boot = get(p + 4, 4);
  frame.seq = get(p + 8, 4);
  frame.uptime = get(p + 12, 4);
  frame.roll = int16_t(get(p + 16, 2));
  frame.pitch = int16_t(get(p + 18, 2));
  frame.mode = p[20];
  frame.gripperEvent = p[21];
  return frame.boot && !(frame.flags & ~7) && frame.mode <= 1 &&
         frame.roll >= -18000 && frame.roll <= 18000 &&
         frame.pitch >= -18000 && frame.pitch <= 18000;
}

inline bool newer(uint32_t next, uint32_t previous) {
  const uint32_t difference = next - previous;
  return difference != 0 && difference < 0x80000000UL;
}
}
