#pragma once

#include <stdint.h>

namespace arm {
constexpr uint8_t count = 4;
constexpr uint8_t gripper = 3;
constexpr uint8_t channels[count] = {14, 15, 13, 12};

struct Joint {
  float minDeg;
  float maxDeg;
  float homeDeg;
  uint16_t minUs;
  uint16_t maxUs;
  uint16_t homeUs;
  bool inverted;
  float speedDegPerSec;
  float accelerationDegPerSec2;
};

struct Region {
  float minDeg[count];
  float maxDeg[count];
};

struct Calibration {
  bool verified;
  const char *id;
  Joint joints[count];
  uint16_t openUs;
  uint16_t closeUs;
  uint32_t closedLimitMs;
  bool gripCaptureSurfaceVerified;
  float tickUs;
  int8_t rollSign;
  int8_t pitchSign;
  float deadzoneDeg;
  uint8_t regionCount;
  Region regions[8];
};
}
