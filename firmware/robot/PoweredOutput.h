#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

namespace robot {
// Mapping evidence and this trial's operating limits are intentionally separate.
struct TrialProfile {
  const char *hash;
  float low[3], high[3], startup[3];
  float speed, acceleration, gripRate;
  float baseUs[3], shoulderUs[2], elbowUs[2];
  uint16_t openUs, closeUs;
  float tickUs;
};
struct OutputTransport {
  virtual bool writeTicks(const uint16_t *ticks) = 0; // order base, shoulder, elbow, gripper
  virtual bool fullOff() = 0;
  virtual ~OutputTransport() = default;
};
class PoweredOutput {
 public:
  enum class Stage { Locked, Holding, Unknown, Disabled };
  const TrialProfile &profile;
  Stage stage = Stage::Locked;
  bool prepared = false, powerConfirmed = false, committed = false;
  uint16_t ticks[4]{};
  float target[3]{}, opening = 1;
  uint32_t lastWriteMs = 0, writeErrors = 0;
  explicit PoweredOutput(const TrialProfile &p):profile(p) {}
  bool enabled() const { return stage == Stage::Holding; }
  bool locked() const { return stage == Stage::Locked || stage == Stage::Disabled; }
  const char *stageName() const {
    switch(stage) { case Stage::Holding:return "holding";case Stage::Unknown:return "unknown";
      case Stage::Disabled:return "disabled";default:return "locked"; }
  }
  // Nominal calibration units; not measured oscillator microseconds.
  float mappedUs(unsigned axis, float angle) const {
    if (axis == 0) return angle < 0 ? profile.baseUs[1] + angle*(profile.baseUs[1]-profile.baseUs[0])/45.f
      : profile.baseUs[1] + angle*(profile.baseUs[2]-profile.baseUs[1])/45.f;
    const float *p = axis == 1 ? profile.shoulderUs : profile.elbowUs;
    return p[0] + angle*(p[1]-p[0])/45.f;
  }
  bool quantize(const float *q, float open, uint16_t *out) const {
    if (!std::isfinite(open) || open < 0 || open > 1) return false;
    for (unsigned i=0;i<3;++i) {
      if (!std::isfinite(q[i]) || q[i] < profile.low[i]-.0001f || q[i] > profile.high[i]+.0001f) return false;
      out[i] = uint16_t(std::lround(mappedUs(i, q[i])/profile.tickUs));
      const float a=mappedUs(i,profile.low[i]),b=mappedUs(i,profile.high[i]);
      const uint16_t low=uint16_t(std::ceil(std::fmin(a,b)/profile.tickUs));
      const uint16_t high=uint16_t(std::floor(std::fmax(a,b)/profile.tickUs));
      if(low>high)return false;
      if(out[i]<low)out[i]=low;if(out[i]>high)out[i]=high;
    }
    out[3] = uint16_t(std::lround((profile.closeUs + open*(float(profile.openUs)-profile.closeUs))/profile.tickUs));
    for (unsigned i=0;i<4;++i) if (!out[i] || out[i] >= 4096) return false;
    return true;
  }
  bool enable(OutputTransport &bus, bool supported, bool powered, bool aligned, uint32_t now) {
    if (!prepared || !locked() || !supported || !powered || !aligned) return false;
    uint16_t values[4]; if (!quantize(profile.startup,1,values)) return false;
    if (!bus.writeTicks(values)) { fail();return false; }
    std::memcpy(ticks,values,sizeof(ticks));std::memcpy(target,profile.startup,sizeof(target));
    opening=1;lastWriteMs=now;committed=true;powerConfirmed=true;stage=Stage::Holding;return true;
  }
  bool submit(OutputTransport &bus, const float *q, float open, uint32_t now) {
    if (!enabled()) return false;
    uint16_t values[4];if(!quantize(q,open,values)){fail();return false;}
    // Software targets advance only after successful transport (or identical quantized PWM).
    if (std::memcmp(values,ticks,sizeof(ticks)) && !bus.writeTicks(values)) {fail();return false;}
    if (std::memcmp(values,ticks,sizeof(ticks))) lastWriteMs=now;
    std::memcpy(ticks,values,sizeof(ticks));std::memcpy(target,q,sizeof(target));opening=open;return true;
  }
  bool disable(OutputTransport &bus, bool supported, uint32_t now) {
    if(!supported)return false;
    if(!bus.fullOff()){fail();return false;}
    stage=Stage::Disabled;prepared=false;powerConfirmed=false;lastWriteMs=now;return true;
  }
  void fail() { ++writeErrors;stage=Stage::Unknown;prepared=false; }
};
} // namespace robot
