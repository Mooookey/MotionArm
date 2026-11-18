#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <cJSON.h>
#include <MotionArmLink.h>
#include <MotionArmPeers.h>

#include "ArmProfile.h"

namespace {
constexpr uint8_t kPca = 0x40;
constexpr uint8_t kPrescale = 121;
constexpr uint32_t kTickMs = 20;
constexpr uint32_t kLeaseMs = 250;
constexpr uint32_t kRearmMs = 100;
constexpr size_t kLineLimit = 512;
constexpr const char *kVersion = "0.1.0";

enum class Owner { None, Wearable, Pc, Replay };
struct Received {
  motionarm::Frame frame;
  uint32_t arrived;
  int rssi;
};

QueueHandle_t mailbox = nullptr;
Received input{};
bool haveInput = false;
bool haveNeutral = false;
uint32_t neutralSince = 0;
bool wearableEngaged = false;
uint32_t priorBoot = 0;
uint32_t dropped = 0;
uint32_t duplicates = 0;

Owner owner = Owner::None;
bool holding = false;
bool initialized = false;
bool pcaReady = false;
bool releasingGrip = false;
bool closedTimerActive = false;
uint32_t closedStarted = 0;
const char *fault = nullptr;

float requested[arm::count]{};
float applied[arm::count]{};
float speed[arm::count]{};
uint16_t lastPulse[arm::count]{};
uint32_t leaseStarted = 0;
uint32_t targetStarted = 0;
uint32_t targetTtl = 0;
uint32_t statusSeq = 0;
uint32_t lastPcSeq = 0;
char bootId[16]{}, sessionId[16]{}, leaseId[16]{};
bool haveSession = false;
uint32_t serialErrors = 0, telemetryDrops = 0, overruns = 0;

float clamp(float value, float low, float high) {
  return fminf(high, fmaxf(low, value));
}

float stepTowards(float from, float to, float bound) {
  return from + clamp(to - from, -bound, bound);
}

const char *ownerName(Owner value) {
  switch (value) {
    case Owner::Wearable: return "WEARABLE";
    case Owner::Pc: return "PC_MANUAL";
    case Owner::Replay: return "REPLAY";
    default: return "NONE";
  }
}

const char *stateName() {
  if (fault && strcmp(fault, "GRIPPER_HOLD_TIMEOUT")) return "FAULT";
  if (fault || holding) return "HOLD";
  if (owner == Owner::Wearable) return "TELEOP";
  if (owner != Owner::None) return ownerName(owner);
  return initialized ? "READY" : "SAFE_BOOT";
}

bool inside(const arm::Region &region, const float *q) {
  for (uint8_t i = 0; i < arm::count; ++i) {
    if (!isfinite(q[i]) || q[i] < region.minDeg[i] || q[i] > region.maxDeg[i]) return false;
  }
  return true;
}

bool pathInside(const float *from, const float *to) {
  for (uint8_t i = 0; i < kArmProfile.regionCount; ++i) {
    const auto &region = kArmProfile.regions[i];
    if (inside(region, from) && inside(region, to)) return true;
  }
  return false;
}

float pulseUs(uint8_t index, float value) {
  const auto &j = kArmProfile.joints[index];
  const float low = j.inverted ? j.maxUs : j.minUs;
  const float high = j.inverted ? j.minUs : j.maxUs;
  if (value == j.homeDeg) return j.homeUs;
  if (value < j.homeDeg) {
    return low + (j.homeUs - low) * ((value - j.minDeg) / (j.homeDeg - j.minDeg));
  }
  return j.homeUs + (high - j.homeUs) * ((value - j.homeDeg) / (j.maxDeg - j.homeDeg));
}

float gripperClosedDeg() {
  const auto &j = kArmProfile.joints[arm::gripper];
  const float low = j.inverted ? j.maxUs : j.minUs;
  const float high = j.inverted ? j.minUs : j.maxUs;
  const float pulse = kArmProfile.closeUs;
  if ((pulse < j.homeUs) != j.inverted) {
    return j.minDeg + (j.homeDeg - j.minDeg) * (pulse - low) / (j.homeUs - low);
  }
  return j.homeDeg + (j.maxDeg - j.homeDeg) * (pulse - j.homeUs) / (high - j.homeUs);
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kPca);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(kPca);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(kPca, 1) != 1) return false;
  value = Wire.read();
  return true;
}

void appendPulse(uint16_t ticks) {
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(ticks));
  Wire.write(uint8_t(ticks >> 8));
}

bool initializePca() {
  if (!writeRegister(0, 0x30)) return false;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    Wire.beginTransmission(kPca);
    Wire.write(uint8_t(6 + 4 * ch));
    appendPulse(4096);  // Per-channel FULL_OFF, including the unused channels.
    if (Wire.endTransmission() != 0) return false;
  }
  if (!writeRegister(0xFE, kPrescale) || !writeRegister(1, 0x04) ||
      !writeRegister(0, 0x20)) return false;
  delay(5);
  return writeRegister(0, 0xA0);
}

bool pcaConfigured() {
  uint8_t mode1 = 0, mode2 = 0, prescale = 0;
  return readRegister(0, mode1) && (mode1 & 0x30) == 0x20 &&
         readRegister(1, mode2) && mode2 == 0x04 &&
         readRegister(0xFE, prescale) && prescale == kPrescale;
}

bool writeTargets(const float *q) {
  if (!pcaReady || !kArmProfile.verified) return false;
  uint16_t ticks[arm::count]{};
  bool changed = !initialized;
  for (uint8_t i = 0; i < arm::count; ++i) {
    const auto &joint = kArmProfile.joints[i];
    const float width = pulseUs(i, q[i]);
    // Inward quantization guarantees the emitted pulse remains inside the profile.
    const float lowerUs = i == arm::gripper ? fminf(kArmProfile.openUs, kArmProfile.closeUs) : joint.minUs;
    const float upperUs = i == arm::gripper ? fmaxf(kArmProfile.openUs, kArmProfile.closeUs) : joint.maxUs;
    const long low = long(ceilf(lowerUs / kArmProfile.tickUs));
    const long high = long(floorf(upperUs / kArmProfile.tickUs));
    if (low <= 0 || high > 4095 || low > high || !isfinite(width)) return false;
    ticks[i] = lroundf(clamp(width / kArmProfile.tickUs, low, high));
    changed |= ticks[i] != lastPulse[i];
  }
  if (!changed) return true;
  // MODE2 OCH=0 latches the four contiguous channels on one STOP.
  Wire.beginTransmission(kPca);
  Wire.write(uint8_t(6 + 4 * 12));
  const uint8_t registerOrder[] = {3, 2, 0, 1};
  for (uint8_t index : registerOrder) appendPulse(ticks[index]);
  if (Wire.endTransmission() != 0) return false;
  memcpy(lastPulse, ticks, sizeof(lastPulse));
  return true;
}

bool writeGrip(float position) {
  const float tick = kArmProfile.tickUs;
  const float low = ceilf(fminf(kArmProfile.openUs, kArmProfile.closeUs) / tick);
  const float high = floorf(fmaxf(kArmProfile.openUs, kArmProfile.closeUs) / tick);
  const uint16_t count = lroundf(clamp(pulseUs(arm::gripper, position) / tick, low, high));
  if (count == lastPulse[arm::gripper] && (!fault || strcmp(fault, "PCA_WRITE_FAILED"))) return true;
  Wire.beginTransmission(kPca);
  Wire.write(uint8_t(6 + 4 * arm::channels[arm::gripper]));
  appendPulse(count);
  if (Wire.endTransmission() != 0) return false;
  lastPulse[arm::gripper] = count;
  return true;
}

void freeze() {
  memcpy(requested, applied, sizeof(applied));
  for (float &value : speed) value = 0;
}

void hold(const char *why = nullptr) {
  freeze();
  owner = Owner::None;
  leaseId[0] = 0;
  targetTtl = 0;
  holding = initialized;
  wearableEngaged = false;
  releasingGrip = false;
  haveNeutral = false;
  if (why && (!fault || strcmp(fault, "GRIPPER_HOLD_TIMEOUT") == 0)) fault = why;
  // Closing is enabled only in a cell with a verified capture surface. Open only
  // the gripper after stopping, without moving the load-bearing joints.
  if (initialized && pcaReady && closedTimerActive && kArmProfile.gripCaptureSurfaceVerified) {
    requested[arm::gripper] = kArmProfile.joints[arm::gripper].homeDeg;
    releasingGrip = pathInside(applied, requested);
  }
}

void onReceived(const esp_now_recv_info_t *info, const uint8_t *bytes, int length) {
  if (!info || !info->src_addr ||
      memcmp(info->src_addr, motionarm::wearableMac, sizeof(motionarm::wearableMac))) return;
  Received packet{};
  if (!motionarm::decode(bytes, length, packet.frame)) return;
  packet.arrived = millis();
  packet.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  xQueueOverwrite(mailbox, &packet);
}

bool deadman() { return haveInput && (input.frame.flags & motionarm::deadman); }

bool validImu(uint32_t now) {
  constexpr uint16_t valid = motionarm::imuValid | motionarm::neutral;
  return haveInput && now - input.arrived <= motionarm::timeoutMs &&
         (input.frame.flags & valid) == valid;
}

bool freshlyNeutral(uint32_t now) {
  return haveNeutral && validImu(now) && !deadman() && now - neutralSince >= kRearmMs;
}

void processPacket(Received packet) {
  const uint32_t now = millis();
  const bool staleInput = haveInput && now - input.arrived > motionarm::timeoutMs;
  if (staleInput) {
    haveNeutral = false;
    if (owner == Owner::Wearable) hold();
  }
  const bool changedBoot = haveInput && packet.frame.boot != input.frame.boot;
  if (haveInput && !changedBoot) {
    if (!motionarm::newer(packet.frame.seq, input.frame.seq) ||
        !motionarm::newer(packet.frame.uptime, input.frame.uptime)) {
      ++duplicates;
      return;
    }
    dropped += uint32_t(packet.frame.seq - input.frame.seq) - 1;
  } else if (changedBoot) {
    if (packet.frame.boot == priorBoot) {
      ++duplicates;
      return;
    }
    priorBoot = input.frame.boot;
    haveNeutral = false;
    if (owner == Owner::Wearable) hold();
  }
  const bool previousDeadman = deadman() && !changedBoot;
  const bool changedMode = haveInput && packet.frame.mode != input.frame.mode;
  const uint8_t eventDelta = haveInput && !changedBoot
    ? uint8_t(packet.frame.gripperEvent - input.frame.gripperEvent) : 0;
  input = packet;
  haveInput = true;
  const bool wasNeutral = haveNeutral && now - neutralSince >= kRearmMs;
  if (!validImu(now)) {
    if (owner == Owner::Wearable) hold();
    haveNeutral = false;
    return;
  }
  const bool level = abs(input.frame.roll) <= kArmProfile.deadzoneDeg * 100 &&
                     abs(input.frame.pitch) <= kArmProfile.deadzoneDeg * 100;
  if (!deadman()) {
    if (owner == Owner::Wearable) freeze();
    wearableEngaged = false;
    if (level && !haveNeutral) neutralSince = now;
    haveNeutral = level;
    return;
  }
  haveNeutral = false;
  if (owner != Owner::Wearable) return;
  if (changedMode) {
    freeze();
    wearableEngaged = false;
    return;
  }
  if (!previousDeadman) wearableEngaged = wasNeutral;
  if (wearableEngaged && previousDeadman && eventDelta == 1 && kArmProfile.gripCaptureSurfaceVerified) {
    const auto g = arm::gripper;
    const float opened = kArmProfile.joints[g].homeDeg;
    float target[arm::count];
    memcpy(target, applied, sizeof(target));
    target[g] = fabsf(requested[g] - opened) < 0.001f ? gripperClosedDeg() : opened;
    if (pathInside(applied, target)) requested[g] = target[g];
  }
}

void startGripTimer(uint32_t now, const float *q) {
  const float opened = kArmProfile.joints[arm::gripper].homeDeg;
  if (q[arm::gripper] != opened) {
    if (!closedTimerActive) closedStarted = now;
    closedTimerActive = true;
  } else {
    closedTimerActive = false;
  }
}

float wristVelocity(float angle, float cap) {
  const float deadzone = kArmProfile.deadzoneDeg;
  const float weight = clamp((fabsf(angle) - deadzone) / (30 - deadzone), 0, 1);
  return copysignf(weight * cap, angle);
}

float goalVelocity(float delta, const arm::Joint &joint, float dt) {
  const float maximum = fminf(sqrtf(2 * joint.accelerationDegPerSec2 * fabsf(delta)),
                             joint.speedDegPerSec);
  return copysignf(fminf(maximum, fabsf(delta) / dt), delta);
}

void updateGripRecovery(uint32_t elapsed) {
  const auto g = arm::gripper;
  const auto &joint = kArmProfile.joints[g];
  const float dt = fminf(elapsed, kTickMs) / 1000.0f;
  const float delta = joint.homeDeg - applied[g];
  const float desired = goalVelocity(delta, joint, dt);
  speed[g] = stepTowards(speed[g], desired, joint.accelerationDegPerSec2 * dt);
  const float next = applied[g] + copysignf(fminf(fabsf(delta), fabsf(speed[g] * dt)), delta);
  if (!pcaReady || !writeGrip(next)) {
    pcaReady = false;
    hold("PCA_GRIP_RELEASE_FAILED");
    return;
  }
  applied[g] = next;
  if (next == joint.homeDeg) {
    closedTimerActive = false;
    releasingGrip = false;
    speed[g] = 0;
  }
}

void control(uint32_t now, uint32_t elapsed) {
  if (!initialized) return;
  if (releasingGrip) { updateGripRecovery(elapsed); return; }
  if (fault) return;
  if (closedTimerActive && now - closedStarted >= kArmProfile.closedLimitMs) {
    hold("GRIPPER_HOLD_TIMEOUT");
    return;
  }
  if (owner == Owner::Wearable && !validImu(now)) { hold(); return; }
  if ((owner == Owner::Pc || owner == Owner::Replay) && now - leaseStarted >= kLeaseMs) {
    hold();
    return;
  }
  if (owner == Owner::None && !releasingGrip) return;
  if (owner == Owner::Wearable && (!deadman() || !wearableEngaged)) return;
  if (elapsed > 2 * kTickMs) {
    ++overruns;
    hold();
    return;
  }
  if (owner == Owner::Pc || owner == Owner::Replay) {
    if (!targetTtl) return;
    if (now - targetStarted >= targetTtl) {
      hold();
      return;
    }
  }
  const float dt = elapsed / 1000.0f;
  float proposed[arm::count]{};
  const uint8_t pitchAxis = input.frame.mode == 0 ? 1 : 2;
  for (uint8_t i = 0; i < arm::count; ++i) {
    const auto &j = kArmProfile.joints[i];
    float desired = goalVelocity(requested[i] - applied[i], j, dt);
    if (owner == Owner::Wearable && i != arm::gripper) {
      desired = 0;
      if (i == 0) desired = wristVelocity(input.frame.roll * kArmProfile.rollSign / 100.0f, j.speedDegPerSec);
      if (i == pitchAxis) desired = wristVelocity(input.frame.pitch * kArmProfile.pitchSign / 100.0f, j.speedDegPerSec);
    }
    speed[i] = stepTowards(speed[i], desired, j.accelerationDegPerSec2 * dt);
    proposed[i] = clamp(applied[i] + speed[i] * dt, j.minDeg, j.maxDeg);
    if (owner != Owner::Wearable || i == arm::gripper) {
      if ((requested[i] - applied[i]) * (requested[i] - proposed[i]) <= 0) {
        proposed[i] = requested[i];
        speed[i] = 0;
      }
    }
  }
  const auto g = arm::gripper;
  const float open = kArmProfile.joints[g].homeDeg;
  const float closed = gripperClosedDeg();
  const float boundedGrip = clamp(proposed[g], fminf(open, closed), fmaxf(open, closed));
  if (proposed[g] != boundedGrip) {
    proposed[g] = boundedGrip;
    speed[g] = 0;
  }
  if (!pathInside(applied, proposed)) { hold(); return; }
  if (!writeTargets(proposed)) {
    closedTimerActive |= proposed[g] != open;
    hold("PCA_WRITE_FAILED");
    return;
  }
  memcpy(applied, proposed, sizeof(applied));
  if (owner == Owner::Wearable) {
    memcpy(requested, applied, sizeof(float) * arm::gripper);
  }
  startGripTimer(now, applied);
}

bool emit(const char *line, int n, bool priority = false) {
  const int reserve = priority ? 0 : kLineLimit;
  if (n < 0 || size_t(n) > kLineLimit || Serial.availableForWrite() < n + 1 + reserve) {
    ++telemetryDrops;
    return false;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(line), n);
  Serial.write('\n');
  return true;
}

void newId(char *out, char prefix) {
  snprintf(out, 16, "%c-%08lx", prefix, (unsigned long)esp_random());
}

void formatArray(char (&out)[80], const float *q, bool present) {
  if (present) snprintf(out, sizeof(out), "[%.3f,%.3f,%.3f,%.3f]", q[0], q[1], q[2], q[3]);
  else strcpy(out, "null");
}

void quoted(char (&out)[96], const char *value) {
  if (value && *value) snprintf(out, sizeof(out), "\"%s\"", value);
  else strcpy(out, "null");
}

void telemetry(uint32_t now) {
  char line[kLineLimit + 1], qr[80], qa[80], calibration[96], error[96], lease[96];
  formatArray(qr, requested, initialized);
  formatArray(qa, applied, initialized);
  quoted(calibration, kArmProfile.id);
  quoted(error, fault);
  quoted(lease, leaseId);
  int n = snprintf(line, sizeof(line),
    "{\"v\":1,\"type\":\"state\",\"seq\":%lu,\"boot_id\":\"%s\",\"t_ms\":%lu,"
    "\"state\":\"%s\",\"owner\":\"%s\",\"q_req\":%s,\"q_out\":%s,\"q_measured_deg\":null,"
    "\"fault\":%s,\"calib\":%s,\"operator_initial_pose_confirmed\":%s,\"lease_id\":%s,"
    "\"pca_ok\":%s,\"overruns\":%lu}",
    (unsigned long)++statusSeq, bootId, (unsigned long)now, stateName(), ownerName(owner),
    qr, qa, error, calibration, initialized ? "true" : "false", lease, pcaReady ? "true" : "false",
    (unsigned long)overruns);
  emit(line, n, fault != nullptr);
  n = snprintf(line, sizeof(line),
    "{\"v\":1,\"type\":\"imu\",\"seq\":%lu,\"boot_id\":\"%s\",\"t_ms\":%lu,"
    "\"imu_seq\":%lu,\"roll_deg\":%.2f,\"pitch_deg\":%.2f,\"mode\":%u,\"deadman\":%s,"
    "\"valid\":%s,\"age_ms\":%lu,\"drop\":%lu,\"old\":%lu,\"rssi\":%d,\"serial_errors\":%lu,"
    "\"telemetry_drop\":%lu}",
    (unsigned long)++statusSeq, bootId, (unsigned long)now, (unsigned long)input.frame.seq,
    input.frame.roll / 100.0, input.frame.pitch / 100.0, input.frame.mode,
    deadman() && validImu(now) ? "true" : "false", validImu(now) ? "true" : "false",
    (unsigned long)(haveInput ? now - input.arrived : UINT32_MAX), (unsigned long)dropped,
    (unsigned long)duplicates, input.rssi, (unsigned long)serialErrors, (unsigned long)telemetryDrops);
  emit(line, n);
}

const cJSON *field(const cJSON *object, const char *name) {
  return cJSON_GetObjectItemCaseSensitive(object, name);
}

bool equals(const cJSON *object, const char *name, const char *value) {
  const auto *item = field(object, name);
  return cJSON_IsString(item) && strcmp(item->valuestring, value) == 0;
}

bool integer(const cJSON *object, const char *name, uint32_t &value) {
  const auto *item = field(object, name);
  if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble) || item->valuedouble < 0 ||
      item->valuedouble > UINT32_MAX || floor(item->valuedouble) != item->valuedouble) return false;
  value = uint32_t(item->valuedouble);
  return true;
}

void ack(uint32_t seq, const char *type, const char *result, const char *why = nullptr) {
  char line[kLineLimit + 1], error[96], lease[96];
  quoted(error, why);
  quoted(lease, leaseId);
  const int n = snprintf(line, sizeof(line),
    "{\"v\":1,\"type\":\"ack\",\"seq\":%lu,\"request_type\":\"%s\",\"result\":\"%s\","
    "\"robot_state\":\"%s\",\"lease_id\":%s,\"reason\":%s}",
    (unsigned long)seq, type, result, stateName(), lease, error);
  emit(line, n, true);
}

const char *activate(Owner next, uint32_t now) {
  if (!kArmProfile.verified) return "CALIBRATION_REQUIRED";
  const bool recovering = fault && strcmp(fault, "GRIPPER_HOLD_TIMEOUT") == 0 &&
                          !closedTimerActive && !releasingGrip;
  if ((fault && !recovering) || !pcaReady || !pcaConfigured()) return "PCA_OR_FAULT";
  if (owner != Owner::None || releasingGrip) return "OWNER_CONFLICT";
  if (next == Owner::Wearable && !freshlyNeutral(now)) return "RELEASE_A_AT_NEUTRAL";
  if (!initialized) {
    float home[arm::count];
    for (uint8_t i = 0; i < arm::count; ++i) home[i] = kArmProfile.joints[i].homeDeg;
    if (!pathInside(home, home) || !writeTargets(home)) {
      hold("PCA_WRITE_FAILED");
      return "CANNOT_START_PWM";
    }
    memcpy(applied, home, sizeof(home));
    freeze();
    initialized = true;
  }
  owner = next;
  holding = false;
  fault = nullptr;
  wearableEngaged = false;
  newId(leaseId, 'l');
  leaseStarted = now;
  return nullptr;
}

bool validArray(const cJSON *node, float *values) {
  if (!cJSON_IsArray(node) || cJSON_GetArraySize(node) != arm::count) return false;
  for (uint8_t i = 0; i < arm::count; ++i) {
    const auto *item = cJSON_GetArrayItem(node, i);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
    const auto &j = kArmProfile.joints[i];
    if (item->valuedouble < j.minDeg || item->valuedouble > j.maxDeg) return false;
    values[i] = item->valuedouble;
  }
  return true;
}

void dispatch(const cJSON *object, uint32_t now) {
  uint32_t v = 0, seq = 0;
  const auto *typeField = field(object, "type");
  if (!integer(object, "v", v) || v != 1 || !integer(object, "seq", seq) ||
      !cJSON_IsString(typeField)) { ++serialErrors; return; }
  const char *type = typeField->valuestring;
  if (!strcmp(type, "hello") && equals(object, "role", "pc")) {
    if (owner == Owner::Pc || owner == Owner::Replay) hold();
    newId(sessionId, 's');
    haveSession = true;
    lastPcSeq = seq;
    char line[kLineLimit + 1], calibration[96];
    quoted(calibration, kArmProfile.id);
    const int n = snprintf(line, sizeof(line),
      "{\"v\":1,\"type\":\"hello_ack\",\"seq\":%lu,\"role\":\"robot\",\"boot_id\":\"%s\","
      "\"session_id\":\"%s\",\"fw_version\":\"%s\",\"protocol_version\":1,\"calibration_id\":%s,"
      "\"channels\":[14,15,13,12],\"joint_names\":[\"base\",\"shoulder\",\"elbow\",\"gripper\"],\"state\":\"%s\"}",
      (unsigned long)seq, bootId, sessionId, kVersion, calibration, stateName());
    emit(line, n, true);
    return;
  }

  const char *known = nullptr;
  const char *commands[] = {"stop", "request_owner", "release_owner", "heartbeat", "joint_target"};
  for (const char *name : commands) {
    if (!strcmp(type, name)) known = name;
  }
  if (!known) { ++serialErrors; return; }
  if (!haveSession || !equals(object, "session_id", sessionId) || !motionarm::newer(seq, lastPcSeq)) {
    ack(seq, known, "NACK", "STALE_SESSION_OR_SEQUENCE");
    return;
  }
  lastPcSeq = seq;
  if (!strcmp(type, "stop")) {
    hold();
    ack(seq, known, "OK");
    return;
  }
  const bool confirmed = cJSON_IsTrue(field(object, "operator_confirmed_safe"));
  if (!strcmp(type, "request_owner")) {
    Owner next = Owner::None;
    if (equals(object, "requested_owner", "WEARABLE")) next = Owner::Wearable;
    if (equals(object, "requested_owner", "PC_MANUAL")) next = Owner::Pc;
    if (equals(object, "requested_owner", "REPLAY")) next = Owner::Replay;
    const char *failure = nullptr;
    if (!kArmProfile.verified) failure = "CALIBRATION_REQUIRED";
    else if (!equals(object, "calibration_id", kArmProfile.id)) failure = "CALIBRATION_MISMATCH";
    else if (!confirmed || next == Owner::None ||
             (!initialized && !cJSON_IsTrue(field(object, "operator_confirmed_initial_pose")))) {
      failure = "UNCONFIRMED_OR_INVALID_OWNER";
    } else {
      failure = activate(next, now);
    }
    ack(seq, known, failure ? "NACK" : "OK", failure);
    return;
  }
  if (owner == Owner::None || !equals(object, "lease_id", leaseId)) {
    ack(seq, known, "NACK", "OWNER_OR_LEASE_REQUIRED");
    return;
  }
  if (!strcmp(type, "release_owner")) {
    hold();
    ack(seq, known, "OK");
    return;
  }
  if (owner == Owner::Wearable) {
    ack(seq, known, "NACK", "WEARABLE_CONTROLS_MOTION");
    return;
  }
  if (now - leaseStarted >= kLeaseMs) {
    hold();
    ack(seq, known, "NACK", "LEASE_EXPIRED");
    return;
  }
  if (!strcmp(type, "heartbeat")) {
    leaseStarted = now;
    ack(seq, known, "OK");
    return;
  }
  uint32_t ttl = 0, segment = 0;
  float next[arm::count];
  if (!integer(object, "ttl_ms", ttl) || !ttl || ttl > kLeaseMs ||
      !integer(object, "segment_ms", segment) || !segment || segment > 1000 ||
      !validArray(field(object, "q_deg"), next) || !pathInside(applied, next)) {
    ack(seq, known, "NACK", "INVALID_OR_UNSAFE_TARGET");
    return;
  }
  // The verified gripper working span ends at the measured close position.
  const auto g = arm::gripper;
  const float opened = kArmProfile.joints[g].homeDeg;
  const float closed = gripperClosedDeg();
  if ((!kArmProfile.gripCaptureSurfaceVerified && fabsf(next[g] - opened) > 0.001f) ||
      next[g] < fminf(opened, closed) - 0.001f || next[g] > fmaxf(opened, closed) + 0.001f) {
    ack(seq, known, "NACK", "UNSAFE_GRIP_TARGET");
    return;
  }
  next[g] = kArmProfile.gripCaptureSurfaceVerified
    ? clamp(next[g], fminf(opened, closed), fmaxf(opened, closed)) : opened;
  memcpy(requested, next, sizeof(requested));
  leaseStarted = targetStarted = now;
  targetTtl = ttl;
  ack(seq, known, "ACCEPTED");
}

bool supportedJsonShape(const char *line) {
  // Keep untrusted nesting small before cJSON recurses. Object members only use
  // scalars or a flat array. Also reject encoded NULs before C-string comparisons.
  int depth = 0;
  bool inString = false;
  for (const char *p = line; *p; ++p) {
    if (inString) {
      if (*p == '\\') {
        if (!p[1] || !strncmp(p + 1, "u0000", 5)) return false;
        ++p;
      } else if (*p == '"') {
        inString = false;
      }
      continue;
    }
    if (*p == '"') inString = true;
    else if (*p == '{' || *p == '[') {
      if (++depth > 2) return false;
    } else if (*p == '}' || *p == ']') {
      if (--depth < 0) return false;
    }
  }
  return !inString && depth == 0;
}

void parse(char *line, uint32_t now) {
  if (!supportedJsonShape(line)) { ++serialErrors; return; }
  cJSON *root = cJSON_ParseWithOpts(line, nullptr, true);
  bool ok = cJSON_IsObject(root);
  if (ok) {
    for (const cJSON *item = root->child; item; item = item->next) {
      if (!item->string || field(root, item->string) != item || cJSON_IsObject(item)) {
        ok = false;
        break;
      }
    }
  }
  if (ok) dispatch(root, now);
  else ++serialErrors;
  cJSON_Delete(root);
}

void readSerial() {
  static char line[kLineLimit + 1];
  static size_t length = 0;
  static bool discarded = false;
  for (unsigned i = 0; i < 96 && Serial.available(); ++i) {
    const char value = Serial.read();
    if (value == '\n') {
      if (length && !discarded) {
        line[length] = 0;
        parse(line, millis());
      } else if (discarded) {
        ++serialErrors;
      }
      length = 0;
      discarded = false;
    } else if (!discarded && value && length < kLineLimit) {
      line[length++] = value;
    } else {
      discarded = true;
    }
  }
}
}  // namespace

void setup() {
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  newId(bootId, 'r');
  Wire.begin(21, 22, 100000);
  Wire.setTimeOut(4);
  pcaReady = initializePca() && pcaConfigured();
  if (!pcaReady) hold("PCA_NOT_READY");
  WiFi.mode(WIFI_STA);
  mailbox = xQueueCreate(1, sizeof(Received));
  if (!mailbox || esp_wifi_set_channel(motionarm::channel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK || esp_now_register_recv_cb(onReceived) != ESP_OK) {
    hold("WIRELESS_INIT_FAILED");
  }
}

void loop() {
  Received packet{};
  if (mailbox && xQueueReceive(mailbox, &packet, 0) == pdTRUE) processPacket(packet);
  readSerial();
  const uint32_t now = millis();
  static uint32_t lastControl = now, lastHealth = now, lastReport = now;
  if (now - lastHealth >= 500) {
    lastHealth = now;
    if (pcaReady && !pcaConfigured()) {
      pcaReady = false;
      hold("PCA_RESET_OR_OFFLINE");
    }
  }
  if (now - lastControl >= kTickMs) {
    const uint32_t dt = now - lastControl;
    lastControl = now;
    control(now, dt);
  }
  if (now - lastReport >= 100) {
    lastReport = now;
    telemetry(now);
  }
  delay(1);
}
