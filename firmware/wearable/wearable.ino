#include <Wire.h>
#include <math.h>
#include <esp_random.h>
#include "OrientationFilter.h"
#include "DebouncedButton.h"

constexpr uint8_t kImuAddress = 0x68;
constexpr uint32_t kPeriodUs = 10000;
constexpr uint16_t kCalibrationSamples = 250;
constexpr uint32_t kLongPressMs = 700;
constexpr uint32_t kTelemetryPeriodMs = 40;
constexpr size_t kTelemetryMaxBytes = 384;
constexpr uint16_t kGyroRangeDps = 2000;
constexpr float kGyroCountsPerDps = 16.4f;
constexpr int32_t kGyroSaturationRaw = 32700;

enum class Rejection : uint8_t {
  None = 0, Read = 1, SampleGap = 2, Acceleration = 3,
  GyroSaturation = 4, Filter = 5
};
Rejection rejection = Rejection::None;



StationaryRecovery recovery;

DebouncedButton deadman{25}, gripperButton{26};
bool havePress = false;
uint32_t pressedAt = 0, shortEvents = 0, longEvents = 0;
constexpr unsigned mode = 0;  // Legacy protocol field; B no longer selects an arm.
uint8_t who = 0;
bool configured = false, valid = false, calibrated = false;
uint32_t errors = 0, rejected = 0, lastSample = 0;
uint32_t lastSuccessfulSample = 0;
bool haveSample = false;
uint16_t samples = 0;
float sum[3] = {}, bias[3] = {};
float gyroMin[3] = {}, gyroMax[3] = {}, latestGyro[3] = {};
float firstAccel[3] = {}, accelNorm = 0;
float gravitySum = 0, gravityReference = 1;
float roll = 0, pitch = 0, temperature = 0;
OrientationFilter attitude;
uint32_t bootId = 0, telemetrySequence = 0;

bool readRegisters(uint8_t reg, uint8_t *bytes, size_t count) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(kImuAddress, count) != count) return false;
  for (size_t i = 0; i < count; ++i) bytes[i] = Wire.read();
  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool configureImu() {
  return readRegisters(0x75, &who, 1) && who == kImuAddress &&
         writeRegister(0x6B, 1) && writeRegister(0x1A, 3) &&
         writeRegister(0x19, 9) && writeRegister(0x1B, 0x18) &&
         writeRegister(0x1C, 0x08);
}

int16_t word(const uint8_t *bytes) {
  return int16_t((uint16_t(bytes[0]) << 8) | bytes[1]);
}

void zeroSamples() {
  samples = 0;
  gravitySum = 0;
  for (float &value : sum) value = 0;
}

float wrapDegrees(float angle) {
  return remainderf(angle, 360.0f);
}

void updateImu(uint32_t now) {
  uint32_t elapsed = now - lastSample;
  if (elapsed < kPeriodUs) return;
  lastSample = now;
  uint8_t raw[14];
  if (!readRegisters(0x3B, raw, sizeof(raw))) {
    ++errors;
    ++rejected;
    rejection = Rejection::Read;
    valid = false;
    recovery.reset();
    configured = false;
    if (!calibrated) zeroSamples();
    return;
  }

  lastSuccessfulSample = micros();
  haveSample = true;
  const float ax = word(raw) / 8192.0f;
  const float ay = word(raw + 2) / 8192.0f;
  const float az = word(raw + 4) / 8192.0f;
  temperature = word(raw + 6) / 340.0f + 36.53f;
  // AFS_SEL=3: +/-2000 deg/s,16.4 counts/(deg/s). Check saturation in raw
  // counts so clipped samples remain invalid instead of appearing as real tilt.
  const int16_t gyroRaw[] = {word(raw + 8), word(raw + 10), word(raw + 12)};
  const float gyro[] = {gyroRaw[0] / kGyroCountsPerDps, gyroRaw[1] / kGyroCountsPerDps,
                        gyroRaw[2] / kGyroCountsPerDps};
  const float norm = sqrtf(ax * ax + ay * ay + az * az);
  accelNorm = norm;
  for (size_t i = 0; i < 3; ++i) latestGyro[i] = gyro[i];
  if (!calibrated) {
    const float accel[] = {ax, ay, az};
    bool still = elapsed <= 50000 && norm >= 0.3f && norm <= 2.0f &&
                 !deadman.stable && !gripperButton.stable;
    for (size_t i = 0; i < 3; ++i) {
      if (samples == 0) {
        gyroMin[i] = gyroMax[i] = gyro[i];
        firstAccel[i] = accel[i];
      }
      gyroMin[i] = fminf(gyroMin[i], gyro[i]);
      gyroMax[i] = fmaxf(gyroMax[i], gyro[i]);
      still &= fabsf(gyro[i]) < 30 && gyroMax[i] - gyroMin[i] < 4 &&
               fabsf(accel[i] - firstAccel[i]) < 0.06f;
    }
    if (!still) {
      ++rejected;
      zeroSamples();
      return;
    }
    for (size_t i = 0; i < 3; ++i) sum[i] += gyro[i];
    gravitySum += norm;
    if (++samples < kCalibrationSamples) return;
    for (size_t i = 0; i < 3; ++i) bias[i] = sum[i] / samples;
    gravityReference = gravitySum / samples;
    if (!attitude.initialize(ax, ay, az)) {
      ++rejected;
      rejection = Rejection::Filter;
      zeroSamples();
      return;
    }
    attitude.relativeTilt(roll, pitch);
    calibrated = valid = true;
    rejection = Rejection::None;
    return;
  }
  bool saturated = false;
  for (int16_t axis : gyroRaw)
    saturated |= axis <= -kGyroSaturationRaw || axis >= kGyroSaturationRaw;
  if (elapsed > 50000 || norm < 0.05f || saturated) {
    ++rejected;
    rejection = elapsed > 50000 ? Rejection::SampleGap :
                (norm < 0.05f ? Rejection::Acceleration : Rejection::GyroSaturation);
    valid = false;
    recovery.reset();
    return;
  }
  const float accel[] = {ax, ay, az};
  const bool released = deadman.primed && gripperButton.primed &&
                        !deadman.stable && !gripperButton.stable;
  const bool settled = recovery.update(accel, gyro, bias, norm, elapsed, released);
  if (settled) {
    // Refresh the filter reference even while valid. Raw norm may change with
    // orientation on an uncalibrated module; it must remain visible unchanged.
    gravityReference = recovery.reference();
    recovery.reset();
  }
  if (!valid) {
    if (settled) {
      if (attitude.alignGravity(ax, ay, az)) {
        attitude.relativeTilt(roll, pitch);
        valid = true;
        rejection = Rejection::None;
      } else {
        ++rejected;
        rejection = Rejection::Filter;
      }
    }
    return;
  }
  const bool trustGravity = fabsf(norm - gravityReference) < 0.12f;
  if (!attitude.update(gyro[0] - bias[0], gyro[1] - bias[1], gyro[2] - bias[2],
                       ax, ay, az, elapsed * 1e-6f, trustGravity)) {
    ++rejected;
    rejection = Rejection::Filter;
    valid = false;
    recovery.reset();
    return;
  }
  attitude.relativeTilt(roll, pitch);
  valid = true;
  rejection = Rejection::None;
}

void updateButtons(uint32_t now) {
  deadman.update(now, digitalRead(deadman.pin) == LOW);
  if (!gripperButton.update(now, digitalRead(gripperButton.pin) == LOW)) return;
  if (gripperButton.stable) {
    pressedAt = now;
    havePress = true;
  } else if (havePress) {
    havePress = false;
    if (now - pressedAt >= kLongPressMs) {
      ++longEvents;
    } else {
      ++shortEvents;
    }
  }
}

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  pinMode(deadman.pin, INPUT);
  pinMode(gripperButton.pin, INPUT);
  bootId = esp_random();
  if (!bootId) bootId = 1;
  // USB-only firmware: no WiFi or ESP-NOW transmitter is started.
  Serial.println("role=wearable transport=usb protocol=1 orientation=rotvec_v2 gyro_range_dps=2000 buttons=press_toggle_v1");
  Wire.begin(21, 22, 100000);
  Wire.setTimeOut(20);
  configured = configureImu();
  if (!configured) rejection = Rejection::Read;
  lastSample = micros();
  Serial.printf("who=0x%02X configured=%u\n", who, unsigned(configured));
}

void loop() {
  const uint32_t now = millis();
  updateButtons(now);
  static uint32_t lastProbe = 0;
  if (!configured && now - lastProbe >= 1000) {
    lastProbe = now;
    configured = configureImu();
    if (configured) lastSample = micros();
    else rejection = Rejection::Read;
  }
  if (configured) updateImu(micros());
  static uint32_t previous = 0;
  if (now - previous < kTelemetryPeriodMs) return;
  previous = now;

  const uint32_t ageUs = haveSample ? uint32_t(micros() - lastSuccessfulSample) : UINT32_MAX;
  const uint32_t ageMs = haveSample ? ageUs / 1000 : UINT32_MAX;
  const bool fresh = configured && calibrated && valid && haveSample && ageUs < 50000;
  // Queue a whole bounded frame only when it fits; the IMU loop never waits
  // for UART transmission. This is the only runtime UART producer.
  char frame[kTelemetryMaxBytes + 1];
  const int length = snprintf(frame, sizeof(frame),
      "{\"v\":1,\"att\":2,\"role\":\"wearable\",\"boot\":%lu,\"seq\":%lu,\"ms\":%lu,"
      "\"age_ms\":%lu,\"who\":%u,\"valid\":%u,\"cal\":%u,"
      "\"roll\":%.2f,\"pitch\":%.2f,\"a\":%u,\"b\":%u,\"mode\":%u,"
      "\"short\":%lu,\"long\":%lu,\"norm_g\":%.3f,\"temp_c\":%.2f,"
      "\"errors\":%lu,\"rej\":%u,\"gr\":%u,\"rc\":%lu,\"ac\":%lu,\"bc\":%lu}\n",
      (unsigned long)bootId, (unsigned long)++telemetrySequence, (unsigned long)now,
      (unsigned long)ageMs, unsigned(who), unsigned(fresh), unsigned(samples),
      wrapDegrees(roll), wrapDegrees(pitch),
      unsigned(deadman.primed && deadman.stable),
      unsigned(gripperButton.primed && gripperButton.stable), mode,
      (unsigned long)shortEvents, (unsigned long)longEvents, accelNorm, temperature,
      (unsigned long)errors, unsigned(rejection), unsigned(kGyroRangeDps),
      (unsigned long)rejected, (unsigned long)deadman.presses,
      (unsigned long)gripperButton.presses);
  if (length <= 0 || size_t(length) > kTelemetryMaxBytes ||
      Serial.availableForWrite() < length) return;
  Serial.write(reinterpret_cast<const uint8_t *>(frame), size_t(length));
}
