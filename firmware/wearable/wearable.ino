#include <Wire.h>
#include <WiFi.h>
#include <math.h>
#include <atomic>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_random.h>
#include <MotionArmLink.h>
#include <MotionArmPeers.h>

constexpr uint8_t kImuAddress = 0x68;
constexpr uint32_t kPeriodUs = 10000;
constexpr uint16_t kCalibrationSamples = 250;
constexpr uint32_t kDebounceMs = 30;
constexpr uint32_t kLongPressMs = 700;

struct Button {
  uint8_t pin;
  bool raw = false;
  bool stable = false;
  bool primed = false;
  uint32_t changed = 0;

  bool update(uint32_t now) {
    bool pressed = digitalRead(pin) == LOW;
    if (pressed != raw) {
      raw = pressed;
      changed = now;
    }
    if (now - changed < kDebounceMs) return false;
    if (!primed) {
      primed = true;
      stable = raw;
      return false;
    }
    if (stable == raw) return false;
    stable = raw;
    return true;
  }
};

Button deadman{25}, modeButton{26};
bool havePress = false;
uint32_t pressedAt = 0, shortEvents = 0, longEvents = 0;
unsigned mode = 0;
uint8_t who = 0;
bool configured = false, valid = false, calibrated = false;
uint32_t errors = 0, rejected = 0, lastSample = 0;
uint16_t samples = 0;
uint8_t recoverySamples = 0;
float sum[3] = {}, bias[3] = {};
float gyroMin[3] = {}, gyroMax[3] = {}, latestGyro[3] = {};
float firstAccel[3] = {}, accelNorm = 0;
float gravitySum = 0, gravityReference = 1;
float roll = 0, pitch = 0, rollZero = 0, pitchZero = 0;
float accelRoll = 0, accelPitch = 0, temperature = 0;
uint32_t bootId = 0, wirelessSequence = 0;
uint8_t gripperEvent = 0;
bool radioReady = false;
std::atomic<bool> sendPending{false};
std::atomic<uint32_t> delivered{0}, failed{0};

void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) ++delivered;
  else ++failed;
  sendPending = false;
}

bool setupRadio() {
  if (esp_wifi_set_channel(motionarm::channel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK || esp_now_register_send_cb(onSent) != ESP_OK) return false;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, motionarm::robotMac, sizeof(peer.peer_addr));
  peer.channel = motionarm::channel;
  peer.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void sendFrame(uint32_t now) {
  static uint32_t lastSent = 0;
  if (!radioReady || now - lastSent < 40 || sendPending.exchange(true)) return;
  lastSent = now;
  motionarm::Frame frame{};
  const bool fresh = valid && uint32_t(micros() - lastSample) < 50000;
  frame.flags = (fresh ? motionarm::imuValid | motionarm::neutral : 0) |
                (deadman.primed && deadman.stable ? motionarm::deadman : 0);
  frame.boot = bootId;
  frame.seq = ++wirelessSequence;
  frame.uptime = now;
  frame.roll = lroundf(remainderf(roll - rollZero, 360.0f) * 100);
  frame.pitch = lroundf(remainderf(pitch - pitchZero, 360.0f) * 100);
  frame.mode = mode;
  frame.gripperEvent = gripperEvent;
  uint8_t bytes[motionarm::frameSize];
  motionarm::encode(frame, bytes);
  if (esp_now_send(motionarm::robotMac, bytes, sizeof(bytes)) != ESP_OK) {
    ++failed;
    sendPending = false;
  }
}

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
         writeRegister(0x19, 9) && writeRegister(0x1B, 0x08) &&
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
    valid = false;
    recoverySamples = 0;
    configured = false;
    if (!calibrated) zeroSamples();
    return;
  }

  const float ax = word(raw) / 8192.0f;
  const float ay = word(raw + 2) / 8192.0f;
  const float az = word(raw + 4) / 8192.0f;
  temperature = word(raw + 6) / 340.0f + 36.53f;
  const float gyro[] = {word(raw + 8) / 65.5f, word(raw + 10) / 65.5f,
                        word(raw + 12) / 65.5f};
  const float norm = sqrtf(ax * ax + ay * ay + az * az);
  accelNorm = norm;
  for (size_t i = 0; i < 3; ++i) latestGyro[i] = gyro[i];
  accelRoll = atan2f(ay, az) * RAD_TO_DEG;
  accelPitch = atan2f(-ax, hypotf(ay, az)) * RAD_TO_DEG;
  const bool nearGravity = fabsf(norm - gravityReference) < 0.12f;
  if (!calibrated) {
    const float accel[] = {ax, ay, az};
    bool still = elapsed <= 50000 && norm >= 0.5f && norm <= 1.5f &&
                 !deadman.stable && !modeButton.stable;
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
    roll = rollZero = accelRoll;
    pitch = pitchZero = accelPitch;
    calibrated = valid = true;
    return;
  }
  if (elapsed > 50000 || norm < 0.05f || fabsf(gyro[0]) >= 499 ||
      fabsf(gyro[1]) >= 499 || fabsf(gyro[2]) >= 499) {
    ++rejected;
    valid = false;
    recoverySamples = 0;
    return;
  }
  if (!valid) {
    const bool settled = nearGravity && !deadman.stable && !modeButton.stable &&
                         fabsf(gyro[0] - bias[0]) < 5 && fabsf(gyro[1] - bias[1]) < 5 &&
                         fabsf(gyro[2] - bias[2]) < 5;
    if (!settled) recoverySamples = 0;
    else if (++recoverySamples >= 25) {
      roll = accelRoll;
      pitch = accelPitch;
      valid = true;
    }
    return;
  }
  const float dt = elapsed * 1e-6f;
  const float correction = nearGravity ? dt / (0.49f + dt) : 0;
  roll = wrapDegrees(roll + (gyro[0] - bias[0]) * dt);
  pitch = wrapDegrees(pitch + (gyro[1] - bias[1]) * dt);
  roll = wrapDegrees(roll + correction * wrapDegrees(accelRoll - roll));
  pitch = wrapDegrees(pitch + correction * wrapDegrees(accelPitch - pitch));
  valid = true;
}

void updateButtons(uint32_t now) {
  deadman.update(now);
  if (!modeButton.update(now)) return;
  if (modeButton.stable) {
    pressedAt = now;
    havePress = true;
  } else if (havePress) {
    havePress = false;
    if (now - pressedAt >= kLongPressMs) {
      mode ^= 1;
      ++longEvents;
    } else {
      ++shortEvents;
      if (valid && deadman.primed && deadman.stable) ++gripperEvent;
    }
  }
}

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  pinMode(deadman.pin, INPUT);
  pinMode(modeButton.pin, INPUT);
  WiFi.mode(WIFI_STA);
  bootId = esp_random();
  if (!bootId) bootId = 1;
  radioReady = setupRadio();
  Serial.printf("role=wearable mac=%s\n", WiFi.macAddress().c_str());
  Wire.begin(21, 22, 100000);
  Wire.setTimeOut(20);
  configured = configureImu();
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
    else Serial.printf("IMU probe failed: expected WHO_AM_I=0x68, got 0x%02X\n", who);
  }
  if (configured) updateImu(micros());
  sendFrame(now);
  static uint32_t previous = 0;
  if (now - previous < 200) return;
  previous = now;
  if (Serial.availableForWrite() < 600) return;
  Serial.printf("{\"role\":\"wearable\",\"ms\":%lu,\"who\":%u,\"valid\":%u,"
                "\"cal\":%u,\"raw_roll\":%.2f,\"raw_pitch\":%.2f,"
                "\"roll\":%.2f,\"pitch\":%.2f,\"neutral\":[%.2f,%.2f],"
                "\"bias\":[%.3f,%.3f,%.3f],\"gyro\":[%.3f,%.3f,%.3f],"
                "\"norm_g\":%.3f,\"gravity_ref_g\":%.3f,\"accel_correction\":%u,\"temp_c\":%.2f,"
                "\"gpio\":[%d,%d],\"a\":%u,\"b\":%u,\"short\":%lu,\"long\":%lu,"
                "\"mode\":%u,\"errors\":%lu,\"rejected\":%lu,"
                "\"radio\":%u,\"tx_ok\":%lu,\"tx_failed\":%lu}\n",
                (unsigned long)now, who, unsigned(valid), samples, accelRoll, accelPitch,
                wrapDegrees(roll - rollZero), wrapDegrees(pitch - pitchZero), rollZero, pitchZero,
                bias[0], bias[1], bias[2], latestGyro[0], latestGyro[1], latestGyro[2],
                accelNorm, gravityReference,
                unsigned(valid && fabsf(accelNorm - gravityReference) < 0.12f),
                temperature, digitalRead(25), digitalRead(26),
                unsigned(deadman.primed && deadman.stable), unsigned(modeButton.primed && modeButton.stable),
                (unsigned long)shortEvents, (unsigned long)longEvents, mode,
                (unsigned long)errors, (unsigned long)rejected, unsigned(radioReady),
                (unsigned long)delivered.load(), (unsigned long)failed.load());
}
