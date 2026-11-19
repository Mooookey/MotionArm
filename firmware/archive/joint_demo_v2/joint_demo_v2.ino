#include <Wire.h>

constexpr uint8_t kAddress = 0x40;
constexpr int kCenterUs = 1500;
constexpr int kBaseHomeUs = 1610;
constexpr int kShoulderHomeUs = 1630;
constexpr int kElbowHomeUs = 1570;
constexpr int kGripperOpenUs = 1130;
constexpr int kLoadedStartMinUs = 1450;
constexpr int kLoadedStartMaxUs = 1550;
constexpr int kProbeMinUs = 1000;
constexpr int kProbeMaxUs = 2000;
constexpr int kMaxStepUs = 150;
constexpr uint32_t kGripperLimitMs = 700;
constexpr uint32_t kLoadedProbeMs = 60;

struct Joint {
  int channel;
  const char *name;
};
const Joint joints[] = {
  { 12, "gripper" },
  { 13, "elbow" },
  { 14, "base" },
  { 15, "shoulder" },
};
const Joint *selected = nullptr;
bool ready = false;
bool outputActive = false;
bool shortProbe = false;
int currentUs = kCenterUs;
uint32_t outputStarted = 0;
int parkedUs[16] = {};

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool writePulse(int channel, int us) {
  const uint16_t ticks = us == 0 ? 4096 : (us * 25 + 61) / 122;
  Wire.beginTransmission(kAddress);
  Wire.write(uint8_t(6 + 4 * channel));
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(ticks));
  Wire.write(uint8_t(ticks >> 8));
  return Wire.endTransmission() == 0;
}

#include "ArmDemo.h"

bool initializePca() {
  if (!writeReg(0x00, 0x30)) return false;
  for (int ch = 0; ch < 16; ++ch) {
    if (!writePulse(ch, 0)) return false;
  }
  if (!writeReg(0xFE, 121) || !writeReg(0x01, 4) || !writeReg(0x00, 0x20)) return false;
  delay(5);
  return writeReg(0x00, 0xA0);
}

void status() {
  if (demo::ownsOutputs()) Serial.printf("DEMO: %s\n", demo::name());
  bool parked = false;
  for (const Joint &joint : joints) {
    if (!parkedUs[joint.channel]) continue;
    Serial.printf("PARKED CH%d %s=%dus\n", joint.channel, joint.name, parkedUs[joint.channel]);
    parked = true;
  }
  if (selected == nullptr) {
    Serial.println(parked ? "HOLD: no joint selected" : "IDLE: no joint selected");
    return;
  }
  Serial.printf("CH%d %s command=%dus target=%dus PWM=%s probe=%d..%dus\n",
                selected->channel, selected->name, currentUs, currentUs,
                outputActive ? "ON" : "OFF", kProbeMinUs, kProbeMaxUs);
}

void releaseOutput() {
  if (!outputActive) return;
  if (writePulse(selected->channel, 0)) {
    outputActive = false;
    Serial.println("PWM OFF ACK");
  } else {
    ready = false;
    Serial.println("PWM OFF FAILED. Support arm and cut servo power.");
  }
}

void finishShortProbe() {
  releaseOutput();
  if (outputActive) return;
  shortProbe = false;
  selected = nullptr;
  status();
}

void releaseParked() {
  for (const Joint &joint : joints) {
    const int ch = joint.channel;
    if (!parkedUs[ch]) continue;
    if (writePulse(ch, 0)) {
      parkedUs[ch] = 0;
      Serial.printf("PWM OFF ACK CH%d\n", ch);
    } else {
      ready = false;
      Serial.printf("PWM OFF FAILED CH%d. Support arm and cut servo power.\n", ch);
    }
  }
}

bool applyPulse(int us) {
  if (writePulse(selected->channel, us)) return true;
  ready = false;
  Serial.println("I2C WRITE ERROR. Last PWM may remain active. Support arm and cut servo power.");
  return false;
}

void handle(const char *line) {
  if (!strcmp(line, "STATUS")) {
    status();
    return;
  }
  if (!strcmp(line, "STOP")) {
    if (demo::ownsOutputs()) {
      demo::stop(millis(), true);
      status();
      return;
    }
    if (shortProbe) {
      finishShortProbe();
      return;
    }
    if (selected && selected->channel == 12) releaseOutput();
    status();
    return;
  }
  if (!strcmp(line, "OFF")) {
    demo::reset();
    releaseOutput();
    releaseParked();
    if (!outputActive) {
      selected = nullptr;
      shortProbe = false;
    }
    status();
    return;
  }
  if (strncmp(line, "DEMO ", 5) == 0) {
    demo::command(line + 5, millis());
    return;
  }
  if (demo::ownsOutputs()) {
    Serial.println("DEMO owns outputs. Support both arms and send OFF before manual control.");
    return;
  }
  if (!ready) {
    Serial.println("PCA9685 unavailable. Check wiring and reset.");
    return;
  }
  if (shortProbe) {
    Serial.println("Wait for the short probe to finish.");
    return;
  }

  if (!strcmp(line, "PARK")) {
    if (!selected || !outputActive || selected->channel == 12) {
      Serial.println("PARK requires an active non-gripper joint.");
      return;
    }
    parkedUs[selected->channel] = currentUs;
    outputActive = false;
    selected = nullptr;
    status();
    return;
  }
  if (!strcmp(line, "NEXT")) {
    if (!selected || selected->channel != 12 || outputActive) {
      Serial.println("NEXT requires the gripper test to have timed out.");
      return;
    }
    selected = nullptr;
    status();
    return;
  }

  int value;
  int startUs;
  char extra;
  if (sscanf(line, "RESUME %d %c", &value, &extra) == 1) {
    if (selected) {
      Serial.println("PARK the current joint first.");
      return;
    }
    for (const Joint &joint : joints) {
      if (joint.channel != value || !parkedUs[value]) continue;
      selected = &joint;
      currentUs = parkedUs[value];
      parkedUs[value] = 0;
      outputActive = true;
      status();
      return;
    }
    Serial.println("That joint is not parked.");
    return;
  }
  const bool probe = sscanf(line, "PROBE %d %d %c", &value, &startUs, &extra) == 2;
  const bool fixture = !probe && sscanf(line, "FIXTURE %d %d %c", &value, &startUs, &extra) == 2;
  if (probe || fixture || sscanf(line, "ARM %d %c", &value, &extra) == 1) {
    if (!probe && !fixture) startUs = value == 14 ? kBaseHomeUs : kGripperOpenUs;
    const bool loaded = value == 13 || value == 15;
    if (loaded != (fixture || probe)) {
      Serial.println("Use PROBE/FIXTURE <13|15> <us> with external support; ARM for CH12/14.");
      return;
    }
    if (selected) {
      Serial.println("PARK the current joint or use OFF with support.");
      return;
    }
    const bool knownHome = fixture && ((value == 15 && startUs == kShoulderHomeUs) ||
                                      (value == 13 && startUs == kElbowHomeUs));
    if (loaded && !knownHome && (startUs < kLoadedStartMinUs || startUs > kLoadedStartMaxUs)) {
      Serial.printf("Probe %d..%d us, or use FIXTURE 15 %d / FIXTURE 13 %d.\n",
                    kLoadedStartMinUs, kLoadedStartMaxUs, kShoulderHomeUs, kElbowHomeUs);
      return;
    }
    for (const Joint &joint : joints) {
      if (joint.channel == value) {
        if (parkedUs[value]) {
          Serial.println("Use RESUME to select a parked joint.");
          return;
        }
        selected = &joint;
      }
    }
    if (!selected) {
      Serial.println("Invalid channel or starting pulse outside probe limits.");
      return;
    }
    outputActive = true;
    shortProbe = probe;
    outputStarted = millis();
    if (!applyPulse(startUs)) {
      releaseOutput();
      return;
    }
    currentUs = startUs;
    status();
    return;
  }
  if (sscanf(line, "STEP %d %c", &value, &extra) == 1) {
    if (!selected) {
      Serial.println("Select a joint first.");
      return;
    }
    if (value < -kMaxStepUs || value > kMaxStepUs || value == 0) {
      Serial.printf("Use STEP -%d..%d, excluding zero.\n", kMaxStepUs, kMaxStepUs);
      return;
    }
    const int requested = currentUs + value;
    if (requested < kProbeMinUs || requested > kProbeMaxUs) {
      Serial.println("Probe limit reached. STATUS shows the current limits.");
      return;
    }
    if (!outputActive) {
      if (selected->channel != 12) return;
      outputActive = true;
      outputStarted = millis();
    }
    if (!applyPulse(requested)) {
      if (selected->channel == 12) releaseOutput();
      return;
    }
    currentUs = requested;
    status();
    return;
  }
  Serial.println("Use ARM, PROBE, FIXTURE, STEP, PARK, RESUME, NEXT, DEMO, STATUS, STOP or OFF.");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  ready = initializePca();
  Serial.println(ready ? "READY: PCA9685 0x40, all PWM off" : "ERROR: PCA9685 initialization failed");
  Serial.printf("Support the arm. ARM 14 applies %dus immediately.\n", kBaseHomeUs);
  Serial.printf("STEP directly applies a change up to %d us. OFF releases torque.\n", kMaxStepUs);
  Serial.println("Support and align to ready pose (1610/1030/1050 us): DEMO ARM, APPROACH, DRY, GRIP. Demo grip limit 3000 ms.");
}

void loop() {
  static char line[40];
  static size_t size = 0;
  static bool overflow = false;
  for (int i = 0; i < 32 && Serial.available(); ++i) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (overflow) Serial.println("Command too long");
      else if (size) {
        line[size] = '\0';
        handle(line);
      }
      size = 0;
      overflow = false;
    } else if (size + 1 < sizeof(line)) {
      line[size++] = c;
    } else {
      overflow = true;
    }
  }
  const uint32_t now = millis();
  demo::update(now);
  if (!ready || !selected || !outputActive) return;
  if (shortProbe && now - outputStarted >= kLoadedProbeMs) {
    finishShortProbe();
    return;
  }
  if (selected->channel == 12 && now - outputStarted >= kGripperLimitMs) {
    releaseOutput();
    return;
  }
}
