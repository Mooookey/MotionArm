#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <cJSON.h>
#include <atomic>
#include <MotionArmPeers.h>
#include "RobotController.h"
#include "RobotTelemetry.h"

// This image has NO powered output implementation and NO unlock command.
// FULL_OFF here is a commissioning lock, never a way to release a loaded arm.
namespace {
constexpr uint8_t kPca = 0x40;
constexpr uint8_t kPrescale = 121;
constexpr uint32_t kManagerTimeoutMs = 300;
constexpr size_t kLineLimit = 512;
robot::Controller controller;
struct Received { motionarm::Frame frame; uint32_t arrived; int rssi; };
QueueHandle_t mailbox = nullptr;
bool pcaReady = false, haveSession = false, managerActive = false, wirelessReady = false;
uint32_t managerAt = 0, lastCommand = 0, statusSeq = 0, snapshotId = 0;
std::atomic<uint32_t> rejectedPackets{0};
uint32_t receivedPackets = 0, telemetryDrops = 0, serialErrors = 0;
char bootId[16]{}, sessionId[16]{}, lastInitRequest[65]{}, mac[18]{};
const char *boolean(bool value) { return value ? "true" : "false"; }
void newId(char *out, char prefix) { snprintf(out, 16, "%c-%08lx", prefix, (unsigned long)esp_random()); }
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kPca); Wire.write(reg); Wire.write(value); return Wire.endTransmission() == 0;
}
bool readRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(kPca); Wire.write(reg);
  if (Wire.endTransmission(false) || Wire.requestFrom(kPca, 1) != 1) return false;
  value = Wire.read(); return true;
}
bool initializeLockedPca() {
  if (!writeRegister(0, 0x30)) return false;
  for (uint8_t ch = 0; ch < 16; ++ch) {
    Wire.beginTransmission(kPca); Wire.write(uint8_t(6 + 4 * ch));
    Wire.write(uint8_t(0)); Wire.write(uint8_t(0));
    Wire.write(uint8_t(0)); Wire.write(uint8_t(0x10)); // FULL_OFF only.
    if (Wire.endTransmission()) return false;
  }
  if (!writeRegister(0xFE, kPrescale) || !writeRegister(1, 4) || !writeRegister(0, 0x20)) return false;
  delay(5); return writeRegister(0, 0xA0);
}
bool lockedPcaReadable() {
  uint8_t value;
  if (!readRegister(0, value) || (value & 0x30) != 0x20 || !readRegister(1, value) || value != 4 ||
      !readRegister(0xFE, value) || value != kPrescale) return false;
  for (uint8_t ch = 12; ch <= 15; ++ch) for (uint8_t i = 0; i < 4; ++i)
    if (!readRegister(6 + 4 * ch + i, value) || value != (i == 3 ? 0x10 : 0)) return false;
  return true;
}
bool emit(const char *line, size_t length) {
  if (!length || length > kLineLimit || Serial.availableForWrite() < int(length + 1)) {
    ++telemetryDrops; return false;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(line), length); Serial.write('\n'); return true;
}
void ack(uint32_t seq, const char *kind, const char *error = nullptr) {
  char line[kLineLimit + 1];
  const int n = snprintf(line, sizeof(line),
    "{\"v\":2,\"type\":\"%s\",\"seq\":%lu,\"boot_id\":\"%s\",\"request_type\":\"%s\",\"ok\":%s,\"error\":%s%s%s}",
    error ? "nack" : "ack", (unsigned long)seq, bootId, kind, boolean(!error),
    error ? "\"" : "", error ? error : "null", error ? "\"" : "");
  if (n > 0 && n <= int(kLineLimit)) emit(line, size_t(n));
}
void stop(const char *reason, uint32_t now) { managerActive = false; controller.stop(reason, now); }
void onReceived(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  if (!info || !info->src_addr || memcmp(info->src_addr, motionarm::wearableMac, 6)) return;
  Received packet{};
  if (!motionarm::decode(data, length, packet.frame)) { rejectedPackets.fetch_add(1, std::memory_order_relaxed); return; }
  packet.arrived = millis(); packet.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  xQueueOverwrite(mailbox, &packet);
}
const cJSON *field(const cJSON *root, const char *key) { return cJSON_GetObjectItemCaseSensitive(root, key); }
const char *string(const cJSON *root, const char *key) {
  const auto *item = field(root, key); return cJSON_IsString(item) ? item->valuestring : nullptr;
}
bool equal(const cJSON *root, const char *key, const char *value) {
  const auto *item = string(root, key); return item && !strcmp(item, value);
}
bool integer(const cJSON *root, const char *key, uint32_t &out) {
  const auto *item = field(root, key);
  if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble) || item->valuedouble < 0 ||
      item->valuedouble > UINT32_MAX || floor(item->valuedouble) != item->valuedouble) return false;
  out = uint32_t(item->valuedouble); return true;
}
void dispatch(const cJSON *root, uint32_t now) {
  uint32_t version, seq;
  const char *kind = string(root, "type");
  if (!kind || !integer(root, "v", version) || version != 2 || !integer(root, "seq", seq)) { ++serialErrors; return; }
  if (!strcmp(kind, "hello") && equal(root, "role", "pc")) {
    stop("MANAGER_SESSION_CHANGED", now);
    newId(sessionId, 's'); haveSession = true; lastCommand = seq; lastInitRequest[0] = 0;
    char line[kLineLimit + 1];
    const int n = snprintf(line, sizeof(line),
      "{\"v\":2,\"type\":\"hello_ack\",\"seq\":%lu,\"role\":\"robot\",\"protocol_version\":2,"
      "\"boot_id\":\"%s\",\"session_id\":\"%s\",\"mac\":\"%s\",\"mode\":\"robot_no_power\","
      "\"output_locked\":true,\"fw_version\":\"2.0.0-no-power\",\"calibration_id\":null}",
      (unsigned long)seq, bootId, sessionId, mac);
    if (n > 0 && n <= int(kLineLimit)) emit(line, n);
    return;
  }
  if (!haveSession || !equal(root, "session_id", sessionId) || !motionarm::newer(seq, lastCommand)) {
    ack(seq, "command", "STALE_SESSION_OR_SEQUENCE"); return;
  }
  lastCommand = seq;
  if (!strcmp(kind, "stop")) { stop("SOFTWARE_STOP", now); ack(seq, kind); return; }
  if (!strcmp(kind, "heartbeat")) {
    if (!managerActive || uint32_t(now - managerAt) >= kManagerTimeoutMs || !controller.enabled) {
      stop("MANAGER_TIMEOUT", now); ack(seq, kind, "INITIALIZATION_REQUIRED"); return;
    }
    managerAt = now; return; // Periodic heartbeats are not queued RPCs; status confirms the lease.
  }
  if (!strcmp(kind, "init")) {
    const char *request = string(root, "request_id");
    if (!request || !*request || strlen(request) > 64) { ack(seq, kind, "REQUEST_ID_REQUIRED"); return; }
    if (!strcmp(request, lastInitRequest)) { ack(seq, kind); return; }
    if (!pcaReady || !lockedPcaReadable() || !wirelessReady) { ack(seq, kind, "HARDWARE_NOT_READY"); return; }
    if (controller.enabled || controller.openOut < 1.0f) { ack(seq, kind, "STOP_AND_OPEN_FIRST"); return; }
    controller.clearFault(now);
    if (!controller.initialize(now)) { ack(seq, kind, "INPUT_NOT_STABLE"); return; }
    strncpy(lastInitRequest, request, sizeof(lastInitRequest) - 1);
    managerAt = now; managerActive = true; ack(seq, kind); return;
  }
  ack(seq, "command", "POWERED_OUTPUT_UNAVAILABLE");
}
void parse(const char *line, uint32_t now) {
  // Scalar-only management messages; bound nesting before cJSON recurses.
  bool inString = false, escape = false; unsigned opens = 0;
  for (const char *p = line; *p; ++p) {
    if (inString) {
      if (escape) { if (*p == 'u' && !strncmp(p, "u0000", 5)) { ++serialErrors; return; } escape = false; }
      else if (*p == '\\') escape = true;
      else if (*p == '"') inString = false;
    } else if (*p == '"') inString = true;
    else if (*p == '[' || (*p == '{' && ++opens > 1)) { ++serialErrors; return; }
  }
  cJSON *root = cJSON_ParseWithOpts(line, nullptr, true);
  bool ok = cJSON_IsObject(root);
  if (ok) for (const auto *item = root->child; item; item = item->next)
    if (!item->string || field(root, item->string) != item || cJSON_IsObject(item) || cJSON_IsArray(item)) ok = false;
  if (ok) dispatch(root, now); else ++serialErrors;
  cJSON_Delete(root);
}
void readSerial() {
  static char line[kLineLimit + 1]; static size_t length = 0; static bool discarded = false;
  for (unsigned count = 0; count < 96 && Serial.available(); ++count) {
    const char ch = Serial.read();
    if (ch == '\n') {
      if (length && !discarded) { line[length] = 0; parse(line, millis()); }
      else if (discarded) ++serialErrors;
      length = 0; discarded = false;
    } else if (ch && !discarded && length < kLineLimit) line[length++] = ch;
    else discarded = true;
  }
}
void telemetry(uint32_t now) {
  robot::TelemetryBatch batch;
  batch.format(controller, now, bootId, ++snapshotId, statusSeq, mac, pcaReady,
    managerActive, receivedPackets, rejectedPackets.load(std::memory_order_relaxed), telemetryDrops, serialErrors);
  if (!batch.valid || Serial.availableForWrite() < int(batch.length + 512)) { ++telemetryDrops; return; }
  Serial.write(reinterpret_cast<const uint8_t *>(batch.data), batch.length);
}
void emitEvents() {
  static robot::Event pending{}; static bool havePending = false;
  if (!havePending) havePending = controller.takeEvent(pending);
  if (!havePending) return;
  char line[kLineLimit + 1];
  const int n = snprintf(line, sizeof(line),
    "{\"v\":2,\"type\":\"event\",\"seq\":%lu,\"boot_id\":\"%s\",\"t_ms\":%lu,\"event_seq\":%lu,\"code\":\"%s\"}",
    (unsigned long)++statusSeq, bootId, (unsigned long)pending.timeMs, (unsigned long)pending.seq, pending.code);
  if (n > 0 && n <= int(kLineLimit) && emit(line, n)) havePending = false;
}
} // namespace
void setup() {
  Serial.setTxBufferSize(4096); Serial.begin(115200); newId(bootId, 'r');
  Wire.begin(21, 22, 100000); Wire.setTimeOut(4);
  pcaReady = initializeLockedPca() && lockedPcaReadable();
  if (!pcaReady) controller.setFault("PCA_NOT_READY", millis());
  WiFi.mode(WIFI_STA);
  uint8_t staMac[6]{};
  const bool identityReady = esp_wifi_get_mac(WIFI_IF_STA, staMac) == ESP_OK;
  snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X", staMac[0], staMac[1], staMac[2], staMac[3], staMac[4], staMac[5]);
  // Arduino's netif MAC getter can still return zero before its STA event runs.
  // Read the driver identity directly, and fail closed on a different board.
  wirelessReady = (mailbox = xQueueCreate(1, sizeof(Received))) && identityReady && !memcmp(staMac, motionarm::robotMac, 6) &&
    esp_wifi_set_channel(motionarm::channel, WIFI_SECOND_CHAN_NONE) == ESP_OK &&
    esp_now_init() == ESP_OK && esp_now_register_recv_cb(onReceived) == ESP_OK;
  if (!wirelessReady) controller.setFault("WIRELESS_INIT_OR_IDENTITY", millis());
}
void loop() {
  uint32_t now = millis();
  controller.tick(now);
  if (managerActive && uint32_t(now - managerAt) >= kManagerTimeoutMs) stop("MANAGER_TIMEOUT", now);
  Received packet{};
  if (mailbox && xQueueReceive(mailbox, &packet, 0) == pdTRUE) {
    ++receivedPackets; controller.receive(packet.frame, packet.arrived, now);
  }
  readSerial(); now = millis();
  static uint32_t lastHealth = now, lastReport = now;
  if (uint32_t(now - lastHealth) >= 500) {
    lastHealth = now;
    if (!lockedPcaReadable()) {
      pcaReady = false; stop("PCA_LOCK_OR_I2C_FAULT", now); controller.setFault("PCA_LOCK_OR_I2C_FAULT", now);
      initializeLockedPca(); // Retry FULL_OFF only; never restores authority.
    } else pcaReady = true;
  }
  emitEvents();
  if (uint32_t(now - lastReport) >= 200) { lastReport = now; telemetry(now); }
  delay(1);
}
