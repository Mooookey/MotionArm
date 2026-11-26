#ifdef MOTIONARM_HOST_TEST
#include "../tests/RobotHostShim.h"
#else
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <cJSON.h>
#endif
#include <atomic>
#include <MotionArmPeers.h>
#include "RobotController.h"
#include "RobotTelemetry.h"
#include "ManagementOutbox.h"
#ifndef MOTIONARM_POWERED_TRIAL
#define MOTIONARM_POWERED_TRIAL 0
#endif
#if MOTIONARM_POWERED_TRIAL
#include "PoweredTrialProfile.h"
#endif

// Default image permanently locks PWM. The explicit powered-trial build starts
// locked too; only a boot/profile/session-bound operator command can enable it.
namespace {
constexpr unsigned kVersion = MOTIONARM_POWERED_TRIAL ? 3 : 2;
constexpr uint8_t kPca = 0x40;
constexpr uint8_t kPrescale = 121;
constexpr uint32_t kManagerTimeoutMs = 300;
constexpr size_t kLineLimit = 511; // JSON payload + one LF <=512 wire bytes.
robot::Controller controller;
robot::ManagementOutbox managementResponses;
#if MOTIONARM_POWERED_TRIAL
robot::PoweredOutput output(kPoweredTrialProfile);
#endif
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
#if MOTIONARM_POWERED_TRIAL
struct PcaTransport final : robot::OutputTransport {
  bool writeTicks(const uint16_t *ticks) override {
    // CH12..15 are contiguous: a single auto-increment transaction commits all
    // four channels at STOP (MODE2 OCH=0). Partial/failing I2C remains unknown.
    const uint8_t order[4]={3,2,0,1};
    for(unsigned i=0;i<4;++i)if(!ticks[i] || ticks[i]>=4096)return false;
    Wire.beginTransmission(kPca);Wire.write(uint8_t(6+4*12));
    for(unsigned i=0;i<4;++i) {
      const uint16_t n=ticks[order[i]];
      Wire.write(uint8_t(0));Wire.write(uint8_t(0));Wire.write(uint8_t(n));Wire.write(uint8_t(n>>8));
    }
    return Wire.endTransmission()==0;
  }
  bool fullOff() override {
    Wire.beginTransmission(kPca);Wire.write(uint8_t(6+4*12));
    for(unsigned i=0;i<4;++i){Wire.write(uint8_t(0));Wire.write(uint8_t(0));Wire.write(uint8_t(0));Wire.write(uint8_t(0x10));}
    return Wire.endTransmission()==0;
  }
} pcaBus;
bool poweredPcaReadable() {
  uint8_t v;
  if(!readRegister(0,v)||(v&0x30)!=0x20 || !readRegister(1,v)||v!=4 ||
     !readRegister(0xFE,v)||v!=kPrescale)return false;
  const uint8_t order[4]={3,2,0,1};
  for(unsigned i=0;i<4;++i)for(unsigned j=0;j<4;++j) {
    const uint16_t n=output.ticks[order[i]];
    const uint8_t expected=j<2?0:(j==2?uint8_t(n):uint8_t(n>>8));
    if(!readRegister(6+4*(12+i)+j,v)||v!=expected)return false;
  }
  return true;
}
#endif
bool emit(const char *line, size_t length) {
  if (!length || length > kLineLimit || Serial.availableForWrite() < int(length + 1)) {
    ++telemetryDrops; return false;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(line), length); Serial.write('\n'); return true;
}
void queueResponse(const char *line,size_t length) {
  if(!managementResponses.enqueue(line,length))++telemetryDrops;
}
void flushManagementResponses() {
  for(unsigned i=0;i<robot::ManagementOutbox::capacity && !managementResponses.empty();++i) {
    const size_t length=managementResponses.length();
    if(Serial.availableForWrite()<int(length))return;
    Serial.write(reinterpret_cast<const uint8_t*>(managementResponses.data()),length);
    managementResponses.pop();
  }
}
void ack(uint32_t seq, const char *kind, const char *error = nullptr) {
  char line[kLineLimit + 1];
  const int n = snprintf(line, sizeof(line),
    "{\"v\":%u,\"type\":\"%s\",\"seq\":%lu,\"boot_id\":\"%s\",\"request_type\":\"%s\",\"ok\":%s,\"error\":%s%s%s}",
    kVersion,error ? "nack" : "ack", (unsigned long)seq, bootId, kind, boolean(!error),
    error ? "\"" : "", error ? error : "null", error ? "\"" : "");
  if (n > 0 && n <= int(kLineLimit)) queueResponse(line, size_t(n));
}
void stop(const char *reason, uint32_t now) {
  managerActive = false;
#if MOTIONARM_POWERED_TRIAL
  output.prepared=false;
  if(output.committed) {
    memcpy(controller.qOut,output.target,sizeof(controller.qOut));controller.openOut=output.opening;
  }
#endif
  controller.stop(reason, now);
}
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
#if MOTIONARM_POWERED_TRIAL
struct Request { char id[65]{}, payload[513]{};const char *error=nullptr; } requests[32];
char canonicalRequest[513]{};
unsigned requestIndex=0;
bool confirmed(const cJSON *root,const char *name){return cJSON_IsTrue(field(root,name));}
uint32_t fingerprint(const cJSON *root) {
  // Canonicalize every scalar field except the monotonic transport sequence.
  const cJSON *items[24]{};unsigned count=0;
  for(const cJSON *p=root->child;p;p=p->next) {
    if(!strcmp(p->string,"seq"))continue;
    if(count==24)return 0;
    unsigned at=count++;while(at && strcmp(items[at-1]->string,p->string)>0){items[at]=items[at-1];--at;}items[at]=p;
  }
  size_t length=0;canonicalRequest[0]=0;
  for(unsigned i=0;i<count;++i) {
    const auto *p=items[i];char *encoded=cJSON_PrintUnformatted(p);
    if(!encoded)return 0;
    const size_t keyLength=strlen(p->string),valueLength=strlen(encoded);
    const int n=snprintf(canonicalRequest+length,sizeof(canonicalRequest)-length,"%u:%s%u:%s",
      unsigned(keyLength),p->string,unsigned(valueLength),encoded);
    cJSON_free(encoded);
    if(n<0 || size_t(n)>=sizeof(canonicalRequest)-length)return 0;
    length+=size_t(n);
  }
  return 1;
}
// Returns 0=new request, 1=identical accepted request, -1=reject.
int boundRequest(const cJSON *root,uint32_t seq,const char *kind,uint32_t &hash) {
  if(!equal(root,"boot_id",bootId)||!equal(root,"profile_hash",kPoweredTrialProfile.hash)) {
    ack(seq,kind,"STALE_BOOT_OR_PROFILE");return -1;
  }
  const char *id=string(root,"request_id");
  if(!id||!*id||strlen(id)>64 || !(hash=fingerprint(root))){ack(seq,kind,"REQUEST_ID_REQUIRED");return -1;}
  for(const auto &r:requests)if(!strcmp(r.id,id)) {
    const bool same=!strcmp(r.payload,canonicalRequest);
    ack(seq,kind,same?r.error:"REQUEST_ID_REUSED");return same?1:-1;
  }
  if(requestIndex>=32){ack(seq,kind,"REQUEST_CACHE_FULL_RECONNECT");return -1;}
  return 0;
}
void rememberRequest(const cJSON *root,uint32_t hash,const char *error=nullptr) {
  auto &r=requests[requestIndex++];strncpy(r.id,string(root,"request_id"),64);strcpy(r.payload,canonicalRequest);r.error=error;
}
#endif
void dispatch(const cJSON *root, uint32_t now) {
  uint32_t version, seq;
  const char *kind = string(root, "type");
  if (!kind || !integer(root, "v", version) || version != kVersion || !integer(root, "seq", seq)) { ++serialErrors; return; }
  if (!strcmp(kind, "hello") && equal(root, "role", "pc")) {
    managementResponses.clear(); // A new session invalidates unsent old replies.
    stop("MANAGER_SESSION_CHANGED", now);
    newId(sessionId, 's'); haveSession = true; lastCommand = seq; lastInitRequest[0] = 0;
#if MOTIONARM_POWERED_TRIAL
    memset(requests,0,sizeof(requests));requestIndex=0;
#endif
    char line[kLineLimit + 1];
    const int n = snprintf(line, sizeof(line),
      "{\"v\":%u,\"type\":\"hello_ack\",\"seq\":%lu,\"role\":\"robot\",\"protocol_version\":%u,"
#if MOTIONARM_POWERED_TRIAL
      "\"boot_id\":\"%s\",\"session_id\":\"%s\",\"mac\":\"%s\",\"mode\":\"robot_powered\","
      "\"output_locked\":%s,\"fw_version\":\"3.0.0-powered-trial\",\"calibration_id\":\"%s\",\"profile_hash\":\"%s\",\"baud\":230400}",
      kVersion,(unsigned long)seq,kVersion,bootId,sessionId,mac,boolean(output.locked()),kPoweredTrialProfile.hash,kPoweredTrialProfile.hash);
#else
      "\"boot_id\":\"%s\",\"session_id\":\"%s\",\"mac\":\"%s\",\"mode\":\"robot_no_power\","
      "\"output_locked\":true,\"fw_version\":\"2.0.0-no-power\",\"calibration_id\":null}",
      kVersion,(unsigned long)seq,kVersion,bootId,sessionId,mac);
#endif
    if (n > 0 && n <= int(kLineLimit)) queueResponse(line, size_t(n));
    return;
  }
  if (!haveSession || !equal(root, "session_id", sessionId) || !motionarm::newer(seq, lastCommand)) {
    ack(seq, "command", "STALE_SESSION_OR_SEQUENCE"); return;
  }
  if(managementResponses.full() && strcmp(kind,"stop") && strcmp(kind,"heartbeat")) {
    ++serialErrors;return; // Never execute an action that cannot retain its reply.
  }
  lastCommand = seq;
  if (!strcmp(kind, "stop")) {
    stop("SOFTWARE_STOP", now);
    if(managementResponses.full())managementResponses.clear();
    ack(seq, kind); return;
  }
  if (!strcmp(kind, "heartbeat")) {
    if (!managerActive || uint32_t(now - managerAt) >= kManagerTimeoutMs
#if !MOTIONARM_POWERED_TRIAL
        || !controller.enabled
#endif
      ) {
      stop("MANAGER_TIMEOUT", now); ack(seq, kind, "INITIALIZATION_REQUIRED"); return;
    }
    managerAt = now; return; // Periodic heartbeats are not queued RPCs; status confirms the lease.
  }
#if MOTIONARM_POWERED_TRIAL
  if(!strcmp(kind,"prepare") || !strcmp(kind,"enable_output") || !strcmp(kind,"test_joint") ||
      !strcmp(kind,"disable_output") || !strcmp(kind,"init")) {
    uint32_t hash=0;if(boundRequest(root,seq,kind,hash)!=0)return;
    const char *error=nullptr;
    if(!strcmp(kind,"disable_output")) {
      if(!confirmed(root,"supported"))error="SUPPORT_CONFIRMATION_REQUIRED";
      else if(output.stage!=robot::PoweredOutput::Stage::Unknown && (controller.enabled || controller.diagnosticActive || controller.openReq<1 || controller.openOut<1))error="STOP_AND_OPEN_FIRST";
      else {
        stop("OUTPUT_DISABLED",now);
        if(!output.disable(pcaBus,true,now)) {controller.setFault("PCA_OUTPUT_UNKNOWN",now);error="PCA_OUTPUT_UNKNOWN";}
        else if(!lockedPcaReadable()){output.fail();controller.setFault("PCA_OUTPUT_UNKNOWN",now);error="PCA_OUTPUT_UNKNOWN";}
        pcaReady=!error;
      }
    } else if(!strcmp(kind,"prepare")) {
      if(output.stage==robot::PoweredOutput::Stage::Unknown)error="OUTPUT_UNKNOWN_MANUAL_INTERVENTION";
      else if(controller.enabled||controller.diagnosticActive||(output.enabled()&&controller.openOut<1))error="STOP_AND_OPEN_FIRST";
      else if(!controller.valid(now))error="INPUT_INVALID";
      else if(!pcaReady||!wirelessReady|| !(output.enabled()?poweredPcaReadable():lockedPcaReadable()))error="HARDWARE_NOT_READY";
      else {controller.clearFault(now);output.prepared=true;managerActive=true;managerAt=now;controller.managementEvent("OUTPUT_PREPARED",now);}
    } else if(!output.prepared||!managerActive||uint32_t(now-managerAt)>=kManagerTimeoutMs)error="PREPARE_REQUIRED";
    else if(!strcmp(kind,"enable_output")) {
      if(!output.locked())error="OUTPUT_NOT_LOCKED";
      else if(!confirmed(root,"supported")||!confirmed(root,"power_confirmed")||!confirmed(root,"startup_aligned"))error="ONSITE_CONFIRMATION_REQUIRED";
      else if(!pcaReady||!lockedPcaReadable())error="HARDWARE_NOT_READY";
      else if(!output.enable(pcaBus,true,true,true,now)) {controller.setFault("PCA_OUTPUT_UNKNOWN",now);error="PCA_OUTPUT_UNKNOWN";}
      else {controller.seedStartupOutput();controller.reason="PWM_HOLDING_PAUSED";controller.managementEvent("PWM_ENABLED_ONSITE_CONFIRMED",now);}
    } else if(!output.enabled())error="PWM_HOLD_REQUIRED";
    else if(!strcmp(kind,"test_joint")) {
      uint32_t axis;const auto *offset=field(root,"offset_deg");
      if(!integer(root,"axis",axis)||!cJSON_IsNumber(offset)||!isfinite(offset->valuedouble)||
          floor(offset->valuedouble)!=offset->valuedouble||!controller.testJoint(axis,float(offset->valuedouble),now))error="DIAGNOSTIC_REJECTED";
    } else {
      if(controller.enabled||controller.diagnosticActive||controller.openOut<1)error="STOP_AND_OPEN_FIRST";
      else if(!controller.initialize(now))error="INPUT_NOT_STABLE";
    }
    if(error){rememberRequest(root,hash,error);ack(seq,kind,error);return;}
    rememberRequest(root,hash);managerAt=now;ack(seq,kind);return;
  }
#else
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
#endif
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
  if(!managementResponses.empty()){++telemetryDrops;return;}
  robot::TelemetryBatch batch;
  batch.format(controller, now, bootId, ++snapshotId, statusSeq, mac, pcaReady,
    managerActive, receivedPackets, rejectedPackets.load(std::memory_order_relaxed), telemetryDrops, serialErrors
#if MOTIONARM_POWERED_TRIAL
    , &output
#endif
    );
  if (!batch.valid || Serial.availableForWrite() < int(batch.length + 512)) { ++telemetryDrops; return; }
  Serial.write(reinterpret_cast<const uint8_t *>(batch.data), batch.length);
}
void emitEvents() {
  if(!managementResponses.empty())return;
  static robot::Event pending{}; static bool havePending = false;
  if (!havePending) havePending = controller.takeEvent(pending);
  if (!havePending) return;
  char line[kLineLimit + 1];
  const int n = snprintf(line, sizeof(line),
    "{\"v\":%u,\"type\":\"event\",\"seq\":%lu,\"boot_id\":\"%s\",\"t_ms\":%lu,\"event_seq\":%lu,\"code\":\"%s\"}",
    kVersion,(unsigned long)++statusSeq, bootId, (unsigned long)pending.timeMs, (unsigned long)pending.seq, pending.code);
  if (n > 0 && n <= int(kLineLimit) && emit(line, n)) havePending = false;
}
} // namespace
void setup() {
#if MOTIONARM_POWERED_TRIAL
  // ESP32 core auto-selects 1MHz REF_TICK at <=250kbaud. Use the supported
  // 80MHz APB source for this 230400 link without changing wire baud/format.
  Serial.setClockSource(UART_CLK_SRC_APB);
#endif
  Serial.setTxBufferSize(4096); Serial.begin(MOTIONARM_POWERED_TRIAL ? 230400 : 115200); newId(bootId, 'r');
#if MOTIONARM_POWERED_TRIAL
  controller.configureTrial(kPoweredTrialProfile);
#endif
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
  if (managerActive && uint32_t(now - managerAt) >= kManagerTimeoutMs) stop("MANAGER_TIMEOUT", now);
#if MOTIONARM_POWERED_TRIAL
  const bool hadControl=controller.enabled||controller.diagnosticActive;
  const uint32_t wearableBoot=controller.haveInput?controller.input.boot:0;
  float previous[3];memcpy(previous,controller.qOut,sizeof(previous));const float previousOpen=controller.openOut;
#endif
  controller.tick(now
#if MOTIONARM_POWERED_TRIAL
    ,output.enabled()
#endif
    );
  Received packet{};
  if (mailbox && xQueueReceive(mailbox, &packet, 0) == pdTRUE) {
    ++receivedPackets; controller.receive(packet.frame, packet.arrived, now);
  }
#if MOTIONARM_POWERED_TRIAL
  if(managerActive && (!controller.valid(now) || (wearableBoot && wearableBoot!=controller.input.boot) ||
      (hadControl && !controller.enabled && !controller.diagnosticActive && strcmp(controller.reason,"DIAGNOSTIC_COMPLETE"))))
    stop(controller.valid(now)?controller.reason:"INPUT_INVALID",now);
#endif
  flushManagementResponses();readSerial();flushManagementResponses();now = millis();
#if MOTIONARM_POWERED_TRIAL
  if(managerActive && !controller.valid(now))
    stop(controller.valid(now)?controller.reason:"INPUT_INVALID",now);
  if(output.enabled()) {
    if(!output.submit(pcaBus,controller.qOut,controller.openOut,now)) {
      memcpy(controller.qOut,output.target,sizeof(controller.qOut));controller.openOut=output.opening;
      stop("PCA_OUTPUT_UNKNOWN",now);controller.setFault("PCA_OUTPUT_UNKNOWN",now);pcaReady=false;
    }
  } else {
    // No speculative progress is presented as committed output while disabled
    // or after I2C uncertainty. Timers/request-open still run locally.
    memcpy(controller.qOut,previous,sizeof(previous));controller.openOut=previousOpen;
  }
#endif
  static uint32_t lastHealth = now, lastReport = now;
  if (uint32_t(now - lastHealth) >= 500) {
    lastHealth = now;
#if MOTIONARM_POWERED_TRIAL
    if(output.stage!=robot::PoweredOutput::Stage::Unknown) {
      if(!(output.enabled()?poweredPcaReadable():lockedPcaReadable())) {
        output.fail();pcaReady=false;stop("PCA_OUTPUT_UNKNOWN",now);controller.setFault("PCA_OUTPUT_UNKNOWN",now);
      } else pcaReady=true;
    }
#else
    if (!lockedPcaReadable()) {
      pcaReady = false; stop("PCA_LOCK_OR_I2C_FAULT", now); controller.setFault("PCA_LOCK_OR_I2C_FAULT", now);
      initializeLockedPca(); // Retry FULL_OFF only; never restores authority.
    } else pcaReady = true;
#endif
  }
  flushManagementResponses();emitEvents();
  if (uint32_t(now - lastReport) >= 200) { lastReport = now; telemetry(now); }
  delay(1);
}
