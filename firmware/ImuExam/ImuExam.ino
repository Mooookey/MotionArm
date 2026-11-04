#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "ImuMath.h"

#ifndef IMU_SERIAL_BAUD
#define IMU_SERIAL_BAUD 460800
#endif
static constexpr uint8_t IMU_ADDRESS = 0x68;
static constexpr uint32_t PERIOD_US = 5000;
enum class State { Calibrating, Idle, Recording, Draining, Error };
enum Event : uint32_t { META_CHANGED=1, INIT_FAILED=2, READ_FAILED=4, STALE_DATA=8, CAL_MOVED=16, SENSOR_LOST=32 };
struct WireFrame { uint8_t bytes[90]; };
struct Counters {
  uint32_t generated=0, sent=0, sampleFail=0, stale=0, missed=0, queueDrops=0, gaps=0;
  uint64_t firstUs=0, lastUs=0;
  uint32_t dtCount=0;
  double dtMean=0, dtM2=0;
  uint32_t dtMin=0, dtMax=0;
  void addTime(uint64_t t) {
    if (!firstUs) firstUs=t;
    if (lastUs) {
      uint32_t d=(uint32_t)(t-lastUs);
      ++dtCount; double delta=d-dtMean; dtMean+=delta/dtCount; dtM2+=delta*(d-dtMean);
      if (dtCount==1 || d<dtMin) dtMin=d;
      if (d>dtMax) dtMax=d;
    }
    lastUs=t;
  }
};
struct Calibration {
  uint32_t count=0;
  uint64_t firstUs=0;
  double mean[6]={}, m2[6]={};
  float first[6]={};
  const char *lastResetReason="startup";
  const char *phase="waiting";
  uint32_t resets=0;
  void reset(const char *reason="command") {
    uint32_t previousResets=resets;
    *this=Calibration();resets=previousResets+1;lastResetReason=reason;
  }
  bool add(const float value[6], uint64_t now) {
    float an=sqrtf(value[0]*value[0]+value[1]*value[1]+value[2]*value[2]);
    // A stationary sensor may have a substantial scale or bias error. Motion
    // rejection uses axis stability/variance rather than the gyro DC offset.
    if (an<.5f*ImuMath::G || an>1.5f*ImuMath::G) { reset("accel_norm"); return false; }
    if (count) for (int i=0;i<6;++i) {
      float tolerance=i<3 ? .05f*ImuMath::G : 1.5f;
      if (fabsf(value[i]-first[i])>tolerance) { reset(i<3?"accel_change":"gyro_change"); break; }
    }
    if (!count) { firstUs=now; memcpy(first,value,sizeof(first)); }
    ++count;
    phase="collecting";
    for (int i=0;i<6;++i) { double delta=value[i]-mean[i]; mean[i]+=delta/count; m2[i]+=delta*(value[i]-mean[i]); }
    if (now-firstUs<3000000 || count<600) return false;
    for (int i=0;i<6;++i) {
      double limit=i<3 ? .025*ImuMath::G : .5;
      if (sqrt(m2[i]/(count-1))>limit) { reset(i<3?"accel_std":"gyro_std"); return false; }
    }
    phase="complete";
    return true;
  }
};

static Adafruit_MPU6050 imu;
static ImuMath motion;
static Calibration calibration;
static Counters counters, finalCounters;
static State state=State::Calibrating;
static QueueHandle_t dataQueue;
static SemaphoreHandle_t stateMutex;
static TaskHandle_t samplingTaskHandle=nullptr;
static hw_timer_t *sampleTimer=nullptr;
static bool calibrated=false, online=false, haveEnd=false;
static bool finalCalibrated=false;
static State finalState=State::Idle;
static float gyroBias[3]={};
static float accelNormScale=1;
static uint32_t sessionId=0, finalSessionId=0, calibrationSamples=0, pendingEvents=0;
static uint64_t sessionStartUs=0;
static uint32_t consecutiveFailures=0;
static int16_t lastRawCounts[6]={};
static float rawAccelNorm=0,rawGyroNorm=0;
static uint64_t lastFreshUs=0,lastAttemptUs=0;
static uint32_t totalFreshSamples=0,totalReadErrors=0,totalStaleSamples=0;
static int lastReadResult=-1;

static const char *stateName(State s) {
  switch(s) {
    case State::Calibrating:return "calibrating";
    case State::Idle:return "idle";
    case State::Recording:return "recording";
    case State::Draining:return "draining";
    default:return "error";
  }
}
static void lockState() { xSemaphoreTake(stateMutex,portMAX_DELAY); }
static void unlockState() { xSemaphoreGive(stateMutex); }
static uint16_t crc16(const uint8_t *p,size_t n) {
  uint16_t crc=0xffff;
  while(n--) { crc^=(uint16_t)*p++<<8; for(int b=0;b<8;++b) crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1; }
  return crc;
}
static void put16(uint8_t *p,uint16_t x) { p[0]=x; p[1]=x>>8; }
static void put32(uint8_t *p,uint32_t x) { for(int i=0;i<4;++i)p[i]=x>>(8*i); }
static void put64(uint8_t *p,uint64_t x) { for(int i=0;i<8;++i)p[i]=x>>(8*i); }
static void putFloat(uint8_t *p,float x) { memcpy(p,&x,4); }
static void envelope(uint8_t *p,uint8_t type,uint16_t length) { p[0]=0xa5;p[1]=0x5a;p[2]=1;p[3]=type;put16(p+4,length); }
static void sendJson(uint8_t type,const char *json) {
  size_t n=strlen(json); uint8_t buffer[1600];
  if(n+8>sizeof(buffer))return;
  envelope(buffer,type,n+8); memcpy(buffer+6,json,n); put16(buffer+6+n,crc16(buffer,6+n));
  Serial.write(buffer,n+8);
}
static void sendError(const char *code,const char *message) {
  char json[300]; snprintf(json,sizeof(json),"{\"code\":\"%s\",\"message\":\"%s\",\"session_id\":%lu}",code,message,(unsigned long)sessionId);
  sendJson(5,json);
}
static void sendMeta() {
  char json[1100]; lockState();
  snprintf(json,sizeof(json),
    "{\"session_id\":%lu,\"state\":\"%s\",\"sample_hz\":200,\"baud\":%lu,\"accel_lsb_per_g\":8192,\"gyro_lsb_per_dps\":65.5,\"g\":9.80665,\"accel_range_g\":4,\"gyro_range_dps\":500,\"hardware_dlpf_hz\":44,\"sensor_output_hz\":1000,\"filter_hz\":20,\"mahony_kp\":0.5,\"mahony_ki\":0,\"gyro_bias_dps\":[%.8f,%.8f,%.8f],\"accel_norm_scale\":%.9f,\"accel_calibration_method\":\"single_pose_norm_scale\",\"calibrated\":%s,\"calibration_samples\":%lu,\"quaternion_order\":\"wxyz\",\"coordinate_frame\":\"right-handed body-to-world, world Z up\",\"firmware_version\":\"1.0.0\"}",
    (unsigned long)sessionId,stateName(state),(unsigned long)IMU_SERIAL_BAUD,gyroBias[0],gyroBias[1],gyroBias[2],accelNormScale,calibrated?"true":"false",(unsigned long)calibrationSamples);
  unlockState(); sendJson(2,json);
}
static void sendSummary(uint8_t type,const Counters &c,uint32_t id,const char *name,bool cal) {
  char json[1500];
  snprintf(json,sizeof(json),
    "{\"session_id\":%lu,\"state\":\"%s\",\"generated\":%lu,\"sent\":%lu,\"sample_fail\":%lu,\"stale_samples\":%lu,\"missed_deadlines\":%lu,\"queue_drops\":%lu,\"integration_gaps\":%lu,\"first_t_us\":%llu,\"last_t_us\":%llu,\"dt_count\":%lu,\"dt_mean_us\":%.4f,\"dt_std_us\":%.4f,\"dt_min_us\":%lu,\"dt_max_us\":%lu,\"calibrated\":%s}",
    (unsigned long)id,name,(unsigned long)c.generated,(unsigned long)c.sent,(unsigned long)c.sampleFail,(unsigned long)c.stale,(unsigned long)c.missed,(unsigned long)c.queueDrops,(unsigned long)c.gaps,
    (unsigned long long)c.firstUs,(unsigned long long)c.lastUs,(unsigned long)c.dtCount,c.dtMean,c.dtCount>1?sqrt(c.dtM2/(c.dtCount-1)):0,(unsigned long)c.dtMin,(unsigned long)c.dtMax,cal?"true":"false");
  if(type==4) {
    size_t offset=strlen(json)-1;
    lockState();
    uint64_t elapsed=calibration.count&&lastFreshUs>=calibration.firstUs?(lastFreshUs-calibration.firstUs)/1000:0;
    snprintf(json+offset,sizeof(json)-offset,
      ",\"last_raw_counts\":[%d,%d,%d,%d,%d,%d],\"raw_accel_norm\":%.6f,\"raw_gyro_norm\":%.6f,\"calibration_progress\":%lu,\"calibration_elapsed_ms\":%llu,\"calibration_phase\":\"%s\",\"calibration_reset_reason\":\"%s\",\"calibration_resets\":%lu,\"last_fresh_t_us\":%llu,\"last_attempt_t_us\":%llu,\"last_read_result\":%d,\"total_fresh_samples\":%lu,\"total_read_errors\":%lu,\"total_stale_samples\":%lu}",
      lastRawCounts[0],lastRawCounts[1],lastRawCounts[2],lastRawCounts[3],lastRawCounts[4],lastRawCounts[5],rawAccelNorm,rawGyroNorm,
      (unsigned long)calibration.count,(unsigned long long)elapsed,calibration.phase,calibration.lastResetReason,(unsigned long)calibration.resets,
      (unsigned long long)lastFreshUs,(unsigned long long)lastAttemptUs,lastReadResult,(unsigned long)totalFreshSamples,(unsigned long)totalReadErrors,(unsigned long)totalStaleSamples);
    unlockState();
  }
  sendJson(type,json);
}
static void sendStatus() { lockState(); Counters c=counters; uint32_t id=sessionId;State s=state;bool cal=calibrated;unlockState();sendSummary(4,c,id,stateName(s),cal); }

static bool writeRegister(uint8_t reg,uint8_t value) { Wire.beginTransmission(IMU_ADDRESS);Wire.write(reg);Wire.write(value);return Wire.endTransmission()==0; }
static bool readRegister(uint8_t reg,uint8_t &value) {
  Wire.beginTransmission(IMU_ADDRESS);Wire.write(reg);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom(IMU_ADDRESS,(uint8_t)1)!=(uint8_t)1)return false;
  value=Wire.read();return true;
}
static bool initializeSensor() {
  if(!imu.begin(IMU_ADDRESS,&Wire))return false;
  imu.setClock(MPU6050_PLL_GYROX); imu.setAccelerometerRange(MPU6050_RANGE_4_G);imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_44_HZ);imu.setSampleRateDivisor(0);
  if(!writeRegister(0x37,0) || !writeRegister(0x38,1))return false;
  uint8_t who=0,acc=0,gyr=0,dlpf=0,div=255;
  return readRegister(0x75,who)&&who==0x68&&readRegister(0x1c,acc)&&(acc&0x18)==8&&readRegister(0x1b,gyr)&&(gyr&0x18)==8&&readRegister(0x1a,dlpf)&&(dlpf&7)==3&&readRegister(0x19,div)&&div==0;
}
// 0=bus error, 1=fresh, 2=no new sensor data. Checked read includes DATA_RDY.
static int readSample(int16_t raw[6]) {
  Wire.beginTransmission(IMU_ADDRESS);Wire.write((uint8_t)0x3a);
  if(Wire.endTransmission(false)!=0)return 0;
  if(Wire.requestFrom(IMU_ADDRESS,(uint8_t)15)!=(uint8_t)15) { while(Wire.available())Wire.read();return 0; }
  uint8_t b[15];for(int i=0;i<15;++i)b[i]=Wire.read();
  if(!(b[0]&1))return 2;
  for(int i=0;i<3;++i)raw[i]=(int16_t)((b[1+2*i]<<8)|b[2+2*i]);
  for(int i=0;i<3;++i)raw[i+3]=(int16_t)((b[9+2*i]<<8)|b[10+2*i]);
  return 1;
}
static void makeData(WireFrame &f,const int16_t raw[6],uint64_t now,uint16_t flags) {
  uint8_t *p=f.bytes;envelope(p,1,90);put32(p+6,sessionId);put32(p+10,counters.generated);put64(p+14,now);put16(p+22,flags);
  for(int i=0;i<6;++i)put16(p+24+2*i,(uint16_t)raw[i]);
  for(int i=0;i<6;++i)putFloat(p+36+4*i,motion.filtered[i]);
  for(int i=0;i<4;++i)putFloat(p+60+4*i,motion.q[i]);
  for(int i=0;i<3;++i)putFloat(p+76+4*i,motion.position[i]);
  put16(p+88,crc16(p,88));
}

static void samplingTask(void *) {
  Wire.begin(21,22,100000);Wire.setTimeOut(10);
  uint64_t retryAt=0;
  for(;;) {
    uint32_t ticks=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    uint64_t now=esp_timer_get_time();
    if(!online) {
      lockState();bool draining=state==State::Draining;unlockState();
      if(draining)continue; // Preserve the stopped session until END is emitted.
      if(now<retryAt)continue;
      bool ok=initializeSensor(); retryAt=esp_timer_get_time()+1000000;
      lockState();online=ok;
      if(ok) { calibrated=false;accelNormScale=1;calibration.reset("initialized");calibrationSamples=0;state=State::Calibrating;pendingEvents|=META_CHANGED;consecutiveFailures=0; }
      else { state=State::Error;pendingEvents|=INIT_FAILED; }
      unlockState();continue;
    }
    int16_t raw[6];now=esp_timer_get_time();int result=readSample(raw);
    lockState();
    lastAttemptUs=now;lastReadResult=result;
    if(state==State::Recording && ticks>1)counters.missed+=ticks-1;
    if(result!=1) {
      if(state==State::Recording) { if(result==0)++counters.sampleFail;else ++counters.stale; }
      if(result==0)++totalReadErrors;else ++totalStaleSamples;
      if(state==State::Calibrating)calibration.reset(result==0?"bus_error":"stale_data");
      ++consecutiveFailures;
      if(consecutiveFailures==1)pendingEvents|=(result==0?READ_FAILED:STALE_DATA);
      if(consecutiveFailures>=100) {
        online=false;calibrated=false;pendingEvents|=SENSOR_LOST;
        if(state==State::Recording)state=State::Draining;else if(state!=State::Draining)state=State::Error;
      }
      unlockState();continue;
    }
    consecutiveFailures=0;
    float value[6];for(int i=0;i<3;++i)value[i]=raw[i]*(ImuMath::G/8192);for(int i=3;i<6;++i)value[i]=raw[i]/65.5f;
    memcpy(lastRawCounts,raw,sizeof(lastRawCounts));lastFreshUs=now;++totalFreshSamples;
    rawAccelNorm=sqrtf(value[0]*value[0]+value[1]*value[1]+value[2]*value[2]);
    rawGyroNorm=sqrtf(value[3]*value[3]+value[4]*value[4]+value[5]*value[5]);
    if(state==State::Calibrating) {
      uint32_t previousCount=calibration.count;
      if(calibration.add(value,now)) {
        for(int i=0;i<3;++i)gyroBias[i]=calibration.mean[i+3];
        float a[3]={(float)calibration.mean[0],(float)calibration.mean[1],(float)calibration.mean[2]};
        accelNormScale=ImuMath::G/sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
        motion.initialize(a);calibrationSamples=calibration.count;calibrated=true;state=State::Idle;pendingEvents|=META_CHANGED;
      } else if(previousCount>100 && calibration.count<previousCount)pendingEvents|=CAL_MOVED;
    }
    if(calibrated) {
      bool gap=motion.update(value,gyroBias,now,accelNormScale);
      if(state==State::Recording && now>=sessionStartUs) {
        counters.addTime(now);if(gap)++counters.gaps;
        WireFrame frame;makeData(frame,raw,now,1|(motion.stationary?2:0)|(gap?4:0));
        ++counters.generated;
        if(xQueueSend(dataQueue,&frame,0)!=pdTRUE)++counters.queueDrops;
      }
    }
    unlockState();
  }
}

static void ARDUINO_ISR_ATTR onSampleTimer() {
  BaseType_t wake=pdFALSE;
  if(samplingTaskHandle)vTaskNotifyGiveFromISR(samplingTaskHandle,&wake);
  if(wake)portYIELD_FROM_ISR();
}
static void handleCommand(const char *command) {
  if(strcmp(command,"INFO")==0) {
    sendMeta();sendStatus();
    lockState();bool has=haveEnd;Counters c=finalCounters;uint32_t id=finalSessionId;bool cal=finalCalibrated;State ended=finalState;unlockState();
    if(has)sendSummary(3,c,id,stateName(ended),cal);
    return;
  }
  if(strcmp(command,"START")==0) {
    lockState();bool ok=state==State::Idle&&calibrated;
    if(ok) { counters=Counters();sessionId=esp_random();if(!sessionId)sessionId=1;haveEnd=false;sessionStartUs=esp_timer_get_time();state=State::Recording; }
    unlockState();if(ok)sendMeta();else sendError("NOT_READY","START requires calibrated idle device");return;
  }
  if(strcmp(command,"STOP")==0) {
    lockState();bool replay=haveEnd&&state!=State::Recording&&state!=State::Draining;
    if(state==State::Recording)state=State::Draining;
    Counters c=finalCounters;uint32_t id=finalSessionId;bool cal=finalCalibrated;State ended=finalState;unlockState();
    if(replay)sendSummary(3,c,id,stateName(ended),cal);else sendStatus();return;
  }
  if(strcmp(command,"CALIBRATE")==0) {
    lockState();bool ok=state==State::Idle||state==State::Calibrating||state==State::Error;
    if(ok) { calibrated=false;accelNormScale=1;calibration.reset();calibrationSamples=0;state=online?State::Calibrating:State::Error; }
    unlockState();if(ok)sendMeta();else sendError("BUSY","Stop recording before calibration");return;
  }
  if(strcmp(command,"ZERO")==0) {
    lockState();bool ok=state==State::Idle&&calibrated;if(ok)motion.zeroPosition();unlockState();
    if(ok)sendMeta();else sendError("BUSY","ZERO requires calibrated idle device");return;
  }
  sendError("BAD_COMMAND","Use INFO START STOP CALIBRATE or ZERO");
}

void setup() {
  Serial.setRxBufferSize(512);Serial.setTxBufferSize(2048);Serial.begin(IMU_SERIAL_BAUD);
  stateMutex=xSemaphoreCreateMutex();dataQueue=xQueueCreate(64,sizeof(WireFrame));
  if(!stateMutex||!dataQueue) { sendError("MEMORY","Cannot allocate task resources");return; }
  if(xTaskCreatePinnedToCore(samplingTask,"imu-sample",6144,nullptr,3,&samplingTaskHandle,1)!=pdPASS) { samplingTaskHandle=nullptr;sendError("TASK","Cannot create sampling task");return; }
  sampleTimer=timerBegin(1000000);
  if(!sampleTimer) { sendError("TIMER","Cannot allocate hardware timer");return; }
  timerAttachInterrupt(sampleTimer,&onSampleTimer);timerAlarm(sampleTimer,PERIOD_US,true,0);
  sendMeta();
}
void loop() {
  if(!stateMutex||!dataQueue||!sampleTimer) { delay(100);return; }
  static char command[64];static size_t length=0;static bool overflow=false;
  while(Serial.available()) {
    char ch=Serial.read();
    if(ch=='\n') { if(overflow)sendError("BAD_COMMAND","Command too long");else { command[length]=0;if(length)handleCommand(command); }length=0;overflow=false; }
    else if(ch!='\r') { if(length<sizeof(command)-1)command[length++]=ch;else overflow=true; }
  }
  lockState();uint32_t events=pendingEvents;pendingEvents=0;unlockState();
  if(events&META_CHANGED)sendMeta();
  if(events&INIT_FAILED)sendError("IMU_INIT","MPU6050 initialization failed; retrying every second");
  if(events&READ_FAILED)sendError("I2C_READ","Sensor read failed; sample not recycled");
  if(events&STALE_DATA)sendError("NO_NEW_DATA","Sensor DATA_RDY is clear; sample not recycled");
  if(events&CAL_MOVED)sendError("CAL_MOVING","Calibration restarted; hold sensor still for three seconds");
  if(events&SENSOR_LOST)sendError("SENSOR_LOST","No fresh data for 100 attempts; reinitializing");
  WireFrame frame;
  // Bounded drain keeps commands responsive, while the independent sampling
  // task never waits on UART. Every frame retains its own sample timestamp.
  for(int i=0;i<8&&xQueueReceive(dataQueue,&frame,0)==pdTRUE;++i) {
    Serial.write(frame.bytes,sizeof(frame.bytes));lockState();++counters.sent;unlockState();
  }
  lockState();bool finish=state==State::Draining&&uxQueueMessagesWaiting(dataQueue)==0;
  if(finish) { finalCounters=counters;finalSessionId=sessionId;haveEnd=true;state=calibrated?State::Idle:State::Error;finalState=state;finalCalibrated=calibrated; }
  Counters final=finalCounters;uint32_t id=finalSessionId;bool cal=finalCalibrated;State ended=finalState;unlockState();
  if(finish)sendSummary(3,final,id,stateName(ended),cal);
  static uint32_t lastStatus=0;
  if(millis()-lastStatus>=1000) {lastStatus=millis();sendStatus();}
  delay(1);
}
