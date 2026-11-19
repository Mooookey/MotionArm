#pragma once
#include <MotionArmLink.h>
#include <cmath>
#include <cstring>
#include <cstdint>

// No Arduino or PWM dependency. All positions are software targets, not feedback.
namespace robot {
constexpr uint32_t gripLimitMs = 5000;
enum class GripState { OpenReady, Closing, Closed, Opening };
struct Event { uint32_t seq, timeMs; const char *code; };
class Controller {
 public:
  motionarm::Frame input{};
  bool haveInput = false, enabled = false, armed = false, hasReference = false, ambiguous = false;
  float qReq[3] = {0,45,45}, qOut[3] = {0,45,45}, velocity[3] = {};
  float reference[4] = {1,0,0,0}, rotation[3] = {}, openReq = 1, openOut = 1;
  const char *reason = "INITIALIZATION_REQUIRED", *fault = nullptr, *gripReason = nullptr;
  GripState gripState = GripState::OpenReady;
  uint32_t gripCycle = 0, gripStarted = 0, eventSeq = 0, dropped = 0, duplicates = 0, restarts = 0;
  uint32_t eventDrops = 0, outOfOrder = 0;

  uint32_t ageMs(uint32_t now) const {
    if (!haveInput) return UINT32_MAX;
    const uint64_t age = uint64_t(input.ageMs) + uint32_t(now - arrived_);
    return age > UINT32_MAX ? UINT32_MAX : uint32_t(age);
  }
  bool valid(uint32_t now) const {
    return haveInput && (input.flags & (motionarm::imuValid | motionarm::buttonsPrimed)) ==
      (motionarm::imuValid | motionarm::buttonsPrimed) && input.who == 0x68 &&
      input.calibrationSamples == 250 && input.rejection == 0 && ageMs(now) < motionarm::timeoutMs && quaternionValid_;
  }
  bool canEnable(uint32_t now) const {
    return !fault && canEnableInput(now);
  }
  // Allows the management UI to offer an explicit retry after the outer PCA
  // health check succeeds. initialize itself still rejects a latched fault.
  bool canEnableInput(uint32_t now) const {
    if (armed || !valid(now) || (input.flags & (motionarm::buttonA | motionarm::buttonB)) ||
        gripState != GripState::OpenReady || !gripReady_ || stableCount_ < 2) return false;
    const uint8_t start = recentStart(now);
    if (start >= stableCount_ || uint32_t(stable_[stableCount_-1].ms - stable_[start].ms) < 480) return false;
    for (uint8_t i = start; i < stableCount_; ++i) {
      if (i > start && uint32_t(stable_[i].ms - stable_[i-1].ms) > 100) return false;
      for (uint8_t j = i + 1; j < stableCount_; ++j)
        if (distance(stable_[i].q, stable_[j].q) > 3.0001f) return false;
    }
    return true;
  }
  bool initialize(uint32_t now) {
    serviceDeadline(now);
    if (!canEnable(now)) return false;
    float sum[4] = {};
    const uint8_t start = recentStart(now);
    for (uint8_t i = start; i < stableCount_; ++i) {
      const float sign = dot(stable_[start].q, stable_[i].q) < 0 ? -1 : 1;
      for (uint8_t a = 0; a < 4; ++a) sum[a] += sign * stable_[i].q[a];
    }
    normalize(sum); std::memcpy(reference, sum, sizeof(reference));
    hasReference = enabled = true; armed = ambiguous = false;
    qReq[0] = qOut[0] = 0; qReq[1] = qReq[2] = qOut[1] = qOut[2] = 45;
    for (float &v : velocity) v = 0;
    openReq = openOut = 1; gripState = GripState::OpenReady; gripReady_ = true;
    baselineA_ = input.ac; baselineB_ = input.bc;
    lastTick_ = now; haveTick_ = true; reason = "INITIALIZED_PAUSED";
    updateRotation(); event("INITIALIZE", now); return true;
  }
  void stop(const char *why, uint32_t now) {
    const bool changed = enabled || armed || hasReference || std::strcmp(reason, why) != 0;
    freeze(); enabled = false; hasReference = false; ambiguous = false;
    reason = why;
    if (changed) stableCount_ = 0;
    requestOpen(why, now);
    if (changed) event(why, now);
  }
  void setFault(const char *why, uint32_t now) { fault = why; stop(why, now); }
  void clearFault(uint32_t now) {
    if (!fault) return;
    fault = nullptr; reason = "FAULT_CLEARED_REINITIALIZE"; event(reason, now);
  }
  bool receive(const motionarm::Frame &frame, uint32_t arrived, uint32_t now) {
    serviceDeadline(now);
    const bool restart = !haveInput || input.boot != frame.boot;
    if (haveInput && !restart) {
      if (frame.seq == input.seq) { ++duplicates; return false; }
      if (!motionarm::newer(frame.seq, input.seq) || !motionarm::newer(frame.uptime, input.uptime)) {
        ++outOfOrder; return false;
      }
    }
    if (haveInput && restart && retired(frame.boot)) { ++outOfOrder; return false; }
    const bool lost = haveInput && !valid(now);
    const motionarm::Frame old = input;
    const uint32_t oldArrival = arrived_;
    if (restart) {
      if (haveInput) { rememberBoot(input.boot); ++restarts; }
      stop("WEARABLE_RESTART", now);
    } else dropped += uint32_t(frame.seq - input.seq) - 1;
    input = frame; haveInput = true; arrived_ = arrived;
    quaternionValid_ = normalizeInput();
    if (lost || !valid(now)) stop("INPUT_INVALID", now);
    const bool still = valid(now) && !(input.flags & (motionarm::buttonA | motionarm::buttonB)) &&
      std::sqrt(input.gyroDps[0]*input.gyroDps[0]+input.gyroDps[1]*input.gyroDps[1]+input.gyroDps[2]*input.gyroDps[2]) < 5;
    if (!still || restart || lost || (!restart && uint32_t(arrived - oldArrival) > 100)) stableCount_ = 0;
    if (still) addStable(arrived);
    const uint32_t da = input.ac - baselineA_, db = input.bc - baselineB_;
    baselineA_ = input.ac; baselineB_ = input.bc;
    // Release rearming is allowed after STOP as well: it never reenables follow.
    if (valid(now) && gripState == GripState::OpenReady && !gripReady_ &&
        !(input.flags & motionarm::buttonB) && db == 0) gripReady_ = true;
    if (!enabled || restart || lost || !valid(now)) return true;
    if (da >= 0x80000000UL || db >= 0x80000000UL) { stop("BUTTON_COUNTER_INVALID", now); return true; }
    trackWrist(old.q, now);
    if (db > 1) { requestOpen("BUTTON_EVENTS_COALESCED", now); event("BUTTON_EVENTS_COALESCED", now); }
    else if (db == 1) clickGrip(now);
    if (da & 1) {
      if (armed) { freeze(); reason = "A_PAUSED"; event("PAUSE", now); }
      else { armed = true; reason = "FOLLOWING"; event("RUN", now); }
    }
    if (armed && !ambiguous) mapRequests();
    return true;
  }
  void tick(uint32_t now) {
    serviceDeadline(now);
    const uint32_t elapsed = haveTick_ ? uint32_t(now - lastTick_) : 0;
    if (enabled && !valid(now)) stop("INPUT_TIMEOUT", now);
    if (!haveTick_) { lastTick_ = now; haveTick_ = true; return; }
    if (elapsed < 20) return;
    lastTick_ = now;
    if (enabled && elapsed > 100) {
      freeze(); reason = "CONTROL_OVERRUN"; requestOpen(reason, now); event(reason, now);
    }
    const float dt = float(elapsed > 50 ? 50 : elapsed) / 1000;
    if (enabled && armed && !ambiguous && !fault) {
      slew(qOut[0], qReq[0], velocity[0], -45,45,20,60,dt);
      slew(qOut[1], qReq[1], velocity[1], 0,60,15,60,dt);
      qOut[2] = qOut[1]; velocity[2] = velocity[1];
    }
    // Release advances regardless of following, control ownership or faults.
    openOut += clamp(openReq-openOut, -dt/.2f, dt/.2f);
    if (openOut >= 1-1e-6f) {
      openOut = 1;
      if (gripState == GripState::Opening) {
        gripState = GripState::OpenReady; gripReady_ = false; event("GRIPPER_OPEN_TARGET", now);
      }
    } else if (openOut <= 1e-6f) {
      openOut = 0; if (gripState == GripState::Closing) gripState = GripState::Closed;
    }
  }
  uint32_t gripElapsedMs(uint32_t now) const {
    if (!haveGripCycle_) return 0;
    const uint32_t elapsed = uint32_t(now - gripStarted);
    return elapsed > gripLimitMs ? gripLimitMs : elapsed;
  }
  uint32_t gripRemainingMs(uint32_t now) const {
    return gripState == GripState::Closing || gripState == GripState::Closed ? gripLimitMs - gripElapsedMs(now) : 0;
  }
  bool gripCanClose() const { return gripState == GripState::OpenReady && gripReady_; }
  const char *gripStateName() const {
    switch (gripState) { case GripState::Closing:return "CLOSING"; case GripState::Closed:return "CLOSED";
      case GripState::Opening:return "OPENING"; default:return "OPEN_READY"; }
  }
  bool takeEvent(Event &out) {
    if (!eventCount_) return false;
    out = events_[eventRead_]; eventRead_ = (eventRead_+1)%eventCapacity; --eventCount_; return true;
  }
 private:
  static constexpr uint8_t stableCapacity = 24, eventCapacity = 16;
  struct Sample { uint32_t ms; float q[4]; };
  Sample stable_[stableCapacity]{};
  Event events_[eventCapacity]{};
  uint8_t stableCount_ = 0, eventRead_ = 0, eventCount_ = 0;
  uint32_t arrived_ = 0, lastTick_ = 0, baselineA_ = 0, baselineB_ = 0;
  uint32_t retiredBoots_[8]{};
  uint8_t retiredIndex_ = 0;
  bool quaternionValid_ = false, haveTick_ = false, gripReady_ = true, haveGripCycle_ = false;
  static float clamp(float v,float a,float b) { return std::fmin(b,std::fmax(a,v)); }
  static float dot(const float *a,const float *b) { float v=0;for(int i=0;i<4;++i)v+=a[i]*b[i];return v; }
  static bool normalize(float *q) {
    const float norm=std::sqrt(dot(q,q)); if (!std::isfinite(norm)||norm<1e-8f)return false;
    for(int i=0;i<4;++i)q[i]/=norm;
    return true;
  }
  bool normalizeInput() {
    const float norm=std::sqrt(dot(input.q,input.q));
    for (float v : input.gyroDps) if (!motionarm::scalar(v,-2031,2031)) return false;
    for (float v : input.accelG) if (!motionarm::scalar(v,-4,4)) return false;
    return std::isfinite(norm) && std::fabs(norm-1)<=.010001f && normalize(input.q);
  }
  static void multiply(const float *a,const float *b,float *q) {
    q[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    q[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    q[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    q[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
  }
  static void relative(const float *a,const float *b,float *q) {
    const float inverse[4]={a[0],-a[1],-a[2],-a[3]}; multiply(inverse,b,q);normalize(q);
  }
  static float distance(const float *a,const float *b) {
    float q[4];relative(a,b,q);return 114.591559f*std::atan2(std::sqrt(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]),std::fabs(q[0]));
  }
  uint8_t recentStart(uint32_t now) const {
    uint8_t i=0;while(i<stableCount_&&uint32_t(now-stable_[i].ms)>650)++i;return i;
  }
  void addStable(uint32_t ms) {
    uint8_t first=recentStart(ms);
    if(first){for(uint8_t i=first;i<stableCount_;++i)stable_[i-first]=stable_[i];stableCount_-=first;}
    if(stableCount_==stableCapacity){for(uint8_t i=1;i<stableCount_;++i)stable_[i-1]=stable_[i];--stableCount_;}
    stable_[stableCount_].ms=ms;std::memcpy(stable_[stableCount_++].q,input.q,sizeof(input.q));
  }
  void event(const char *code,uint32_t now) {
    if(eventCount_==eventCapacity){eventRead_=(eventRead_+1)%eventCapacity;--eventCount_;++eventDrops;}
    events_[(eventRead_+eventCount_)%eventCapacity]={++eventSeq,now,code};++eventCount_;
  }
  void freeze() { for(int i=0;i<3;++i){qReq[i]=qOut[i];velocity[i]=0;}armed=false; }
  bool retired(uint32_t boot) const { for(uint32_t v:retiredBoots_)if(v==boot)return true;return false; }
  void rememberBoot(uint32_t boot) { retiredBoots_[retiredIndex_++%8]=boot; }
  void requestOpen(const char *why,uint32_t now) {
    const bool closing=gripState==GripState::Closing||gripState==GripState::Closed;
    const bool change=openReq!=1||openOut<1;
    openReq=1;
    if(change){gripState=GripState::Opening;gripReady_=false;}
    if(closing){gripReason=why;event("GRIPPER_AUTO_OPEN",now);}
  }
  void serviceDeadline(uint32_t now) {
    if((gripState==GripState::Closing||gripState==GripState::Closed)&&uint32_t(now-gripStarted)>=gripLimitMs)
      requestOpen("GRIPPER_TIMEOUT",now);
  }
  void clickGrip(uint32_t now) {
    if(gripState==GripState::Closing||gripState==GripState::Closed){requestOpen("B_OPEN",now);event("GRIPPER_OPEN",now);return;}
    if(gripState!=GripState::OpenReady||!gripReady_){event("GRIPPER_CLOSE_BLOCKED",now);return;}
    openReq=0;gripStarted=now;++gripCycle;haveGripCycle_=true;
    gripState=GripState::Closing;gripReady_=false;gripReason=nullptr;event("GRIPPER_CLOSE",now);
  }
  void updateRotation() {
    if(!hasReference)return;
    float q[4];relative(reference,input.q,q);
    const float wrist[3]={-q[2],q[1],q[3]};
    const float sign=q[0]<0?-1:1;
    const float length=std::sqrt(wrist[0]*wrist[0]+wrist[1]*wrist[1]+wrist[2]*wrist[2]);
    const float scale=length>1e-8f?114.591559f*std::atan2(length,std::fabs(q[0]))/length:114.591559f;
    for(int i=0;i<3;++i)rotation[i]=wrist[i]*sign*scale;
  }
  void trackWrist(const float *old,uint32_t now) {
    float previous[4],current[4];relative(reference,old,previous);relative(reference,input.q,current);
    if(dot(previous,current)<0)for(float &v:current)v=-v;
    const float angle=distance(reference,input.q);
    const bool crossed=previous[0]*current[0]<0;
    if((angle>=174.9999f||crossed)&&!ambiguous){
      const bool wasArmed=armed;freeze();armed=wasArmed;ambiguous=true;reason="ANGLE_AMBIGUOUS";event(reason,now);
    } else if(ambiguous&&angle<=90.0001f){ambiguous=false;reason=armed?"FOLLOWING":"INITIALIZED_PAUSED";event("ANGLE_RECOVERED",now);}
    updateRotation();
  }
  void mapRequests() {
    const float roll=std::copysign(std::fmax(0.0f,std::fabs(rotation[0])-5),rotation[0]);
    const float pitch=-std::copysign(std::fmax(0.0f,std::fabs(rotation[1])-5),rotation[1]);
    qReq[0]=clamp(roll,-45,45);qReq[1]=qReq[2]=45+clamp(pitch,-45,15);
  }
  static void slew(float &current,float requested,float &v,float low,float high,float speed,float accel,float dt) {
    const float error=requested-current;
    const float desired=std::copysign(std::fmin(speed,std::sqrt(2*accel*std::fabs(error))),error);
    v+=clamp(desired-v,-accel*dt,accel*dt);
    const float movement=v*dt;
    current=error*movement>=0&&std::fabs(movement)>=std::fabs(error)?requested:clamp(current+movement,low,high);
    if((current<=low&&v<0)||(current>=high&&v>0))v=0;
  }
};
}  // namespace robot
