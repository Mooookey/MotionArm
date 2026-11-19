#include "../robot/RobotController.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>

using motionarm::Frame;
using robot::Controller;
using robot::GripState;
void near(float a,float b,float tolerance=.002f) { assert(std::fabs(a-b)<=tolerance); }
Frame sample(uint32_t seq=1,uint32_t ms=0) {
  Frame f; f.boot=7; f.seq=seq;f.uptime=ms;f.ageMs=0;
  f.flags=motionarm::imuValid|motionarm::buttonsPrimed;f.who=0x68;f.calibrationSamples=250;
  f.accelG[2]=f.normG=1;return f;
}
void sensorQuaternion(Frame &f,float x,float y,float z=0) {
  const float magnitude=std::sqrt(x*x+y*y+z*z);
  const float half=magnitude*.00872664626f;
  const float scale=magnitude>0?std::sin(half)/magnitude:0;
  f.q[0]=std::cos(half);f.q[1]=x*scale;f.q[2]=y*scale;f.q[3]=z*scale;
}
struct Rig {
  Controller c;
  Frame f=sample();
  uint32_t now=0;
  explicit Rig(uint32_t start=0,const float *reference=nullptr,uint32_t counters=0):now(start) {
    if(reference)std::memcpy(f.q,reference,sizeof(f.q));
    f.ac=f.bc=counters;
    for(int i=0;i<16;++i) { if(i)now+=40;send();c.tick(now); }
    assert(c.canEnable(now));assert(c.initialize(now));assert(c.enabled&&!c.armed);
  }
  void send() { f.uptime=now;++f.seq;assert(c.receive(f,now,now)); }
  void advance(uint32_t duration) {
    const uint32_t end=now+duration;
    while(uint32_t(end-now)>0) {
      const uint32_t step=uint32_t(end-now)<20?uint32_t(end-now):20;
      now+=step;c.tick(now);if(uint32_t(now-f.uptime)>=40)send();
    }
  }
  void a() { ++f.ac;now+=40;c.tick(now);send(); }
  void b() { ++f.bc;now+=40;c.tick(now);send(); }
};
void codec() {
  assert(motionarm::crc(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29b1);
  Frame f=sample(0xffffffff,0xfffffffc);f.ac=0xffffffff;f.bc=0x87654321;
  f.q[0]=f.q[1]=f.q[2]=f.q[3]=-.5f;f.gyroDps[0]=-2028.05f;
  f.accelG[0]=-4;f.tempC=-59.85f;f.errors=f.rejected=0xffffffff;f.ageMs=0xfffffffe;
  uint8_t bytes[motionarm::frameSize];assert(motionarm::encode(f,bytes));
  assert(sizeof(bytes)==82&&bytes[0]==2&&bytes[1]==1);
  assert(bytes[20]==0xe0&&bytes[21]==0x5e&&bytes[22]==0xf8&&bytes[23]==0xff); // -500000 LE
  assert(motionarm::get(bytes+60,4)==0x87654321);
  Frame decoded;assert(motionarm::decode(bytes,sizeof(bytes),decoded));
  near(decoded.q[0],-.5f,.000001f);near(decoded.gyroDps[0],-2028.05f,.001f);
  near(decoded.accelG[0],-4);near(decoded.tempC,-59.85f);
  assert(decoded.ac==0xffffffff&&decoded.ageMs==0xfffffffe&&decoded.errors==0xffffffff);
  assert(!motionarm::decode(nullptr,82,decoded));assert(!motionarm::decode(bytes,24,decoded));
  bytes[0]=1;assert(!motionarm::decode(bytes,82,decoded));bytes[0]=2;
  bytes[78]=1;motionarm::put(bytes+80,motionarm::crc(bytes,80),2);assert(!motionarm::decode(bytes,82,decoded));
  bytes[78]=0;bytes[2]=16;motionarm::put(bytes+80,motionarm::crc(bytes,80),2);assert(!motionarm::decode(bytes,82,decoded));
  bytes[2]=9;motionarm::put(bytes+20,2000000,4);motionarm::put(bytes+80,motionarm::crc(bytes,80),2);
  assert(!motionarm::decode(bytes,82,decoded));
  f.q[0]=std::numeric_limits<float>::quiet_NaN();assert(!motionarm::encode(f,bytes));
}
void angles() {
  Rig r;r.a();assert(r.c.armed);
  sensorQuaternion(r.f,19.13f,34.58f);r.now+=40;r.send();
  // Sensor [X,Y] maps to wrist [roll,pitch]=[-Y,X].
  near(r.c.rotation[0],-34.58f);near(r.c.rotation[1],19.13f);
  near(r.c.qReq[0],-29.58f);near(r.c.qReq[1],30.87f);near(r.c.qReq[2],30.87f);
  const float ref=r.c.reference[0];r.advance(6000);
  near(r.c.qOut[0],r.c.qReq[0],.03f);near(r.c.qOut[1],r.c.qOut[2]);
  assert(std::fabs(r.c.velocity[0])<=20&&std::fabs(r.c.velocity[1])<=15);
  r.a();assert(!r.c.armed);near(r.c.reference[0],ref);
  const float held=r.c.qOut[0];sensorQuaternion(r.f,0,-60);r.advance(200);near(r.c.qOut[0],held);
  r.a();r.advance(200);near(r.c.qReq[0],45);near(r.c.reference[0],ref);
  sensorQuaternion(r.f,0,0);r.advance(40);near(r.c.qReq[0],0);near(r.c.qReq[1],45);
}
void sharedVectors() {
  std::ifstream file("firmware/tests/control_vectors.csv");assert(file.good());
  std::string line;std::getline(file,line);unsigned count=0;
  while(std::getline(file,line)) {
    std::istringstream row(line);std::string field;std::getline(row,field,',');
    float values[14];
    for(float &value:values) { assert(bool(std::getline(row,field,',')));value=std::stof(field); }
    Rig r(0,values);r.a();std::memcpy(r.f.q,values+4,sizeof(r.f.q));r.now+=40;r.send();
    for(int i=0;i<3;++i) { near(r.c.rotation[i],values[8+i]);near(r.c.qReq[i],values[11+i]); }
    ++count;
  }
  assert(count==6);
}
void guards() {
  Rig r;r.a();sensorQuaternion(r.f,0,0,170);r.advance(40);assert(!r.c.ambiguous);
  sensorQuaternion(r.f,0,0,190);r.advance(40);assert(r.c.ambiguous);
  sensorQuaternion(r.f,0,0,100);r.advance(40);assert(r.c.ambiguous);
  sensorQuaternion(r.f,0,0,89);r.advance(40);assert(!r.c.ambiguous);
  for(int i=0;i<4;++i)r.f.q[i]=-r.f.q[i];
  r.advance(40);assert(!r.c.ambiguous);
  near(r.c.qReq[0],0);near(r.c.qReq[1],45);
  Frame bad=r.f;bad.seq+=100;bad.uptime+=40;bad.q[0]=8;
  r.now+=40;assert(r.c.receive(bad,r.now,r.now));assert(!r.c.enabled&&!r.c.hasReference);
}
void stableGate() {
  Controller c;Frame f=sample();uint32_t now=0;
  for(int i=0;i<20;++i){now=i*40;f.seq++;f.uptime=now;sensorQuaternion(f,0,0,i*.3f);c.receive(f,now,now);}
  assert(!c.canEnable(now)); // Pure Z rotation fails full SO(3) stability.
  for(int i=0;i<20;++i){now+=40;f.seq++;f.uptime=now;sensorQuaternion(f,70,-40,23);c.receive(f,now,now);}
  assert(c.canEnable(now));assert(c.initialize(now));
  near(c.rotation[0],0);near(c.rotation[1],0);near(c.rotation[2],0);
  f.seq++;f.uptime=now+40;f.flags|=motionarm::buttonA;f.ac++;
  c.receive(f,now+40,now+40);assert(c.armed);
  c.tick(now+40);c.tick(now+160);assert(!c.armed&&c.enabled); // No implicit resume after loop overrun.
}
void gripper() {
  Rig r;r.b();const uint32_t started=r.c.gripStarted;assert(r.c.openReq==0&&r.c.gripCycle==1);
  r.advance(4999);assert(r.c.openReq==0);assert(r.c.gripRemainingMs(r.now)==1);
  r.c.tick(started+5000);r.now=started+5000;
  assert(r.c.openReq==1);assert(!std::strcmp(r.c.gripReason,"GRIPPER_TIMEOUT"));
  r.b();assert(r.c.openReq==1&&r.c.gripCycle==1); // Click during opening is consumed.
  r.advance(300);assert(r.c.openOut==1&&r.c.gripCanClose());
  r.b();assert(r.c.gripCycle==2&&r.c.openReq==0);
  r.advance(100);r.b();assert(r.c.openReq==1);const auto cycle=r.c.gripCycle;
  r.b();assert(r.c.openReq==1&&r.c.gripCycle==cycle);
  r.advance(300);r.b();assert(r.c.openReq==0);
  r.advance(60);r.f.bc+=2;r.now+=40;r.send();assert(r.c.openReq==1);
  assert(!std::strcmp(r.c.gripReason,"BUTTON_EVENTS_COALESCED"));
  r.advance(300);r.b();r.advance(100);
  r.c.stop("STOP",r.now);assert(!r.c.enabled&&!r.c.hasReference&&r.c.openReq==1);
  r.advance(300);assert(r.c.openOut==1&&r.c.gripCanClose());
  assert(r.c.canEnable(r.now+400)==false); // No stable 480ms window after stop yet.
}
void failureAndOrdering() {
  Rig r;r.a();r.b();r.advance(200);
  const auto prior=r.f;const auto age=r.c.ageMs(r.now);
  assert(!r.c.receive(prior,r.now+200,r.now+200));
  assert(r.c.ageMs(r.now+200)==age+200);assert(r.c.duplicates==1);
  Frame reordered=prior;--reordered.seq;--reordered.uptime;
  assert(!r.c.receive(reordered,r.now+210,r.now+210));assert(r.c.outOfOrder==1);
  near(float(r.c.ageMs(r.now+210)),210);
  r.c.tick(r.now+250);assert(!r.c.enabled&&!r.c.hasReference&&r.c.openReq==1);
  r.now+=280;r.send();assert(!r.c.enabled);
  r.advance(600);assert(r.c.initialize(r.now));r.a();r.b();
  Frame reboot=r.f;reboot.boot=8;reboot.seq=1;reboot.uptime=1;
  r.now+=40;assert(r.c.receive(reboot,r.now,r.now));assert(!r.c.enabled&&r.c.openReq==1&&r.c.restarts==1);
  assert(!r.c.receive(prior,r.now,r.now)); // Retired boot cannot reinstate an old session.
  Controller wrap;Frame f=sample(0xffffffff,0xfffffff0);f.ac=f.bc=0xffffffff;
  assert(wrap.receive(f,0xfffffff0,0xfffffff0));f.seq=0;f.uptime=24;
  assert(wrap.receive(f,24,24));assert(wrap.ageMs(34)==10);
  Rig counters;counters.f.ac=counters.f.bc=0xffffffff;counters.now+=40;counters.send();
  assert(!counters.c.enabled); // Backwards jump is not a wrap.
  Rig timeWrap(0xfffffb00);timeWrap.a();timeWrap.b();timeWrap.advance(5000);
  assert(timeWrap.c.openReq==1&&timeWrap.c.armed); // Deadline survives millis wrap.
  Rig counterWrap(0,nullptr,UINT32_MAX);counterWrap.a();counterWrap.b();
  assert(counterWrap.c.enabled&&counterWrap.c.armed&&counterWrap.c.openReq==0);
  assert(counterWrap.f.ac==0&&counterWrap.f.bc==0);
  Rig cycleWrap;cycleWrap.c.gripCycle=UINT32_MAX;cycleWrap.b();cycleWrap.advance(100);
  assert(cycleWrap.c.gripCycle==0&&cycleWrap.c.gripElapsedMs(cycleWrap.now)==100);
  Rig faults;faults.c.setFault("PCA_RESET_OR_OFFLINE",faults.now);faults.advance(600);
  assert(!faults.c.canEnable(faults.now)&&faults.c.canEnableInput(faults.now));
  faults.c.clearFault(faults.now);assert(faults.c.canEnable(faults.now));
  assert(faults.c.initialize(faults.now)&&!faults.c.armed);
  faults.c.stop("STOP",faults.now);const auto event=faults.c.eventSeq;
  faults.c.stop("STOP",faults.now+1);assert(faults.c.eventSeq==event);
}
int main() {
  codec();angles();sharedVectors();guards();stableGate();gripper();failureAndOrdering();
  std::cout<<"Robot regression passed: v2 codec, SO(3), fixed reference, click semantics, limits, deadlines, disconnects, wrap.\n";
}
