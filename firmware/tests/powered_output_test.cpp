#include "../robot/PoweredOutput.h"
#include "../robot/PoweredTrialProfile.h"
#include "../robot/RobotController.h"
#include <cassert>
#include <cmath>
#include <iostream>
struct Bus : robot::OutputTransport {
  unsigned writes=0,offs=0;bool ok=true;
  bool writeTicks(const uint16_t *) override { ++writes;return ok; }
  bool fullOff() override {++offs;return ok;}
};
void near(float a,float b){assert(std::fabs(a-b)<.01f);}
int main() {
  Bus bus;robot::PoweredOutput out(kPoweredTrialProfile);
  assert(out.locked() && !out.enabled() && !out.committed);
  near(out.mappedUs(0,-45),1010);near(out.mappedUs(0,0),1610);near(out.mappedUs(0,45),2000);
  near(out.mappedUs(1,0),1630);near(out.mappedUs(1,45),1030);near(out.mappedUs(2,0),1570);near(out.mappedUs(2,45),1050);
  uint16_t ticks[4];float edge[3];std::memcpy(edge,kPoweredTrialProfile.high,sizeof(edge));assert(out.quantize(edge,1,ticks));
  for(unsigned i=0;i<3;++i) {
    const float pulse=ticks[i]*4.88f;
    assert(pulse>=std::fmin(out.mappedUs(i,kPoweredTrialProfile.low[i]),out.mappedUs(i,kPoweredTrialProfile.high[i]))-.001f);
    assert(pulse<=std::fmax(out.mappedUs(i,kPoweredTrialProfile.low[i]),out.mappedUs(i,kPoweredTrialProfile.high[i]))+.001f);
  }
  for(unsigned i=0;i<3;++i) {
    edge[i]=kPoweredTrialProfile.high[i]+.01f;assert(!out.quantize(edge,1,ticks));edge[i]=kPoweredTrialProfile.high[i];
    float below[3];std::memcpy(below,kPoweredTrialProfile.startup,sizeof(below));
    below[i]=kPoweredTrialProfile.low[i]-.01f;assert(!out.quantize(below,1,ticks));
    below[i]=kPoweredTrialProfile.low[i];assert(out.quantize(below,1,ticks));
    const float pulse=ticks[i]*kPoweredTrialProfile.tickUs;
    assert(pulse>=std::fmin(out.mappedUs(i,kPoweredTrialProfile.low[i]),out.mappedUs(i,kPoweredTrialProfile.high[i]))-.001f);
    assert(pulse<=std::fmax(out.mappedUs(i,kPoweredTrialProfile.low[i]),out.mappedUs(i,kPoweredTrialProfile.high[i]))+.001f);
  }
  assert(!out.enable(bus,true,true,true,1));out.prepared=true;
  assert(!out.enable(bus,false,true,true,1));assert(!out.enable(bus,true,true,false,1));assert(bus.writes==0);
  assert(out.enable(bus,true,true,true,10));assert(out.enabled() && !out.locked());
  assert(out.submit(bus,kPoweredTrialProfile.startup,1,11));assert(bus.writes==1 && out.lastWriteMs==10);
  bus.ok=false;assert(!out.submit(bus,edge,0,12));assert(out.stage==robot::PoweredOutput::Stage::Unknown);
  assert(out.writeErrors==1 && out.target[0]==0 && out.opening==1 && out.lastWriteMs==10);
  assert(!out.submit(bus,edge,0,13));assert(bus.writes==2);
  assert(!out.disable(bus,false,14));assert(bus.offs==0);
  bus.ok=true;assert(out.disable(bus,true,15));assert(out.locked() && !out.powerConfirmed);
  robot::Controller c;c.configureTrial(kPoweredTrialProfile);
  motionarm::Frame f;f.boot=1;f.ageMs=0;f.flags=motionarm::imuValid|motionarm::buttonsPrimed;f.who=0x68;f.calibrationSamples=250;
  for(unsigned ms=0;ms<=600;ms+=40){f.seq++;f.uptime=ms;c.receive(f,ms,ms);c.tick(ms);}
  c.qOut[0]=2;c.qOut[1]=47;c.qOut[2]=43;
  assert(c.initialize(600));assert(c.initializing && c.qOut[0]==2 && c.qOut[1]==47 && c.qOut[2]==43);
  f.ac++;f.seq++;f.uptime=640;c.receive(f,640,640);assert(!c.armed);
  float before=c.qOut[0];c.tick(640);assert(std::fabs(c.qOut[0]-before)<=kPoweredTrialProfile.speed*.04f+.001f);
  c.stop("STOP",640);const float held=c.qOut[0];assert(!c.enabled && !c.hasReference);
  assert(c.testJoint(0,-5,640));
  for(unsigned ms=660;ms<=760;ms+=20){c.tick(ms);if(ms%40==0){f.seq++;f.uptime=ms;c.receive(f,ms,ms);}}
  // Early, unsaturated motion: semi-implicit discrete acceleration bound.
  assert(std::fabs(c.qOut[0]-held)<=.5f*kPoweredTrialProfile.acceleration*.12f*(.12f+.02f)+.001f);
  c.tick(1000);assert(!c.diagnosticActive && !c.enabled);
  c.qOut[0]=0;c.qOut[1]=c.qOut[2]=45;c.openOut=.5f;c.openReq=1;
  c.tick(1020,false);assert(c.openOut==.5f);
  robot::Controller grip;grip.configureTrial(kPoweredTrialProfile);f=motionarm::Frame{};f.boot=2;f.ageMs=0;
  f.flags=motionarm::imuValid|motionarm::buttonsPrimed;f.who=0x68;f.calibrationSamples=250;
  for(unsigned ms=0;ms<=600;ms+=40){f.seq++;f.uptime=ms;grip.receive(f,ms,ms);grip.tick(ms);}
  assert(grip.initialize(600));++f.bc;f.seq++;f.uptime=640;grip.receive(f,640,640);
  assert(grip.openReq==0 && grip.gripStarted==640 && !grip.armed);
  for(unsigned ms=660;ms<=5620;ms+=20) {
    if(ms%40==0){f.seq++;f.uptime=ms;grip.receive(f,ms,ms);}
    grip.tick(ms);
  }
  grip.tick(5639);assert(grip.openReq==0 && grip.gripRemainingMs(5639)==1);
  grip.tick(5640);assert(grip.openReq==1 && grip.gripRemainingMs(5640)==0);
  ++f.bc;f.seq++;f.uptime=5640;grip.receive(f,5640,5640);
  assert(grip.gripCycle==1 && grip.openReq==1); // opening clicks are consumed
  for(unsigned ms=5660;ms<=6700;ms+=20){if(ms%40==0){f.seq++;f.uptime=ms;grip.receive(f,ms,ms);}grip.tick(ms);}
  assert(grip.openOut==1 && grip.gripCanClose());
  grip.stop("STOP",6700);assert(grip.testJoint(0,0,6700));grip.tick(6720);
  assert(!grip.diagnosticActive && grip.qOut[0]==0);
  for(unsigned axis=0;axis<3;++axis) {
    for(unsigned i=0;i<kPoweredTrialProfile.diagnosticCounts[axis];++i) {
      grip.stop("STOP",6720);
      const float offset=kPoweredTrialProfile.diagnosticOffsets[axis][i];
      assert(grip.testJoint(axis,offset,6720));near(grip.qReq[axis],kPoweredTrialProfile.startup[axis]+offset);
    }
    for(float offset:{-45.f,-25.f,-20.f,-15.f,-7.5f,1.f,3.f,5.01f,15.f,20.f,25.f,45.f}) {
      if(kPoweredTrialProfile.permitsDiagnostic(axis,offset))continue;
      grip.stop("STOP",6720);assert(!grip.testJoint(axis,offset,6720));
    }
  }
  robot::Controller following;following.configureTrial(kPoweredTrialProfile);f=motionarm::Frame{};
  for(float speed:following.speeds)near(speed,15);
  near(following.acceleration,30);
  assert(following.jointSigns[0]==-1 && following.jointSigns[1]==-1);
  assert(!std::strcmp(kPoweredTrialProfile.mappingVersion,"coordinated-angle-v4"));
  f.boot=3;f.ageMs=0;f.flags=9;f.who=0x68;f.calibrationSamples=250;
  for(unsigned ms=0;ms<=600;ms+=40){f.seq++;f.uptime=ms;following.receive(f,ms,ms);following.tick(ms);}
  assert(following.initialize(600));f.ac++;f.seq++;f.uptime=640;following.receive(f,640,640);assert(following.armed);
  unsigned timestamp=640;
  auto pose=[&](unsigned axis,float angle) {
    f.q[0]=std::cos(angle*.00872664626f);f.q[1]=f.q[2]=f.q[3]=0;f.q[axis+1]=std::sin(angle*.00872664626f);
    f.seq++;f.uptime=timestamp+=40;following.receive(f,timestamp,timestamp);
  };
  pose(1,-20);near(following.rotation[0],20);near(following.qReq[0],-15);
  pose(1,20);near(following.rotation[0],-20);near(following.qReq[0],15);
  pose(1,-60);near(following.qReq[0],-45);pose(1,60);near(following.qReq[0],45);
  pose(1,-5);near(following.qReq[0],0);
  pose(0,10);near(following.qReq[1],40);near(following.qReq[2],40);
  pose(0,-10);near(following.qReq[1],50);near(following.qReq[2],50);
  pose(0,30);near(following.qReq[1],20);near(following.qReq[2],20);
  pose(0,-30);near(following.qReq[1],60);near(following.qReq[2],60);
  pose(2,30);near(following.rotation[2],30);near(following.qReq[0],0);near(following.qReq[1],45); // yaw mapping deliberately unchanged
  grip.stop("STOP",6720);assert(!grip.testJoint(3,0,6720));
  robot::Controller noPower;
  assert(noPower.low[0]==-45 && noPower.high[0]==45 && noPower.low[1]==0 && noPower.high[1]==60);
  assert(noPower.speeds[0]==20 && noPower.speeds[1]==15 && noPower.acceleration==60);
  assert(noPower.jointSigns[0]==1 && noPower.jointSigns[1]==-1);
  std::cout<<"powered mapping, lifecycle, commit failure, bounded init/diagnostic passed\n";
}

