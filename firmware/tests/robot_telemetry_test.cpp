#include "../robot/RobotTelemetry.h"
#include <cassert>
#include <iostream>
#include <cstring>
int main() {
  robot::Controller c;
  motionarm::Frame initial;initial.boot=1;initial.flags=9;initial.who=0x68;initial.calibrationSamples=250;initial.ageMs=0;
  for(uint32_t i=0;i<16;++i){initial.seq=i+1;initial.uptime=i*40;c.receive(initial,i*40,i*40);}
  assert(c.initialize(600));initial.seq=17;initial.uptime=640;initial.bc=1;c.receive(initial,640,640);
  c.haveInput=c.hasReference=c.enabled=true;c.gripState=robot::GripState::OpenReady;
  c.input.boot=c.input.seq=c.input.uptime=c.input.ac=c.input.bc=c.input.errors=c.input.rejected=UINT32_MAX;
  c.input.ageMs=UINT32_MAX;c.input.flags=15;c.input.who=255;c.input.calibrationSamples=250;c.input.rejection=5;
  for(float &v:c.input.q)v=-.5f;
  for(float &v:c.reference)v=-.5f;
  for(float &v:c.rotation)v=-179.999f;
  for(float &v:c.input.gyroDps)v=-2028.05f;
  for(float &v:c.input.accelG)v=-4;
  c.input.normG=6.928f;c.input.tempC=-59.85f;
  c.qReq[0]=c.qOut[0]=-45;c.qReq[1]=c.qReq[2]=c.qOut[1]=c.qOut[2]=60;
  c.velocity[0]=-20;c.velocity[1]=c.velocity[2]=-15;
  c.fault="WIRELESS_INIT_OR_IDENTITY";c.reason="FAULT_CLEARED_REINITIALIZE";
  c.gripCycle=UINT32_MAX;c.gripReason="MANAGER_SESSION_CHANGED";
  c.dropped=c.duplicates=c.outOfOrder=c.restarts=c.eventDrops=UINT32_MAX;
  uint32_t seq=UINT32_MAX-8;
  robot::TelemetryBatch batch;
  batch.format(c,UINT32_MAX,"r-ffffffff",UINT32_MAX,seq,"C0:CD:D6:D0:9B:54",false,false,
    UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX);
  std::cerr<<"Robot worst snapshot bytes="<<batch.length<<" valid="<<batch.valid<<"\n";
  for(const char *start=batch.data;*start;) {
    const char *end=std::strchr(start,'\n');assert(end);
    std::cerr<<"line="<<(end-start+1)<<"\n";assert(end-start+1<=512);start=end+1;
  }
  assert(batch.valid);assert(batch.length*5+1000<11520);
}
