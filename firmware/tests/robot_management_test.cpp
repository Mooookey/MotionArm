#define MOTIONARM_HOST_TEST 1
#include "../robot/robot.ino"
#include <cassert>
#include <iostream>
struct Message {
  cJSON root;unsigned seq;
  explicit Message(const char *kind,unsigned n):seq(n){number("v",kVersion);number("seq",n);text("type",kind);}
  Message &add(const char *key,cJSON::Kind kind){auto *n=new cJSON;n->type=kind;n->string=hostCopy(key);
    auto **at=&root.child;while(*at)at=&(*at)->next;*at=n;return *this;}
  cJSON *last(){auto *n=root.child;while(n&&n->next)n=n->next;return n;}
  Message &text(const char *key,const char *v){add(key,cJSON::String);last()->valuestring=hostCopy(v);return *this;}
  Message &number(const char *key,double v){add(key,cJSON::Number);last()->valuedouble=v;return *this;}
  Message &yes(const char *key){return add(key,cJSON::True);}
  Message &session(){return text("session_id",sessionId);}
#if MOTIONARM_POWERED_TRIAL
  Message &bound(const char *id){return session().text("boot_id",bootId).text("profile_hash",kPoweredTrialProfile.hash).text("request_id",id);}
#endif
  std::string send(){Serial.output.clear();dispatch(&root,hostNow);flushManagementResponses();return Serial.output;}
  ~Message(){for(auto *n=root.child;n;){auto *next=n->next;cJSON_Delete(n);n=next;}root.child=nullptr;}
};
int main(){
  setup();assert(pcaReady&&wirelessReady);assert(lockedPcaReadable());assert(Wire.effectiveWrites==0);
#if MOTIONARM_POWERED_TRIAL
  assert(Serial.clockSource==UART_CLK_SRC_APB);
#else
  assert(Serial.clockSource==-1);
#endif
  assert(emit(std::string(511,'x').c_str(),511));assert(Serial.output.size()==512);
  assert(!emit(std::string(512,'x').c_str(),512));Serial.output.clear();
  Serial.input=std::string(512,'x')+"\n";for(unsigned i=0;i<6;++i)readSerial();assert(hostParseCalls==0);
  Serial.input+=std::string(511,'x')+"\n";for(unsigned i=0;i<6;++i)readSerial();assert(hostParseCalls==1);
  motionarm::Frame f;f.boot=9;f.ageMs=0;f.flags=motionarm::imuValid|motionarm::buttonsPrimed;f.who=0x68;f.calibrationSamples=250;
  auto sample=[&](unsigned ms){hostNow=ms;f.seq++;f.uptime=ms;controller.receive(f,ms,ms);};
  for(unsigned ms=40;ms<=640;ms+=40)sample(ms);
  Serial.txCapacity=0;
  Message hello("hello",1);hello.text("role","pc");assert(hello.send().empty());
  assert(managementResponses.count()==1);const std::string acceptedSession=sessionId;
  telemetry(hostNow);emitEvents();flushManagementResponses();assert(Serial.output.empty());
  assert(managementResponses.count()==1 && acceptedSession==sessionId);
  Serial.txCapacity=8192;flushManagementResponses();
  assert(Serial.output.find("hello_ack")!=std::string::npos && managementResponses.empty());
  const auto helloBytes=Serial.output;flushManagementResponses();assert(Serial.output==helloBytes);
  telemetry(hostNow);assert(Serial.output.compare(0,helloBytes.size(),helloBytes)==0);
  assert(Serial.output.find("\"type\":\"control\"",helloBytes.size())!=std::string::npos);
#if MOTIONARM_POWERED_TRIAL
  Message stale("prepare",2);stale.session().text("boot_id","r-stale").text("profile_hash",kPoweredTrialProfile.hash).text("request_id","stale");
  assert(stale.send().find("STALE_BOOT_OR_PROFILE")!=std::string::npos);assert(!output.prepared);
  Message badSession("prepare",3);badSession.text("session_id","old").text("boot_id",bootId).text("profile_hash",kPoweredTrialProfile.hash).text("request_id","bad-session");
  assert(badSession.send().find("STALE_SESSION")!=std::string::npos);
  Message prep("prepare",4);prep.bound("p1");assert(prep.send().find("\"ok\":true")!=std::string::npos);assert(output.prepared);
  Message missing("enable_output",5);missing.bound("e-missing");assert(missing.send().find("ONSITE_CONFIRMATION_REQUIRED")!=std::string::npos);
  assert(Wire.effectiveWrites==0&&output.locked());
  Message changedFailed("enable_output",6);changedFailed.bound("e-missing").yes("supported").yes("power_confirmed").yes("startup_aligned");
  assert(changedFailed.send().find("REQUEST_ID_REUSED")!=std::string::npos);
  Message enable("enable_output",7);enable.bound("e1").yes("supported").yes("power_confirmed").yes("startup_aligned");
  assert(enable.send().find("\"ok\":true")!=std::string::npos);assert(Wire.effectiveWrites==1&&output.enabled());
  Message changed("test_joint",8);changed.bound("e1").number("axis",0).number("offset_deg",3);
  assert(changed.send().find("REQUEST_ID_REUSED")!=std::string::npos);
  Message diagnostic("test_joint",9);diagnostic.bound("d1").number("axis",0).number("offset_deg",3);
  assert(diagnostic.send().find("\"ok\":true")!=std::string::npos);assert(controller.diagnosticActive);
  hostNow=660;loop();const float committed=output.target[0];assert(committed>0&&committed<.01f);
  Message stopMessage("stop",10);stopMessage.session();assert(stopMessage.send().find("\"ok\":true")!=std::string::npos);
  assert(controller.qOut[0]==committed&&!controller.hasReference&&!controller.enabled);
  const unsigned writes=Wire.effectiveWrites;
  Message duplicate("enable_output",11);duplicate.bound("e1").yes("supported").yes("power_confirmed").yes("startup_aligned");
  assert(duplicate.send().find("\"ok\":true")!=std::string::npos);assert(Wire.effectiveWrites==writes&&!output.prepared);
  Message prep2("prepare",12);prep2.bound("p2");assert(prep2.send().find("\"ok\":true")!=std::string::npos);
  Message diag2("test_joint",13);diag2.bound("d2").number("axis",0).number("offset_deg",3);diag2.send();
  hostNow+=301;loop();assert(!managerActive&&!output.prepared&&!controller.diagnosticActive);
  assert(controller.qOut[0]==committed&&output.target[0]==committed);assert(output.enabled());
  // A bus failure never commits a speculative target and never auto-FULL_OFFs.
  sample(hostNow+40);Message prep3("prepare",14);prep3.bound("p3");prep3.send();
  Message diag3("test_joint",15);diag3.bound("d3").number("axis",0).number("offset_deg",3);diag3.send();
  const unsigned offWrites=Wire.fullOffWrites;Wire.ok=false;
  // Quantized targets may legitimately avoid I2C until the next tick changes.
  float lastSuccessful=output.target[0];
  for(unsigned i=0;i<20 && output.enabled();++i){lastSuccessful=output.target[0];sample(hostNow+20);managerAt=hostNow;loop();}
  assert(output.stage==robot::PoweredOutput::Stage::Unknown&&controller.fault);
  assert(controller.qOut[0]==output.target[0]&&Wire.fullOffWrites==offWrites);
  const float faultTarget=output.target[0];
  Wire.ok=true;hostNow+=501;loop();assert(output.stage==robot::PoweredOutput::Stage::Unknown);
  assert(output.target[0]==faultTarget&&controller.qOut[0]==faultTarget);
  Message release("disable_output",16);release.bound("off1");assert(release.send().find("SUPPORT_CONFIRMATION_REQUIRED")!=std::string::npos);
  assert(Wire.fullOffWrites==offWrites);
  Message releaseOk("disable_output",17);releaseOk.bound("off2").yes("supported");releaseOk.send();
  assert(output.locked()&&lockedPcaReadable());
  sample(hostNow+40);Message prep4("prepare",18);prep4.bound("p4");prep4.send();assert(output.prepared);
  --f.bc;sample(hostNow+40);assert(!controller.valid(hostNow));loop();
  assert(!output.prepared&&!managerActive&&!controller.hasReference);
  std::cout<<"actual powered dispatch/loop gates, idempotency, STOP/timeout and I2C failure passed\n";
#else
  Message rejected("enable_output",2);rejected.session().yes("supported").yes("power_confirmed").yes("startup_aligned");
  assert(rejected.send().find("POWERED_OUTPUT_UNAVAILABLE")!=std::string::npos);
  for(unsigned ms=680;ms<=1320;ms+=40){sample(ms);loop();}
  Message init("init",3);init.session().text("request_id","i1");assert(init.send().find("\"ok\":true")!=std::string::npos);
  f.ac++;sample(1360);assert(controller.armed);
  for(unsigned ms=1380;ms<=1600;ms+=20){sample(ms);loop();}
  assert(Wire.effectiveWrites==0&&lockedPcaReadable());
  std::cout<<"actual no-power dispatch/loop cannot unlock PWM\n";
#endif
  // Queue saturation never executes a new action without retaining its reply.
  Serial.output.clear();Serial.txCapacity=0;
  for(unsigned i=0;i<robot::ManagementOutbox::capacity;++i)ack(400+i,"command","OLD_RESPONSE");
  assert(managementResponses.full());
  const auto beforeFull=lastCommand;
  Message overflow("init",500);overflow.session().text("request_id","must-not-run");
  assert(overflow.send().empty());assert(lastCommand==beforeFull);
  // STOP remains effective under saturation, and its reply is retained.
  Message saturatedStop("stop",501);saturatedStop.session();assert(saturatedStop.send().empty());
  assert(managementResponses.count()==1 && !controller.enabled && !controller.hasReference);
  // New HELLO supersedes all queued responses from the old management session.
  Message replacementHello("hello",502);replacementHello.text("role","pc");assert(replacementHello.send().empty());
  assert(managementResponses.count()==1);Serial.txCapacity=8192;flushManagementResponses();
  assert(Serial.output.find("hello_ack")!=std::string::npos && Serial.output.find("OLD_RESPONSE")==std::string::npos);
  assert(Serial.output.find("\"request_type\":\"stop\"")==std::string::npos);
  const auto sentOnce=Serial.output;flushManagementResponses();assert(Serial.output==sentOnce);
  std::cout<<"management outbox pressure, whole-line priority, supersession and once-only drain passed\n";
}
