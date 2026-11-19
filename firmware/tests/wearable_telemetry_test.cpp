#include "../wearable/WearableTelemetry.h"
#include <cassert>
#include <cstring>
#include <iostream>

int main() {
  // Every uint32 counter at its maximum; negative float signs and maximum
  // sensor widths, including bias-corrected gyro beyond nominal +/-2000dps.
  WearableTelemetry t{UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,255,true,250,
      {-.5f,-.5f,-.5f,-.5f}, {-2028.05f,-2028.05f,-2028.05f}, {-4,-4,-4},
      true,true,UINT32_MAX,UINT32_MAX,6.928f,-59.85f,UINT32_MAX,5,UINT32_MAX};
  char frame[kTelemetryMaxBytes+1];
  const int length=serializeWearable(frame,sizeof(frame),t);
  assert(length > 0 && size_t(length)<=kTelemetryMaxBytes);
  assert(size_t(length)==std::strlen(frame) && frame[length-1]=='\n');
  assert(std::strstr(frame,"\"att\":3") && std::strstr(frame,"\"q\":[-0.500000,-0.500000,-0.500000,-0.500000]"));
  for(const char *old:{"\"roll\"","\"pitch\"","\"mode\"","\"short\"","\"long\""})
    assert(!std::strstr(frame,old));
  // 8N1 at 115200 baud, 25Hz; the worst frame leaves UART headroom.
  assert(length*25*10 < 115200);
  char small[10];
  assert(serializeWearable(small,sizeof(small),t)==length);
  assert(small[sizeof(small)-1]=='\0');
  std::cout << "Wearable att3 telemetry regression passed: worst frame " << length
            << " bytes, " << length*25 << " bytes/s at 25Hz.\n";
}
