#pragma once
#include "PoweredOutput.h"
// Generated scoped trial; full travel, load and thermal safety remain unverified.
inline constexpr robot::TrialProfile kPoweredTrialProfile{
  "sha256:395bc263b9436fe91a9607744ce6df19d4a518f7a5f38086dc7406b636821e53",
  {-45,20,20}, {45,60,60}, {0,45,45}, 15,30,1,
  {1010,1610,2000}, {1630,1030}, {1570,1050}, 1130,1630,4.88f,
  5, {{-15,-10,-5,0,5,10,15},{-10,-5,0,5,10,0,0},{-10,-5,0,5,10,0,0}}, {7,5,5},
  {-1,-1}, "coordinated-angle-v4"
};
