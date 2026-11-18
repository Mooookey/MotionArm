#include "../wearable/OrientationFilter.h"
#include <cassert>
#include <iostream>
#include <limits>

constexpr float kPi = 3.14159265358979323846f;
void close(float actual, float expected, float tolerance = 0.003f) {
  if (std::fabs(actual - expected) > tolerance) {
    std::cerr << "expected " << expected << ", actual " << actual << '\n';
    assert(false);
  }
}
void rotate(OrientationFilter &f, float x, float y, float z, int samples = 100) {
  for (int i = 0; i < samples; ++i) assert(f.update(x, y, z, 0, 0, 1, .01f, false));
}
void tilt(const OrientationFilter &f, float expectedRoll, float expectedPitch) {
  float roll, pitch;
  f.relativeTilt(roll, pitch);
  close(roll, expectedRoll); close(pitch, expectedPitch);
}
void unit(const OrientationFilter &f) {
  const auto &q = f.quaternion();
  close(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z, 1, .00001f);
  float roll, pitch;
  f.relativeTilt(roll, pitch);
  assert(std::isfinite(roll) && std::isfinite(pitch));
  assert(std::fabs(roll) <= 180.001f && std::fabs(pitch) <= 180.001f);
}

int main() {
  // Gravity initialization must work in arbitrary initial poses, including
  // upside down. Scaling to1.58g never alters the estimated direction.
  const float directions[][3]={{0,0,1},{0,1,0},{-1,0,0},{0,0,-1},{.3f,.4f,.8660254f}};
  for (const auto &g:directions) {
    OrientationFilter f;
    assert(f.initialize(g[0]*1.58f,g[1]*1.58f,g[2]*1.58f));
    float x,y,z; f.gravity(x,y,z);
    close(x,g[0]); close(y,g[1]); close(z,g[2]);
    tilt(f,0,0); unit(f);
  }

  // Both startup and ongoing stationary reference policy allow1.58g raw
  // magnitude while requiring released buttons and a stable direction/window.
  StationaryRecovery recovery;
  const float accel158[3]={0,0,1.58f}, quiet[3]={0,0,0};
  for(int i=0;i<24;++i) assert(!recovery.update(accel158,quiet,quiet,1.58f,10000,true));
  assert(recovery.update(accel158,quiet,quiet,1.58f,10000,true));
  close(recovery.reference(),1.58f);
  recovery.reset();
  for(int i=0;i<100;++i) assert(!recovery.update(accel158,quiet,quiet,1.58f,10000,false));
  const float moving[3]={.1f,0,1.58f}, fast[3]={0,5,0};
  for(int i=0;i<100;++i)
    assert(!recovery.update(i%2?accel158:moving,quiet,quiet,1.58f,10000,true));
  assert(!recovery.update(accel158,fast,quiet,1.58f,10000,true));
  for(float norm:{.29f,2.01f}) assert(!recovery.update(accel158,quiet,quiet,norm,10000,true));
  assert(!recovery.update(accel158,quiet,quiet,1.58f,50001,true));
  assert(!recovery.update(accel158,quiet,quiet,std::numeric_limits<float>::quiet_NaN(),10000,true));

  OrientationFilter f;
  assert(f.initialize(0,0,1));
  rotate(f,90,0,0);
  tilt(f,90,0);
  // BodyY at roll90 couples into world heading. The result must be the
  // quaternion Rx90*Ry90=(.5,.5,.5,.5), not independent Euler(90,90).
  rotate(f,0,90,0);
  const auto q=f.quaternion();
  close(q.w,.5f);close(q.x,.5f);close(q.y,.5f);close(q.z,.5f);
  const float component=120/std::sqrt(3.0f);
  tilt(f,component,component);
  float x,y,z; f.gravity(x,y,z);
  close(x,0);close(y,1);close(z,0);

  assert(f.initialize(0,0,1));
  rotate(f,0,120,0);
  tilt(f,0,120); // No EulerPitch folding at90deg.
  for(int i=0;i<200;++i)
    assert(f.update(0,0,0,-std::sin(2*kPi/3),0,std::cos(2*kPi/3),.01f,true));
  tilt(f,0,120);

  assert(f.initialize(0,0,1));
  rotate(f,0,0,90);
  tilt(f,0,0); // Pure relative yaw does not invent X/Y tilt.
  close(f.quaternion().z,std::sqrt(.5f));

  // Relative output is anchored in the initial sensor axes, not world Euler.
  assert(f.initialize(.3f,.4f,.8660254f));
  rotate(f,20,0,0);
  tilt(f,20,0);

  assert(f.initialize(0,0,1));
  rotate(f,30,0,0);
  for(int i=0;i<500;++i) assert(f.update(0,0,0,0,0,1.58f,.01f,true));
  tilt(f,0,0); // Gravity feedback sign must converge, including scaled input.

  // Invalid-interval tilt reacquisition preserves neutral and remains finite
  // even when measured and predicted gravity are exactly opposite.
  assert(f.initialize(0,0,1));
  assert(f.alignGravity(0,0,-1));
  f.gravity(x,y,z);close(x,0);close(y,0);close(z,-1);unit(f);

  // Shortest-arc branch is explicit at180; values cannot grow without bound.
  assert(f.initialize(0,0,1));rotate(f,179,0,0);tilt(f,179,0);
  rotate(f,2,0,0);tilt(f,-179,0);

  assert(f.initialize(0,0,1));
  for(int i=0;i<10000;++i) {
    assert(f.update(130+80*std::sin(i*.003f),-180,270,0,0,1,.01f,false));
    unit(f);
  }
  // Natural fast wrist turns above the old +/-500deg/s range remain correct
  // at100Hz in the new +/-2000deg/s mode (1000deg/s *90ms=90deg).
  assert(f.initialize(0,0,1));
  rotate(f,1000,0,0,9);tilt(f,90,0);unit(f);
  for(int i=0;i<10000;++i) {
    assert(f.update(1000,-900,700,0,0,1,.01f,false));
    unit(f);
  }
  assert(!f.update(0,0,0,0,0,1,.051f,true));
  assert(!f.update(std::numeric_limits<float>::quiet_NaN(),0,0,0,0,1,.01f,true));
  assert(!f.update(0,0,0,0,0,0,.01f,true));
  std::cout << "Quaternion orientation regression passed: gravity, all gyros, compound rotation,120deg tilt, relative neutral, feedback, opposite vectors, wrap, normalization,1000deg/s rotation.\n";
}
