#include "../wearable/OrientationFilter.h"
#include <cassert>
#include <iostream>
#include <limits>

constexpr float kPi = 3.14159265358979323846f;
using Quaternion = OrientationFilter::Quaternion;
void close(float actual, float expected, float tolerance = 0.003f) {
  if (std::fabs(actual - expected) > tolerance) {
    std::cerr << "expected " << expected << ", actual " << actual << '\n';
    assert(false);
  }
}
void rotate(OrientationFilter &f, float x, float y, float z, int samples = 100) {
  for (int i = 0; i < samples; ++i) assert(f.update(x, y, z, 0, 0, 1, .01f, false));
}
void quaternion(const OrientationFilter &f, Quaternion expected) {
  const auto &q = f.quaternion();
  // q and -q encode the same body-to-world rotation.
  const float sign = q.w*expected.w+q.x*expected.x+q.y*expected.y+q.z*expected.z < 0 ? -1 : 1;
  close(q.w, sign*expected.w); close(q.x, sign*expected.x);
  close(q.y, sign*expected.y); close(q.z, sign*expected.z);
}
Quaternion axisAngle(float degrees, float x, float y, float z) {
  const float half = degrees*kPi/360;
  return {std::cos(half), x*std::sin(half), y*std::sin(half), z*std::sin(half)};
}
void unit(const OrientationFilter &f) {
  const auto &q = f.quaternion();
  assert(std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z));
  close(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z, 1, .00001f);
}

int main() {
  // Arbitrary initial gravity, including face down, is represented in the
  // transmitted quaternion rather than hidden in a boot-neutral subtraction.
  const float directions[][3]={{0,0,1},{0,1,0},{-1,0,0},{0,0,-1},{.3f,.4f,.8660254f}};
  for (const auto &g:directions) {
    OrientationFilter f;
    assert(f.initialize(g[0]*1.58f,g[1]*1.58f,g[2]*1.58f));
    float x,y,z; f.gravity(x,y,z);
    close(x,g[0]); close(y,g[1]); close(z,g[2]); unit(f);
    const auto initial = f.quaternion();
    rotate(f,20,0,0);
    const float c=std::cos(10*kPi/180), s=std::sin(10*kPi/180);
    quaternion(f, {initial.w*c-initial.x*s, initial.w*s+initial.x*c,
                   initial.y*c+initial.z*s, -initial.y*s+initial.z*c});
  }

  // Stationary recovery checks every gyro axis and raw gravity without scaling.
  StationaryRecovery recovery;
  const float accel158[3]={0,0,1.58f}, quiet[3]={0,0,0};
  for(int i=0;i<24;++i) assert(!recovery.update(accel158,quiet,quiet,1.58f,10000,true));
  assert(recovery.update(accel158,quiet,quiet,1.58f,10000,true));
  close(recovery.reference(),1.58f);
  recovery.reset();
  for(int i=0;i<100;++i) assert(!recovery.update(accel158,quiet,quiet,1.58f,10000,false));
  const float moving[3]={.1f,0,1.58f};
  for(int i=0;i<100;++i)
    assert(!recovery.update(i%2?accel158:moving,quiet,quiet,1.58f,10000,true));
  for(int axis=0;axis<3;++axis) {
    float fast[3]={0,0,0}; fast[axis]=5;
    assert(!recovery.update(accel158,fast,quiet,1.58f,10000,true));
  }
  for(float norm:{.29f,2.01f}) assert(!recovery.update(accel158,quiet,quiet,norm,10000,true));
  assert(!recovery.update(accel158,quiet,quiet,1.58f,50001,true));
  assert(!recovery.update(accel158,quiet,quiet,std::numeric_limits<float>::quiet_NaN(),10000,true));

  OrientationFilter f;
  assert(f.initialize(0,0,1));
  rotate(f,90,0,0); quaternion(f,axisAngle(90,1,0,0));
  // Body Y after body X gives Rx90*Ry90, preserving the full compound attitude.
  rotate(f,0,90,0); quaternion(f,{.5f,.5f,.5f,.5f});
  float x,y,z; f.gravity(x,y,z); close(x,0);close(y,1);close(z,0);

  assert(f.initialize(0,0,1));
  rotate(f,0,120,0); quaternion(f,axisAngle(120,0,1,0));
  for(int i=0;i<200;++i)
    assert(f.update(0,0,0,-std::sin(2*kPi/3),0,std::cos(2*kPi/3),.01f,true));
  quaternion(f,axisAngle(120,0,1,0));

  assert(f.initialize(0,0,1));
  rotate(f,0,0,90); quaternion(f,axisAngle(90,0,0,1));
  assert(f.initialize(0,0,1));
  rotate(f,30,0,0);
  for(int i=0;i<500;++i) assert(f.update(0,0,0,0,0,1.58f,.01f,true));
  quaternion(f,{1,0,0,0}); // Gravity feedback converges without changing raw scale.

  assert(f.initialize(0,0,1));
  rotate(f,0,0,90);
  assert(f.alignGravity(0,0,1));
  quaternion(f,axisAngle(90,0,0,1)); // Reacquisition preserves observable heading.
  assert(f.alignGravity(0,0,-1));
  f.gravity(x,y,z);close(x,0);close(y,0);close(z,-1);unit(f);

  // Firmware does not impose a rotation-vector branch cut at 180 degrees.
  assert(f.initialize(0,0,1)); rotate(f,179,0,0);
  quaternion(f,axisAngle(179,1,0,0));
  rotate(f,2,0,0); quaternion(f,axisAngle(181,1,0,0));
  assert(f.quaternion().w < 0 && f.quaternion().x > 0);

  assert(f.initialize(0,0,1));
  for(int i=0;i<10000;++i) {
    assert(f.update(130+80*std::sin(i*.003f),-180,270,0,0,1,.01f,false)); unit(f);
  }
  assert(f.initialize(0,0,1));
  rotate(f,1000,0,0,9);quaternion(f,axisAngle(90,1,0,0));unit(f);
  for(int i=0;i<10000;++i) {
    assert(f.update(1000,-900,700,0,0,1,.01f,false)); unit(f);
  }
  assert(!f.update(0,0,0,0,0,1,.051f,true));
  assert(!f.update(std::numeric_limits<float>::quiet_NaN(),0,0,0,0,1,.01f,true));
  assert(!f.update(0,0,0,0,0,0,.01f,true));
  std::cout << "Quaternion orientation regression passed: arbitrary gravity, body-to-world composition, all gyro axes, 120deg tilt, feedback, reacquisition, 180deg continuity, normalization and 1000deg/s rotation.\n";
}
