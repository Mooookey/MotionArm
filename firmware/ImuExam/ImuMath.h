#pragma once
#include <math.h>
#include <stdint.h>

// Original implementation of the quaternion complementary-filter equations
// described by Mahony et al.; see ALGORITHM.md. No third-party source copied.
struct ImuMath {
  static constexpr float G = 9.80665f;
  float q[4] = {1, 0, 0, 0}; // wxyz, active rotation body -> world
  float filtered[6] = {};
  float velocity[3] = {}, position[3] = {}, previousAcceleration[3] = {};
  bool filterReady = false;
  uint64_t previousUs = 0, stationarySinceUs = 0;
  bool stationary = false;

  void zeroPosition() {
    for (int i = 0; i < 3; ++i) velocity[i] = position[i] = previousAcceleration[i] = 0;
  }

  void initialize(const float a[3]) {
    float roll = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    float cr = cosf(roll * .5f), sr = sinf(roll * .5f);
    float cp = cosf(pitch * .5f), sp = sinf(pitch * .5f);
    q[0] = cr * cp; q[1] = sr * cp; q[2] = cr * sp; q[3] = -sr * sp;
    previousUs = 0; stationarySinceUs = 0; stationary = false; filterReady = false;
    zeroPosition();
  }

  // Returns true when a gap prevents integration across this sample interval.
  bool update(const float raw[6], const float bias[3], uint64_t now, float accelNormScale = 1) {
    float dt = previousUs ? (now - previousUs) * 1e-6f : .005f;
    bool gap = previousUs && (dt > .020f || dt <= 0);
    previousUs = now;
    float alpha = 1 - expf(-2 * 3.14159265358979323846f * 20 * (gap ? .005f : dt));
    for (int i = 0; i < 6; ++i) {
      float value = i < 3 ? raw[i] * accelNormScale : raw[i] - bias[i - 3];
      if (!filterReady || gap) filtered[i] = value;
      else filtered[i] += alpha * (value - filtered[i]);
    }
    filterReady = true;
    float ax = filtered[0], ay = filtered[1], az = filtered[2];
    float accelerationNorm = sqrtf(ax * ax + ay * ay + az * az);
    float gx = filtered[3] * .0174532925199433f;
    float gy = filtered[4] * .0174532925199433f;
    float gz = filtered[5] * .0174532925199433f;
    float gyroNormDps = sqrtf(filtered[3]*filtered[3] + filtered[4]*filtered[4] + filtered[5]*filtered[5]);
    if (gap) {
      stationarySinceUs = 0; stationary = false;
      for (int i = 0; i < 3; ++i) velocity[i] = previousAcceleration[i] = 0;
      return true;
    }

    // Gravity-based tilt correction; acceleration outside 0.8..1.2g is rejected.
    if (accelerationNorm > .8f * G && accelerationNorm < 1.2f * G) {
      ax /= accelerationNorm; ay /= accelerationNorm; az /= accelerationNorm;
      float vx = 2 * (q[1]*q[3] - q[0]*q[2]);
      float vy = 2 * (q[0]*q[1] + q[2]*q[3]);
      float vz = 1 - 2 * (q[1]*q[1] + q[2]*q[2]);
      gx += .5f * (ay*vz - az*vy);
      gy += .5f * (az*vx - ax*vz);
      gz += .5f * (ax*vy - ay*vx);
    }
    float w = q[0], x = q[1], y = q[2], z = q[3], h = .5f * dt;
    q[0] += (-x*gx - y*gy - z*gz) * h;
    q[1] += ( w*gx + y*gz - z*gy) * h;
    q[2] += ( w*gy - x*gz + z*gx) * h;
    q[3] += ( w*gz + x*gy - y*gx) * h;
    float inverseNorm = 1 / sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; ++i) q[i] *= inverseNorm;
    w=q[0]; x=q[1]; y=q[2]; z=q[3];
    ax=filtered[0]; ay=filtered[1]; az=filtered[2];
    float earth[3] = {
      (1-2*(y*y+z*z))*ax + 2*(x*y-w*z)*ay + 2*(x*z+w*y)*az,
      2*(x*y+w*z)*ax + (1-2*(x*x+z*z))*ay + 2*(y*z-w*x)*az,
      2*(x*z-w*y)*ax + 2*(y*z+w*x)*ay + (1-2*(x*x+y*y))*az - G
    };
    float linearNorm = sqrtf(earth[0]*earth[0] + earth[1]*earth[1] + earth[2]*earth[2]);
    bool quiet = fabsf(accelerationNorm - G) < .04f * G && gyroNormDps < 2 && linearNorm < .35f;
    if (!quiet) stationarySinceUs = 0;
    else if (!stationarySinceUs) stationarySinceUs = now;
    stationary = stationarySinceUs && now - stationarySinceUs >= 200000;
    for (int i = 0; i < 3; ++i) {
      if (stationary) velocity[i] = 0;
      else {
        float oldVelocity = velocity[i];
        velocity[i] += .5f * (previousAcceleration[i] + earth[i]) * dt;
        position[i] += .5f * (oldVelocity + velocity[i]) * dt;
      }
      previousAcceleration[i] = earth[i];
    }
    return false;
  }
};
