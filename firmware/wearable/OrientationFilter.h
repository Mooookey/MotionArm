#pragma once
#include <cmath>
#include <cstdint>
#include <cstddef>

// A recovery window is independent of the gravity magnitude observed at boot.
// Some uncalibrated modules have orientation-dependent scale/bias errors: retain
// the raw norm diagnostics, but refresh only the filter reference after settling.
struct StationaryRecovery {
  uint8_t count = 0;
  float firstAccel[3] = {};
  float firstNorm = 0, normSum = 0;

  void reset() {
    count = 0;
    normSum = 0;
  }

  bool update(const float accel[3], const float gyro[3], const float bias[3],
              float norm, uint32_t elapsedUs, bool buttonsReleased) {
    bool still = buttonsReleased && elapsedUs <= 50000 && norm >= 0.3f && norm <= 2.0f;
    for (size_t i = 0; i < 3; ++i) still &= std::fabs(gyro[i] - bias[i]) < 5;
    if (!still) {
      reset();
      return false;
    }
    if (count == 0) {
      for (size_t i = 0; i < 3; ++i) firstAccel[i] = accel[i];
      firstNorm = norm;
    }
    still = std::fabs(norm - firstNorm) < 0.06f;
    for (size_t i = 0; i < 3; ++i) still &= std::fabs(accel[i] - firstAccel[i]) < 0.06f;
    if (!still) {
      reset();
      return false;
    }
    normSum += norm;
    return ++count >= 25;
  }

  float reference() const { return normSum / count; }
};

// Six-axis body-to-world attitude. Accelerometer magnitude remains a diagnostic;
// only its direction is normalized for gravity fusion. Heading is unobservable
// without a magnetometer and will drift with residual gyro bias.
class OrientationFilter {
 public:
  struct Quaternion { float w, x, y, z; };

  bool initialize(float ax, float ay, float az) {
    q_ = {1, 0, 0, 0};
    if (!alignGravity(ax, ay, az)) return false;
    neutral_ = q_;
    return true;
  }

  // Reacquire tilt after an invalid interval using the smallest correction,
  // preserving the boot-neutral quaternion and as much relative heading as the
  // six-axis measurement permits. No gyro bias or raw sensor scale is changed.
  bool alignGravity(float ax, float ay, float az) {
    if (!unitVector(ax, ay, az)) return false;
    float px, py, pz;
    gravity(px, py, pz);
    const float dot = ax * px + ay * py + az * pz;
    Quaternion delta{1 + dot, ay * pz - az * py,
                      az * px - ax * pz, ax * py - ay * px};
    if (dot < -0.999999f) {
      // Opposite vectors have no unique axis: choose a deterministic orthogonal
      // body axis rather than divide by a nearly zero quaternion norm.
      if (std::fabs(ax) < 0.8f) delta = {0, 0, az, -ay};
      else delta = {0, -az, 0, ax};
    }
    if (!normalize(delta)) return false;
    Quaternion next = multiply(q_, delta);
    if (!normalize(next)) return false;
    q_ = next;
    return true;
  }

  bool update(float gxDps, float gyDps, float gzDps, float ax, float ay,
              float az, float dt, bool trustAccelerometer) {
    if (!std::isfinite(gxDps) || !std::isfinite(gyDps) ||
        !std::isfinite(gzDps) || !std::isfinite(dt) || dt <= 0 || dt > 0.05f) return false;
    constexpr float radians = 0.017453292519943295f;
    float wx = gxDps * radians, wy = gyDps * radians, wz = gzDps * radians;
    if (trustAccelerometer) {
      if (!unitVector(ax, ay, az)) return false;
      float px, py, pz;
      gravity(px, py, pz);
      // Mahony proportional feedback, gain 2/s (~0.5 s tilt time constant).
      // The cross product is in body coordinates, as are all three gyro rates.
      constexpr float kp = 2.0f;
      wx += kp * (ay * pz - az * py);
      wy += kp * (az * px - ax * pz);
      wz += kp * (ax * py - ay * px);
    }
    // Integrate one constant-rate body rotation exactly over this short sample.
    // This avoids Euler-angle cross-axis errors and preserves unit quaternions.
    const float speed = std::sqrt(wx * wx + wy * wy + wz * wz);
    const float halfAngle = 0.5f * speed * dt;
    const float factor = speed > 1e-6f ? std::sin(halfAngle) / speed : 0.5f * dt;
    Quaternion next = multiply(q_, {std::cos(halfAngle), wx * factor, wy * factor, wz * factor});
    if (!normalize(next)) return false;
    q_ = next;
    return true;
  }

  // Relative rotation-vector components about the boot-neutral X/Y axes.
  // Unlike Euler pitch, these do not fold at +/-90 degrees. The SO(3) shortest
  // arc has a +/-180 degree branch cut; a controller must clutch/reanchor there.
  void relativeTilt(float &rollDeg, float &pitchDeg) const {
    Quaternion relative = multiply({neutral_.w, -neutral_.x, -neutral_.y, -neutral_.z}, q_);
    if (relative.w < 0) {
      relative.w = -relative.w;
      relative.x = -relative.x;
      relative.y = -relative.y;
      relative.z = -relative.z;
    }
    const float n = std::sqrt(relative.x * relative.x + relative.y * relative.y + relative.z * relative.z);
    const float scale = n > 1e-6f ? 2 * std::atan2(n, relative.w) / n : 2;
    constexpr float degrees = 57.29577951308232f;
    rollDeg = relative.x * scale * degrees;
    pitchDeg = relative.y * scale * degrees;
  }

  void gravity(float &x, float &y, float &z) const {
    x = 2 * (q_.x * q_.z - q_.w * q_.y);
    y = 2 * (q_.y * q_.z + q_.w * q_.x);
    z = 1 - 2 * (q_.x * q_.x + q_.y * q_.y);
  }

  const Quaternion &quaternion() const { return q_; }

 private:
  Quaternion q_{1, 0, 0, 0}, neutral_{1, 0, 0, 0};

  static Quaternion multiply(const Quaternion &a, const Quaternion &b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
  }

  static bool normalize(Quaternion &q) {
    const float norm = std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if (!std::isfinite(norm) || norm < 1e-6f) return false;
    q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm;
    return true;
  }

  static bool unitVector(float &x, float &y, float &z) {
    const float norm = std::sqrt(x*x+y*y+z*z);
    if (!std::isfinite(norm) || norm < 1e-6f) return false;
    x /= norm; y /= norm; z /= norm;
    return true;
  }
};
