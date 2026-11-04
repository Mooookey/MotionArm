export function normalizedQuaternion(q) {
  const norm = Math.hypot(...q);
  if (!Number.isFinite(norm) || norm < 1e-9) return [1, 0, 0, 0];
  return q.map(v => v / norm);
}

// Hamilton quaternion wxyz, active rotation from sensor/body to world.
export function rotateVector(q, [vx, vy, vz]) {
  const [w, x, y, z] = normalizedQuaternion(q);
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx];
}

export function quaternionToEuler(q) {
  const [w, x, y, z] = normalizedQuaternion(q);
  const degrees = 180 / Math.PI;
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * degrees,
    pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))) * degrees,
    yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * degrees,
  };
}
