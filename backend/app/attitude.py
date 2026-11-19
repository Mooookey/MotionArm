"""Quaternion geometry: Hamilton [w,x,y,z], sensor-to-world attitude."""
import math


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def normalize(q):
    norm = math.hypot(*q)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid quaternion norm")
    return [value / norm for value in q]


def relative(reference, current):
    w, x, y, z = reference
    return normalize(multiply([w, -x, -y, -z], current))


def distance_deg(a, b):
    q = relative(a, b)
    return math.degrees(2 * math.atan2(math.hypot(*q[1:]), abs(q[0])))


def mean(quaternions):
    anchor = quaternions[0]
    aligned = [q if sum(a*b for a, b in zip(anchor, q)) >= 0 else [-v for v in q]
               for q in quaternions]
    return normalize([sum(row[index] for row in aligned) for index in range(4)])


def wrist_rotation_vector(reference, current):
    """Wrist axes: fingertip=-sensor Y, right=+sensor X, down=+sensor Z."""
    w, x, y, z = relative(reference, current)
    q = [w, -y, x, z]
    if q[0] < 0:
        q = [-v for v in q]
    length = math.hypot(*q[1:])
    scale = math.degrees(2 * math.atan2(length, q[0])) / length if length > 1e-12 else math.degrees(2)
    return [v * scale for v in q[1:]]
