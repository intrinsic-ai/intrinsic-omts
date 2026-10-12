# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pure-Python 3D rigid-body pose, quaternion, and Euler RPY math utilities."""

import math

Pose3Tuple = tuple[
  tuple[float, float, float], tuple[float, float, float, float]
]


def normalize_quaternion(
  qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float, float, float]:
  """Normalizes a quaternion `(qx, qy, qz, qw)` to unit length.

  Args:
    qx: Quaternion X component.
    qy: Quaternion Y component.
    qz: Quaternion Z component.
    qw: Quaternion W (scalar) component.

  Returns:
    Unit quaternion `(qx, qy, qz, qw)`, or `(0.0, 0.0, 0.0, 1.0)` if the norm
    is smaller than `1e-12`.
  """
  norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
  if norm < 1e-12:
    return (0.0, 0.0, 0.0, 1.0)
  return (qx / norm, qy / norm, qz / norm, qw / norm)


def quaternion_to_rpy_deg(
  qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float, float]:
  """Converts a quaternion `(qx, qy, qz, qw)` to extrinsic XYZ RPY in degrees.

  Args:
    qx: Quaternion X component.
    qy: Quaternion Y component.
    qz: Quaternion Z component.
    qw: Quaternion W component.

  Returns:
    Tuple `(roll_deg, pitch_deg, yaw_deg)` in degrees.
  """
  x, y, z, w = normalize_quaternion(qx, qy, qz, qw)
  sinp = 2.0 * (w * y - z * x)
  if abs(sinp) >= 1.0 - 1e-7:
    pitch = math.copysign(math.pi / 2.0, sinp)
    yaw = 0.0
    if sinp > 0.0:
      roll = math.atan2(2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z))
    else:
      roll = math.atan2(-2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z))
  else:
    pitch = math.asin(sinp)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

  return (
    math.degrees(roll),
    math.degrees(pitch),
    math.degrees(yaw),
  )


def rpy_deg_to_quaternion(
  roll_deg: float, pitch_deg: float, yaw_deg: float
) -> tuple[float, float, float, float]:
  """Converts extrinsic XYZ / intrinsic ZYX Euler angles (degrees) to quaternion.

  Args:
    roll_deg: Roll rotation around X axis in degrees.
    pitch_deg: Pitch rotation around Y axis in degrees.
    yaw_deg: Yaw rotation around Z axis in degrees.

  Returns:
    Normalized quaternion `(qx, qy, qz, qw)`.
  """
  half_r = math.radians(roll_deg) * 0.5
  half_p = math.radians(pitch_deg) * 0.5
  half_y = math.radians(yaw_deg) * 0.5

  cr = math.cos(half_r)
  sr = math.sin(half_r)
  cp = math.cos(half_p)
  sp = math.sin(half_p)
  cy = math.cos(half_y)
  sy = math.sin(half_y)

  qx = sr * cp * cy - cr * sp * sy
  qy = cr * sp * cy + sr * cp * sy
  qz = cr * cp * sy - sr * sp * cy
  qw = cr * cp * cy + sr * sp * sy
  return normalize_quaternion(qx, qy, qz, qw)


def _quat_multiply(
  q1: tuple[float, float, float, float],
  q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
  """Multiplies two Hamilton quaternions stored as `(qx, qy, qz, qw)`."""
  x1, y1, z1, w1 = q1
  x2, y2, z2, w2 = q2
  return normalize_quaternion(
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
  )


def _rotate_vector_by_quat(
  vec: tuple[float, float, float],
  quat: tuple[float, float, float, float],
) -> tuple[float, float, float]:
  """Rotates a 3D vector `vec` by unit quaternion `quat` `(qx, qy, qz, qw)`."""
  vx, vy, vz = vec
  qx, qy, qz, qw = normalize_quaternion(*quat)
  tx = 2.0 * (qy * vz - qz * vy)
  ty = 2.0 * (qz * vx - qx * vz)
  tz = 2.0 * (qx * vy - qy * vx)
  rx = vx + qw * tx + (qy * tz - qz * ty)
  ry = vy + qw * ty + (qz * tx - qx * tz)
  rz = vz + qw * tz + (qx * ty - qy * tx)
  return (rx, ry, rz)


def compose_transforms(t_a_b: Pose3Tuple, t_b_c: Pose3Tuple) -> Pose3Tuple:
  """Composes two rigid 3D transforms `T_a_c = T_a_b * T_b_c`.

  Args:
    t_a_b: Transform `(position, orientation)` of frame B in frame A.
    t_b_c: Transform `(position, orientation)` of frame C in frame B.

  Returns:
    Composed transform `(position, orientation)` of frame C in frame A.
  """
  p1, q1_raw = t_a_b
  p2, q2_raw = t_b_c
  q1 = normalize_quaternion(*q1_raw)
  q2 = normalize_quaternion(*q2_raw)
  rx, ry, rz = _rotate_vector_by_quat(p2, q1)
  p_out = (p1[0] + rx, p1[1] + ry, p1[2] + rz)
  q_out = _quat_multiply(q1, q2)
  return p_out, q_out


def invert_transform(t_a_b: Pose3Tuple) -> Pose3Tuple:
  """Inverts a rigid 3D transform `T_b_a = (T_a_b)^-1`.

  Args:
    t_a_b: Transform `(position, orientation)` of frame B in frame A.

  Returns:
    Inverted transform `(position, orientation)` of frame A in frame B.
  """
  p, q = t_a_b
  q_inv = normalize_quaternion(-q[0], -q[1], -q[2], q[3])
  p_inv = _rotate_vector_by_quat((-p[0], -p[1], -p[2]), q_inv)
  return p_inv, q_inv


def relative_transform(
  t_root_parent: Pose3Tuple,
  t_root_target: Pose3Tuple,
) -> Pose3Tuple:
  """Computes `T_parent_target = (T_root_parent)^-1 * T_root_target`.

  Args:
    t_root_parent: Transform of parent frame in fixed/root frame.
    t_root_target: Transform of target frame in fixed/root frame.

  Returns:
    Transform of target frame relative to parent frame.
  """
  return compose_transforms(invert_transform(t_root_parent), t_root_target)


def compose_pose_delta(
  position: tuple[float, float, float],
  orientation: tuple[float, float, float, float],
  delta_xyz: tuple[float, float, float],
  delta_rpy_deg: tuple[float, float, float],
  local_frame: bool = False,
) -> Pose3Tuple:
  """Applies a translation and Euler RPY rotation delta to a pose.

  Args:
    position: Current `(x, y, z)` translation in meters.
    orientation: Current `(qx, qy, qz, qw)` orientation quaternion.
    delta_xyz: Translation delta `(dx, dy, dz)` in meters.
    delta_rpy_deg: Rotation delta `(droll, dpitch, dyaw)` in degrees.
    local_frame: If `True`, applies translation and rotation along the frame's
      own local axes; if `False`, applies them along the parent frame's axes.

  Returns:
    Tuple `(new_position, new_orientation)`.
  """
  q_curr = normalize_quaternion(*orientation)
  q_delta = rpy_deg_to_quaternion(*delta_rpy_deg)

  if local_frame:
    dx, dy, dz = _rotate_vector_by_quat(delta_xyz, q_curr)
    new_pos = (position[0] + dx, position[1] + dy, position[2] + dz)
    new_ori = _quat_multiply(q_curr, q_delta)
  else:
    new_pos = (
      position[0] + delta_xyz[0],
      position[1] + delta_xyz[1],
      position[2] + delta_xyz[2],
    )
    new_ori = _quat_multiply(q_delta, q_curr)

  return new_pos, new_ori
