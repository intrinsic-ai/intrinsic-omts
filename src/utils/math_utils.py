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

"""Math, geometric transform, and motion utility functions for OMTS."""

import math
from collections.abc import Sequence
from typing import Any

__all__ = [
  "compute_top_down_grasp_quaternion",
  "create_transform_node_ref",
  "describe_motion_types",
  "extract_in_plane_alignment_axis",
  "normalize_angle",
  "normalize_joint_angles",
  "normalize_motion_types",
]


def normalize_angle(angle: float) -> float:
  """Normalizes an angle to [-pi, pi] radians."""
  return (angle + math.pi) % (2.0 * math.pi) - math.pi


def normalize_joint_angles(joint_angles: Sequence[float]) -> list[float]:
  """Normalizes a sequence of joint angles to [-pi, pi] radians."""
  return [normalize_angle(angle) for angle in joint_angles]


def normalize_motion_types(
  motion_type: str | Sequence[str], num_segments: int
) -> list[str]:
  """Expands a motion type into one entry per trajectory segment."""
  if isinstance(motion_type, str):
    return [motion_type] * num_segments
  types = list(motion_type)
  if len(types) != num_segments:
    raise ValueError(
      f"motion_type has {len(types)} entries but trajectory has"
      f" {num_segments} segments."
    )
  return types


def describe_motion_types(motion_types: Sequence[str]) -> str:
  """Renders motion types for a task name, collapsing a uniform trajectory."""
  if len(set(motion_types)) == 1:
    return motion_types[0]
  return "/".join(motion_types)


def create_transform_node_ref(object_name: str, frame_name: str) -> Any:
  """Builds a TransformNodeReference proto by object and frame name."""
  from intrinsic.world.proto import object_world_refs_pb2

  return object_world_refs_pb2.TransformNodeReference(
    by_name=object_world_refs_pb2.TransformNodeReferenceByName(
      frame=object_world_refs_pb2.FrameReferenceByName(
        object_name=object_name, frame_name=frame_name
      )
    )
  )


def extract_in_plane_alignment_axis(pose: Any) -> tuple[float, float, float]:
  """Extracts deterministic in-plane alignment axis from pose or rotation.

  Identifies which local axis is vertical (aligned with world Z) and selects
  the appropriate in-plane axis according to workpiece conventions:
  - When flat, local Y is vertical (max_z_idx = 1) -> local X is in-plane.
  - If local X is vertical (max_z_idx = 0) -> local Y is in-plane.
  - If local Z is vertical (max_z_idx = 2) -> local X is in-plane.

  Args:
    pose: 3D pose, rotation, or quaternion tuple.

  Returns:
    Normalized (vx, vy, 0.0) vector representing the planar alignment axis.
  """
  if hasattr(pose, "rotate_point"):
    ax = [float(v) for v in pose.rotate_point([1.0, 0.0, 0.0])]
    ay = [float(v) for v in pose.rotate_point([0.0, 1.0, 0.0])]
    az = [float(v) for v in pose.rotate_point([0.0, 0.0, 1.0])]
  elif hasattr(pose, "rotation"):
    return extract_in_plane_alignment_axis(pose.rotation)
  else:
    if hasattr(pose, "quaternion"):
      q = pose.quaternion
      qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
    elif hasattr(pose, "qx"):
      qx, qy, qz, qw = (
        float(pose.qx),
        float(pose.qy),
        float(pose.qz),
        float(pose.qw),
      )
    elif isinstance(pose, (tuple, list)) and len(pose) == 4:
      qx, qy, qz, qw = (
        float(pose[0]),
        float(pose[1]),
        float(pose[2]),
        float(pose[3]),
      )
    else:
      return (1.0, 0.0, 0.0)

    ax = [
      1.0 - 2.0 * (qy * qy + qz * qz),
      2.0 * (qx * qy + qz * qw),
      2.0 * (qx * qz - qy * qw),
    ]
    ay = [
      2.0 * (qx * qy - qz * qw),
      1.0 - 2.0 * (qx * qx + qz * qz),
      2.0 * (qy * qz + qx * qw),
    ]
    az = [
      2.0 * (qx * qz + qy * qw),
      2.0 * (qy * qz - qx * qw),
      1.0 - 2.0 * (qx * qx + qy * qy),
    ]

  abs_z = [abs(ax[2]), abs(ay[2]), abs(az[2])]
  max_z_idx = abs_z.index(max(abs_z))

  if max_z_idx == 0:  # Local X is vertical
    vx, vy = ay[0], ay[1]
    if math.hypot(vx, vy) < 1e-6:
      vx, vy = az[0], az[1]
  elif max_z_idx == 1:  # Local Y is vertical (flat raw_stock)
    vx, vy = ax[0], ax[1]
    if math.hypot(vx, vy) < 1e-6:
      vx, vy = az[0], az[1]
  else:  # Local Z is vertical
    vx, vy = ax[0], ax[1]
    if math.hypot(vx, vy) < 1e-6:
      vx, vy = ay[0], ay[1]

  norm = math.hypot(vx, vy)
  if norm > 1e-6:
    return (vx / norm, vy / norm, 0.0)
  return (1.0, 0.0, 0.0)


def compute_top_down_grasp_quaternion(
  target_pose: Any,
  current_tool_q: tuple[float, float, float, float] | None = None,
) -> tuple[float, float, float, float]:
  """Computes best aligned top-down grasp quaternion minimizing wrist rotation.

  Selects between the two antipodal top-down yaw orientations (yaw and yaw + pi)
  by choosing whichever is closest in wrapped yaw to current_tool_q (at most 90
  degrees rotation), and signs the quaternion to lie in the same S^3 hemisphere
  as current_tool_q to prevent 360-degree wrist unwinds.

  Args:
    target_pose: Pose3D or rotation of the target object.
    current_tool_q: Quaternion (qx, qy, qz, qw) of the current tool frame.

  Returns:
    A quaternion (qx, qy, qz, qw) representing the optimal grasp orientation.
  """
  vx, vy, _ = extract_in_plane_alignment_axis(target_pose)
  yaw = math.atan2(vy, vx)

  if current_tool_q is not None:
    try:
      cur_qx, cur_qy, _cur_qz, _cur_qw = (
        float(current_tool_q[0]),
        float(current_tool_q[1]),
        float(current_tool_q[2]),
        float(current_tool_q[3]),
      )
      cur_yaw = 2.0 * math.atan2(cur_qy, cur_qx)
      d1 = abs(normalize_angle(yaw - cur_yaw))
      d2 = abs(normalize_angle((yaw + math.pi) - cur_yaw))
      best_yaw = yaw if d1 <= d2 else (yaw + math.pi)

      half_psi = best_yaw / 2.0
      qx = math.cos(half_psi)
      qy = math.sin(half_psi)

      dot = qx * cur_qx + qy * cur_qy
      if dot < 0.0:
        qx, qy = -qx, -qy
      return (qx, qy, 0.0, 0.0)
    except (TypeError, ValueError, IndexError):
      pass

  half_psi = yaw / 2.0
  return (math.cos(half_psi), math.sin(half_psi), 0.0, 0.0)
