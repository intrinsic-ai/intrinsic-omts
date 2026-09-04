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

"""Math and geometric transform utility functions for OMTS."""

import math
from collections.abc import Sequence

from src.core.types import Pose3D


def normalize_angle(angle: float) -> float:
  """Normalizes an angle to [-pi, pi] radians."""
  return (angle + math.pi) % (2.0 * math.pi) - math.pi


def normalize_joint_angles(joint_angles: Sequence[float]) -> list[float]:
  """Normalizes a sequence of joint angles to [-pi, pi] radians."""
  return [normalize_angle(angle) for angle in joint_angles]


def compute_euclidean_distance(pose_a: Pose3D, pose_b: Pose3D) -> float:
  """Computes Euclidean translation distance between two 3D poses (meters)."""
  dx = pose_a.x - pose_b.x
  dy = pose_a.y - pose_b.y
  dz = pose_a.z - pose_b.z
  return math.sqrt(dx * dx + dy * dy + dz * dz)


def interpolate_poses(pose_a: Pose3D, pose_b: Pose3D, alpha: float) -> Pose3D:
  """Linearly interpolates translation between pose_a and pose_b by alpha in [0, 1]."""
  alpha = max(0.0, min(1.0, alpha))
  return Pose3D(
    x=pose_a.x + alpha * (pose_b.x - pose_a.x),
    y=pose_a.y + alpha * (pose_b.y - pose_a.y),
    z=pose_a.z + alpha * (pose_b.z - pose_a.z),
    qx=pose_b.qx,
    qy=pose_b.qy,
    qz=pose_b.qz,
    qw=pose_b.qw,
  )
