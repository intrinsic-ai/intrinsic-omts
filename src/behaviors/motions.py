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

"""Reusable motion tasks and building blocks for robot arm movements."""

from collections.abc import Sequence

from intrinsic.solutions import behavior_tree as bt

from src.hardware.robot import RobotInterface


def create_move_to_frame_task(
  robot: RobotInterface,
  frame_name: str | None,
  parent_object: str,
  motion_type: str,
  allow_tool_z_rotation: bool = False,
  cone_opening_half_angle: float = 0.0,
  moving_frame_offset: tuple[float, float, float] | None = None,
  target_frame_offset: tuple[
    tuple[float, float, float], tuple[float, float, float, float]
  ]
  | None = None,
  excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a Cartesian motion task moving the arm tool to a target frame.

  Args:
      robot: Robot controller adapter.
      frame_name: Target frame name on `parent_object`, or None to target the
        object root directly.
      parent_object: World object owning the target frame (e.g. 'root').
      motion_type: Trajectory interpolation mode ('ANY', 'LINEAR', or 'JOINT').
      allow_tool_z_rotation: If True, relaxes orientation to a rotation cone
        around tool Z instead of strict PoseEquality.
      cone_opening_half_angle: Allowed tilt half-angle in radians when
        `allow_tool_z_rotation` is True.
      moving_frame_offset: Optional (x, y, z) translation offset on the tool.
      target_frame_offset: Optional ((x, y, z), (qx, qy, qz, qw)) pose offset
        relative to the target frame.
      excluded_collision_pairs: Optional pairs of object names to exclude from
        collision checking during this motion segment.
      task_name: Optional custom name for the Behavior Tree task node.

  Returns:
      Configured SBL Behavior Tree Task node.
  """
  target_desc = f"{parent_object}/{frame_name}" if frame_name else parent_object
  return robot.build_move_cartesian_task(
    target_frame_name=frame_name,
    target_object_name=parent_object,
    motion_type=motion_type,
    allow_tool_z_rotation=allow_tool_z_rotation,
    cone_opening_half_angle=cone_opening_half_angle,
    moving_frame_offset=moving_frame_offset,
    target_frame_offset=target_frame_offset,
    excluded_collision_pairs=excluded_collision_pairs,
    name=task_name or f"Move to {target_desc} ({motion_type})",
  )


def create_compliant_touchdown_task(
  robot: RobotInterface,
  direction: tuple[float, float, float],
  contact_force_newtons: float,
  timeout_seconds: float,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a force-controlled compliant `move_to_contact` task.

  Args:
      robot: Robot controller adapter.
      direction: (x, y, z) search vector expressed in the moving tool frame.
      contact_force_newtons: Force threshold in Newtons that terminates motion.
      timeout_seconds: Maximum duration in seconds to search for contact.
      task_name: Optional custom name for the Behavior Tree task node.

  Returns:
      Configured SBL Behavior Tree Task node.
  """
  return robot.build_move_to_contact_task(
    direction=direction,
    contact_force_newtons=contact_force_newtons,
    timeout_seconds=timeout_seconds,
    name=task_name or "Compliant Touchdown",
  )


def create_relative_retract_task(
  robot: RobotInterface,
  distance_meters: float,
  excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a relative Cartesian linear retract task along the tool -Z axis.

  Args:
      robot: Robot controller adapter.
      distance_meters: Magnitude in meters to retract along tool -Z.
      excluded_collision_pairs: Optional pairs of object names to exclude from
        collision checking during this motion segment.
      task_name: Optional custom name for the Behavior Tree task node.

  Returns:
      Configured SBL Behavior Tree Task node.
  """
  retract_distance = -abs(distance_meters)
  return robot.build_move_relative_cartesian_task(
    translation=(0.0, 0.0, retract_distance),
    motion_type="LINEAR",
    excluded_collision_pairs=excluded_collision_pairs,
    name=task_name
    or f"Relative Retract ({abs(distance_meters) * 100:.1f} cm, -Z Tool)",
  )
