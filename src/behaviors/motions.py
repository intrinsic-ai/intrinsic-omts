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
  """Builds a Cartesian motion task moving the arm tool to a target frame or object."""
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
  """Builds a compliant move_to_contact task."""
  return robot.build_move_to_contact_task(
    direction=direction,
    contact_force_newtons=contact_force_newtons,
    timeout_seconds=timeout_seconds,
    name=task_name or "Compliant Touchdown",
  )


def create_relative_retract_task(
  robot: RobotInterface,
  distance_meters: float,
  exclude_collision: bool = True,
  excluded_collision_objects: Sequence[str] | None = None,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a relative Cartesian linear retract task along the tool -Z axis."""
  retract_distance = -abs(distance_meters)
  return robot.build_move_relative_cartesian_task(
    translation=(0.0, 0.0, retract_distance),
    motion_type="LINEAR",
    exclude_collision=exclude_collision,
    excluded_collision_objects=excluded_collision_objects,
    name=task_name
    or f"Relative Retract ({abs(distance_meters) * 100:.1f} cm, -Z Tool)",
  )
