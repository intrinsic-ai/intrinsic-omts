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

from src.core.config import AppConfig
from src.core.types import Touchdown
from src.hardware.robot import RobotInterface

__all__ = [
  "DEFAULT_TOUCHDOWN",
  "Touchdown",
  "build_interaction_tasks",
  "create_compliant_touchdown_task",
  "create_move_through_frames_task",
  "create_move_to_frame_task",
  "create_relative_retract_task",
  "create_seated_approach_tasks",
]

DEFAULT_TOUCHDOWN = Touchdown()


def create_move_to_frame_task(
  robot: RobotInterface,
  frame_name: str | None,
  parent_object: str | None = None,
  *,
  config: AppConfig | None = None,
  motion_type: str = "ANY",
  allow_tool_z_rotation: bool = False,
  cone_opening_half_angle: float = 0.0,
  moving_frame_offset: tuple[float, float, float] | None = None,
  target_frame_offset: (
    tuple[tuple[float, float, float], tuple[float, float, float, float]] | None
  ) = None,
  excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a Cartesian motion task moving the arm tool to a target frame."""
  parent = parent_object or (
    config.frames.parent_object if config is not None else "root"
  )
  target_desc = f"{parent}/{frame_name}" if frame_name else parent
  return robot.build_move_cartesian_task(
    target_frame_name=frame_name,
    target_object_name=parent,
    motion_type=motion_type,
    allow_tool_z_rotation=allow_tool_z_rotation,
    cone_opening_half_angle=cone_opening_half_angle,
    moving_frame_offset=moving_frame_offset,
    target_frame_offset=target_frame_offset,
    excluded_collision_pairs=excluded_collision_pairs,
    name=task_name or f"Move to {target_desc} ({motion_type})",
  )


def create_move_through_frames_task(
  robot: RobotInterface,
  frame_names: Sequence[str],
  parent_object: str | None = None,
  *,
  config: AppConfig | None = None,
  motion_type: str | Sequence[str] = "ANY",
  target_frame_offset: (
    tuple[tuple[float, float, float], tuple[float, float, float, float]] | None
  ) = None,
  excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
  task_name: str | None = None,
  max_tries: int = 1,
) -> bt.Node:
  """Builds a task moving through multiple frames in a blended trajectory."""
  if not frame_names:
    raise ValueError("frame_names must not be empty.")
  parent = parent_object or (
    config.frames.parent_object if config is not None else "root"
  )

  if len(frame_names) == 1:
    m_type = motion_type if isinstance(motion_type, str) else motion_type[0]
    return create_move_to_frame_task(
      robot=robot,
      frame_name=frame_names[0],
      parent_object=parent,
      motion_type=m_type,
      target_frame_offset=target_frame_offset,
      excluded_collision_pairs=excluded_collision_pairs,
      task_name=task_name,
    )

  target_frames = [(parent, f) for f in frame_names]
  path_desc = " -> ".join(f"{parent}/{f}" for f in frame_names)
  base_name = task_name or f"Blended Move through {path_desc}"
  task = robot.build_move_blended_cartesian_task(
    target_frames=target_frames,
    motion_type=motion_type,
    target_frame_offset=target_frame_offset,
    excluded_collision_pairs=excluded_collision_pairs,
    name=base_name,
  )
  if max_tries > 1:
    return bt.Retry(
      max_tries=max_tries,
      child=task,
      name=base_name,
    )
  return task


def create_compliant_touchdown_task(
  robot: RobotInterface,
  direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
  contact_force_newtons: float = 8.0,
  timeout_seconds: float = 40.0,
  task_name: str | None = None,
  touchdown: Touchdown | None = None,
) -> bt.Node:
  """Builds a compliant move_to_contact task."""
  if touchdown is not None:
    direction = touchdown.direction
    contact_force_newtons = touchdown.force_n
    timeout_seconds = touchdown.timeout_s
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
  """Builds a relative Cartesian linear retract task along the tool -Z axis."""
  retract_distance = -abs(distance_meters)
  return robot.build_move_relative_cartesian_task(
    translation=(0.0, 0.0, retract_distance),
    motion_type="LINEAR",
    excluded_collision_pairs=excluded_collision_pairs,
    name=(
      task_name
      or f"Relative Retract ({abs(distance_meters) * 100:.1f} cm, -Z Tool)"
    ),
  )


def create_seated_approach_tasks(
  robot: RobotInterface,
  frame_name: str,
  parent_object: str | None = None,
  touchdown: Touchdown = DEFAULT_TOUCHDOWN,
  *,
  config: AppConfig | None = None,
  label: str,
  approach_frames: Sequence[str] = (),
  approach_motion_types: str | Sequence[str] = "ANY",
  excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
) -> list[bt.Node]:
  """Builds a standoff approach, compliant touchdown, and optional lift."""
  parent = parent_object or (
    config.frames.parent_object if config is not None else "root"
  )
  standoff_offset = (
    (0.0, 0.0, -float(touchdown.standoff_m)),
    (0.0, 0.0, 0.0, 1.0),
  )
  if approach_frames:
    motion_types = (
      [approach_motion_types] * len(approach_frames)
      if isinstance(approach_motion_types, str)
      else list(approach_motion_types)
    )
    motion_types.append("LINEAR")
    approach_task = create_move_through_frames_task(
      robot=robot,
      frame_names=[*approach_frames, frame_name],
      parent_object=parent,
      motion_type=motion_types,
      target_frame_offset=standoff_offset,
      excluded_collision_pairs=excluded_collision_pairs,
      task_name=(
        f"{label}: Blended Approach to Standoff ({parent}/{frame_name})"
      ),
    )
  else:
    approach_task = create_move_to_frame_task(
      robot=robot,
      frame_name=frame_name,
      parent_object=parent,
      motion_type="LINEAR",
      target_frame_offset=standoff_offset,
      excluded_collision_pairs=excluded_collision_pairs,
      task_name=(
        f"{label}: Linear Approach to Standoff ({parent}/{frame_name})"
      ),
    )

  tasks = [
    approach_task,
    create_compliant_touchdown_task(
      robot=robot,
      touchdown=touchdown,
      task_name=f"{label}: Compliant Touchdown (+Z Tool)",
    ),
  ]
  if touchdown.retract_after_m > 0.0:
    tasks.append(
      create_relative_retract_task(
        robot=robot,
        distance_meters=touchdown.retract_after_m,
        excluded_collision_pairs=excluded_collision_pairs,
        task_name=(
          f"{label}: Linear Retract"
          f" ({touchdown.retract_after_m * 100:.1f} cm, -Z Tool)"
        ),
      )
    )
  return tasks


def build_interaction_tasks(
  robot: RobotInterface,
  frame_name: str,
  touchdown: Touchdown,
  *,
  config: AppConfig | None = None,
  parent_object: str | None = None,
  label: str,
  approach_frames: Sequence[str] = (),
  approach_motion_types: str | Sequence[str] = "ANY",
  excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
  pre_reparent_tasks: Sequence[bt.Node] = (),
  reparent_task: bt.Node | None = None,
  post_reparent_tasks: Sequence[bt.Node] = (),
) -> list[bt.Node]:
  """Builds seated approach, hardware actuation, reparenting, and post tasks."""
  tasks = create_seated_approach_tasks(
    robot=robot,
    frame_name=frame_name,
    parent_object=parent_object,
    touchdown=touchdown,
    config=config,
    label=label,
    approach_frames=approach_frames,
    approach_motion_types=approach_motion_types,
    excluded_collision_pairs=excluded_collision_pairs,
  )
  tasks.extend(pre_reparent_tasks)
  if reparent_task is not None:
    tasks.append(reparent_task)
  tasks.extend(post_reparent_tasks)
  return tasks
