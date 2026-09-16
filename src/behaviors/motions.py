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

import dataclasses
import logging
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.hardware.robot import RobotInterface
from src.utils.math_utils import describe_motion_types, normalize_motion_types
from src.utils.script_utils import create_dwell_task


@dataclasses.dataclass(frozen=True)
class Touchdown:
  """Compliant seating parameters for one object interaction."""

  force_n: float = 8.0
  standoff_m: float = 0.010
  retract_after_m: float = 0.0
  timeout_s: float = 40.0


def create_seated_approach_tasks(
  robot: RobotInterface,
  frame_name: str,
  parent_object: str,
  touchdown: Touchdown,
  *,
  label: str,
  solution: Any | None = None,
) -> list[bt.Node]:
  """Builds a linear standoff approach, compliant touchdown, and optional lift."""
  tasks = [
    create_move_to_frame_task(
      robot=robot,
      frame_name=frame_name,
      parent_object=parent_object,
      motion_type="LINEAR",
      target_frame_offset=(
        (0.0, 0.0, -float(touchdown.standoff_m)),
        (0.0, 0.0, 0.0, 1.0),
      ),
      solution=solution,
      task_name=(
        f"{label}: Linear Approach to Standoff ({parent_object}/{frame_name})"
      ),
    ),
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
        task_name=(
          f"{label}: Linear Retract"
          f" ({touchdown.retract_after_m * 100:.1f} cm, -Z Tool)"
        ),
      )
    )
  return tasks


def create_move_to_frame_task(
  robot: RobotInterface,
  frame_name: str | None = None,
  parent_object: str = "root",
  motion_type: str = "ANY",
  target_frame_offset: (
    tuple[tuple[float, float, float], tuple[float, float, float, float]] | None
  ) = None,
  task_name: str | None = None,
  max_tries: int = 2,
  retry_delay_sec: float = 1.0,
  solution: Any | None = None,
) -> bt.Node:
  """Builds a Cartesian motion task moving the arm tool to a target frame."""
  target_desc = f"{parent_object}/{frame_name}" if frame_name else parent_object
  base_name = task_name or f"Move to {target_desc} ({motion_type})"
  task = robot.build_move_cartesian_task(
    target_frame_name=frame_name,
    target_object_name=parent_object,
    motion_type=motion_type,
    target_frame_offset=target_frame_offset,
    name=base_name,
  )
  if max_tries > 1:
    recovery = create_motion_recovery_task(
      solution=solution,
      retry_delay_sec=retry_delay_sec,
      task_name=f"Recovery: {base_name}",
    )
    return bt.Retry(
      max_tries=max_tries,
      child=task,
      recovery=recovery,
      name=base_name,
    )
  return task


def create_move_through_frames_task(
  robot: RobotInterface,
  frame_names: Sequence[str],
  parent_object: str = "root",
  motion_type: str | Sequence[str] = "ANY",
  task_name: str | None = None,
  max_tries: int = 2,
  retry_delay_sec: float = 1.0,
  solution: Any | None = None,
) -> bt.Node:
  """Builds a task moving through multiple frames in a blended trajectory."""
  if not frame_names:
    raise ValueError("frame_names must not be empty.")

  motion_types = normalize_motion_types(motion_type, len(frame_names))

  if len(frame_names) == 1:
    return create_move_to_frame_task(
      robot=robot,
      frame_name=frame_names[0],
      parent_object=parent_object,
      motion_type=motion_types[0],
      task_name=task_name,
      max_tries=max_tries,
      retry_delay_sec=retry_delay_sec,
      solution=solution,
    )

  target_frames = [(parent_object, f) for f in frame_names]
  path_desc = " -> ".join(f"{parent_object}/{f}" for f in frame_names)
  base_name = task_name or (
    f"Blended Move through {path_desc} ({describe_motion_types(motion_types)})"
  )
  task = robot.build_move_blended_cartesian_task(
    target_frames=target_frames,
    motion_type=motion_types,
    name=base_name,
  )
  if max_tries > 1:
    recovery = create_motion_recovery_task(
      solution=solution,
      retry_delay_sec=retry_delay_sec,
      task_name=f"Recovery: {base_name}",
    )
    return bt.Retry(
      max_tries=max_tries,
      child=task,
      recovery=recovery,
      name=base_name,
    )
  return task


def create_compliant_touchdown_task(
  robot: RobotInterface,
  direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
  contact_force_newtons: float = 5.0,
  timeout_seconds: float = 40.0,
  touchdown: Touchdown | None = None,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a compliant move_to_contact task."""
  force = touchdown.force_n if touchdown is not None else contact_force_newtons
  timeout = touchdown.timeout_s if touchdown is not None else timeout_seconds
  return robot.build_move_to_contact_task(
    direction=direction,
    contact_force_newtons=force,
    timeout_seconds=timeout,
    name=task_name or "Compliant Touchdown",
  )


def create_relative_retract_task(
  robot: RobotInterface,
  distance_meters: float = 0.03,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a relative Cartesian linear retract task along the tool -Z axis."""
  retract_distance = -abs(distance_meters)
  return robot.build_move_relative_cartesian_task(
    translation=(0.0, 0.0, retract_distance),
    motion_type="LINEAR",
    name=(
      task_name
      or f"Relative Retract ({abs(distance_meters) * 100:.1f} cm, -Z Tool)"
    ),
  )


def create_clear_motion_planner_cache_task(
  solution: Any | None = None,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a behavior tree task to clear the motion planner service cache."""
  name = task_name or "Clear Motion Planner Cache"
  if solution is not None:
    try:
      skills = getattr(solution, "skills", None)
      ai_skills = getattr(skills, "ai", None) if skills else None
      intrinsic_skills = (
        getattr(ai_skills, "intrinsic", None) if ai_skills else None
      )
      if intrinsic_skills and hasattr(
        intrinsic_skills, "clear_motion_planner_service_cache"
      ):
        clear_action = intrinsic_skills.clear_motion_planner_service_cache()
        task = bt.Task(action=clear_action, name=name)
        task.root = task
        return task
    except Exception as e:  # pylint: disable=broad-exception-caught
      logging.warning("Failed to create clear_motion_planner_cache task: %s", e)

  task = bt.Task(
    action=bt.PythonScript(function_body="pass\n"),
    name=name,
  )
  task.root = task
  return task


def create_clear_robot_faults_task(
  solution: Any | None = None,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a behavior tree task to clear robot/ICON faults and re-enable control."""
  name = task_name or "Clear Robot Faults & Enable Robot"
  if solution is not None:
    try:
      skills = getattr(solution, "skills", None)
      ai_skills = getattr(skills, "ai", None) if skills else None
      intrinsic_skills = (
        getattr(ai_skills, "intrinsic", None) if ai_skills else None
      )
      if intrinsic_skills and hasattr(
        intrinsic_skills, "enable_realtime_control"
      ):
        enable_action = intrinsic_skills.enable_realtime_control(
          clear_faults=True
        )
        task = bt.Task(action=enable_action, name=name)
        task.root = task
        return task
    except Exception as e:  # pylint: disable=broad-exception-caught
      logging.warning("Failed to create enable_realtime_control task: %s", e)

  task = bt.Task(
    action=bt.PythonScript(function_body="pass\n"),
    name=name,
  )
  task.root = task
  return task


def create_motion_recovery_task(
  solution: Any | None = None,
  retry_delay_sec: float = 1.0,
  task_name: str | None = None,
) -> bt.Node:
  """Builds a composite recovery task for motion retries."""
  name = task_name or "Motion Fault Recovery Sequence"
  clear_faults_step = create_clear_robot_faults_task(
    solution=solution,
    task_name="Step Rec-1: Clear Faults & Enable Robot",
  )
  clear_cache_step = create_clear_motion_planner_cache_task(
    solution=solution,
    task_name="Step Rec-2: Clear Motion Planner Cache",
  )
  dwell_step = create_dwell_task(
    dwell_time_sec=retry_delay_sec,
    solution=solution,
    task_name=f"Step Rec-3: Dwell ({retry_delay_sec}s)",
  )
  recovery_seq = bt.Sequence(
    name=name,
    children=[clear_faults_step, clear_cache_step, dwell_step],
  )
  recovery_seq.root = recovery_seq
  return recovery_seq
