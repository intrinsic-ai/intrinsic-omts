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

"""Infeed return placement subtree."""

from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  Touchdown,
  create_clear_motion_planner_cache_task,
  create_move_through_frames_task,
  create_move_to_frame_task,
  create_seated_approach_tasks,
)
from src.core.workpiece import Workpiece
from src.core.world import World
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface


def build_return_to_infeed_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  workpiece: Workpiece,
  parent_object: str = "root",
  transit_frame_name: str = "transit",
  preplace_frame_name: str = "infeed_pre_grasp",
  place_frame_name: str = "infeed_grasp",
  machine: Any | None = None,
  touchdown: Touchdown | None = None,
  return_to_view_frame: bool = False,
  solution: Any | None = None,
  name: str = "6. Obstacle-Aware Scatter Return Subtree",
  view_frame_name: str | None = None,
  enable_object_reparenting: bool = False,
  clear_motion_planner_cache: bool = False,
  **kwargs: Any,
) -> bt.Node:
  """Builds Behavior Tree subtree for returning the part to the infeed."""
  tasks: list[bt.Node] = []

  if machine is not None:
    tasks.append(
      machine.build_close_door_task(name="Step 6a-2: Close CNC Door")
    )
    if clear_motion_planner_cache:
      tasks.append(
        create_clear_motion_planner_cache_task(
          solution=solution,
          task_name="Step 6a-4: Clear Motion Planner Cache",
        )
      )

  tasks.append(
    create_move_through_frames_task(
      robot=robot,
      frame_names=[transit_frame_name, preplace_frame_name],
      parent_object=parent_object,
      motion_type="ANY",
      solution=solution,
      task_name=(
        f"Step 6a: Blended Move to Pre-Place via {transit_frame_name}"
        f" ({parent_object}/{transit_frame_name} -> {parent_object}/{preplace_frame_name})"
      ),
    )
  )

  if touchdown is None:
    touchdown = Touchdown(
      force_n=float(kwargs.get("contact_force_newtons", Touchdown.force_n)),
      standoff_m=float(kwargs.get("standoff_distance_m", Touchdown.standoff_m)),
      timeout_s=float(
        kwargs.get("contact_timeout_seconds", Touchdown.timeout_s)
      ),
      retract_after_m=0.0,
    )

  tasks.extend(
    create_seated_approach_tasks(
      robot=robot,
      frame_name=place_frame_name,
      parent_object=parent_object,
      touchdown=touchdown,
      label="Step 6c",
      solution=solution,
    )
  )

  tasks.append(
    gripper.build_open_task(name="Step 6d: Release Part at Scatter Placement")
  )

  if enable_object_reparenting:
    world = kwargs.get("world") or getattr(solution, "world", None)
    if not hasattr(world, "build_reparent_task"):
      world = World(world, solution=solution)
    tasks.append(
      world.build_reparent_task(
        target=workpiece.scene_object_name,
        new_parent=parent_object,
        name=f"Step 6e: Detach Part to {parent_object} in Digital Twin",
      )
    )
    if clear_motion_planner_cache:
      tasks.append(
        create_clear_motion_planner_cache_task(
          solution=solution,
          task_name="Step 6e-2: Clear Motion Planner Cache",
        )
      )

  if return_to_view_frame:
    actual_view = view_frame_name or "view"
    tasks.append(
      create_move_through_frames_task(
        robot=robot,
        frame_names=[preplace_frame_name, actual_view],
        parent_object=parent_object,
        motion_type=["LINEAR", "ANY"],
        solution=solution,
        task_name=(
          f"Step 6f: Linear Retract to {preplace_frame_name} Blended to View"
          f" Frame ({parent_object}/{preplace_frame_name} ->"
          f" {parent_object}/{actual_view})"
        ),
      )
    )
  else:
    tasks.append(
      create_move_to_frame_task(
        robot=robot,
        frame_name=preplace_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        solution=solution,
        task_name=(
          f"Step 6f: Linear Retract to Pre-Place ({parent_object}/{preplace_frame_name})"
        ),
      )
    )

  return bt.Sequence(name=name, children=tasks)
