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

"""CNC machine unloading and extraction subtree."""

from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  Touchdown,
  create_clear_motion_planner_cache_task,
  create_move_to_frame_task,
  create_seated_approach_tasks,
)
from src.core.workpiece import Workpiece
from src.core.world import World
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_unload_machine_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  machine: CncMachineInterface,
  workpiece: Workpiece,
  parent_object: str = "root",
  machine_approach_frame_name: str = "machine_approach",
  vise_approach_frame_name: str = "vise_pre_place",
  vise_place_frame_name: str = "vise_place",
  touchdown: Touchdown | None = None,
  grasp_offset_z: float = 0.0,
  solution: Any | None = None,
  name: str = "5. Unload Machine Subtree",
  clear_motion_planner_cache: bool = False,
  enable_object_reparenting: bool = False,
  **kwargs: Any,
) -> bt.Node:
  """Builds Behavior Tree subtree for unloading a machined part from the CNC."""
  tasks: list[bt.Node] = [
    create_move_to_frame_task(
      robot=robot,
      frame_name=machine_approach_frame_name,
      parent_object=parent_object,
      motion_type="ANY",
      solution=solution,
      task_name=(
        f"Step 5a: Move to Machine Approach ({parent_object}/{machine_approach_frame_name})"
      ),
    ),
    create_move_to_frame_task(
      robot=robot,
      frame_name=vise_approach_frame_name,
      parent_object=parent_object,
      motion_type="LINEAR",
      solution=solution,
      task_name=(
        f"Step 5b: Move to Vise Approach ({parent_object}/{vise_approach_frame_name})"
      ),
    ),
  ]

  if touchdown is None:
    grasp_offset_z = kwargs.get("grasp_offset_z", grasp_offset_z)
    touchdown = Touchdown(
      force_n=float(kwargs.get("contact_force_newtons", Touchdown.force_n)),
      standoff_m=float(kwargs.get("standoff_distance_m", Touchdown.standoff_m)),
      timeout_s=float(
        kwargs.get("contact_timeout_seconds", Touchdown.timeout_s)
      ),
      retract_after_m=grasp_offset_z,
    )

  tasks.extend(
    create_seated_approach_tasks(
      robot=robot,
      frame_name=vise_place_frame_name,
      parent_object=parent_object,
      touchdown=touchdown,
      label="Step 5c",
      solution=solution,
    )
  )

  tasks.append(gripper.build_close_task(name="Step 5e: Grasp Machined Part"))

  if enable_object_reparenting:
    world = kwargs.get("world") or getattr(solution, "world", None)
    if not hasattr(world, "build_reparent_task"):
      world = World(world, solution=solution)
    tasks.append(
      world.build_reparent_task(
        target=workpiece.scene_object_name,
        new_parent="gripper",
        name="Step 5f: Attach Part to Gripper in Digital Twin",
      )
    )

  tasks.append(
    machine.build_open_vise_task(name="Step 5g: Open CNC Vise (Unclamp Part)")
  )

  if clear_motion_planner_cache:
    tasks.append(
      create_clear_motion_planner_cache_task(
        solution=solution,
        task_name="Step 5h: Clear Motion Planner Cache",
      )
    )

  tasks.extend(
    [
      create_move_to_frame_task(
        robot=robot,
        frame_name=vise_approach_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        solution=solution,
        task_name=(
          f"Step 5i: Linear Retract to Vise Approach ({parent_object}/{vise_approach_frame_name})"
        ),
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=machine_approach_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        solution=solution,
        task_name=(
          f"Step 5j: Linear Retract to Machine Approach ({parent_object}/{machine_approach_frame_name})"
        ),
      ),
    ]
  )

  return bt.Sequence(name=name, children=tasks)
