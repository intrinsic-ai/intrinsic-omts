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
  DEFAULT_TOUCHDOWN,
  Touchdown,
  build_interaction_tasks,
  create_move_through_frames_task,
)
from src.core.workpiece import Workpiece
from src.core.world import WorldInterface, resolve_world
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
  touchdown: Touchdown = DEFAULT_TOUCHDOWN,
  solution: Any | None = None,
  world: WorldInterface | None = None,
  name: str = "5. Unload Machine Subtree",
  enable_object_reparenting: bool = False,
) -> bt.Node:
  """Builds Behavior Tree subtree for unloading a machined part from the CNC."""
  tasks: list[bt.Node] = [
    create_move_through_frames_task(
      robot=robot,
      frame_names=[machine_approach_frame_name, vise_approach_frame_name],
      parent_object=parent_object,
      motion_type=["ANY", "LINEAR"],
      solution=solution,
      task_name=(
        f"Step 5a: Blended Move to Vise Approach"
        f" ({parent_object}/{machine_approach_frame_name} ->"
        f" {parent_object}/{vise_approach_frame_name})"
      ),
    )
  ]

  reparent_task = (
    resolve_world(solution, world).build_attach_to_gripper_task(
      object_name=workpiece.object_name,
      name="Step 5f: Attach Part to Gripper in Digital Twin",
    )
    if enable_object_reparenting
    else None
  )

  tasks.extend(
    build_interaction_tasks(
      robot=robot,
      frame_name=vise_place_frame_name,
      parent_object=parent_object,
      touchdown=touchdown,
      label="Step 5c",
      pre_reparent_tasks=[
        gripper.build_close_task(name="Step 5e: Grasp Machined Part")
      ],
      reparent_task=reparent_task,
      post_reparent_tasks=[
        machine.build_open_vise_task(
          name="Step 5g: Open CNC Vise (Unclamp Part)"
        )
      ],
      solution=solution,
    )
  )

  tasks.append(
    create_move_through_frames_task(
      robot=robot,
      frame_names=[vise_approach_frame_name, machine_approach_frame_name],
      parent_object=parent_object,
      motion_type="LINEAR",
      solution=solution,
      task_name=(
        f"Step 5i: Blended Linear Retract to Machine Approach"
        f" ({parent_object}/{vise_approach_frame_name} ->"
        f" {parent_object}/{machine_approach_frame_name})"
      ),
    )
  )

  return bt.Sequence(name=name, children=tasks)
