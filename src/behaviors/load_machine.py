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

"""CNC machine loading and fixturing subtree."""

from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  DEFAULT_TOUCHDOWN,
  Touchdown,
  build_interaction_tasks,
  create_move_through_frames_task,
  create_move_to_frame_task,
)
from src.core.workpiece import Workpiece
from src.core.world import WorldInterface, resolve_world
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_load_machine_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  machine: CncMachineInterface,
  workpiece: Workpiece,
  parent_object: str = "root",
  entry_via_frame_name: str = "transit",
  machine_approach_frame_name: str = "machine_approach",
  vise_approach_frame_name: str = "vise_pre_place",
  vise_place_frame_name: str = "vise_place",
  touchdown: Touchdown = DEFAULT_TOUCHDOWN,
  solution: Any | None = None,
  world: WorldInterface | None = None,
  name: str = "3. Load Machine Subtree",
  enable_object_reparenting: bool = False,
) -> bt.Node:
  """Builds Behavior Tree subtree for loading raw stock into the CNC machine."""
  tasks: list[bt.Node] = [
    create_move_through_frames_task(
      robot=robot,
      frame_names=[
        entry_via_frame_name,
        machine_approach_frame_name,
        vise_approach_frame_name,
      ],
      parent_object=parent_object,
      motion_type=["ANY", "ANY", "LINEAR"],
      solution=solution,
      task_name=(
        f"Step 3a: Blended Move to Vise Approach"
        f" ({parent_object}/{entry_via_frame_name} ->"
        f" {parent_object}/{machine_approach_frame_name} ->"
        f" {parent_object}/{vise_approach_frame_name})"
      ),
    )
  ]

  reparent_task = (
    resolve_world(solution, world).build_detach_from_gripper_task(
      object_name=workpiece.object_name,
      name=f"Step 3f: Detach Part to {parent_object} in Digital Twin",
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
      label="Step 3c",
      pre_reparent_tasks=[
        machine.build_close_vise_task(name="Step 3d: Clamp CNC Vise"),
        gripper.build_open_task(
          name="Step 3e: Open Gripper (Release Part in Vise)"
        ),
      ],
      reparent_task=reparent_task,
      solution=solution,
    )
  )

  tasks.append(
    create_move_to_frame_task(
      robot=robot,
      frame_name=vise_approach_frame_name,
      parent_object=parent_object,
      motion_type="LINEAR",
      solution=solution,
      task_name=(
        f"Step 3h: Linear Retract to Vise Approach ({parent_object}/{vise_approach_frame_name})"
      ),
    )
  )

  return bt.Sequence(name=name, children=tasks)
