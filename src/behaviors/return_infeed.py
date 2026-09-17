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
  DEFAULT_TOUCHDOWN,
  Touchdown,
  build_interaction_tasks,
  create_move_through_frames_task,
)
from src.core.workpiece import Workpiece
from src.core.world import WorldInterface, resolve_world
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
  view_frame_name: str = "view",
  touchdown: Touchdown = DEFAULT_TOUCHDOWN,
  solution: Any | None = None,
  world: WorldInterface | None = None,
  name: str = "6. Obstacle-Aware Scatter Return Subtree",
  enable_object_reparenting: bool = False,
) -> bt.Node:
  """Builds Behavior Tree subtree for returning the part to the infeed."""
  tasks: list[bt.Node] = [
    create_move_through_frames_task(
      robot=robot,
      frame_names=[transit_frame_name, preplace_frame_name],
      parent_object=parent_object,
      motion_type="ANY",
      solution=solution,
      task_name=(
        f"Step 6a: Blended Move to Pre-Place via {transit_frame_name}"
        f" ({parent_object}/{transit_frame_name} ->"
        f" {parent_object}/{preplace_frame_name})"
      ),
    )
  ]

  reparent_task = (
    resolve_world(solution, world).build_detach_from_gripper_task(
      object_name=workpiece.object_name,
      name=f"Step 6e: Detach Part to {parent_object} in Digital Twin",
    )
    if enable_object_reparenting
    else None
  )

  tasks.extend(
    build_interaction_tasks(
      robot=robot,
      frame_name=place_frame_name,
      parent_object=parent_object,
      touchdown=touchdown,
      label="Step 6c",
      pre_reparent_tasks=[
        gripper.build_open_task(
          name="Step 6d: Release Part at Scatter Placement"
        )
      ],
      reparent_task=reparent_task,
      solution=solution,
    )
  )

  tasks.append(
    create_move_through_frames_task(
      robot=robot,
      frame_names=[preplace_frame_name, view_frame_name],
      parent_object=parent_object,
      motion_type=["LINEAR", "ANY"],
      solution=solution,
      task_name=(
        f"Step 6f: Linear Retract to {preplace_frame_name} Blended to View"
        f" Frame ({parent_object}/{preplace_frame_name} ->"
        f" {parent_object}/{view_frame_name})"
      ),
    )
  )

  return bt.Sequence(name=name, children=tasks)
