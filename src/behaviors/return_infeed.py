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

"""Infeed return / outfeed placement subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  create_compliant_touchdown_task,
  create_move_to_frame_task,
)
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface


def build_return_to_infeed_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  workpiece: Workpiece,
  parent_object: str = "root",
  pregrasp_frame_name: str = "pre_grasp",
  grasp_frame_name: str = "grasp",
  view_frame_name: str = "view",
) -> bt.Node:
  """Builds the Behavior Tree subtree for returning the finished part back to infeed.

  Steps (13 in OMTS pipeline):
  13a. Move arm to infeed pre-grasp approach position (ANY Cartesian motion).
  13b. Perform compliant touchdown via move_to_contact to place part on table surface (-Z).
  13c. Open gripper to release part.
  13d. Retract arm linearly to pre-grasp approach position (LINEAR Cartesian motion).
  13e. Move arm to view position (ANY Cartesian motion).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      workpiece: Workpiece instance being returned.
      parent_object: Name of parent object for target frames (default: 'root').
      pregrasp_frame_name: Name of pre-grasp approach frame (default: 'pre_grasp').
      grasp_frame_name: Name of infeed surface grasp frame (default: 'grasp').
      view_frame_name: Name of view frame (default: 'view').

  Returns:
      Behavior tree sequence node executing infeed return.
  """
  tasks: list[bt.Node] = [
    create_move_to_frame_task(
      robot=robot,
      frame_name=pregrasp_frame_name,
      parent_object=parent_object,
      motion_type="ANY",
      task_name=f"Step 13a: Approach Infeed Placement ({parent_object}/{pregrasp_frame_name})",
    ),
    create_compliant_touchdown_task(
      robot=robot,
      direction=(0.0, 0.0, 1.0),
      contact_force_newtons=5.0,
      task_name="Step 13b: Compliant Touchdown to Table Surface (+Z Tool)",
    ),
    gripper.build_open_task(name="Step 13c: Release Finished Part"),
    create_move_to_frame_task(
      robot=robot,
      frame_name=pregrasp_frame_name,
      parent_object=parent_object,
      motion_type="LINEAR",
      task_name=f"Step 13d: Retract Arm from Table ({parent_object}/{pregrasp_frame_name})",
    ),
    create_move_to_frame_task(
      robot=robot,
      frame_name=view_frame_name,
      parent_object=parent_object,
      motion_type="ANY",
      task_name=f"Step 13e: Return to View Pose ({parent_object}/{view_frame_name})",
    ),
  ]

  return bt.Sequence(name="5. Return to Infeed Subtree", children=tasks)
