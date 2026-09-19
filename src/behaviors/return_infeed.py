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
from src.core.config import AppConfig
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface


def build_return_to_infeed_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  config: AppConfig,
) -> bt.Node:
  """Builds the Behavior Tree subtree for returning the finished part to infeed.

  Sequence:
  1. Approach infeed `pregrasp_frame` (`ANY`), blending through `transit_frame`
     if configured.
  2. Perform compliant touchdown along tool +Z to place finished part on table
     surface.
  3. Open gripper to release part and detach workpiece entity from gripper in
     the belief world.
  4. Retract arm linearly to `pregrasp_frame` (`LINEAR`) with segment-scoped
     `(tool, workpiece)` collision exclusion.
  5. Return arm to `view_frame` (`ANY`).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      config: Validated application configuration dataclass.

  Returns:
      Behavior tree sequence node executing infeed return.
  """
  parent_object = config.frames.parent_object
  pregrasp_frame_name = config.frames.pregrasp_frame
  view_frame_name = config.frames.view_frame
  transit_frame_name = config.frames.transit_frame
  return_touchdown_force_newtons = config.cycle.return_touchdown_force_newtons
  touchdown_timeout_seconds = config.cycle.touchdown_timeout_seconds
  workpiece_object_name = config.cycle.workpiece_id

  tasks: list[bt.Node] = []

  if transit_frame_name:
    tasks.append(
      robot.build_move_blended_cartesian_task(
        target_frames=[
          (parent_object, transit_frame_name),
          (parent_object, pregrasp_frame_name),
        ],
        motion_type="ANY",
        name=f"Blended Transit to Infeed Placement ({parent_object}/{transit_frame_name} -> {parent_object}/{pregrasp_frame_name})",
      )
    )
  else:
    tasks.append(
      create_move_to_frame_task(
        robot=robot,
        frame_name=pregrasp_frame_name,
        parent_object=parent_object,
        motion_type="ANY",
        task_name=f"Approach Infeed Placement ({parent_object}/{pregrasp_frame_name})",
      )
    )

  tasks.extend(
    [
      create_compliant_touchdown_task(
        robot=robot,
        direction=(0.0, 0.0, 1.0),
        contact_force_newtons=return_touchdown_force_newtons,
        timeout_seconds=touchdown_timeout_seconds,
        task_name="Compliant Touchdown to Table Surface (+Z Tool)",
      ),
      gripper.build_open_task(name="Release Finished Part"),
      robot.build_detach_object_task(
        object_name=workpiece_object_name,
        name=f"Detach {workpiece_object_name} from Gripper",
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=pregrasp_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        excluded_collision_pairs=[
          (config.robot.tool_object_name, workpiece_object_name),
        ],
        task_name=f"Retract Arm from Table ({parent_object}/{pregrasp_frame_name})",
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=view_frame_name,
        parent_object=parent_object,
        motion_type="ANY",
        task_name=f"Return to View Pose ({parent_object}/{view_frame_name})",
      ),
    ]
  )

  return bt.Sequence(name="5. Return to Infeed Subtree", children=tasks)
