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

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  build_interaction_tasks,
  create_move_through_frames_task,
)
from src.core.config import AppConfig
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface


def build_return_to_infeed_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  config: AppConfig,
) -> bt.Node:
  """Builds the Behavior Tree subtree for returning the part to the infeed.

  Sequence:
      1. Blended transit (`[transit_frame, pregrasp_frame]`) back to the infeed.
      2. Linear standoff approach + compliant touchdown (`config.return_touchdown`)
         onto `grasp_frame`, open gripper to release part, and detach workpiece
         in `ObjectWorld`.
      3. Blended retract (`[pregrasp_frame, view_frame]`) to return the arm to
         the viewing pose for the next cycle.

  Args:
      robot: Robot hardware interface.
      gripper: Gripper hardware interface.
      config: Application configuration.

  Returns:
      A `bt.Sequence` node executing the return-to-infeed phase.
  """
  entry_frames = [
    f for f in (config.frames.transit_frame, config.frames.pregrasp_frame) if f
  ]
  tasks: list[bt.Node] = [
    create_move_through_frames_task(
      robot=robot,
      frame_names=entry_frames,
      config=config,
      motion_type="ANY",
      task_name=(
        f"Step 13a: Blended Move to Infeed ({' -> '.join(entry_frames)})"
      ),
    )
  ]

  tasks.extend(
    build_interaction_tasks(
      robot=robot,
      config=config,
      frame_name=config.frames.grasp_frame,
      touchdown=config.return_touchdown,
      label="Step 13b",
      pre_reparent_tasks=[
        gripper.build_open_task(
          name="Step 13c: Release Finished Part at Infeed"
        )
      ],
      reparent_task=robot.build_detach_object_task(
        object_name=config.workpiece.object_name,
        name="Step 13d: Detach Finished Part in World",
      ),
    )
  )

  exit_frames = [
    config.frames.pregrasp_frame,
    config.frames.view_frame,
  ]
  retract_to_view = create_move_through_frames_task(
    robot=robot,
    frame_names=exit_frames,
    config=config,
    motion_type=["LINEAR", "ANY"],
    task_name=(
      f"Step 13e: Blended Retract to View ({' -> '.join(exit_frames)})"
    ),
  )
  tasks.append(retract_to_view)
  tasks.append(
    gripper.build_close_task(name="Step 13f: Close Gripper for Next Cycle")
  )

  return bt.Sequence(name="5. Return to Infeed Subtree", children=tasks)
