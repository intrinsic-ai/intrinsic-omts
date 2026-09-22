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

"""Infeed part localization and picking subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  create_move_to_frame_task,
  create_seated_approach_tasks,
)
from src.core.config import AppConfig
from src.core.infeed import InfeedMode, InfeedStrategy, PerceptionInfeedStrategy
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def build_pick_from_infeed_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  vision: VisionInterface,
  infeed_strategy: InfeedStrategy,
  config: AppConfig,
  machine: CncMachineInterface | None = None,
) -> bt.Node:
  """Builds the Behavior Tree subtree for locating and grasping a raw workpiece.

  Sequence:
  1. If `machine` is provided, open CNC door and vise sequentially prior to
     robot motion to avoid `lock_the_universe` resource conflicts.
  2. If `infeed_strategy.mode` is `PERCEPTION`, move to `view_frame` (`ANY`)
     and run the 3-step perception pipeline (capture RGB-D, estimate 6D pose
     via FoundationPose, and update dynamic `pre_grasp`/`grasp` frames).
  3. Open gripper fingers.
  4. Move tool to dynamic `pre_grasp` frame (`ANY`).
  5. Approach `grasp` standoff (`LINEAR`), perform compliant touchdown along
     tool +Z (`config.pick_touchdown`), and execute relative linear retract
     along tool -Z (`create_seated_approach_tasks`).
  6. Close gripper to grasp part and attach workpiece entity to gripper in the
     belief world.
  7. Retract arm linearly back up to `pre_grasp` (`LINEAR`).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      vision: Vision/perception adapter.
      infeed_strategy: Infeed strategy model (`PerceptionInfeedStrategy`).
      config: Validated application configuration dataclass.
      machine: Optional CNC machine adapter to prepare prior to pick.

  Returns:
      Behavior tree sequence executing the infeed pick pipeline.
  """
  parent_object = config.frames.parent_object
  view_frame_name = config.frames.view_frame
  pregrasp_frame_name = config.frames.pregrasp_frame
  grasp_frame_name = config.frames.grasp_frame
  approach_offset_z = config.cycle.approach_offset_z
  workpiece_object_name = config.cycle.workpiece_id

  tasks: list[bt.Node] = []

  if machine is not None:
    tasks.extend(
      [
        machine.build_open_door_task(name="Open CNC Door"),
        machine.build_open_vise_task(name="Open CNC Vise"),
      ]
    )

  if infeed_strategy.mode == InfeedMode.PERCEPTION:
    if not isinstance(infeed_strategy, PerceptionInfeedStrategy):
      raise TypeError(
        "InfeedMode.PERCEPTION requires a PerceptionInfeedStrategy instance."
      )

    tasks.extend(
      [
        create_move_to_frame_task(
          robot=robot,
          frame_name=view_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          task_name=f"Move to View Frame ({parent_object}/{view_frame_name})",
        ),
        gripper.build_close_task(name="Close Gripper (Clear View)"),
        vision.build_perception_and_spawn_task(
          approach_offset_z=approach_offset_z,
          parent_object=parent_object,
          pregrasp_frame_name=pregrasp_frame_name,
          grasp_frame_name=grasp_frame_name,
          tool_object_name=config.robot.tool_object_name,
          tool_frame_name=config.robot.tool_frame_name,
          name="Perception & Dynamic Grasp Frame Update Pipeline",
        ),
      ]
    )

  tasks.append(gripper.build_open_task(name="Open Gripper"))

  tool_to_workpiece_collision_pair = (
    config.robot.tool_object_name,
    workpiece_object_name,
  )
  enclosure_to_workpiece_collision_pair = (
    config.robot.enclosure_object_name,
    workpiece_object_name,
  )

  approach_collision_pairs = [tool_to_workpiece_collision_pair]
  post_attach_collision_pairs = [tool_to_workpiece_collision_pair]
  if config.robot.enclosure_object_name:
    post_attach_collision_pairs.append(enclosure_to_workpiece_collision_pair)

  tasks.append(
    create_move_to_frame_task(
      robot=robot,
      frame_name=pregrasp_frame_name,
      parent_object=parent_object,
      motion_type="ANY",
      task_name=(
        f"Move to Dynamic Pre-Grasp ({parent_object}/{pregrasp_frame_name})"
      ),
    )
  )

  tasks.extend(
    create_seated_approach_tasks(
      robot=robot,
      frame_name=grasp_frame_name,
      parent_object=parent_object,
      touchdown=config.pick_touchdown,
      config=config,
      label="Infeed Pick",
      excluded_collision_pairs=approach_collision_pairs,
    )
  )

  tasks.extend(
    [
      gripper.build_close_task(name="Close Gripper (Grasp Part)"),
      robot.build_attach_object_task(
        object_name=workpiece_object_name,
        name=f"Attach {workpiece_object_name} to Gripper",
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=pregrasp_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        excluded_collision_pairs=post_attach_collision_pairs,
        task_name=(
          "Retract to Dynamic Pre-Grasp"
          f" ({parent_object}/{pregrasp_frame_name})"
        ),
      ),
    ]
  )

  return bt.Sequence(name="1. Infeed Pick Subtree", children=tasks)
