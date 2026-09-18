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
  create_compliant_touchdown_task,
  create_move_to_frame_task,
  create_relative_retract_task,
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
  1. If `machine` is provided, execute CNC machine preparation (open door + open
     vise) prior to robot motion to avoid `lock_the_universe` footprint conflicts.
  2. Move robot to view frame, run 6D pose estimation, and open gripper.
  3. Move to dynamic pre_grasp frame (ANY Cartesian motion).
  4. Perform compliant touchdown (move_to_contact in +Z tool).
  5. Linear retract along tool -Z to align finger pads with part.
  6. Close gripper to grasp part.
  7. Attach workpiece entity to robot gripper in the belief world.
  8. Retract arm linearly back up to pre_grasp (LINEAR Cartesian motion).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      vision: Vision/perception adapter.
      infeed_strategy: Infeed strategy model.
      config: Application configuration dataclass.
      machine: Optional CNC machine adapter to prepare prior to pick.

  Returns:
      Behavior tree sequence executing the infeed pick pipeline.
  """
  parent_object = config.frames.parent_object
  view_frame_name = config.frames.view_frame
  pregrasp_frame_name = config.frames.pregrasp_frame
  grasp_frame_name = config.frames.grasp_frame
  approach_offset_z = config.cycle.approach_offset_z
  pick_touchdown_force_newtons = config.cycle.pick_touchdown_force_newtons
  touchdown_timeout_seconds = config.cycle.touchdown_timeout_seconds
  retract_distance_meters = config.cycle.retract_distance_meters
  workpiece_object_name = config.cycle.workpiece_id

  tasks: list[bt.Node] = []

  if machine is not None:
    tasks.extend(
      [
        machine.build_open_door_task(name="Prep: Open CNC Door"),
        machine.build_open_vise_task(name="Prep: Open CNC Vise"),
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
          task_name=f"Step 01: Move to View Frame ({parent_object}/{view_frame_name})",
        ),
        vision.build_perception_and_spawn_task(
          approach_offset_z=approach_offset_z,
          parent_object=parent_object,
          pregrasp_frame_name=pregrasp_frame_name,
          grasp_frame_name=grasp_frame_name,
          tool_object_name=config.robot.tool_object_name,
          tool_frame_name=config.robot.tool_frame_name,
          name="Step 02: Perception & Dynamic Grasp Frame Update Pipeline",
        ),
      ]
    )

  tasks.append(gripper.build_open_task(name="Step 03: Open Gripper"))

  tasks.extend(
    [
      create_move_to_frame_task(
        robot=robot,
        frame_name=pregrasp_frame_name,
        parent_object=parent_object,
        motion_type="ANY",
        task_name=f"Step 04: Move to Dynamic Pre-Grasp ({parent_object}/{pregrasp_frame_name})",
      ),
      create_compliant_touchdown_task(
        robot=robot,
        direction=(0.0, 0.0, 1.0),
        contact_force_newtons=pick_touchdown_force_newtons,
        timeout_seconds=touchdown_timeout_seconds,
        task_name="Step 05: Compliant Touchdown to Part (+Z Tool)",
      ),
      create_relative_retract_task(
        robot=robot,
        distance_meters=retract_distance_meters,
        exclude_collision=True,
        excluded_collision_objects=(workpiece_object_name,),
        task_name=f"Step 06: Linear Retract ({retract_distance_meters * 100:.1f} cm, -Z Tool)",
      ),
      gripper.build_close_task(name="Step 07a: Close Gripper (Grasp Part)"),
      robot.build_attach_object_task(
        object_name=workpiece_object_name,
        name=f"Step 07b: Attach {workpiece_object_name} to Gripper",
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=pregrasp_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        task_name=f"Step 08: Linear Retract to Pre-Grasp ({parent_object}/{pregrasp_frame_name})",
      ),
    ]
  )

  return bt.Sequence(name="1. Infeed Pick Subtree", children=tasks)
