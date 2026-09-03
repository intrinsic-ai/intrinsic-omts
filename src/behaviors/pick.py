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
from src.core.infeed import InfeedMode, InfeedStrategy, PerceptionInfeedStrategy
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def build_pick_from_infeed_subtree(
    robot: RobotInterface,
    gripper: GripperInterface,
    vision: VisionInterface,
    infeed_strategy: InfeedStrategy,
    workpiece: Workpiece,
    parent_object: str = "root",
    view_frame_name: str = "view",
    pregrasp_frame_name: str = "pre_grasp",
    grasp_frame_name: str = "grasp",
    approach_offset_z: float = 0.08,
) -> bt.Node:
  """Builds the Behavior Tree subtree for locating and grasping a raw workpiece.

  Sequence:
  1. Move robot to view frame (ANY Cartesian motion).
  2. Perception acquisition & dynamic grasp frame update step:
     - capture RGB-D -> estimate 6D pose -> dynamically update root/pre_grasp and root/grasp
       via indirect transforms from the camera with zero hardcoded extrinsics.
  3. Open gripper fingers before approach.
  4. Move to dynamic pre_grasp frame (ANY Cartesian motion with tool Z rotation relaxed).
  5. Perform compliant touchdown (move_to_contact in +Z tool).
  6. Linear retract 3 cm along tool -Z to align finger pads with part before closing.
  7. Close gripper to grasp part.
  8. Retract arm linearly back up to pre_grasp (LINEAR Cartesian motion).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      vision: Vision/perception adapter.
      infeed_strategy: Infeed strategy model.
      workpiece: Workpiece model instance.
      parent_object: Name of parent object for target frames (default: 'root').
      view_frame_name: Name of perception view frame (default: 'view').
      pregrasp_frame_name: Name of pre-grasp approach frame (default: 'pre_grasp').
      grasp_frame_name: Name of grasp target frame (default: 'grasp').
      approach_offset_z: Approach standoff distance in meters (default: 0.05).

  Returns:
      Behavior tree sequence executing the infeed pick pipeline.
  """
  tasks: list[bt.Node] = []

  if infeed_strategy.mode == InfeedMode.PERCEPTION:
    target_object_id = (
        infeed_strategy.scene_object_id
        if isinstance(infeed_strategy, PerceptionInfeedStrategy)
        else "ai.intrinsic.raw_stock_2x3x5"
    )
    pose_estimator_id = (
        infeed_strategy.pose_estimator_id
        if isinstance(infeed_strategy, PerceptionInfeedStrategy)
        else "ai.intrinsic.raw_stock_2x3x5_estimator"
    )
    min_instances = (
        infeed_strategy.min_num_instances
        if isinstance(infeed_strategy, PerceptionInfeedStrategy)
        else 1
    )

    tasks.extend([
        create_move_to_frame_task(
            robot=robot,
            frame_name=view_frame_name,
            parent_object=parent_object,
            motion_type="ANY",
            task_name=f"Step 01: Move to View Frame ({parent_object}/{view_frame_name})",
        ),
        vision.build_perception_and_spawn_task(
            target_scene_object_id=target_object_id,
            pose_estimator_id=pose_estimator_id,
            min_num_instances=min_instances,
            approach_offset_z=approach_offset_z,
            parent_object=parent_object,
            pregrasp_frame_name=pregrasp_frame_name,
            grasp_frame_name=grasp_frame_name,
            name="Step 02: Perception & Dynamic Grasp Frame Update Pipeline",
        ),
    ])

  tasks.extend([
      gripper.build_open_task(name="Step 03: Open Gripper"),
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
          contact_force_newtons=15.0,
          task_name="Step 05: Compliant Touchdown to Part (+Z Tool)",
      ),
      create_relative_retract_task(
          robot=robot,
          distance_meters=0.03,
          task_name="Step 06: Linear Retract (3 cm, -Z Tool)",
      ),
      gripper.build_close_task(name="Step 07: Close Gripper (Grasp Part)"),
      create_move_to_frame_task(
          robot=robot,
          frame_name=pregrasp_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          task_name=f"Step 08: Linear Retract to Pre-Grasp ({parent_object}/{pregrasp_frame_name})",
      ),
  ])

  return bt.Sequence(name="1. Infeed Pick Subtree", children=tasks)
