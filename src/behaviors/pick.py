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

from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  DEFAULT_TOUCHDOWN,
  Touchdown,
  build_interaction_tasks,
  create_move_to_frame_task,
)
from src.core.infeed import InfeedMode, InfeedStrategy, PerceptionInfeedStrategy
from src.core.workpiece import Workpiece
from src.core.world import WorldInterface, resolve_world
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def _build_hardware_prep_task(
  gripper: GripperInterface,
  machine: CncMachineInterface | None = None,
) -> bt.Node:
  """Builds opening actions for CNC door/vise and gripper."""
  children: list[bt.Node] = []
  if machine is not None:
    children.append(machine.build_open_door_task(name="Open CNC Door"))
    children.append(machine.build_open_vise_task(name="Open CNC Vise"))
  children.append(gripper.build_open_task(name="Open Gripper"))
  if len(children) == 1:
    return children[0]
  return bt.Sequence(name="CNC Machine & Gripper Prep", children=children)


def build_pick_from_infeed_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  vision: VisionInterface,
  infeed_strategy: InfeedStrategy,
  workpiece: Workpiece,
  machine: CncMachineInterface | None = None,
  parent_object: str = "root",
  view_frame_name: str = "view",
  pregrasp_frame_name: str = "infeed_pre_grasp",
  grasp_frame_name: str = "infeed_grasp",
  approach_offset_z: float = 0.08,
  touchdown: Touchdown = DEFAULT_TOUCHDOWN,
  min_safe_z: float = 0.95,
  solution: Any | None = None,
  world: WorldInterface | None = None,
  enable_object_reparenting: bool = False,
  perception_max_retries: int = 3,
  perception_retry_delay_sec: float = 1.0,
) -> bt.Node:
  """Builds the Behavior Tree subtree for locating and grasping a workpiece."""
  tasks: list[bt.Node] = []
  w = resolve_world(solution, world)

  if infeed_strategy.mode == InfeedMode.PERCEPTION:
    target_object_id = (
      infeed_strategy.scene_object_id
      if isinstance(infeed_strategy, PerceptionInfeedStrategy)
      else workpiece.asset_id
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

    # TODO: on subsequent cycles, return_infeed.py will have already moved
    # us back to the view frame, so we could skip it to save cycle time
    step_01_children: list[bt.Node] = [
      create_move_to_frame_task(
        robot=robot,
        frame_name=view_frame_name,
        parent_object=parent_object,
        motion_type="ANY",
        max_tries=2,
        retry_delay_sec=1.0,
        solution=solution,
        task_name=(
          f"Step 01a: Move to View Frame ({parent_object}/{view_frame_name})"
        ),
      ),
      gripper.build_close_task(
        name="Step 01b: Close Gripper (Clear Camera FOV)"
      ),
    ]
    if machine is not None:
      step_01_children.append(
        machine.build_open_door_task(name="Open CNC Door")
      )
      step_01_children.append(
        machine.build_open_vise_task(name="Open CNC Vise")
      )

    tasks.append(
      bt.Parallel(
        name="Step 01: Move to View & Prep Machine",
        children=step_01_children,
      )
    )

    capture_node, capture_data = vision.build_capture_image_task(
      max_tries=perception_max_retries,
      retry_delay_sec=perception_retry_delay_sec,
      name="Step 02a: Capture RGB-D Images",
    )
    tasks.append(capture_node)

    estimate_node, estimates = vision.build_estimate_pose_task(
      capture_data=capture_data,
      pose_estimator_id=pose_estimator_id,
      min_num_instances=min_instances,
      name="Estimate 6D Workpiece Poses",
    )
    update_frames_node = w.build_update_grasp_frames_task(
      estimates=estimates,
      camera_name=getattr(vision, "_camera_name", "orbbec_camera"),
      target_scene_object_id=target_object_id,
      parent_object=parent_object,
      pregrasp_frame_name=pregrasp_frame_name,
      grasp_frame_name=grasp_frame_name,
      approach_offset_z=approach_offset_z,
      min_safe_z=min_safe_z,
    )
    estimate_and_update_seq = bt.Sequence(
      name="Perception & Dynamic Grasp Frame Update Pipeline",
      children=[estimate_node, update_frames_node],
    )

    tasks.append(
      bt.Parallel(
        name="Step 02b: Parallel Estimation & Open Gripper",
        children=[
          estimate_and_update_seq,
          gripper.build_open_task(name="Open Gripper"),
        ],
      )
    )
  else:
    tasks.append(_build_hardware_prep_task(gripper, machine))

  reparent_task = (
    w.build_attach_to_gripper_task(
      object_name=workpiece.object_name,
      name="Step 05: Attach Part to Gripper in Digital Twin",
    )
    if enable_object_reparenting
    else None
  )

  tasks.extend(
    build_interaction_tasks(
      robot=robot,
      frame_name=grasp_frame_name,
      parent_object=parent_object,
      touchdown=touchdown,
      label="Step 03",
      approach_frames=[pregrasp_frame_name],
      approach_motion_types="ANY",
      pre_reparent_tasks=[
        gripper.build_close_task(name="Step 04: Close Gripper (Grasp Part)")
      ],
      reparent_task=reparent_task,
      solution=solution,
    )
  )

  tasks.append(
    create_move_to_frame_task(
      robot=robot,
      frame_name=pregrasp_frame_name,
      parent_object=parent_object,
      motion_type="LINEAR",
      max_tries=2,
      retry_delay_sec=1.0,
      solution=solution,
      task_name=(
        f"Step 06: Linear Retract to Pre-Grasp"
        f" ({parent_object}/{pregrasp_frame_name})"
      ),
    )
  )

  return bt.Sequence(name="1. Infeed Pick Subtree", children=tasks)
