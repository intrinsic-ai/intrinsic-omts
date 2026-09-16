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
  Touchdown,
  create_clear_motion_planner_cache_task,
  create_move_to_frame_task,
  create_seated_approach_tasks,
)
from src.core.infeed import InfeedMode, InfeedStrategy, PerceptionInfeedStrategy
from src.core.types import GripperState
from src.core.workpiece import Workpiece
from src.core.world import World
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def _build_hardware_prep_task(
  gripper: GripperInterface,
  machine: CncMachineInterface | None = None,
) -> bt.Node | None:
  """Builds opening actions for CNC door/vise and gripper if needed."""
  children: list[bt.Node] = []
  if machine is not None:
    children.append(machine.build_open_door_task(name="Open CNC Door"))
    children.append(machine.build_open_vise_task(name="Open CNC Vise"))
  if gripper.commanded_state != GripperState.OPEN:
    children.append(gripper.build_open_task(name="Open Gripper"))
  if not children:
    return None
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
  move_to_view_first: bool = True,
  grasp_offset_z: float = 0.0,
  touchdown: Touchdown | None = None,
  close_gripper_before_perception: bool = False,
  min_safe_z: float | None = 0.95,
  solution: Any | None = None,
  enable_object_reparenting: bool = False,
  clear_motion_planner_cache: bool = False,
  perception_max_retries: int = 3,
  perception_retry_delay_sec: float = 1.0,
  **kwargs: Any,
) -> bt.Node:
  """Builds the Behavior Tree subtree for locating and grasping a raw workpiece."""
  tasks: list[bt.Node] = []

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

    if move_to_view_first:
      tasks.append(
        create_move_to_frame_task(
          robot=robot,
          frame_name=view_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          max_tries=2,
          retry_delay_sec=1.0,
          solution=solution,
          task_name=(
            f"Step 01: Move to View Frame ({parent_object}/{view_frame_name})"
          ),
        )
      )

    if (
      close_gripper_before_perception
      and gripper.commanded_state != GripperState.CLOSED
    ):
      tasks.append(
        gripper.build_close_task(
          name="Step 01b: Close Gripper (Clear Camera FOV)"
        )
      )

    tasks.append(
      vision.build_capture_task(name="Step 02a: Capture RGB-D Images")
    )

    estimate_task = vision.build_estimate_task(
      target_scene_object_id=target_object_id,
      pose_estimator_id=pose_estimator_id,
      min_num_instances=min_instances,
      approach_offset_z=approach_offset_z,
      parent_object=parent_object,
      pregrasp_frame_name=pregrasp_frame_name,
      grasp_frame_name=grasp_frame_name,
      min_safe_z=min_safe_z if min_safe_z is not None else 0.95,
      max_tries=perception_max_retries,
      retry_delay_sec=perception_retry_delay_sec,
      name="Perception & Dynamic Grasp Frame Update Pipeline",
    )

    prep_task = _build_hardware_prep_task(gripper, machine)
    if prep_task is not None:
      tasks.append(
        bt.Parallel(
          name="Step 02b: Parallel Estimation & Machine Prep",
          children=[
            estimate_task,
            prep_task,
          ],
        )
      )
    else:
      tasks.append(estimate_task)
  else:
    prep_task = _build_hardware_prep_task(gripper, machine)
    if prep_task is not None:
      tasks.append(prep_task)

  tasks.append(
    create_move_to_frame_task(
      robot=robot,
      frame_name=pregrasp_frame_name,
      parent_object=parent_object,
      motion_type="ANY",
      max_tries=2,
      retry_delay_sec=1.0,
      solution=solution,
      task_name=(
        f"Step 03: Move to Dynamic Pre-Grasp"
        f" ({parent_object}/{pregrasp_frame_name})"
      ),
    )
  )

  if touchdown is None:
    touchdown = Touchdown(
      force_n=float(kwargs.get("contact_force_newtons", Touchdown.force_n)),
      standoff_m=float(kwargs.get("standoff_distance_m", Touchdown.standoff_m)),
      timeout_s=float(
        kwargs.get("contact_timeout_seconds", Touchdown.timeout_s)
      ),
      retract_after_m=grasp_offset_z,
    )

  tasks.extend(
    create_seated_approach_tasks(
      robot=robot,
      frame_name=grasp_frame_name,
      parent_object=parent_object,
      touchdown=touchdown,
      label="Step 04",
      solution=solution,
    )
  )

  tasks.append(
    gripper.build_close_task(name="Step 05: Close Gripper (Grasp Part)")
  )

  if enable_object_reparenting:
    world = kwargs.get("world") or getattr(solution, "world", None)
    if not hasattr(world, "build_reparent_task"):
      world = World(world, solution=solution)
    tasks.append(
      world.build_reparent_task(
        target=workpiece.scene_object_name,
        new_parent="gripper",
        name="Step 06: Attach Part to Gripper in Digital Twin",
      )
    )
    if clear_motion_planner_cache:
      tasks.append(
        create_clear_motion_planner_cache_task(
          solution=solution,
          task_name="Step 06b: Clear Motion Planner Cache",
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
        f"Step 07: Linear Retract to Pre-Grasp"
        f" ({parent_object}/{pregrasp_frame_name})"
      ),
    )
  )

  return bt.Sequence(name="1. Infeed Pick Subtree", children=tasks)
