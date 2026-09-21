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
  build_interaction_tasks,
  create_move_to_frame_task,
)
from src.core.config import AppConfig
from src.core.infeed import InfeedMode, InfeedStrategy
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
  loop_counter_key: str | None = "cycle_index",
) -> bt.Node:
  """Builds the Behavior Tree subtree for locating and grasping a workpiece.

  Sequence:
      1. Parallel prep: Move robot arm to `view_frame`, close gripper to clear
         camera field of view, and open CNC door + vise.
      2. Capture RGB-D images via `vision.build_capture_image_task`.
      3. Parallel estimation & open gripper: Estimate 6D workpiece poses +
         update dynamic `grasp` and `pre_grasp` frames in `ObjectWorld` while
         opening the gripper fingers.
      4. Approach `pre_grasp` (`LINEAR`), descend to standoff above `grasp`,
         execute compliant touchdown (`config.pick_touchdown`), retract by
         `grasp_offset_z`, close gripper, and attach workpiece in `ObjectWorld`.
      5. Linearly retract to `pre_grasp`.

  Args:
      robot: Robot hardware interface.
      gripper: Gripper hardware interface.
      vision: Vision hardware interface.
      infeed_strategy: Strategy for locating parts at the infeed station.
      config: Application configuration containing frame, cycle, and vision
        settings.
      machine: Optional CNC machine hardware interface for prep actions.
      loop_counter_key: Optional blackboard key of the enclosing `bt.Loop`
        counter (`google.protobuf.Int64Value`, unboxed to `int64` in CEL). When
        provided, guards Step 01 with `<loop_counter_key> == 0` so cycles after
        the first skip the redundant move-to-view and gripper-close (already
        performed by Step 13e/13f of `return_infeed`). When `None` (single-cycle
        execution without a `bt.Loop`), executes Step 01 directly without
        referencing an undefined blackboard key.

  Returns:
      A `bt.Sequence` node executing the infeed pick phase.
  """
  tasks: list[bt.Node] = []
  parent = config.frames.parent_object

  if infeed_strategy.mode == InfeedMode.PERCEPTION:
    if machine is not None:
      tasks.append(machine.build_open_door_task(name="Prep: Open CNC Door"))
      tasks.append(machine.build_open_vise_task(name="Prep: Open CNC Vise"))
    move_to_view = create_move_to_frame_task(
      robot=robot,
      frame_name=config.frames.view_frame,
      config=config,
      motion_type="ANY",
      task_name=(
        f"Step 01a: Move to View Frame ({parent}/{config.frames.view_frame})"
      ),
    )
    step_01_seq = bt.Sequence(
      name="Step 01: Move to View & Close Gripper",
      children=[
        gripper.build_close_task(
          name="Step 01b: Close Gripper (Clear Camera FOV)"
        ),
        move_to_view,
      ],
    )
    if loop_counter_key:
      tasks.append(
        bt.Branch(
          if_condition=bt.Blackboard(f"{loop_counter_key} == 0"),
          then_child=step_01_seq,
          name="Step 01: First-Cycle Move to View & Close Gripper Guard",
        )
      )
    else:
      tasks.append(step_01_seq)

    capture_node, capture_data = vision.build_capture_image_task(
      max_tries=1,
      task_name="Step 02a: Capture RGB-D Images",
    )

    estimate_node, estimates = vision.build_estimate_pose_task(
      capture_data=capture_data,
      config=config.vision,
      task_name="Estimate 6D Workpiece Poses",
    )
    update_frames_node = vision.build_update_grasp_frames_task(
      estimates=estimates,
      config=config,
    )
    perception_cycle_seq = bt.Sequence(
      name="Step 02: Capture, Estimate Poses & Open Gripper",
      children=[
        capture_node,
        estimate_node,
        update_frames_node,
        gripper.build_open_task(name="Step 03: Open Gripper"),
      ],
    )
    tasks.append(
      bt.Retry(
        max_tries=3,
        child=perception_cycle_seq,
        recovery=gripper.build_close_task(
          name="Recovery: Re-Close Gripper (Clear Camera FOV)"
        ),
        name="Step 02: Retryable Perception & Gripper Open (max 3 tries)",
      )
    )
  else:
    if machine is not None:
      tasks.append(machine.build_open_door_task(name="Prep: Open CNC Door"))
      tasks.append(machine.build_open_vise_task(name="Prep: Open CNC Vise"))
    tasks.append(gripper.build_open_task(name="Step 03: Open Gripper"))

  tasks.extend(
    build_interaction_tasks(
      robot=robot,
      config=config,
      frame_name=config.frames.grasp_frame,
      touchdown=config.pick_touchdown,
      label="Step 04",
      approach_frames=[config.frames.pregrasp_frame],
      approach_motion_types="LINEAR",
      excluded_collision_pairs=config.pick_collision_pairs,
      pre_reparent_tasks=[
        gripper.build_close_task(name="Step 05a: Grasp Workpiece")
      ],
      reparent_task=robot.build_attach_object_task(
        object_name=config.workpiece.object_name,
        name="Step 05b: Attach Workpiece in World",
      ),
    )
  )

  tasks.append(
    create_move_to_frame_task(
      robot=robot,
      frame_name=config.frames.pregrasp_frame,
      config=config,
      motion_type="LINEAR",
      excluded_collision_pairs=config.pick_collision_pairs,
      task_name="Step 06: Retract to Dynamic Pre-Grasp",
    )
  )

  return bt.Sequence(name="1. Infeed Pick Subtree", children=tasks)
