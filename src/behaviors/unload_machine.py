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

"""CNC machine unloading and extraction subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  create_move_through_frames_task,
  create_move_to_frame_task,
  create_seated_approach_tasks,
)
from src.core.config import AppConfig
from src.hardware.grasping import GraspPlannerInterface
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import Robot


def build_unload_machine_subtree(
  robot: Robot,
  gripper: GripperInterface,
  machine: CncMachineInterface | None,
  config: AppConfig,
  grasp_planner: GraspPlannerInterface | None = None,
) -> bt.Node:
  """Builds the Behavior Tree subtree for unloading a finished part from the CNC.

  Sequence:
  1. If `machine` is provided, open CNC door prior to robot entry (keeping the
     CNC vise clamped so the part remains secured during compliant touchdown).
  2. Move robot arm linearly from `machine_approach_frame` to
     `preplace_vise_frame` (`LINEAR`) with segment-scoped vise collision
     exclusions.
  3. Approach `place_vise_frame` standoff (`LINEAR`), compliantly touch down to
     machined part along tool +Z (`config.unload_touchdown`), and execute
     relative linear retract (`create_seated_approach_tasks`). Planners without
     `uses_contact_touchdown` instead open the gripper, plan a grasp on the
     placed part, and move to it through `pre_grasp`, lifting back to
     `pre_grasp` before the retract.
  4. Close gripper to grasp machined part and attach workpiece entity to
     gripper in the belief world.
  5. If `machine` is provided, open CNC vise to unclamp the grasped part.
  6. Blended retract out of enclosure (`[preplace_vise_frame,
     machine_approach_frame]`, `["LINEAR", "ANY"]`).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: Optional CNC machine controller adapter (`None` when cell has no
        CNC enclosure/vise).
      config: Validated application configuration dataclass.
      grasp_planner: Optional grasp planner. Only planners without
        `uses_contact_touchdown` are used here; the others keep the touchdown.

  Returns:
      Behavior tree sequence node executing machine unloading.
  """
  parent_object = config.frames.parent_object
  machine_approach_frame_name = config.frames.machine_approach_frame
  preplace_vise_frame_name = config.frames.preplace_vise_frame
  place_vise_frame_name = config.frames.place_vise_frame
  pregrasp_frame_name = config.frames.pregrasp_frame
  grasp_frame_name = config.frames.grasp_frame
  workpiece_object_name = config.cycle.workpiece_id
  vise_object_name = (
    config.machine.vise_object_name if config.machine is not None else None
  )
  vise_collision_pairs = (
    [
      (workpiece_object_name, vise_object_name),
      (config.robot.tool_object_name, vise_object_name),
      (config.robot.tool_object_name, workpiece_object_name),
    ]
    if vise_object_name
    else [
      (config.robot.tool_object_name, workpiece_object_name),
    ]
  )
  if not vise_object_name and config.robot.enclosure_object_name:
    vise_collision_pairs.extend(
      [
        (config.robot.enclosure_object_name, workpiece_object_name),
        (config.robot.tool_object_name, config.robot.enclosure_object_name),
      ]
    )

  tasks: list[bt.Node] = []
  if machine is not None:
    tasks.append(machine.build_open_door_task(name="Open CNC Door"))

  exact_contacts = (
    grasp_planner is not None and not grasp_planner.uses_contact_touchdown
  )
  if exact_contacts:
    # As in the infeed pick: plan with open jaws, then move to the contacts.
    tasks.extend(
      [
        gripper.build_open_task(name="Open Gripper for Unload"),
        grasp_planner.build_plan_grasp_task(
          workpiece_object_name=workpiece_object_name,
          parent_object=parent_object,
          grasp_frame_name=grasp_frame_name,
          pregrasp_frame_name=pregrasp_frame_name,
          name="Plan Unload Grasp",
        ),
        create_move_through_frames_task(
          robot=robot,
          frame_names=[preplace_vise_frame_name, pregrasp_frame_name],
          parent_object=parent_object,
          config=config,
          motion_type=["LINEAR", "ANY"],
          excluded_collision_pairs=vise_collision_pairs,
          task_name=(
            f"Approach Unload Pre-Grasp ({preplace_vise_frame_name} ->"
            f" {pregrasp_frame_name})"
          ),
        ),
        create_move_to_frame_task(
          robot=robot,
          frame_name=grasp_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          excluded_collision_pairs=vise_collision_pairs,
          task_name="Approach Unload Contact Pose",
        ),
      ]
    )
  else:
    tasks.extend(
      create_seated_approach_tasks(
        robot=robot,
        frame_name=place_vise_frame_name,
        parent_object=parent_object,
        touchdown=config.unload_touchdown,
        config=config,
        label="Unload Vise",
        approach_frames=[preplace_vise_frame_name],
        approach_motion_types="LINEAR",
        excluded_collision_pairs=vise_collision_pairs,
      )
    )

  tasks.extend(
    [
      gripper.build_close_task(name="Grasp Machined Part"),
      robot.build_attach_object_task(
        object_name=workpiece_object_name,
        name=f"Attach {workpiece_object_name} to Gripper",
      ),
    ]
  )
  if machine is not None:
    tasks.append(machine.build_open_vise_task(name="Open CNC Vise"))

  exit_frames = [
    preplace_vise_frame_name,
    machine_approach_frame_name,
  ]
  exit_motion_types = ["LINEAR", "ANY"]
  if exact_contacts:
    # Lift straight up off the planned contacts first.
    exit_frames.insert(0, pregrasp_frame_name)
    exit_motion_types.insert(0, "LINEAR")
  tasks.append(
    create_move_through_frames_task(
      robot=robot,
      frame_names=exit_frames,
      parent_object=parent_object,
      config=config,
      motion_type=exit_motion_types,
      excluded_collision_pairs=vise_collision_pairs,
      task_name=f"Blended Retract from Vise ({' -> '.join(exit_frames)})",
    )
  )

  return bt.Sequence(name="4. Unload Machine Subtree", children=tasks)
