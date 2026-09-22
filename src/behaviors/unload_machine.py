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
  create_seated_approach_tasks,
)
from src.core.config import AppConfig
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_unload_machine_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  machine: CncMachineInterface | None,
  config: AppConfig,
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
     relative linear retract (`create_seated_approach_tasks`).
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

  Returns:
      Behavior tree sequence node executing machine unloading.
  """
  parent_object = config.frames.parent_object
  machine_approach_frame_name = config.frames.machine_approach_frame
  preplace_vise_frame_name = config.frames.preplace_vise_frame
  place_vise_frame_name = config.frames.place_vise_frame
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

  tasks: list[bt.Node] = []
  if machine is not None:
    tasks.append(machine.build_open_door_task(name="Open CNC Door"))

  tasks.extend(
    create_seated_approach_tasks(
      robot=robot,
      frame_name=place_vise_frame_name,
      parent_object=parent_object,
      touchdown=config.unload_touchdown,
      config=config,
      label="Unload Vise",
      approach_frames=[preplace_vise_frame_name],
      approach_motion_types=["LINEAR"],
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
  tasks.append(
    create_move_through_frames_task(
      robot=robot,
      frame_names=exit_frames,
      parent_object=parent_object,
      config=config,
      motion_type=["LINEAR", "ANY"],
      excluded_collision_pairs=vise_collision_pairs,
      task_name=f"Blended Retract from Vise ({' -> '.join(exit_frames)})",
    )
  )

  return bt.Sequence(name="4. Unload Machine Subtree", children=tasks)
