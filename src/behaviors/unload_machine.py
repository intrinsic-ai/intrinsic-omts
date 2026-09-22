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
  create_compliant_touchdown_task,
  create_move_to_frame_task,
  create_relative_retract_task,
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
  1. If `machine` is provided, open CNC door and open CNC vise prior to robot
     entry.
  2. Move robot arm to machine entry approach position (`machine_approach_frame`,
     `ANY`).
  3. Move robot arm to vise pre-place approach position (`preplace_vise_frame`,
     `ANY`) with segment-scoped vise collision exclusions.
  4. Compliantly touch down to machined part along tool +Z.
  5. Execute relative linear retract (`retract_distance_meters`, `-Z` tool)
     with workpiece collision exclusion to align finger pads.
  6. Close gripper to grasp machined part and attach workpiece entity to
     gripper in the belief world.
  7. Execute relative linear retract (`retract_distance_meters`, `-Z` tool)
     with workpiece collision exclusion to lift part clear of vise jaws.
  8. Retract arm linearly out of enclosure to `machine_approach_frame`
     (`LINEAR`).

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
  unload_touchdown_force_newtons = config.cycle.unload_touchdown_force_newtons
  touchdown_timeout_seconds = config.cycle.touchdown_timeout_seconds
  retract_distance_meters = config.cycle.retract_distance_meters
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
    tasks.extend(
      [
        machine.build_open_door_task(name="Open CNC Door"),
        machine.build_open_vise_task(name="Open CNC Vise"),
      ]
    )

  tasks.extend(
    [
      create_move_to_frame_task(
        robot=robot,
        frame_name=machine_approach_frame_name,
        parent_object=parent_object,
        motion_type="ANY",
        task_name=f"Approach Machine Entry ({parent_object}/{machine_approach_frame_name})",
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=preplace_vise_frame_name,
        parent_object=parent_object,
        motion_type="ANY",
        excluded_collision_pairs=vise_collision_pairs,
        task_name=f"Approach Machined Part ({parent_object}/{preplace_vise_frame_name})",
      ),
      create_compliant_touchdown_task(
        robot=robot,
        direction=(0.0, 0.0, 1.0),
        contact_force_newtons=unload_touchdown_force_newtons,
        timeout_seconds=touchdown_timeout_seconds,
        task_name="Compliant Touchdown to Machined Part (+Z Tool)",
      ),
      create_relative_retract_task(
        robot=robot,
        distance_meters=retract_distance_meters,
        excluded_collision_pairs=[
          (config.robot.tool_object_name, workpiece_object_name),
        ],
        task_name=f"Linear Retract ({retract_distance_meters * 100:.1f} cm, -Z Tool)",
      ),
      gripper.build_close_task(name="Grasp Machined Part"),
      robot.build_attach_object_task(
        object_name=workpiece_object_name,
        name=f"Attach {workpiece_object_name} to Gripper",
      ),
      create_relative_retract_task(
        robot=robot,
        distance_meters=retract_distance_meters,
        excluded_collision_pairs=[
          (config.robot.tool_object_name, workpiece_object_name),
        ],
        task_name=f"Linear Retract Clear of Vise ({retract_distance_meters * 100:.1f} cm, -Z Tool)",
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=machine_approach_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        task_name=f"Retract Machined Part from Machine ({parent_object}/{machine_approach_frame_name})",
      ),
    ]
  )

  return bt.Sequence(name="4. Unload Machine Subtree", children=tasks)
