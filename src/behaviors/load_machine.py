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

"""CNC machine loading and fixturing subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  create_compliant_touchdown_task,
  create_move_to_frame_task,
)
from src.core.config import AppConfig
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_load_machine_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  machine: CncMachineInterface | None,
  config: AppConfig,
) -> bt.Node:
  """Builds the Behavior Tree subtree for loading raw stock into the CNC machine.

  Sequence:
  1. If `machine` is provided, ensure CNC door and vise are open prior to entry.
  2. Transit to `machine_approach_frame` (`ANY`), blending through
     `transit_frame` if configured.
  3. Move arm to `preplace_vise_frame` (`ANY`) with segment-scoped collision
     exclusions between tool, workpiece, and vise.
  4. Seat part into vise via compliant touchdown along tool +Z.
  5. If `machine` is provided, clamp CNC vise.
  6. Open gripper to release part and detach workpiece entity from gripper in
     the belief world.
  7. Retract arm linearly to `preplace_vise_frame` with vise collision
     exclusions (`LINEAR`) and then to `machine_approach_frame` (`LINEAR`).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: Optional CNC machine adapter (`None` when cell has no CNC).
      config: Validated application configuration dataclass.

  Returns:
      Behavior tree sequence executing machine loading and fixturing.
  """
  parent_object = config.frames.parent_object
  machine_approach_frame_name = config.frames.machine_approach_frame
  preplace_vise_frame_name = config.frames.preplace_vise_frame
  transit_frame_name = config.frames.transit_frame
  load_seat_force_newtons = config.cycle.load_seat_force_newtons
  touchdown_timeout_seconds = config.cycle.touchdown_timeout_seconds
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
        machine.build_open_door_task(name="Ensure CNC Door Open"),
        machine.build_open_vise_task(name="Ensure CNC Vise Open"),
      ]
    )

  if transit_frame_name:
    tasks.append(
      robot.build_move_blended_cartesian_task(
        target_frames=[
          (parent_object, transit_frame_name),
          (parent_object, machine_approach_frame_name),
        ],
        motion_type="ANY",
        name=f"Blended Transit to Machine Entry ({parent_object}/{transit_frame_name} -> {parent_object}/{machine_approach_frame_name})",
      )
    )
  else:
    tasks.append(
      create_move_to_frame_task(
        robot=robot,
        frame_name=machine_approach_frame_name,
        parent_object=parent_object,
        motion_type="ANY",
        task_name=f"Approach Machine Entry ({parent_object}/{machine_approach_frame_name})",
      )
    )

  tasks.extend(
    [
      create_move_to_frame_task(
        robot=robot,
        frame_name=preplace_vise_frame_name,
        parent_object=parent_object,
        motion_type="ANY",
        excluded_collision_pairs=vise_collision_pairs,
        task_name=f"Approach CNC Vise ({parent_object}/{preplace_vise_frame_name})",
      ),
      create_compliant_touchdown_task(
        robot=robot,
        direction=(0.0, 0.0, 1.0),
        contact_force_newtons=load_seat_force_newtons,
        timeout_seconds=touchdown_timeout_seconds,
        task_name="Compliant Seat Part into Vise (+Z Tool)",
      ),
    ]
  )
  if machine is not None:
    tasks.append(machine.build_close_vise_task(name="Clamp CNC Vise"))

  tasks.extend(
    [
      gripper.build_open_task(name="Release Part in Vise"),
      robot.build_detach_object_task(
        object_name=workpiece_object_name,
        name=f"Detach {workpiece_object_name} from Gripper",
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=preplace_vise_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        excluded_collision_pairs=vise_collision_pairs,
        task_name=f"Retract Arm to Vise Approach ({parent_object}/{preplace_vise_frame_name})",
      ),
      create_move_to_frame_task(
        robot=robot,
        frame_name=machine_approach_frame_name,
        parent_object=parent_object,
        motion_type="LINEAR",
        task_name=f"Retract Arm to Machine Entry ({parent_object}/{machine_approach_frame_name})",
      ),
    ]
  )

  return bt.Sequence(name="2. Load Machine Subtree", children=tasks)
