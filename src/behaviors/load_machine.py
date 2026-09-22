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
  create_move_through_frames_task,
  create_seated_approach_tasks,
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
  2. Blended transit to `preplace_vise_frame` (`[transit_frame,
     machine_approach_frame, preplace_vise_frame]`) with segment-scoped vise
     collision exclusions.
  3. Approach `place_vise_frame` standoff (`LINEAR`) and seat part into vise via
     compliant touchdown along tool +Z (`create_seated_approach_tasks` with
     `config.load_touchdown`).
  4. If `machine` is provided, clamp CNC vise.
  5. Open gripper to release part and detach workpiece entity from gripper in
     the belief world.
  6. Blended exit (`[preplace_vise_frame, machine_approach_frame]`,
     `["LINEAR", "ANY"]`) with vise collision exclusions.

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
  place_vise_frame_name = config.frames.place_vise_frame
  transit_frame_name = config.frames.transit_frame
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

  entry_frames = [
    f
    for f in (
      transit_frame_name,
      machine_approach_frame_name,
      preplace_vise_frame_name,
    )
    if f
  ]
  entry_motions = ["ANY"] * (len(entry_frames) - 1) + ["LINEAR"]

  tasks: list[bt.Node] = []
  if machine is not None:
    tasks.extend(
      [
        machine.build_open_door_task(name="Ensure CNC Door Open"),
        machine.build_open_vise_task(name="Ensure CNC Vise Open"),
      ]
    )

  tasks.extend(
    create_seated_approach_tasks(
      robot=robot,
      frame_name=place_vise_frame_name,
      parent_object=parent_object,
      touchdown=config.load_touchdown,
      config=config,
      label="Load Vise",
      approach_frames=entry_frames,
      approach_motion_types=entry_motions,
      excluded_collision_pairs=vise_collision_pairs,
    )
  )

  if machine is not None:
    tasks.append(machine.build_close_vise_task(name="Clamp CNC Vise"))

  exit_frames = [
    preplace_vise_frame_name,
    machine_approach_frame_name,
  ]
  tasks.extend(
    [
      gripper.build_open_task(name="Release Part in Vise"),
      robot.build_detach_object_task(
        object_name=workpiece_object_name,
        name=f"Detach {workpiece_object_name} from Gripper",
      ),
      create_move_through_frames_task(
        robot=robot,
        frame_names=exit_frames,
        parent_object=parent_object,
        config=config,
        motion_type=["LINEAR", "ANY"],
        excluded_collision_pairs=vise_collision_pairs,
        task_name=f"Blended Exit from Vise ({' -> '.join(exit_frames)})",
      ),
    ]
  )

  return bt.Sequence(name="2. Load Machine Subtree", children=tasks)
