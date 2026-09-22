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

"""CNC machine loading and vise seating subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  build_interaction_tasks,
  create_move_through_frames_task,
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
  include_entry_guard: bool = False,
) -> bt.Node:
  """Builds the Behavior Tree subtree for loading raw stock into the CNC vise.

  Sequence:
      1. Optional mid-cycle entry guard (`include_entry_guard=True`): open CNC
         door and vise if entering directly at `Phase.LOAD`.
      2. Blended transit (`[transit_frame, machine_approach_frame,
         preplace_vise_frame]`).
      3. Linear standoff approach + compliant touchdown (`config.load_touchdown`)
         into `place_vise_frame`.
      4. Clamp CNC vise, open gripper to release part, and detach workpiece in
         `ObjectWorld`.
      5. Blended exit (`[preplace_vise_frame, machine_approach_frame]`) leaving
         the arm outside the enclosure at `machine_approach_frame`.

  Args:
      robot: Robot hardware interface.
      gripper: Gripper hardware interface.
      machine: Optional CNC machine hardware interface.
      config: Application configuration.
      include_entry_guard: Whether to prepend door/vise open guards for
        mid-cycle entry.

  Returns:
      A `bt.Sequence` node executing the machine loading phase.
  """
  entry_frames = [
    f
    for f in (
      config.frames.transit_frame,
      config.frames.machine_approach_frame,
      config.frames.preplace_vise_frame,
    )
    if f
  ]
  entry_motions = ["ANY"] * (len(entry_frames) - 1) + ["LINEAR"]

  tasks: list[bt.Node] = []
  if include_entry_guard and machine is not None:
    tasks.append(
      machine.build_open_door_task(name="Prep: Open CNC Door (Entry Guard)")
    )
    tasks.append(
      machine.build_open_vise_task(name="Prep: Open CNC Vise (Entry Guard)")
    )
  tasks.append(
    create_move_through_frames_task(
      robot=robot,
      frame_names=entry_frames,
      config=config,
      motion_type=entry_motions,
      excluded_collision_pairs=config.vise_collision_pairs,
      task_name=(
        f"Step 07a: Blended Transit to Vise ({' -> '.join(entry_frames)})"
      ),
    )
  )

  pre_reparent: list[bt.Node] = []
  if machine is not None:
    pre_reparent.append(
      machine.build_close_vise_task(name="Step 07c: Clamp CNC Vise")
    )
  pre_reparent.append(
    gripper.build_open_task(name="Step 07d: Release Workpiece in Vise")
  )

  tasks.extend(
    build_interaction_tasks(
      robot=robot,
      config=config,
      frame_name=config.frames.place_vise_frame,
      touchdown=config.load_touchdown,
      label="Step 07b",
      excluded_collision_pairs=config.vise_collision_pairs,
      pre_reparent_tasks=pre_reparent,
      reparent_task=robot.build_detach_object_task(
        object_name=config.workpiece.object_name,
        name="Step 07e: Detach Workpiece in World",
      ),
    )
  )

  exit_frames = [
    config.frames.preplace_vise_frame,
    config.frames.machine_approach_frame,
  ]
  tasks.append(
    create_move_through_frames_task(
      robot=robot,
      frame_names=exit_frames,
      config=config,
      motion_type=["LINEAR", "ANY"],
      excluded_collision_pairs=config.vise_collision_pairs,
      task_name=(
        f"Step 08: Blended Exit from Vise ({' -> '.join(exit_frames)})"
      ),
    )
  )

  return bt.Sequence(name="2. Load Machine Subtree", children=tasks)
