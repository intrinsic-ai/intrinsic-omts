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

"""CNC machine unloading and part extraction subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  build_interaction_tasks,
  create_move_through_frames_task,
  create_move_to_frame_task,
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
  include_entry_guard: bool = False,
) -> bt.Node:
  """Builds the Behavior Tree subtree for unloading a machined part.

  Sequence:
      1. Optional mid-cycle entry guard (`include_entry_guard=True`): move arm
         to `machine_approach_frame` if entering directly at `Phase.UNLOAD`.
      2. Open CNC enclosure door (keeping the CNC vise clamped so the part is
         securely held during compliant touchdown).
      3. Linearly enter through the door from `machine_approach_frame` to
         `preplace_vise_frame`.
      4. Approach `place_vise_frame` (`standoff_m = 0.010 + grasp_offset_z`),
         compliantly touch down (`config.unload_touchdown`), retract by
         `grasp_offset_z`, close gripper (`pre_reparent_tasks`), attach part in
         `ObjectWorld` (`reparent_task`), and **then** open the CNC vise
         (`post_reparent_tasks`) to unclamp the grasped part.
      5. Blended retract (`[preplace_vise_frame, machine_approach_frame]`) out
         of the enclosure.

  Args:
      robot: Robot hardware interface.
      gripper: Gripper hardware interface.
      machine: Optional CNC machine hardware interface.
      config: Application configuration.
      include_entry_guard: Whether to prepend a safety move to
        `machine_approach_frame` for mid-cycle entry.

  Returns:
      A `bt.Sequence` node executing the machine unloading phase.
  """
  tasks: list[bt.Node] = []
  if include_entry_guard:
    tasks.append(
      create_move_to_frame_task(
        robot=robot,
        frame_name=config.frames.machine_approach_frame,
        config=config,
        motion_type="ANY",
        task_name="Step 10a: Move to Machine Approach (Entry Guard)",
      )
    )
  if machine is not None:
    tasks.append(machine.build_open_door_task(name="Step 10: Open CNC Door"))

  tasks.append(
    create_move_to_frame_task(
      robot=robot,
      frame_name=config.frames.preplace_vise_frame,
      config=config,
      motion_type="LINEAR",
      excluded_collision_pairs=config.vise_collision_pairs,
      task_name=f"Step 11a: Linear Move to {config.frames.preplace_vise_frame}",
    )
  )

  post_reparent: list[bt.Node] = []
  if machine is not None:
    post_reparent.append(
      machine.build_open_vise_task(
        name="Step 11e: Open CNC Vise (Unclamp Part)"
      )
    )

  tasks.extend(
    build_interaction_tasks(
      robot=robot,
      config=config,
      frame_name=config.frames.place_vise_frame,
      touchdown=config.unload_touchdown,
      label="Step 11b",
      excluded_collision_pairs=config.vise_collision_pairs,
      pre_reparent_tasks=[
        gripper.build_close_task(name="Step 11c: Grasp Machined Part")
      ],
      reparent_task=robot.build_attach_object_task(
        object_name=config.workpiece.object_name,
        name="Step 11d: Attach Machined Part in World",
      ),
      post_reparent_tasks=post_reparent,
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
        f"Step 12: Blended Retract from Vise ({' -> '.join(exit_frames)})"
      ),
    )
  )

  return bt.Sequence(name="4. Unload Machine Subtree", children=tasks)
