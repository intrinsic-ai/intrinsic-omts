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

"""CNC machining process cycle handshake subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import create_move_to_frame_task
from src.core.config import AppConfig
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_machining_handshake_subtree(
  robot: RobotInterface,
  machine: CncMachineInterface | None,
  config: AppConfig,
  include_entry_guard: bool = False,
) -> bt.Node:
  """Builds the Behavior Tree subtree for the CNC machining handshake.

  Sequence:
      1. Optional mid-cycle entry guard (`include_entry_guard=True`): move arm
         to `machine_approach_frame` if entering directly at `Phase.MACHINING`.
      2. Close CNC enclosure door (arm is already at `machine_approach_frame`
         after `load_machine`).
      3. Trigger CNC cycle start signal (`pulse_high -> 0.5s dwell ->
         pulse_low`).
      4. Wait for CNC machining cycle completion (`machining_timeout_seconds`).

  Args:
      robot: Robot hardware interface.
      machine: Optional CNC machine hardware interface.
      config: Application configuration.
      include_entry_guard: Whether to prepend a safety move to
        `machine_approach_frame` for mid-cycle entry.

  Returns:
      A `bt.Sequence` node executing the machining handshake phase.
  """
  steps: list[bt.Node] = []
  if include_entry_guard:
    steps.append(
      create_move_to_frame_task(
        robot=robot,
        frame_name=config.frames.machine_approach_frame,
        config=config,
        motion_type="ANY",
        task_name="Step 09a: Move to Machine Approach (Entry Guard)",
      )
    )
  if machine is not None:
    steps.extend([
      machine.build_close_door_task(name="Step 09a: Close CNC Door"),
      machine.build_trigger_cycle_task(name="Step 09b: Trigger CNC Cycle"),
      machine.build_wait_cycle_complete_task(
        timeout_seconds=config.cycle.machining_timeout_seconds,
        name="Step 09c: Wait for Machining Complete",
      ),
    ])
  return bt.Sequence(name="3. Machining Handshake Subtree", children=steps)
