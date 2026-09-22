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

from src.core.config import AppConfig
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_machining_handshake_subtree(
  robot: RobotInterface,
  machine: CncMachineInterface | None,
  config: AppConfig,
) -> bt.Node:
  """Builds the Behavior Tree subtree for executing the CNC machining cycle.

  Sequence:
  1. If `machine` is provided (with the arm already positioned at
     `machine_approach_frame` after `load_machine`):
     a. Close CNC enclosure door.
     b. Pulse CNC cycle start digital output.
     c. Wait for CNC cycle completion signal or timeout.

  Args:
      robot: Robot controller adapter.
      machine: Optional CNC machine controller adapter (`None` when cell has no
        CNC enclosure/vise).
      config: Validated application configuration dataclass.

  Returns:
      Behavior tree sequence node executing machining cycle handshake.
  """
  machining_timeout_seconds = config.cycle.machining_timeout_seconds

  tasks: list[bt.Node] = []
  if machine is not None:
    tasks.extend(
      [
        machine.build_close_door_task(name="Close CNC Door"),
        machine.build_trigger_cycle_task(name="Trigger CNC Cycle Start"),
        machine.build_wait_cycle_complete_task(
          timeout_seconds=machining_timeout_seconds,
          name="Wait for CNC Cycle Complete",
        ),
      ]
    )

  return bt.Sequence(name="3. Machining Handshake Subtree", children=tasks)
