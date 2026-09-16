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

from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  create_move_to_frame_task,
)
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_machining_handshake_subtree(
  robot: RobotInterface,
  machine: CncMachineInterface,
  parent_object: str = "root",
  standby_frame_name: str = "machine_approach",
  machining_timeout_seconds: float = 30.0,
  solution: Any | None = None,
  name: str = "4. Machining Handshake Subtree",
) -> bt.Node:
  """Builds the retract, close door, run cycle, open door handshake."""
  tasks: list[bt.Node] = [
    create_move_to_frame_task(
      robot,
      standby_frame_name,
      parent_object,
      motion_type="LINEAR",
      max_tries=2,
      retry_delay_sec=1.0,
      solution=solution,
      task_name=(
        f"Step 4a: Retract to Standby ({parent_object}/{standby_frame_name})"
      ),
    ),
    machine.build_close_door_task(name="Step 4b: Close CNC Door"),
    machine.build_trigger_cycle_task(name="Step 4c: Trigger CNC Cycle Start"),
    machine.build_wait_cycle_complete_task(
      timeout_seconds=machining_timeout_seconds,
      name="Step 4d: Wait for CNC Cycle Complete",
    ),
    machine.build_open_door_task(name="Step 4e: Open CNC Door"),
  ]
  return bt.Sequence(name=name, children=tasks)
