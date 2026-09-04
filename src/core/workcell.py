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

"""Workcell state tracking and process metrics."""

from dataclasses import dataclass, field
import time
from typing import Optional
from src.core.tray import Tray
from src.core.types import FixtureState, MachineDoorState
from src.core.workpiece import Workpiece


@dataclass
class WorkcellState:
  """Maintains state and runtime metrics across the machine tending cell.

  Attributes:
      current_workpiece: Workpiece currently gripped by the robot.
      workpiece_in_machine: Workpiece currently mounted in the CNC vise.
      infeed_tray: Optional grid tray instance if infeed is grid-based.
      door_state: Current state of CNC door.
      vise_state: Current state of CNC vise.
      cycles_completed: Total successful machine tending cycles.
      cycles_failed: Total aborted/failed machine tending cycles.
      cycle_start_timestamp: Unix timestamp when current cycle started.
  """

  current_workpiece: Optional[Workpiece] = None
  workpiece_in_machine: Optional[Workpiece] = None
  infeed_tray: Optional[Tray] = None
  door_state: MachineDoorState = MachineDoorState.CLOSED
  vise_state: FixtureState = FixtureState.OPEN
  cycles_completed: int = 0
  cycles_failed: int = 0
  cycle_start_timestamp: Optional[float] = None

  def start_new_cycle(self, workpiece: Workpiece) -> None:
    """Begins a new tending cycle with the given workpiece."""
    self.current_workpiece = workpiece
    self.cycle_start_timestamp = time.time()

  def record_cycle_success(self) -> float:
    """Marks cycle as successful and returns duration in seconds."""
    self.cycles_completed += 1
    duration = 0.0
    if self.cycle_start_timestamp is not None:
      duration = time.time() - self.cycle_start_timestamp
    self.current_workpiece = None
    self.cycle_start_timestamp = None
    return duration

  def record_cycle_failure(self) -> None:
    """Marks cycle as failed."""
    self.cycles_failed += 1
    self.current_workpiece = None
    self.cycle_start_timestamp = None
