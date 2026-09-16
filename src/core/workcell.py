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

import logging
import time
from dataclasses import dataclass

from src.core.tray import Tray
from src.core.types import Phase, WorkpieceLocation
from src.core.workpiece import Workpiece


@dataclass
class WorkcellState:
  """Tracks cycle progress and workpiece custody across the tending cell.

  Attributes:
      total_cycles: Number of cycles to execute, or <= 0 for unbounded.
      phase: Initial or current execution phase of the tending cycle.
      current_workpiece: Workpiece currently handled in the cycle.
      infeed_tray: Optional grid tray instance if infeed is grid-based.
      workpiece_location: Current logical location of active workpiece.
      workpiece_parent: Name of world object parenting the workpiece ('root',
        'gripper', or vise object name).
      cycles_completed: Total successful machine tending cycles.
      cycle_start_timestamp: Unix timestamp when current cycle started.
  """

  total_cycles: int = 1
  phase: Phase = Phase.PICK
  current_workpiece: Workpiece | None = None
  infeed_tray: Tray | None = None
  workpiece_location: WorkpieceLocation = WorkpieceLocation.INFEED
  workpiece_parent: str = "root"
  cycles_completed: int = 0
  cycle_start_timestamp: float | None = None

  @classmethod
  def create(
    cls,
    total_cycles: int = 1,
    phase: Phase | str = Phase.PICK,
    current_workpiece: Workpiece | None = None,
    infeed_tray: Tray | None = None,
  ) -> "WorkcellState":
    """Constructs a WorkcellState, clamping total_cycles to 1 if resuming mid-cycle."""
    resolved_phase = Phase(phase) if not isinstance(phase, Phase) else phase
    resolved_cycles = total_cycles
    if resolved_phase is not Phase.PICK and resolved_cycles != 1:
      logging.warning(
        "Resuming from non-infeed phase '%s'. Clamping total_cycles from %d to 1"
        " for safety.",
        resolved_phase,
        resolved_cycles,
      )
      resolved_cycles = 1
    return cls(
      total_cycles=resolved_cycles,
      phase=resolved_phase,
      current_workpiece=current_workpiece,
      infeed_tray=infeed_tray,
    )

  @property
  def cycles_remaining(self) -> int | None:
    """Returns cycles left to run, or None if unbounded."""
    if self.total_cycles <= 0:
      return None
    return max(0, self.total_cycles - self.cycles_completed)

  @property
  def has_work_remaining(self) -> bool:
    return self.cycles_remaining is None or self.cycles_remaining > 0

  def start_new_cycle(self, workpiece: Workpiece) -> None:
    """Begins a new tending cycle with the given workpiece."""
    self.current_workpiece = workpiece
    self.workpiece_location = WorkpieceLocation.INFEED
    self.workpiece_parent = "root"
    self.cycle_start_timestamp = time.time()

  def record_infeed_pick(
    self,
    workpiece: Workpiece | None = None,
    tool_name: str = "gripper",
  ) -> None:
    """Records that the workpiece has been grasped and parented to the gripper."""
    if workpiece is not None:
      self.current_workpiece = workpiece
    self.workpiece_location = WorkpieceLocation.GRIPPER
    self.workpiece_parent = tool_name

  def record_machine_load(self, vise_name: str = "schunk_egp_64nnb") -> None:
    """Records that the workpiece has been clamped into the vise and detached from gripper."""
    self.workpiece_location = WorkpieceLocation.VISE
    self.workpiece_parent = vise_name

  def record_machine_unload(self, tool_name: str = "gripper") -> None:
    """Records that the workpiece has been re-grasped by gripper and unclamped from vise."""
    self.workpiece_location = WorkpieceLocation.GRIPPER
    self.workpiece_parent = tool_name

  def record_infeed_return(self, parent_name: str = "root") -> None:
    """Records that the workpiece has been returned to the infeed and detached to root."""
    self.workpiece_location = WorkpieceLocation.RETURNED
    self.workpiece_parent = parent_name
    self.current_workpiece = None

  def record_cycle_success(self) -> float:
    """Marks cycle as successful and returns duration in seconds."""
    self.cycles_completed += 1
    duration = 0.0
    if self.cycle_start_timestamp is not None:
      duration = time.time() - self.cycle_start_timestamp
    self.record_infeed_return()
    self.cycle_start_timestamp = None
    return duration
