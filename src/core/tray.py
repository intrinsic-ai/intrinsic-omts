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

"""Pallet and grid tray domain model for blind infeed/outfeed."""

from dataclasses import dataclass
from typing import Optional
from src.core.types import Pose3D, SlotState
from src.core.workpiece import Workpiece


@dataclass
class TraySlot:
  """Represents a single slot in an NxM tray pallet.

  Attributes:
      row: 0-indexed row number.
      col: 0-indexed column number.
      state: Occupancy and processing state.
      workpiece: Optional workpiece currently occupying the slot.
  """

  row: int
  col: int
  state: SlotState = SlotState.EMPTY
  workpiece: Optional[Workpiece] = None


class Tray:
  """Manages an NxM grid of workpiece positions on a physical tray/table.

  Attributes:
      name: Identifier for the tray (e.g. 'infeed_tray', 'outfeed_tray').
      rows: Number of rows in grid.
      cols: Number of columns in grid.
      pitch_x: Center-to-center distance between columns along X (meters).
      pitch_y: Center-to-center distance between rows along Y (meters).
      origin_frame: Base coordinate frame name for slot offsets.
  """

  def __init__(
      self,
      name: str,
      rows: int,
      cols: int,
      pitch_x: float,
      pitch_y: float,
      origin_frame: str,
  ) -> None:
    if rows <= 0 or cols <= 0:
      raise ValueError("Tray rows and columns must be strictly positive.")
    self.name = name
    self.rows = rows
    self.cols = cols
    self.pitch_x = pitch_x
    self.pitch_y = pitch_y
    self.origin_frame = origin_frame
    self._slots: list[list[TraySlot]] = [
        [TraySlot(row=r, col=c) for c in range(cols)] for r in range(rows)
    ]

  def get_slot(self, row: int, col: int) -> TraySlot:
    """Returns the TraySlot at (row, col)."""
    if not (0 <= row < self.rows and 0 <= col < self.cols):
      raise IndexError(f"Slot ({row}, {col}) out of bounds for tray '{self.name}'.")
    return self._slots[row][col]

  def get_slot_relative_pose(self, row: int, col: int) -> Pose3D:
    """Calculates relative 3D pose of slot (row, col) from the tray origin frame."""
    if not (0 <= row < self.rows and 0 <= col < self.cols):
      raise IndexError(f"Slot ({row}, {col}) out of bounds for tray '{self.name}'.")
    return Pose3D(
        x=col * self.pitch_x,
        y=row * self.pitch_y,
        z=0.0,
    )

  def get_next_available_slot(self, target_state: SlotState = SlotState.OCCUPIED) -> Optional[TraySlot]:
    """Finds the next slot matching target_state in row-major order."""
    for r in range(self.rows):
      for c in range(self.cols):
        if self._slots[r][c].state == target_state:
          return self._slots[r][c]
    return None

  def populate_all_slots(self, part_cad_model: str = "raw_stock_2x3x5") -> None:
    """Populates all slots with new raw stock workpieces."""
    for r in range(self.rows):
      for c in range(self.cols):
        part_id = f"{self.name}_r{r}_c{c}"
        self._slots[r][c].workpiece = Workpiece(
            id=part_id,
            cad_model_name=part_cad_model,
        )
        self._slots[r][c].state = SlotState.OCCUPIED

  def count_slots_by_state(self, state: SlotState) -> int:
    """Returns the total number of slots with the given state."""
    count = 0
    for r in range(self.rows):
      for c in range(self.cols):
        if self._slots[r][c].state == state:
          count += 1
    return count
