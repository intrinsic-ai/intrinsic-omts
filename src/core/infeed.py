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

"""Infeed strategy definitions for vision vs. blind grid acquisition."""

import abc
from collections.abc import Sequence

from src.core.tray import Tray, TraySlot
from src.core.types import InfeedMode
from src.core.workpiece import Workpiece

__all__ = [
  "GridInfeedStrategy",
  "InfeedMode",
  "InfeedStrategy",
  "PerceptionInfeedStrategy",
]


class InfeedStrategy(abc.ABC):
  """Abstract base class defining how parts are localized and acquired."""

  @abc.abstractmethod
  def get_target_part(self) -> Workpiece | None:
    """Retrieves or instantiates the next workpiece to process."""
    raise NotImplementedError


class PerceptionInfeedStrategy(InfeedStrategy):
  """Vision-guided infeed strategy for randomly placed parts."""

  mode: InfeedMode = InfeedMode.PERCEPTION

  def __init__(
    self,
    camera_name: str = "orbbec_camera",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
    sensor_ids: Sequence[int] = (1, 4),
    min_num_instances: int = 1,
    refinement_iters: int = 3,
    view_frame_name: str = "view",
  ) -> None:
    self.mode = InfeedMode.PERCEPTION
    self.camera_name = camera_name
    self.pose_estimator_id = pose_estimator_id
    self.scene_object_id = scene_object_id
    self.sensor_ids = list(sensor_ids)
    self.min_num_instances = min_num_instances
    self.refinement_iters = refinement_iters
    self.view_frame_name = view_frame_name
    self._current_part_count = 0

  def get_target_part(self) -> Workpiece | None:
    """Instantiates a new workpiece representation for vision acquisition."""
    self._current_part_count += 1
    short_name = self.scene_object_id.split(".")[-1]
    return Workpiece(
      asset_id=self.scene_object_id,
      object_name=short_name,
    )


class GridInfeedStrategy(InfeedStrategy):
  """Deterministic tray grid infeed strategy for blind acquisition."""

  mode: InfeedMode = InfeedMode.GRID

  def __init__(self, tray: Tray) -> None:
    self.mode = InfeedMode.GRID
    self.tray = tray

  def get_target_part(self) -> Workpiece | None:
    """Retrieves the next occupied slot's workpiece from the tray."""
    slot = self.tray.get_next_available_slot()
    if slot is not None:
      return slot.workpiece
    return None

  def get_next_slot(self) -> TraySlot | None:
    """Returns the next slot to pick from."""
    return self.tray.get_next_available_slot()
