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

from src.core.config import VisionConfig
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

  @property
  @abc.abstractmethod
  def mode(self) -> InfeedMode:
    """Returns the infeed operational mode."""
    raise NotImplementedError

  @abc.abstractmethod
  def get_target_part(self) -> Workpiece | None:
    """Retrieves or instantiates the next workpiece to process."""
    raise NotImplementedError


class PerceptionInfeedStrategy(InfeedStrategy):
  """Vision-guided infeed strategy for randomly placed parts."""

  def __init__(
    self,
    config: VisionConfig,
    view_frame_name: str,
    workpiece: Workpiece | None = None,
    refinement_iters: int = 3,
  ) -> None:
    self.camera_name = config.camera_name
    self.pose_estimator_id = config.pose_estimator_id
    self.scene_object_id = config.scene_object_id
    self.sensor_ids = list(config.sensor_ids)
    self.min_num_instances = config.min_num_instances
    self.refinement_iters = refinement_iters
    self.view_frame_name = view_frame_name
    self._template_workpiece = workpiece or Workpiece(id="raw_stock_2x3x5")
    self._current_part_count = 0

  @property
  def mode(self) -> InfeedMode:
    return InfeedMode.PERCEPTION

  def get_target_part(self) -> Workpiece | None:
    """Instantiates a new workpiece representation for vision acquisition."""
    self._current_part_count += 1
    return Workpiece(
      id=f"workpiece_{self._current_part_count - 1}",
      cad_model_name=self._template_workpiece.cad_model_name,
      object_name=self._template_workpiece.object_name,
      grasp_offset=self._template_workpiece.grasp_offset,
    )


class GridInfeedStrategy(InfeedStrategy):
  """Deterministic tray grid infeed strategy for blind acquisition."""

  def __init__(self, tray: Tray) -> None:
    self.tray = tray

  @property
  def mode(self) -> InfeedMode:
    return InfeedMode.GRID

  def get_target_part(self) -> Workpiece | None:
    """Retrieves the next occupied slot's workpiece from the tray."""
    slot = self.tray.get_next_available_slot()
    if slot is not None:
      return slot.workpiece
    return None

  def get_next_slot(self) -> TraySlot | None:
    """Returns the next slot to pick from."""
    return self.tray.get_next_available_slot()
