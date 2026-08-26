"""Infeed strategy definitions for vision vs. blind grid acquisition."""

import abc
from typing import Optional
from src.core.tray import Tray, TraySlot
from src.core.types import InfeedMode, Pose3D
from src.core.workpiece import Workpiece


class InfeedStrategy(abc.ABC):
  """Abstract base class defining how parts are localized and acquired."""

  @property
  @abc.abstractmethod
  def mode(self) -> InfeedMode:
    """Returns the infeed operational mode."""
    raise NotImplementedError

  @abc.abstractmethod
  def get_target_part(self) -> Optional[Workpiece]:
    """Retrieves or instantiates the next workpiece to process."""
    raise NotImplementedError


class PerceptionInfeedStrategy(InfeedStrategy):
  """Vision-guided infeed strategy for randomly placed parts."""

  def __init__(
      self,
      camera_name: str = "orbbec_camera",
      estimator_name: str = "raw_stock_2x3x5_estimator",
      view_frame_name: str = "view",
  ) -> None:
    self.camera_name = camera_name
    self.estimator_name = estimator_name
    self.view_frame_name = view_frame_name
    self._current_part_count = 0

  @property
  def mode(self) -> InfeedMode:
    return InfeedMode.PERCEPTION

  def get_target_part(self) -> Optional[Workpiece]:
    """Instantiates a new workpiece representation for vision acquisition."""
    self._current_part_count += 1
    return Workpiece(id=f"vision_raw_stock_{self._current_part_count:03d}")


class GridInfeedStrategy(InfeedStrategy):
  """Deterministic tray grid infeed strategy for blind acquisition."""

  def __init__(self, tray: Tray) -> None:
    self.tray = tray

  @property
  def mode(self) -> InfeedMode:
    return InfeedMode.GRID

  def get_target_part(self) -> Optional[Workpiece]:
    """Retrieves the next occupied slot's workpiece from the tray."""
    slot = self.tray.get_next_available_slot()
    if slot is not None:
      return slot.workpiece
    return None

  def get_next_slot(self) -> Optional[TraySlot]:
    """Returns the next slot to pick from."""
    return self.tray.get_next_available_slot()
