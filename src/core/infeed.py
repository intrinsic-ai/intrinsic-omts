"""Infeed strategy definitions for vision vs. blind grid acquisition."""

import abc
from typing import Optional, Sequence
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
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
      sensor_ids: Sequence[int] = (1, 4),
      min_num_instances: int = 1,
      refinement_iters: int = 3,
      view_frame_name: str = "view",
  ) -> None:
    self.camera_name = camera_name
    self.pose_estimator_id = pose_estimator_id
    self.scene_object_id = scene_object_id
    self.sensor_ids = list(sensor_ids)
    self.min_num_instances = min_num_instances
    self.refinement_iters = refinement_iters
    self.view_frame_name = view_frame_name
    self._current_part_count = 0

  @property
  def mode(self) -> InfeedMode:
    return InfeedMode.PERCEPTION

  def get_target_part(self) -> Optional[Workpiece]:
    """Instantiates a new workpiece representation for vision acquisition."""
    self._current_part_count += 1
    return Workpiece(id=f"workpiece_{self._current_part_count - 1}")


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
