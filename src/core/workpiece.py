"""Workpiece domain model for machine tending."""

from dataclasses import dataclass
from typing import Optional
from src.core.types import PartState, Pose3D


@dataclass
class Workpiece:
  """Represents an individual physical workpiece being processed.

  Attributes:
      id: Unique identifier for the part instance (e.g. 'raw_stock_01').
      cad_model_name: Name of the 3D model asset (e.g. 'raw_stock_2x3x5').
      state: Lifecycle status in the manufacturing flow.
      grasp_offset: Relative offset from part origin to robot grasp point.
      insertion_depth: Distance to seat part into CNC vise (meters).
      initial_infeed_pose: Recorded pose on infeed table if detected by vision.
  """

  id: str
  cad_model_name: str = "raw_stock_2x3x5"
  state: PartState = PartState.RAW
  grasp_offset: Pose3D = Pose3D(x=0.0, y=0.0, z=0.05)
  insertion_depth: float = 0.03
  initial_infeed_pose: Optional[Pose3D] = None

  def mark_detected(self, detected_pose: Pose3D) -> None:
    """Updates part state to DETECTED and records its initial location."""
    self.state = PartState.DETECTED
    self.initial_infeed_pose = detected_pose

  def mark_picked(self) -> None:
    """Updates part state to IN_TRANSIT."""
    self.state = PartState.IN_TRANSIT

  def mark_loaded(self) -> None:
    """Updates part state to IN_MACHINE."""
    self.state = PartState.IN_MACHINE

  def mark_machined(self) -> None:
    """Updates part state to MACHINED."""
    self.state = PartState.MACHINED

  def mark_finished(self) -> None:
    """Updates part state to INSPECTED_OK."""
    self.state = PartState.INSPECTED_OK

  def mark_rejected(self) -> None:
    """Updates part state to REJECTED."""
    self.state = PartState.REJECTED
