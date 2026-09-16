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

"""Workpiece domain model for machine tending."""

from dataclasses import dataclass

from src.core.types import PartState, Pose3D


@dataclass
class Workpiece:
  """Represents an individual physical workpiece being processed.

  Attributes:
      asset_id: Asset or package identifier (e.g. 'ai.intrinsic.raw_stock_2x3x5').
      object_name: Scene object name in ObjectWorld (e.g. 'raw_stock_2x3x5').
      object_id: Optional integer entity ID in ObjectWorld.
      state: Lifecycle status in the manufacturing flow.
      initial_infeed_pose: Recorded pose on infeed table if detected by vision.
  """

  id: str = "raw_stock_01"
  cad_model_name: str = "raw_stock_2x3x5"
  asset_id: str = "ai.intrinsic.raw_stock_2x3x5"
  object_name: str = "raw_stock_2x3x5"
  object_id: int | None = None
  state: PartState = PartState.RAW
  initial_infeed_pose: Pose3D | None = None

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
