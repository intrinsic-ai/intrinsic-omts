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

"""Core data types, enums, and geometric primitives for OMTS."""

import enum
from dataclasses import dataclass


class PartState(enum.Enum):
  """State lifecycle of a workpiece in the machine tending cell."""

  RAW = "raw"
  DETECTED = "detected"
  IN_TRANSIT = "in_transit"
  IN_MACHINE = "in_machine"
  MACHINED = "machined"
  INSPECTED_OK = "inspected_ok"
  REJECTED = "rejected"


class SlotState(enum.Enum):
  """Occupancy status of a tray or pallet slot."""

  EMPTY = "empty"
  OCCUPIED = "occupied"
  RESERVED = "reserved"
  PROCESSED = "processed"
  FAULT = "fault"


class InfeedMode(enum.Enum):
  """Infeed part localization and acquisition strategy."""

  PERCEPTION = "perception"
  GRID = "grid"


class FixtureState(enum.Enum):
  """Status of the CNC machine vise or chuck clamping mechanism."""

  OPEN = "open"
  CLAMPED = "clamped"
  ERROR = "error"


class MachineDoorState(enum.Enum):
  """Status of the CNC enclosure safety door."""

  OPEN = "open"
  CLOSED = "closed"
  MOVING = "moving"
  ERROR = "error"


@dataclass(frozen=True)
class Pose3D:
  """Represents a 3D Cartesian position and orientation quaternion.

  Attributes:
      x: Translation along X-axis (meters).
      y: Translation along Y-axis (meters).
      z: Translation along Z-axis (meters).
      qx: Quaternion X component.
      qy: Quaternion Y component.
      qz: Quaternion Z component.
      qw: Quaternion W component (scalar).
  """

  x: float = 0.0
  y: float = 0.0
  z: float = 0.0
  qx: float = 0.0
  qy: float = 0.0
  qz: float = 0.0
  qw: float = 1.0

  def to_translation_tuple(self) -> tuple[float, float, float]:
    """Returns (x, y, z) translation coordinates."""
    return (self.x, self.y, self.z)

  def to_quaternion_tuple(self) -> tuple[float, float, float, float]:
    """Returns (qx, qy, qz, qw) orientation values."""
    return (self.qx, self.qy, self.qz, self.qw)


@dataclass(frozen=True)
class JointPosition:
  """Represents an N-DoF robot joint configuration.

  Attributes:
      positions: Joint angles in radians.
  """

  positions: tuple[float, ...]

  def to_list(self) -> list[float]:
    """Returns joint angles as a list."""
    return list(self.positions)
