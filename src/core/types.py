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

"""Core domain types, enumerations, and geometric primitives for OMTS."""

import enum
from dataclasses import dataclass


class InfeedMode(enum.Enum):
  """Defines how workpieces are localized at the infeed station."""

  PERCEPTION = "perception"
  GRID = "grid"


class PartState(enum.Enum):
  """Lifecycle status of an individual workpiece."""

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


class WorkpieceLocation(enum.Enum):
  """Physical or logical station holding the workpiece."""

  INFEED = "infeed"
  GRIPPER = "gripper"
  VISE = "vise"
  OUTFEED = "outfeed"
  SCRAP = "scrap"


class FixtureState(enum.Enum):
  """Clamping state of the CNC workholding fixture (vise)."""

  OPEN = "open"
  CLAMPED = "clamped"
  FAULT = "fault"


class MachineDoorState(enum.Enum):
  """State of the CNC machine enclosure access door."""

  OPEN = "open"
  CLOSED = "closed"
  MOVING = "moving"
  FAULT = "fault"


class Phase(enum.StrEnum):
  """Ordered stages of one machine tending cycle."""

  PICK = "pick"
  LOAD = "load"
  MACHINING = "machining"
  UNLOAD = "unload"
  RETURN = "return"

  @property
  def remaining(self) -> tuple["Phase", ...]:
    """Returns this phase and every subsequent phase in execution order."""
    order = tuple(Phase)
    return order[order.index(self) :]


class SimulationMode(enum.Enum):
  """Executive execution mode selector."""

  REALITY = "reality"
  PREVIEW = "preview"
  FAST_PREVIEW = "fast_preview"


@dataclass(frozen=True)
class Pose3D:
  """Represents a 6-DOF pose in 3D space (meters and unit quaternion)."""

  x: float = 0.0
  y: float = 0.0
  z: float = 0.0
  qx: float = 0.0
  qy: float = 0.0
  qz: float = 0.0
  qw: float = 1.0

  @property
  def position(self) -> tuple[float, float, float]:
    """Returns the (x, y, z) translation tuple."""
    return (self.x, self.y, self.z)

  @property
  def orientation(self) -> tuple[float, float, float, float]:
    """Returns the (qx, qy, qz, qw) quaternion tuple."""
    return (self.qx, self.qy, self.qz, self.qw)

  def to_translation_tuple(self) -> tuple[float, float, float]:
    """Returns (x, y, z) translation coordinates."""
    return (self.x, self.y, self.z)

  def to_quaternion_tuple(self) -> tuple[float, float, float, float]:
    """Returns (qx, qy, qz, qw) orientation values."""
    return (self.qx, self.qy, self.qz, self.qw)


@dataclass(frozen=True)
class JointPosition:
  """Represents a 6-DOF robot arm joint configuration in radians."""

  j1: float
  j2: float
  j3: float
  j4: float
  j5: float
  j6: float

  def __init__(
    self,
    j1: float | list[float] | tuple[float, ...] = 0.0,
    j2: float = 0.0,
    j3: float = 0.0,
    j4: float = 0.0,
    j5: float = 0.0,
    j6: float = 0.0,
  ) -> None:
    if isinstance(j1, (list, tuple)):
      if len(j1) != 6:
        raise ValueError(f"Expected 6 joint values, got {len(j1)}")
      vals = [float(v) for v in j1]
    else:
      vals = [float(j1), float(j2), float(j3), float(j4), float(j5), float(j6)]
    object.__setattr__(self, "j1", vals[0])
    object.__setattr__(self, "j2", vals[1])
    object.__setattr__(self, "j3", vals[2])
    object.__setattr__(self, "j4", vals[3])
    object.__setattr__(self, "j5", vals[4])
    object.__setattr__(self, "j6", vals[5])

  @property
  def positions(self) -> tuple[float, ...]:
    """Returns the 6 joint angles as a tuple."""
    return (self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)

  def as_list(self) -> list[float]:
    """Returns the 6 joint angles as an ordered list of floats."""
    return [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]

  def to_list(self) -> list[float]:
    """Returns the 6 joint angles as an ordered list of floats."""
    return self.as_list()

  @classmethod
  def from_sequence(
    cls, values: list[float] | tuple[float, ...]
  ) -> "JointPosition":
    """Constructs a JointPosition from a 6-element sequence of radians."""
    if len(values) != 6:
      raise ValueError(f"Expected 6 joint values, got {len(values)}")
    return cls(*[float(v) for v in values])


@dataclass(frozen=True)
class Touchdown:
  """Compliant seating parameters for a robot-workpiece contact interaction."""

  force_n: float = 8.0
  standoff_m: float = 0.010
  timeout_s: float = 40.0
  retract_after_m: float = 0.005
  direction: tuple[float, float, float] = (0.0, 0.0, 1.0)
