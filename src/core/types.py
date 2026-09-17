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
from typing import Any


class Phase(enum.StrEnum):
  """Ordered stages of one machine tending cycle."""

  PICK = "pick"
  LOAD = "load"
  MACHINING = "machining"
  UNLOAD = "unload"
  RETURN = "return"

  @property
  def remaining(self) -> tuple["Phase", ...]:
    """Returns this phase and every phase after it, in execution order."""
    order = tuple(Phase)
    return order[order.index(self) :]


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


class SimulationMode(enum.Enum):
  """Execution mode requested from the Flowstate executive.

  Mirrors `intrinsic_proto.executive.SimulationMode` without importing the
  Intrinsic SDK, so that `src.core` stays dependency free.

  Attributes:
      REALITY: Full physics. Executes on real hardware, or in the simulator as
        close to reality as possible.
      PREVIEW: Executes skills in preview mode and visualizes world updates.
      FAST_PREVIEW: Executes skills in preview mode without visualization.
  """

  REALITY = "reality"
  PREVIEW = "preview"
  FAST_PREVIEW = "fast_preview"


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


class InfeedMode(enum.StrEnum):
  """Acquisition mode for raw workpieces at the cell infeed."""

  PERCEPTION = "perception"
  GRID = "grid"


class GripperState(enum.Enum):
  """Commanded aperture of the end effector."""

  OPEN = "open"
  CLOSED = "closed"
  UNKNOWN = "unknown"


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

  @property
  def position(self) -> tuple[float, float, float]:
    """Returns (x, y, z) translation tuple."""
    return (self.x, self.y, self.z)

  @property
  def orientation(self) -> tuple[float, float, float, float]:
    """Returns (qx, qy, qz, qw) quaternion tuple."""
    return (self.qx, self.qy, self.qz, self.qw)

  def to_translation_tuple(self) -> tuple[float, float, float]:
    """Returns (x, y, z) translation coordinates."""
    return (self.x, self.y, self.z)

  def to_quaternion_tuple(self) -> tuple[float, float, float, float]:
    """Returns (qx, qy, qz, qw) orientation values."""
    return (self.qx, self.qy, self.qz, self.qw)

  def to_proto(self) -> Any:
    """Converts this Pose3D into an intrinsic_proto.Pose protobuf message."""
    from intrinsic.math.proto import point_pb2, pose_pb2, quaternion_pb2

    return pose_pb2.Pose(
      position=point_pb2.Point(x=self.x, y=self.y, z=self.z),
      orientation=quaternion_pb2.Quaternion(
        x=self.qx, y=self.qy, z=self.qz, w=self.qw
      ),
    )

  @classmethod
  def from_proto(cls, proto: Any) -> "Pose3D":
    """Constructs a Pose3D from a Pose proto or data_types.Pose3 object."""
    pos = getattr(proto, "position", None)
    if pos is None:
      pos = getattr(proto, "translation", None)
    ori = getattr(proto, "orientation", None)
    if ori is None:
      rot = getattr(proto, "rotation", None)
      ori = getattr(rot, "quaternion", None)
    if pos is not None and ori is not None:
      x = float(pos.x) if hasattr(pos, "x") else float(pos[0])
      y = float(pos.y) if hasattr(pos, "y") else float(pos[1])
      z = float(pos.z) if hasattr(pos, "z") else float(pos[2])
      return cls(
        x=x,
        y=y,
        z=z,
        qx=float(ori.x),
        qy=float(ori.y),
        qz=float(ori.z),
        qw=float(ori.w),
      )
    raise TypeError(f"Unsupported pose representation: {type(proto)}")


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


@dataclass(frozen=True)
class Frames:
  """World frame names the tending cycle drives to."""

  root: str = "root"
  view: str = "view"
  transit: str = "transit"
  machine_approach: str = "machine_approach"
  infeed_pre_grasp: str = "infeed_pre_grasp"
  infeed_grasp: str = "infeed_grasp"
  vise_pre_place: str = "vise_pre_place"
  vise_place: str = "vise_place"


DEFAULT_FRAMES = Frames()
