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

"""Cell configuration dataclasses and strict YAML loader for OMTS workcells."""

import dataclasses
import pathlib
from typing import Any, TypeVar

import yaml

_T = TypeVar("_T")


@dataclasses.dataclass(frozen=True)
class RobotConfig:
  """Configuration for the robot arm and moving tool frame."""

  arm_part_name: str
  tool_object_name: str
  tool_frame_name: str


@dataclasses.dataclass(frozen=True)
class GripperConfig:
  """Configuration for the end-effector gripper adapter."""

  type: str
  joint_name: str | None = None
  open_position: float | None = None
  close_position: float | None = None
  action_name: str | None = None
  dio_open_pin: int | None = None
  dio_close_pin: int | None = None
  dio_device_name: str | None = None
  dio_output_block_name: str | None = None

  def __post_init__(self) -> None:
    if self.type == "robotiq":
      required = ("joint_name", "open_position", "close_position")
    elif self.type == "dio":
      required = ("dio_open_pin", "dio_close_pin", "dio_output_block_name")
    else:
      return
    missing = [name for name in required if getattr(self, name) is None]
    if missing:
      raise KeyError(
        f"Missing required configuration field(s) {sorted(missing)} for "
        f"gripper type '{self.type}'"
      )


@dataclasses.dataclass(frozen=True)
class MachineConfig:
  """Configuration for CNC enclosure door, vise, and cycle handshake DIO."""

  door_open_pin: int
  door_close_pin: int
  vise_open_pin: int
  vise_close_pin: int
  cycle_start_pin: int
  cycle_complete_input_pin: int | None
  device_name: str
  enclosure_object_name: str | None
  vise_object_name: str | None
  door_open_joints: tuple[float, ...]
  door_closed_joints: tuple[float, ...]
  vise_open_joints: tuple[float, ...]
  vise_closed_joints: tuple[float, ...]
  output_block_name: str
  input_block_name: str


@dataclasses.dataclass(frozen=True)
class VisionConfig:
  """Configuration for 3D perception and pose estimation."""

  camera_name: str
  perception_service_name: str
  pose_estimator_id: str
  scene_object_id: str
  sensor_ids: tuple[int, ...]
  min_num_instances: int
  infeed_mode: str
  min_safe_z: float


@dataclasses.dataclass(frozen=True)
class FramesConfig:
  """World transform frame names used across machine tending motions."""

  parent_object: str
  view_frame: str
  pregrasp_frame: str
  grasp_frame: str
  machine_approach_frame: str
  preplace_vise_frame: str
  place_vise_frame: str
  transit_frame: str | None = None


@dataclasses.dataclass(frozen=True)
class CycleConfig:
  """Execution, force, and motion parameters for the machine tending cycle."""

  num_cycles: int
  workpiece_id: str
  approach_offset_z: float
  retract_distance_meters: float
  pick_touchdown_force_newtons: float
  load_seat_force_newtons: float
  unload_touchdown_force_newtons: float
  return_touchdown_force_newtons: float
  touchdown_timeout_seconds: float
  machining_timeout_seconds: float


@dataclasses.dataclass(frozen=True)
class AppConfig:
  """Top-level configuration for a machine tending cell deployment."""

  cell_name: str
  robot: RobotConfig
  gripper: GripperConfig
  vision: VisionConfig
  frames: FramesConfig
  cycle: CycleConfig
  machine: MachineConfig | None = None


def _construct_section(
  cls: type[_T],
  raw_data: dict[str, Any],
  section_name: str,
  file_path: pathlib.Path,
) -> _T:
  """Validates that all fields of `cls` exist in `raw_data[section_name]` and constructs it."""
  if section_name not in raw_data or not isinstance(
    raw_data[section_name], dict
  ):
    raise KeyError(
      f"Missing required configuration section '{section_name}' in {file_path}"
    )

  section_dict = dict(raw_data[section_name])
  if section_name == "gripper":
    gripper_type = section_dict.get("type")
    if gripper_type == "robotiq":
      required_fields = {
        "type",
        "joint_name",
        "open_position",
        "close_position",
      }
    elif gripper_type == "dio":
      required_fields = {
        "type",
        "dio_open_pin",
        "dio_close_pin",
        "dio_output_block_name",
      }
    else:
      required_fields = {"type"}
  else:
    required_fields = {
      f.name
      for f in dataclasses.fields(cls)  # type: ignore[arg-type]
      if f.default is dataclasses.MISSING
      and f.default_factory is dataclasses.MISSING
    }
  missing_fields = required_fields - set(section_dict.keys())
  if missing_fields:
    sorted_missing = sorted(missing_fields)
    raise KeyError(
      f"Missing required configuration field(s) {sorted_missing} in section "
      f"'{section_name}' of {file_path}"
    )

  if section_name == "vision" and isinstance(
    section_dict.get("sensor_ids"), list
  ):
    section_dict["sensor_ids"] = tuple(
      int(x) for x in section_dict["sensor_ids"]
    )
  elif section_name == "machine":
    for joint_key in (
      "door_open_joints",
      "door_closed_joints",
      "vise_open_joints",
      "vise_closed_joints",
    ):
      if isinstance(section_dict.get(joint_key), list):
        section_dict[joint_key] = tuple(
          float(x) for x in section_dict[joint_key]
        )

  return cls(**section_dict)


def load_app_config(path: str | pathlib.Path) -> AppConfig:
  """Loads and strictly validates an AppConfig from a YAML or JSON configuration file.

  Fails loudly with KeyError if any required section or field is omitted.

  Args:
      path: File path to the YAML configuration file.

  Returns:
      Populated AppConfig dataclass instance.
  """
  file_path = pathlib.Path(path)
  content = file_path.read_text(encoding="utf-8")
  raw_data = yaml.safe_load(content)

  if not isinstance(raw_data, dict):
    raise ValueError(
      f"Configuration file {file_path} must contain a top-level mapping."
    )
  if "cell_name" not in raw_data or not raw_data["cell_name"]:
    raise KeyError(f"Missing required field 'cell_name' in {file_path}")

  machine_config = (
    _construct_section(MachineConfig, raw_data, "machine", file_path)
    if "machine" in raw_data and raw_data["machine"] is not None
    else None
  )

  return AppConfig(
    cell_name=str(raw_data["cell_name"]),
    robot=_construct_section(RobotConfig, raw_data, "robot", file_path),
    gripper=_construct_section(GripperConfig, raw_data, "gripper", file_path),
    machine=machine_config,
    vision=_construct_section(VisionConfig, raw_data, "vision", file_path),
    frames=_construct_section(FramesConfig, raw_data, "frames", file_path),
    cycle=_construct_section(CycleConfig, raw_data, "cycle", file_path),
  )
