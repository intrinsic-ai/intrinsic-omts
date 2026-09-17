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

"""Mapping OMTS options to Intrinsic executive settings and world refs."""

from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import execution
from intrinsic.world.proto import object_world_refs_pb2

from src.core.types import SimulationMode

__all__ = [
  "create_transform_node_ref",
  "describe_motion_types",
  "normalize_motion_types",
  "to_executive_simulation_mode",
]

_SIMULATION_MODE_MAP: dict[
  SimulationMode, execution.Executive.SimulationMode
] = {
  SimulationMode.REALITY: execution.Executive.SimulationMode.REALITY,
  SimulationMode.PREVIEW: execution.Executive.SimulationMode.PREVIEW,
  SimulationMode.FAST_PREVIEW: execution.Executive.SimulationMode.FAST_PREVIEW,
}


def to_executive_simulation_mode(
  mode: SimulationMode | None,
) -> execution.Executive.SimulationMode | None:
  """Converts an OMTS simulation mode to the executive SDK enum."""
  if mode is None:
    return None

  try:
    return _SIMULATION_MODE_MAP[mode]
  except KeyError as err:
    raise ValueError(f"Unsupported simulation mode: {mode}") from err


def normalize_motion_types(
  motion_type: str | Sequence[str], num_segments: int
) -> list[str]:
  """Expands a motion type into one entry per trajectory segment."""
  if isinstance(motion_type, str):
    return [motion_type] * num_segments
  types = list(motion_type)
  if len(types) != num_segments:
    raise ValueError(
      f"motion_type has {len(types)} entries but trajectory has"
      f" {num_segments} segments."
    )
  return types


def describe_motion_types(motion_types: Sequence[str]) -> str:
  """Renders motion types for a task name, collapsing a uniform trajectory."""
  if len(set(motion_types)) == 1:
    return motion_types[0]
  return "/".join(motion_types)


def create_transform_node_ref(object_name: str, frame_name: str) -> Any:
  """Builds a TransformNodeReference proto by object and frame name."""
  return object_world_refs_pb2.TransformNodeReference(
    by_name=object_world_refs_pb2.TransformNodeReferenceByName(
      frame=object_world_refs_pb2.FrameReferenceByName(
        object_name=object_name, frame_name=frame_name
      )
    )
  )
