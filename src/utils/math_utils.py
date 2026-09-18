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

"""Math, geometric transform, and motion utility functions for OMTS."""

import math
from collections.abc import Sequence
from typing import Any


def normalize_angle(angle: float) -> float:
  """Normalizes an angle to [-pi, pi] radians."""
  return (angle + math.pi) % (2.0 * math.pi) - math.pi


def normalize_joint_angles(joint_angles: Sequence[float]) -> list[float]:
  """Normalizes a sequence of joint angles to [-pi, pi] radians."""
  return [normalize_angle(angle) for angle in joint_angles]


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


def object_exists_in_world(solution: Any, object_name: str | None) -> bool:
  """Checks whether the named object exists in the solution world."""
  if not object_name:
    return False
  world = getattr(solution, "world", None)
  if world is None:
    return False
  list_object_names = getattr(world, "list_object_names", None)
  if callable(list_object_names):
    names = list_object_names()
    if isinstance(names, (list, tuple, set)):
      return object_name in names
  return hasattr(world, object_name)


def resolve_adio_resource(solution: Any, device_name: str | None) -> Any | None:
  """Resolves an optional ADIO device resource handle if present and compatible."""
  if not device_name:
    return None
  resources = getattr(solution, "resources", None)
  if resources is None:
    return None
  handle: Any = None
  if isinstance(resources, dict):
    handle = resources.get(device_name)
  else:
    try:
      handle = resources[device_name]
    except (KeyError, TypeError):
      try:
        handle = getattr(resources, device_name)
      except (KeyError, AttributeError):
        handle = None
  if handle is None:
    return None
  resource_types = getattr(handle, "types", None)
  if (
    isinstance(resource_types, (list, tuple, set))
    and "Icon2AdioPart" not in resource_types
  ):
    return None
  return handle


def create_transform_node_ref(
  object_name: str, frame_name: str | None = None
) -> Any:
  """Builds a TransformNodeReference proto by object and optional frame name."""
  from intrinsic.world.proto import object_world_refs_pb2

  if frame_name:
    return object_world_refs_pb2.TransformNodeReference(
      by_name=object_world_refs_pb2.TransformNodeReferenceByName(
        frame=object_world_refs_pb2.FrameReferenceByName(
          object_name=object_name, frame_name=frame_name
        )
      )
    )
  return object_world_refs_pb2.TransformNodeReference(
    by_name=object_world_refs_pb2.TransformNodeReferenceByName(
      object=object_world_refs_pb2.ObjectReferenceByName(
        object_name=object_name
      )
    )
  )
