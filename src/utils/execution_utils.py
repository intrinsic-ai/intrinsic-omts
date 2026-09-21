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

"""Helpers for mapping OMTS options onto Intrinsic executive settings."""

import logging
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import execution

from src.core.types import SimulationMode

__all__ = [
  "build_pose_estimator_proto",
  "create_transform_node_ref",
  "describe_motion_types",
  "export_behavior_tree_dot",
  "normalize_motion_types",
  "object_exists_in_world",
  "resolve_adio_resource",
  "resolve_resource",
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
  """Converts an OMTS simulation mode to the executive SDK enum.

  Args:
      mode: OMTS simulation mode, or None to keep whatever mode is currently
        set in the executive.

  Returns:
      The matching `Executive.SimulationMode`, or None if `mode` is None.

  Raises:
      ValueError: If the mode has no executive counterpart.
  """
  if mode is None:
    return None

  try:
    return _SIMULATION_MODE_MAP[mode]
  except KeyError as err:
    raise ValueError(f"Unsupported simulation mode: {mode}") from err


def normalize_motion_types(
  motion_type: str | Sequence[str], num_segments: int
) -> list[str]:
  """Expands a motion type into one entry per trajectory segment.

  Args:
      motion_type: A single motion type string or sequence of strings.
      num_segments: Number of trajectory segments.

  Returns:
      List of motion type strings of length `num_segments`.

  Raises:
      ValueError: If `motion_type` length does not match `num_segments`.
  """
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
  """Renders motion types for a task name, collapsing a uniform trajectory.

  Args:
      motion_types: Sequence of motion type strings.

  Returns:
      Single motion type string if uniform, or slash-separated string.
  """
  if len(set(motion_types)) == 1:
    return motion_types[0]
  return "/".join(motion_types)


def object_exists_in_world(solution: Any, object_name: str | None) -> bool:
  """Checks whether the named object exists in the solution world."""
  world = getattr(solution, "world", None)
  if not object_name or world is None:
    return False
  if hasattr(world, "list_object_names"):
    names = world.list_object_names()
    if isinstance(names, (list, tuple, set)):
      return object_name in names
  return hasattr(world, object_name)


def resolve_adio_resource(solution: Any, device_name: str | None) -> Any | None:
  """Resolves an optional ADIO device resource handle if present."""
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
  """Builds a TransformNodeReference proto by object and optional frame name.

  Args:
      object_name: Name of the object.
      frame_name: Optional name of the frame on the object.

  Returns:
      TransformNodeReference proto message.
  """
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


def resolve_resource(
  solution: Any,
  target_name: str,
  capability_type: str,
  label: str = "Resource",
) -> Any:
  """Resolves a named resource handle from a connected SBL solution deployment.

  Args:
      solution: Connected SBL solution deployment object.
      target_name: Preferred resource name in `solution.resources`.
      capability_type: Substring or full proto type name to match if direct
        lookup fails.
      label: Human-readable resource description for error reporting.

  Returns:
      The resolved resource handle from `solution.resources`.

  Raises:
      ValueError: If no matching resource is found in `solution.resources`.
  """
  resources = getattr(solution, "resources", None)
  if resources is None:
    raise ValueError(
      f"{label} '{target_name}' unavailable: solution has no resources."
    )
  try:
    return resources[target_name]
  except (KeyError, TypeError):
    pass
  if hasattr(resources, "items"):
    for _, handle in resources.items():
      if hasattr(handle, "types") and capability_type in handle.types:
        return handle
  raise ValueError(f"{label} resource '{target_name}' not found in solution.")


def build_pose_estimator_proto(
  pose_estimator_id: str,
) -> Any:
  """Constructs a PoseEstimatorId proto from an asset or package ID.

  Args:
      pose_estimator_id: Asset or package ID string.

  Returns:
      Constructed PoseEstimatorId protobuf message.
  """
  from intrinsic.assets import id_utils
  from intrinsic.perception.proto.v1 import pose_estimator_id_pb2

  pkg = (
    id_utils.package_from(pose_estimator_id)
    if id_utils.is_id(pose_estimator_id)
    else "ai.intrinsic"
  )
  est_name = (
    id_utils.name_from(pose_estimator_id)
    if id_utils.is_id(pose_estimator_id)
    else pose_estimator_id
  )
  return pose_estimator_id_pb2.PoseEstimatorId(id=est_name, package=pkg)


def export_behavior_tree_dot(tree: Any, output_path: str) -> None:
  """Exports the Behavior Tree to a Graphviz DOT file if supported."""
  dot_content = getattr(tree, "dot_graph", None)
  if dot_content is not None:
    if callable(dot_content):
      dot_content = dot_content()
    with open(output_path, "w", encoding="utf-8") as f:
      f.write(str(dot_content))
    logging.info("Exported behavior tree DOT graph to %s", output_path)
