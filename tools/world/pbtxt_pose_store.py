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

"""ObjectWorldUpdates (.pbtxt) pose storage, path safety, and live world sync."""

import dataclasses
import logging
import os
from collections.abc import Sequence
from typing import Any

from google.protobuf import text_format
from intrinsic.world.proto import object_world_updates_pb2

from tools.world.apply_scene_updates import (
  adapt_updates_for_live_world,
  find_file,
)
from tools.world.pose_math import (
  Pose3Tuple,
  normalize_quaternion,
  quaternion_to_rpy_deg,
)

DEFAULT_PBTXT_HEADER = (
  "# proto-file: intrinsic/world/public/proto/object_world_updates.proto\n"
  "# proto-message: intrinsic_proto.world.ObjectWorldUpdates\n\n"
)


@dataclasses.dataclass
class PoseEntry:
  """Represents a single editable frame or transform rule in a .pbtxt file."""

  entry_id: str
  marker_name: str = ""
  file_path: str = ""
  update_index: int = 0
  entry_type: str = "create_frame"
  parent_name: str = "root"
  target_name: str = ""
  position: tuple[float, float, float] = (0.0, 0.0, 0.0)
  orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
  rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)

  def __post_init__(self) -> None:
    """Populates default `marker_name` and `rpy_deg` when omitted."""
    if not self.marker_name and self.target_name:
      self.marker_name = f"{self.parent_name or 'root'}/{self.target_name}"
    if self.orientation != (0.0, 0.0, 0.0, 1.0) and self.rpy_deg == (
      0.0,
      0.0,
      0.0,
    ):
      rpy = tuple(round(v, 4) for v in quaternion_to_rpy_deg(*self.orientation))
      self.rpy_deg = (rpy[0], rpy[1], rpy[2])

  def to_dict(self) -> dict[str, Any]:
    """Serializes the pose entry into a JSON-compatible dictionary."""
    return {
      "entry_id": self.entry_id,
      "marker_name": self.marker_name,
      "file_path": self.file_path,
      "update_index": self.update_index,
      "entry_type": self.entry_type,
      "parent_name": self.parent_name,
      "target_name": self.target_name,
      "position": [
        float(self.position[0]),
        float(self.position[1]),
        float(self.position[2]),
      ],
      "orientation": [
        float(self.orientation[0]),
        float(self.orientation[1]),
        float(self.orientation[2]),
        float(self.orientation[3]),
      ],
      "rpy_deg": [
        float(self.rpy_deg[0]),
        float(self.rpy_deg[1]),
        float(self.rpy_deg[2]),
      ],
    }


def resolve_safe_pbtxt_path(
  filepath: str, allowed_root: str | None = None
) -> str:
  """Resolves a `.pbtxt` file path safely, preventing path traversal (CWE-22).

  Args:
    filepath: Relative or absolute path to a `.pbtxt` file.
    allowed_root: Optional directory root that the resolved realpath must stay
      within.

  Returns:
    Canonical resolved path to the `.pbtxt` file.

  Raises:
    ValueError: If `filepath` does not end with `.pbtxt` or resolves outside
      `allowed_root`.
  """
  if not filepath or not filepath.endswith(".pbtxt"):
    raise ValueError(f"Only .pbtxt files are permitted, got: {filepath!r}")

  if allowed_root is not None:
    root_real = os.path.realpath(allowed_root)
    candidate = (
      filepath if os.path.isabs(filepath) else os.path.join(root_real, filepath)
    )
    resolved = os.path.realpath(candidate)
    if not resolved.startswith(root_real + os.sep):
      raise ValueError(
        f"Path {filepath!r} resolves outside allowed root {allowed_root!r}"
      )
    return resolved

  if os.path.isabs(filepath):
    return os.path.realpath(filepath)

  workspace_dir = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
  if workspace_dir:
    ws_candidate = os.path.join(workspace_dir, filepath)
    if os.path.exists(ws_candidate) or os.path.exists(
      os.path.dirname(ws_candidate)
    ):
      return os.path.realpath(ws_candidate)

  working_dir = os.environ.get("BUILD_WORKING_DIRECTORY")
  if working_dir:
    wd_candidate = os.path.join(working_dir, filepath)
    if os.path.exists(wd_candidate) or os.path.exists(
      os.path.dirname(wd_candidate)
    ):
      return os.path.realpath(wd_candidate)

  return os.path.realpath(find_file(filepath))


def _extract_parent_name_from_node_a(node_a: Any) -> str:
  """Extracts parent identifier from an `UpdateTransform.node_a` proto."""
  if node_a.HasField("by_name"):
    if node_a.by_name.HasField("frame"):
      obj_name = node_a.by_name.frame.object_name
      frame_name = node_a.by_name.frame.frame_name
      if obj_name and frame_name:
        return f"{obj_name}/{frame_name}"
      return frame_name or obj_name or "root"
    if node_a.by_name.HasField("object") and node_a.by_name.object.object_name:
      return node_a.by_name.object.object_name
  return "root"


def _extract_target_name_from_node(node: Any) -> str:
  """Extracts target object or frame name from `node_to_update` or `node_b`."""
  if node.HasField("by_name"):
    if node.by_name.HasField("object") and node.by_name.object.object_name:
      return node.by_name.object.object_name
    if node.by_name.HasField("frame"):
      return (
        node.by_name.frame.frame_name or node.by_name.frame.object_name or ""
      )
  return ""


def _pose_entry_from_update(
  filepath: str,
  update_index: int,
  update: object_world_updates_pb2.ObjectWorldUpdate,
) -> PoseEntry | None:
  """Builds a `PoseEntry` from an `ObjectWorldUpdate` if it is a pose rule."""
  if update.HasField("create_frame"):
    cf = update.create_frame
    parent_name = (
      cf.parent_object_with_filter.reference.by_name.object_name or "root"
    )
    target_name = cf.new_frame_name
    pos = cf.parent_t_new_frame.position
    ori = cf.parent_t_new_frame.orientation
    position = (float(pos.x), float(pos.y), float(pos.z))
    raw_q = (float(ori.x), float(ori.y), float(ori.z), float(ori.w))
    norm_sq = sum(c * c for c in raw_q)
    orientation = (0.0, 0.0, 0.0, 1.0) if norm_sq < 1e-12 else raw_q
    rpy_deg = tuple(round(v, 4) for v in quaternion_to_rpy_deg(*orientation))
    return PoseEntry(
      entry_id=f"{filepath}::{update_index}",
      marker_name=f"{parent_name}/{target_name}",
      file_path=filepath,
      update_index=update_index,
      entry_type="create_frame",
      parent_name=parent_name,
      target_name=target_name,
      position=position,
      orientation=orientation,
      rpy_deg=(rpy_deg[0], rpy_deg[1], rpy_deg[2]),
    )

  if update.HasField("update_transform"):
    ut = update.update_transform
    parent_name = _extract_parent_name_from_node_a(ut.node_a)
    target_name = (
      _extract_target_name_from_node(ut.node_to_update)
      or _extract_target_name_from_node(ut.node_b)
      or f"transform_{update_index}"
    )
    pos = ut.a_t_b.position
    ori = ut.a_t_b.orientation
    position = (float(pos.x), float(pos.y), float(pos.z))
    raw_q = (float(ori.x), float(ori.y), float(ori.z), float(ori.w))
    norm_sq = sum(c * c for c in raw_q)
    orientation = (0.0, 0.0, 0.0, 1.0) if norm_sq < 1e-12 else raw_q
    rpy_deg = tuple(round(v, 4) for v in quaternion_to_rpy_deg(*orientation))
    return PoseEntry(
      entry_id=f"{filepath}::{update_index}",
      marker_name=f"{parent_name}/{target_name}",
      file_path=filepath,
      update_index=update_index,
      entry_type="update_transform",
      parent_name=parent_name,
      target_name=target_name,
      position=position,
      orientation=orientation,
      rpy_deg=(rpy_deg[0], rpy_deg[1], rpy_deg[2]),
    )

  return None


def _assign_unique_marker_names(entries: list[PoseEntry]) -> list[PoseEntry]:
  """Ensures every `PoseEntry` has a unique `marker_name`."""
  counts: dict[str, int] = {}
  for entry in entries:
    base = f"{entry.parent_name}/{entry.target_name}"
    counts[base] = counts.get(base, 0) + 1

  used: set[str] = set()
  result: list[PoseEntry] = []
  for entry in entries:
    base = f"{entry.parent_name}/{entry.target_name}"
    if counts[base] == 1 and base not in used:
      name = base
    else:
      name = f"{base}#{entry.update_index}"
      if name in used:
        name = (
          f"{base}#{os.path.basename(entry.file_path)}:{entry.update_index}"
        )
    used.add(name)
    result.append(dataclasses.replace(entry, marker_name=name))
  return result


def _extract_header_comments(content: str) -> str:
  """Preserves leading `#` comment lines or returns `DEFAULT_PBTXT_HEADER`."""
  header_lines: list[str] = []
  for line in content.splitlines():
    stripped = line.strip()
    if stripped.startswith("#"):
      header_lines.append(line)
    elif not stripped and header_lines:
      break
    else:
      break
  if header_lines and any("proto-file" in ln for ln in header_lines):
    return "\n".join(header_lines) + "\n\n"
  return DEFAULT_PBTXT_HEADER


def load_pose_entries_from_file(
  filepath: str, allowed_root: str | None = None
) -> list[PoseEntry]:
  """Loads all `create_frame` and `update_transform` entries from a `.pbtxt` file.

  Args:
    filepath: Path to the `.pbtxt` `ObjectWorldUpdates` file.
    allowed_root: Optional root directory to enforce path containment.

  Returns:
    List of `PoseEntry` items in file order.
  """
  resolved_path = resolve_safe_pbtxt_path(filepath, allowed_root=allowed_root)
  if not os.path.exists(resolved_path):
    logging.warning("ObjectWorldUpdates file not found: %s", resolved_path)
    return []

  with open(resolved_path, encoding="utf-8") as f:
    content = f.read()

  updates = object_world_updates_pb2.ObjectWorldUpdates()
  if content.strip():
    text_format.Parse(content, updates)

  entries: list[PoseEntry] = []
  for idx, update in enumerate(updates.updates):
    entry = _pose_entry_from_update(filepath, idx, update)
    if entry is not None:
      entries.append(entry)
  return _assign_unique_marker_names(entries)


def load_pose_entries(
  files: Sequence[str], allowed_root: str | None = None
) -> list[PoseEntry]:
  """Loads `PoseEntry` objects across multiple `.pbtxt` files in order."""
  all_entries: list[PoseEntry] = []
  for filepath in files:
    all_entries.extend(
      load_pose_entries_from_file(filepath, allowed_root=allowed_root)
    )
  return _assign_unique_marker_names(all_entries)


def update_pose_in_pbtxt(
  filepath: str,
  update_index: int,
  position: tuple[float, float, float],
  orientation: tuple[float, float, float, float],
  allowed_root: str | None = None,
) -> PoseEntry:
  """Updates a `create_frame` or `update_transform` entry in a `.pbtxt` file.

  Preserves all other `updates` in the file and the `# proto-file:` header.

  Args:
    filepath: Path to the `.pbtxt` file.
    update_index: 0-based index into `ObjectWorldUpdates.updates`.
    position: New `(x, y, z)` position in meters.
    orientation: New `(qx, qy, qz, qw)` orientation quaternion.
    allowed_root: Optional allowed root directory for path validation.

  Returns:
    The updated `PoseEntry`.

  Raises:
    IndexError: If `update_index` is out of bounds.
    ValueError: If the update at `update_index` is not a pose entry.
  """
  resolved_path = resolve_safe_pbtxt_path(filepath, allowed_root=allowed_root)
  with open(resolved_path, encoding="utf-8") as f:
    content = f.read()

  header = _extract_header_comments(content)
  updates = object_world_updates_pb2.ObjectWorldUpdates()
  if content.strip():
    text_format.Parse(content, updates)

  if update_index < 0 or update_index >= len(updates.updates):
    raise IndexError(
      f"update_index {update_index} out of range for {filepath} "
      f"(total updates: {len(updates.updates)})"
    )

  norm_q = normalize_quaternion(*orientation)
  px = round(float(position[0]), 4)
  py = round(float(position[1]), 4)
  pz = round(float(position[2]), 4)
  qx = round(float(norm_q[0]), 4)
  qy = round(float(norm_q[1]), 4)
  qz = round(float(norm_q[2]), 4)
  qw = round(float(norm_q[3]), 4)

  target_update = updates.updates[update_index]
  if target_update.HasField("create_frame"):
    pose_proto = target_update.create_frame.parent_t_new_frame
  elif target_update.HasField("update_transform"):
    pose_proto = target_update.update_transform.a_t_b
  else:
    raise ValueError(
      f"Update at index {update_index} in {filepath} is not a "
      "create_frame or update_transform entry."
    )

  pose_proto.position.x = px
  pose_proto.position.y = py
  pose_proto.position.z = pz
  pose_proto.orientation.x = qx
  pose_proto.orientation.y = qy
  pose_proto.orientation.z = qz
  pose_proto.orientation.w = qw

  body = text_format.MessageToString(updates)
  with open(resolved_path, "w", encoding="utf-8") as f:
    f.write(header + body)

  updated_entry = _pose_entry_from_update(filepath, update_index, target_update)
  if updated_entry is None:
    raise ValueError(f"Failed to build PoseEntry for index {update_index}")
  return updated_entry


def add_frame_to_pbtxt(
  filepath: str,
  frame_name: str,
  parent_name: str,
  position: tuple[float, float, float],
  orientation: tuple[float, float, float, float],
  allowed_root: str | None = None,
) -> PoseEntry:
  """Adds or updates a `create_frame` entry in the specified `.pbtxt` file.

  Args:
    filepath: Target `.pbtxt` file path.
    frame_name: Name of the frame to create or update.
    parent_name: Parent object name (defaults to `'root'`).
    position: `(x, y, z)` translation in meters.
    orientation: `(qx, qy, qz, qw)` orientation quaternion.
    allowed_root: Optional root directory to enforce path containment.

  Returns:
    The created or updated `PoseEntry`.
  """
  clean_frame = frame_name.strip()
  if not clean_frame:
    raise ValueError("frame_name must not be empty.")
  clean_parent = parent_name.strip() or "root"

  resolved_path = resolve_safe_pbtxt_path(filepath, allowed_root=allowed_root)
  content = ""
  if os.path.exists(resolved_path):
    with open(resolved_path, encoding="utf-8") as f:
      content = f.read()

  header = _extract_header_comments(content)
  updates = object_world_updates_pb2.ObjectWorldUpdates()
  if content.strip():
    text_format.Parse(content, updates)

  norm_q = normalize_quaternion(*orientation)
  px = round(float(position[0]), 4)
  py = round(float(position[1]), 4)
  pz = round(float(position[2]), 4)
  qx = round(float(norm_q[0]), 4)
  qy = round(float(norm_q[1]), 4)
  qz = round(float(norm_q[2]), 4)
  qw = round(float(norm_q[3]), 4)

  found_index: int | None = None
  for idx, update in enumerate(updates.updates):
    if update.HasField("create_frame"):
      cf = update.create_frame
      existing_parent = (
        cf.parent_object_with_filter.reference.by_name.object_name or "root"
      )
      if cf.new_frame_name == clean_frame and existing_parent == clean_parent:
        cf.parent_t_new_frame.position.x = px
        cf.parent_t_new_frame.position.y = py
        cf.parent_t_new_frame.position.z = pz
        cf.parent_t_new_frame.orientation.x = qx
        cf.parent_t_new_frame.orientation.y = qy
        cf.parent_t_new_frame.orientation.z = qz
        cf.parent_t_new_frame.orientation.w = qw
        found_index = idx
        break

  if found_index is None:
    found_index = len(updates.updates)
    new_update = updates.updates.add()
    cf = new_update.create_frame
    cf.parent_object_with_filter.reference.by_name.object_name = clean_parent
    cf.new_frame_name = clean_frame
    cf.parent_t_new_frame.position.x = px
    cf.parent_t_new_frame.position.y = py
    cf.parent_t_new_frame.position.z = pz
    cf.parent_t_new_frame.orientation.x = qx
    cf.parent_t_new_frame.orientation.y = qy
    cf.parent_t_new_frame.orientation.z = qz
    cf.parent_t_new_frame.orientation.w = qw

  body = text_format.MessageToString(updates)
  with open(resolved_path, "w", encoding="utf-8") as f:
    f.write(header + body)

  entry = _pose_entry_from_update(
    filepath, found_index, updates.updates[found_index]
  )
  if entry is None:
    raise ValueError("Failed to construct PoseEntry for created frame.")
  return entry


def _build_single_entry_updates(
  entry: PoseEntry,
) -> object_world_updates_pb2.ObjectWorldUpdates:
  """Constructs a single-update `ObjectWorldUpdates` proto for `entry`."""
  raw_updates = object_world_updates_pb2.ObjectWorldUpdates()
  up = raw_updates.updates.add()

  if entry.entry_type == "create_frame":
    cf = up.create_frame
    cf.parent_object_with_filter.reference.by_name.object_name = (
      entry.parent_name or "root"
    )
    cf.new_frame_name = entry.target_name
    cf.parent_t_new_frame.position.x = entry.position[0]
    cf.parent_t_new_frame.position.y = entry.position[1]
    cf.parent_t_new_frame.position.z = entry.position[2]
    cf.parent_t_new_frame.orientation.x = entry.orientation[0]
    cf.parent_t_new_frame.orientation.y = entry.orientation[1]
    cf.parent_t_new_frame.orientation.z = entry.orientation[2]
    cf.parent_t_new_frame.orientation.w = entry.orientation[3]
    return raw_updates

  copied_from_file = False
  try:
    if entry.file_path and entry.file_path.endswith(".pbtxt"):
      resolved = find_file(entry.file_path)
      if os.path.exists(resolved):
        with open(resolved, encoding="utf-8") as f:
          file_updates = object_world_updates_pb2.ObjectWorldUpdates()
          text_format.Parse(f.read(), file_updates)
        if 0 <= entry.update_index < len(file_updates.updates):
          src = file_updates.updates[entry.update_index]
          if src.HasField("update_transform"):
            up.CopyFrom(src)
            copied_from_file = True
  except Exception:  # pylint: disable=broad-exception-caught
    copied_from_file = False

  ut = up.update_transform
  if not copied_from_file:
    if "/" in entry.parent_name:
      p_obj, p_frame = entry.parent_name.split("/", 1)
      ut.node_a.by_name.frame.object_name = p_obj
      ut.node_a.by_name.frame.frame_name = p_frame
    else:
      ut.node_a.by_name.object.object_name = entry.parent_name or "root"

    ut.node_b.by_name.object.object_name = entry.target_name
    ut.node_to_update.by_name.object.object_name = entry.target_name

  ut.a_t_b.position.x = entry.position[0]
  ut.a_t_b.position.y = entry.position[1]
  ut.a_t_b.position.z = entry.position[2]
  ut.a_t_b.orientation.x = entry.orientation[0]
  ut.a_t_b.orientation.y = entry.orientation[1]
  ut.a_t_b.orientation.z = entry.orientation[2]
  ut.a_t_b.orientation.w = entry.orientation[3]
  return raw_updates


def apply_pose_entry_to_world(
  world: Any, entry: PoseEntry, init_world: Any | None = None
) -> bool:
  """Applies a single `PoseEntry` live to `world` (and optionally `init_world`).

  Args:
    world: Connected SBL `ObjectWorld` instance.
    entry: The `PoseEntry` to apply.
    init_world: Optional initial `ObjectWorld` instance (`EditWorldId.INITIAL`).

  Returns:
    `True` if applied to `world` without error, `False` otherwise.
  """
  if world is None:
    return False
  try:
    raw_updates = _build_single_entry_updates(entry)
    adapted = adapt_updates_for_live_world(world=world, updates=raw_updates)
    if adapted.updates:
      world.batch_update(adapted)
    if init_world is not None:
      adapted_init = adapt_updates_for_live_world(
        world=init_world, updates=raw_updates
      )
      if adapted_init.updates:
        init_world.batch_update(adapted_init)
    return True
  except Exception as e:  # pylint: disable=broad-exception-caught
    logging.warning("Failed to apply pose entry to live world: %s", e)
    return False


def _extract_pose_from_transform(transform: Any) -> Pose3Tuple:
  """Extracts `((x, y, z), (qx, qy, qz, qw))` from an SBL Pose3/Transform."""
  if hasattr(transform, "position"):
    pos = transform.position
    if hasattr(pos, "x"):
      pos_x, pos_y, pos_z = float(pos.x), float(pos.y), float(pos.z)
    elif hasattr(pos, "xyz"):
      pos_x, pos_y, pos_z = (
        float(pos.xyz[0]),
        float(pos.xyz[1]),
        float(pos.xyz[2]),
      )
    else:
      pos_x, pos_y, pos_z = float(pos[0]), float(pos[1]), float(pos[2])
  elif hasattr(transform, "translation"):
    trans = transform.translation
    pos_x, pos_y, pos_z = float(trans[0]), float(trans[1]), float(trans[2])
  else:
    pos_x, pos_y, pos_z = 0.0, 0.0, 0.0

  if hasattr(transform, "rotation"):
    rot = transform.rotation
    if hasattr(rot, "quaternion"):
      quat = rot.quaternion
      if hasattr(quat, "xyzw"):
        ori_x, ori_y, ori_z, ori_w = (
          float(quat.xyzw[0]),
          float(quat.xyzw[1]),
          float(quat.xyzw[2]),
          float(quat.xyzw[3]),
        )
      elif hasattr(quat, "x"):
        ori_x, ori_y, ori_z, ori_w = (
          float(quat.x),
          float(quat.y),
          float(quat.z),
          float(quat.w),
        )
      else:
        ori_x, ori_y, ori_z, ori_w = (
          float(quat[0]),
          float(quat[1]),
          float(quat[2]),
          float(quat[3]),
        )
    elif hasattr(rot, "xyzw"):
      ori_x, ori_y, ori_z, ori_w = (
        float(rot.xyzw[0]),
        float(rot.xyzw[1]),
        float(rot.xyzw[2]),
        float(rot.xyzw[3]),
      )
    else:
      ori_x, ori_y, ori_z, ori_w = 0.0, 0.0, 0.0, 1.0
  else:
    ori_x, ori_y, ori_z, ori_w = 0.0, 0.0, 0.0, 1.0

  return (pos_x, pos_y, pos_z), normalize_quaternion(ori_x, ori_y, ori_z, ori_w)


def _resolve_world_node(world: Any, node_name: str) -> Any:
  """Resolves an object or `object/frame` node on an SBL `ObjectWorld`."""
  clean_name = node_name or "root"
  if "/" in clean_name:
    obj_name, frame_name = clean_name.split("/", 1)
    obj = getattr(world, obj_name)
    return getattr(obj, frame_name)
  return getattr(world, clean_name)


def snap_entry_to_tool_pose(
  world: Any,
  filepath: str,
  update_index: int,
  tool_object: str = "gripper",
  tool_frame: str = "tool_frame",
  init_world: Any | None = None,
  allowed_root: str | None = None,
) -> PoseEntry:
  """Snaps a pose entry to the current live transform of `tool_object.tool_frame`.

  Args:
    world: Connected SBL `ObjectWorld` instance.
    filepath: Path to the `.pbtxt` file containing the target entry.
    update_index: Index of the update entry in `filepath`.
    tool_object: Tool object name in `world` (default: `'gripper'`).
    tool_frame: Frame name on `tool_object` (default: `'tool_frame'`).
    init_world: Optional initial `ObjectWorld` instance.
    allowed_root: Optional root directory to enforce path containment.

  Returns:
    The updated `PoseEntry`.
  """
  if world is None:
    raise ValueError("Cannot snap to tool pose when offline (world is None).")

  entries = load_pose_entries_from_file(filepath, allowed_root=allowed_root)
  target_entry: PoseEntry | None = None
  for entry in entries:
    if entry.update_index == update_index:
      target_entry = entry
      break
  if target_entry is None:
    raise IndexError(
      f"No pose entry found at update_index={update_index} in {filepath}"
    )

  parent_node = _resolve_world_node(world, target_entry.parent_name)

  tool_obj = getattr(world, tool_object)
  tool_node = getattr(tool_obj, tool_frame)
  transform = world.get_transform(parent_node, tool_node)
  position, orientation = _extract_pose_from_transform(transform)

  updated_entry = update_pose_in_pbtxt(
    filepath=filepath,
    update_index=update_index,
    position=position,
    orientation=orientation,
    allowed_root=allowed_root,
  )
  updated_entry = dataclasses.replace(
    updated_entry, marker_name=target_entry.marker_name
  )
  apply_pose_entry_to_world(
    world=world, entry=updated_entry, init_world=init_world
  )
  return updated_entry
