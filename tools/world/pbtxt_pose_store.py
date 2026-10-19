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
from intrinsic.math.python import data_types, proto_conversion
from intrinsic.world.proto import object_world_updates_pb2

from src.utils.world_utils import find_file
from tools.world.apply_scene_updates import adapt_updates_for_live_world

Pose3Tuple = tuple[
  tuple[float, float, float], tuple[float, float, float, float]
]

DEFAULT_PBTXT_HEADER = (
  "# proto-file: intrinsic/world/public/proto/object_world_updates.proto\n"
  "# proto-message: intrinsic_proto.world.ObjectWorldUpdates\n\n"
)


def _normalize_quaternion(
  orientation: Sequence[float],
) -> tuple[float, float, float, float]:
  """Normalizes `(qx, qy, qz, qw)` via `Rotation3`, defaulting to identity."""
  qx = float(orientation[0])
  qy = float(orientation[1])
  qz = float(orientation[2])
  qw = float(orientation[3])
  norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
  if norm_sq < 1e-12:
    return (0.0, 0.0, 0.0, 1.0)
  q_xyzw = data_types.Rotation3.from_xyzw(
    [qx, qy, qz, qw], normalize=True
  ).quaternion.xyzw
  return (
    float(q_xyzw[0]),
    float(q_xyzw[1]),
    float(q_xyzw[2]),
    float(q_xyzw[3]),
  )


def _quaternion_to_rpy_deg(
  orientation: Sequence[float],
) -> tuple[float, float, float]:
  """Converts `(qx, qy, qz, qw)` to rounded extrinsic XYZ RPY in degrees."""
  norm_q = _normalize_quaternion(orientation)
  rpy = data_types.Rotation3.from_xyzw(
    list(norm_q), normalize=True
  ).euler_angles(radians=False)
  return (
    round(float(rpy[0]), 4),
    round(float(rpy[1]), 4),
    round(float(rpy[2]), 4),
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
      self.rpy_deg = _quaternion_to_rpy_deg(self.orientation)

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


def _pose_from_proto_normalized(pose_proto: Any) -> data_types.Pose3:
  """Converts a `pose_pb2.Pose` to `data_types.Pose3`, normalizing safely."""
  ori = pose_proto.orientation
  raw_q = (float(ori.x), float(ori.y), float(ori.z), float(ori.w))
  norm_sq = sum(c * c for c in raw_q)
  if norm_sq < 1e-12:
    norm_q = (0.0, 0.0, 0.0, 1.0)
  else:
    q_xyzw = data_types.Rotation3.from_xyzw(
      raw_q, normalize=True
    ).quaternion.xyzw
    norm_q = (
      float(q_xyzw[0]),
      float(q_xyzw[1]),
      float(q_xyzw[2]),
      float(q_xyzw[3]),
    )
  sanitized = proto_conversion.pose_to_proto(
    data_types.Pose3(
      rotation=data_types.Rotation3(data_types.Quaternion(list(norm_q))),
      translation=[
        float(pose_proto.position.x),
        float(pose_proto.position.y),
        float(pose_proto.position.z),
      ],
    )
  )
  return proto_conversion.pose_from_proto(sanitized)


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
    pose = _pose_from_proto_normalized(cf.parent_t_new_frame)
    position, orientation = _extract_pose_from_transform(pose)
    rpy_deg = _quaternion_to_rpy_deg(orientation)
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
      rpy_deg=rpy_deg,
    )

  if update.HasField("update_transform"):
    ut = update.update_transform
    parent_name = _extract_parent_name_from_node_a(ut.node_a)
    target_name = (
      _extract_target_name_from_node(ut.node_to_update)
      or _extract_target_name_from_node(ut.node_b)
      or f"transform_{update_index}"
    )
    pose = _pose_from_proto_normalized(ut.a_t_b)
    position, orientation = _extract_pose_from_transform(pose)
    rpy_deg = _quaternion_to_rpy_deg(orientation)
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
      rpy_deg=rpy_deg,
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


def _find_named_blocks(
  text: str, field_name: str
) -> list[tuple[int, int, int, int]]:
  """Finds top-level `field_name [:] { ... }` blocks in protobuf text.

  Ignores braces and identifiers inside `#` comments and quoted string literals.

  Args:
    text: Protobuf text-format string to scan.
    field_name: Top-level message field name to locate.

  Returns:
    List of `(block_start, open_brace_idx, close_brace_idx, block_end)` tuples.
  """
  blocks: list[tuple[int, int, int, int]] = []
  n = len(text)
  i = 0
  depth = 0
  current_start: int | None = None
  current_open: int | None = None

  def _is_ident_char(ch: str) -> bool:
    return ch.isalnum() or ch == "_"

  while i < n:
    ch = text[i]
    if ch == "#":
      newline_idx = text.find("\n", i)
      if newline_idx == -1:
        break
      i = newline_idx + 1
      continue

    if ch in ('"', "'"):
      quote = ch
      i += 1
      while i < n:
        if text[i] == "\\":
          i += 2
          continue
        if text[i] == quote:
          i += 1
          break
        i += 1
      continue

    if depth == 0:
      if text.startswith(field_name, i) and (
        i == 0 or not _is_ident_char(text[i - 1])
      ):
        after_ident = i + len(field_name)
        if after_ident == n or not _is_ident_char(text[after_ident]):
          j = after_ident
          saw_colon = False
          while j < n:
            if text[j].isspace():
              j += 1
            elif text[j] == "#" and not saw_colon:
              nl = text.find("\n", j)
              j = n if nl == -1 else nl + 1
            elif text[j] == ":" and not saw_colon:
              saw_colon = True
              j += 1
            elif text[j] == "#":
              nl = text.find("\n", j)
              j = n if nl == -1 else nl + 1
            else:
              break
          if j < n and text[j] == "{":
            current_start = i
            current_open = j
            depth = 1
            i = j + 1
            continue

    if ch == "{":
      depth += 1
    elif ch == "}":
      if depth > 0:
        depth -= 1
        if (
          depth == 0 and current_start is not None and current_open is not None
        ):
          blocks.append((current_start, current_open, i, i + 1))
          current_start = None
          current_open = None
    i += 1

  return blocks


def _format_scalar_subblock(
  parent_text: str,
  block_span: tuple[int, int, int, int],
  fields: Sequence[tuple[str, float]],
) -> str:
  """Formats a `position` or `orientation` block while keeping `#` comments."""
  sub_start, sub_open, sub_close, _ = block_span
  line_start = parent_text.rfind("\n", 0, sub_start)
  prefix = (
    parent_text[line_start + 1 : sub_start]
    if line_start != -1
    else parent_text[:sub_start]
  )
  block_indent = prefix if prefix.isspace() else "      "
  field_indent = block_indent + "  "

  sub_header = parent_text[sub_start : sub_open + 1]
  sub_inner = parent_text[sub_open + 1 : sub_close]

  comment_lines: list[str] = []
  trailing_comments: dict[str, str] = {}
  for line in sub_inner.splitlines():
    stripped = line.strip()
    if not stripped:
      continue
    if stripped.startswith("#"):
      comment_lines.append(f"{field_indent}{stripped}")
    elif "#" in stripped:
      code_part, comment_part = stripped.split("#", 1)
      key_token = code_part.split(":", 1)[0].strip()
      if key_token:
        trailing_comments[key_token] = f"  #{comment_part}"

  out_lines = [sub_header]
  out_lines.extend(comment_lines)
  for key, val in fields:
    suffix = trailing_comments.get(key, "")
    out_lines.append(f"{field_indent}{key}: {val}{suffix}")
  out_lines.append(f"{block_indent}}}")
  return "\n".join(out_lines)


def _replace_pose_in_pbtxt_content(
  content: str,
  update_index: int,
  total_updates: int,
  position: tuple[float, float, float],
  orientation: tuple[float, float, float, float],
) -> str | None:
  """Updates `position` and `orientation` in-place for `updates[update_index]`.

  Preserves all header comments, inter-block comments, untouched `updates`
  blocks, and inline `#` comments inside the updated block.

  Args:
    content: Original `.pbtxt` file text.
    update_index: 0-based index of the `updates` block to modify.
    total_updates: Total number of parsed `updates` entries in the proto.
    position: Rounded `(x, y, z)` translation values.
    orientation: Full-precision `(qx, qy, qz, qw)` unit quaternion values.

  Returns:
    Updated `.pbtxt` text with comments preserved, or `None` if in-place
    replacement is not applicable.
  """
  update_blocks = _find_named_blocks(content, "updates")
  if len(update_blocks) != total_updates:
    return None
  if update_index < 0 or update_index >= len(update_blocks):
    return None

  _, u_open, u_close, _ = update_blocks[update_index]
  u_inner = content[u_open + 1 : u_close]

  cf_blocks = _find_named_blocks(u_inner, "create_frame")
  ut_blocks = _find_named_blocks(u_inner, "update_transform")
  if cf_blocks:
    _, in_open, in_close, _ = cf_blocks[0]
    pose_field = "parent_t_new_frame"
  elif ut_blocks:
    _, in_open, in_close, _ = ut_blocks[0]
    pose_field = "a_t_b"
  else:
    return None

  in_inner = u_inner[in_open + 1 : in_close]
  pose_blocks = _find_named_blocks(in_inner, pose_field)
  if not pose_blocks:
    return None

  _, p_open, p_close, _ = pose_blocks[0]
  pose_inner = in_inner[p_open + 1 : p_close]

  pos_blocks = _find_named_blocks(pose_inner, "position")
  ori_blocks = _find_named_blocks(pose_inner, "orientation")
  if not pos_blocks or not ori_blocks:
    return None

  pos_str = _format_scalar_subblock(
    pose_inner,
    pos_blocks[0],
    [("x", position[0]), ("y", position[1]), ("z", position[2])],
  )
  ori_str = _format_scalar_subblock(
    pose_inner,
    ori_blocks[0],
    [
      ("x", orientation[0]),
      ("y", orientation[1]),
      ("z", orientation[2]),
      ("w", orientation[3]),
    ],
  )

  replacements = sorted(
    [
      (pos_blocks[0][0], pos_blocks[0][3], pos_str),
      (ori_blocks[0][0], ori_blocks[0][3], ori_str),
    ],
    key=lambda item: item[0],
    reverse=True,
  )
  new_pose_inner = pose_inner
  for start_idx, end_idx, rep_text in replacements:
    new_pose_inner = (
      new_pose_inner[:start_idx] + rep_text + new_pose_inner[end_idx:]
    )

  new_in_inner = in_inner[: p_open + 1] + new_pose_inner + in_inner[p_close:]
  new_u_inner = u_inner[: in_open + 1] + new_in_inner + u_inner[in_close:]
  new_content = content[: u_open + 1] + new_u_inner + content[u_close:]
  if "proto-file" not in new_content:
    new_content = DEFAULT_PBTXT_HEADER + new_content.lstrip("\n")

  try:
    check_proto = object_world_updates_pb2.ObjectWorldUpdates()
    text_format.Parse(new_content, check_proto)
  except Exception:  # pylint: disable=broad-exception-caught
    return None

  return new_content


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

  Preserves inline `#` comments, untouched `updates` blocks, and the
  `# proto-file:` header. Position is rounded to 4 decimal places while unit
  quaternion components retain full float precision.

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

  rounded_pos = (
    round(float(position[0]), 4),
    round(float(position[1]), 4),
    round(float(position[2]), 4),
  )
  norm_q = _normalize_quaternion(orientation)
  pose_obj = data_types.Pose3(
    rotation=data_types.Rotation3.from_xyzw(list(norm_q), normalize=True),
    translation=list(rounded_pos),
  )
  pose_proto_val = proto_conversion.pose_to_proto(pose_obj)

  target_update = updates.updates[update_index]
  if target_update.HasField("create_frame"):
    target_update.create_frame.parent_t_new_frame.CopyFrom(pose_proto_val)
  elif target_update.HasField("update_transform"):
    target_update.update_transform.a_t_b.CopyFrom(pose_proto_val)
  else:
    raise ValueError(
      f"Update at index {update_index} in {filepath} is not a "
      "create_frame or update_transform entry."
    )

  new_content = _replace_pose_in_pbtxt_content(
    content=content,
    update_index=update_index,
    total_updates=len(updates.updates),
    position=rounded_pos,
    orientation=norm_q,
  )
  if new_content is None:
    new_content = header + text_format.MessageToString(updates)

  with open(resolved_path, "w", encoding="utf-8") as f:
    f.write(new_content)

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

  rounded_pos = (
    round(float(position[0]), 4),
    round(float(position[1]), 4),
    round(float(position[2]), 4),
  )
  norm_q = _normalize_quaternion(orientation)
  pose_obj = data_types.Pose3(
    rotation=data_types.Rotation3.from_xyzw(list(norm_q), normalize=True),
    translation=list(rounded_pos),
  )
  pose_proto_val = proto_conversion.pose_to_proto(pose_obj)

  found_index: int | None = None
  for idx, update in enumerate(updates.updates):
    if update.HasField("create_frame"):
      cf = update.create_frame
      existing_parent = (
        cf.parent_object_with_filter.reference.by_name.object_name or "root"
      )
      if cf.new_frame_name == clean_frame and existing_parent == clean_parent:
        cf.parent_t_new_frame.CopyFrom(pose_proto_val)
        found_index = idx
        break

  new_content: str | None = None
  if found_index is not None:
    new_content = _replace_pose_in_pbtxt_content(
      content=content,
      update_index=found_index,
      total_updates=len(updates.updates),
      position=rounded_pos,
      orientation=norm_q,
    )
  else:
    found_index = len(updates.updates)
    new_update = updates.updates.add()
    cf = new_update.create_frame
    cf.parent_object_with_filter.reference.by_name.object_name = clean_parent
    cf.new_frame_name = clean_frame
    cf.parent_t_new_frame.CopyFrom(pose_proto_val)

    if content.strip():
      single_msg = object_world_updates_pb2.ObjectWorldUpdates()
      single_msg.updates.add().CopyFrom(new_update)
      base_text = content
      if "proto-file" not in base_text:
        base_text = DEFAULT_PBTXT_HEADER + base_text.lstrip("\n")
      if not base_text.endswith("\n"):
        base_text += "\n"
      candidate_text = base_text + text_format.MessageToString(single_msg)
      try:
        check_proto = object_world_updates_pb2.ObjectWorldUpdates()
        text_format.Parse(candidate_text, check_proto)
        new_content = candidate_text
      except Exception:  # pylint: disable=broad-exception-caught
        new_content = None

  if new_content is None:
    new_content = header + text_format.MessageToString(updates)

  with open(resolved_path, "w", encoding="utf-8") as f:
    f.write(new_content)

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

  norm_q = _normalize_quaternion(entry.orientation)
  pose_proto_val = proto_conversion.pose_to_proto(
    data_types.Pose3(
      rotation=data_types.Rotation3.from_xyzw(list(norm_q), normalize=True),
      translation=list(entry.position),
    )
  )

  if entry.entry_type == "create_frame":
    cf = up.create_frame
    cf.parent_object_with_filter.reference.by_name.object_name = (
      entry.parent_name or "root"
    )
    cf.new_frame_name = entry.target_name
    cf.parent_t_new_frame.CopyFrom(pose_proto_val)
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

  ut.a_t_b.CopyFrom(pose_proto_val)
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


def _extract_pose_from_transform(transform: data_types.Pose3) -> Pose3Tuple:
  """Extracts `((x, y, z), (qx, qy, qz, qw))` from a `data_types.Pose3`."""
  pos = transform.translation
  quat = transform.rotation.quaternion.xyzw
  return (
    (float(pos[0]), float(pos[1]), float(pos[2])),
    (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
  )


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
