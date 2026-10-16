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

"""RViz 6-DoF InteractiveMarker feedback controller."""

import dataclasses
import logging
import os
import threading
import time
from collections.abc import Sequence
from typing import Any

from google.protobuf import text_format
from intrinsic.math.python import data_types
from intrinsic.world.proto import object_world_updates_pb2

from tools.world.apply_scene_updates import (
  DEFAULT_UPDATE_FILES,
  extract_joint_updates,
)
from tools.world.pbtxt_pose_store import (
  Pose3Tuple,
  PoseEntry,
  _extract_pose_from_transform,
  _normalize_quaternion,
  _quaternion_to_rpy_deg,
  _resolve_world_node,
  apply_pose_entry_to_world,
  load_pose_entries,
  load_pose_entries_from_file,
  resolve_safe_pbtxt_path,
  snap_entry_to_tool_pose,
  update_pose_in_pbtxt,
)
from tools.world.rviz_marker_node import (
  AXIS_CONTROL_SPECS,
  MENU_LABELS,
  ControlInteractionMode,
  ControlOrientationMode,
  FeedbackEventType,
  MenuEntryId,
  build_6dof_interactive_marker,
)

__all__ = [
  "AXIS_CONTROL_SPECS",
  "MENU_LABELS",
  "ControlInteractionMode",
  "ControlOrientationMode",
  "FeedbackEventType",
  "MenuEntryId",
  "RvizFrameEditorController",
  "build_6dof_interactive_marker",
]


def _pose_tuple_to_pose3(pose: Pose3Tuple) -> data_types.Pose3:
  """Converts `((x, y, z), (qx, qy, qz, qw))` to a `data_types.Pose3`."""
  pos, ori = pose
  norm_q = _normalize_quaternion(ori)
  return data_types.Pose3(
    rotation=data_types.Rotation3(data_types.Quaternion(list(norm_q))),
    translation=[float(pos[0]), float(pos[1]), float(pos[2])],
  )


class RvizFrameEditorController:
  """Manages RViz 6-DoF InteractiveMarker state and feedback events."""

  def __init__(
    self,
    files: Sequence[str] = DEFAULT_UPDATE_FILES,
    world: Any | None = None,
    init_world: Any | None = None,
    solution: Any | None = None,
    allowed_root: str | None = None,
    marker_scale: float = 0.18,
    default_fixed_orientation: bool = False,
    fixed_orientation: bool | dict[str, bool] | None = None,
    drag_throttle_sec: float = 0.04,
    reset_sim_on_release: bool = False,
    tool_object: str = "gripper",
    tool_frame: str = "tool_frame",
    entries: Sequence[PoseEntry] | None = None,
    fixed_frame: str = "root",
  ) -> None:
    """Initializes the RViz interactive marker controller."""
    self.files: list[str] = list(files)
    self.world: Any | None = world
    self.init_world: Any | None = init_world
    self.solution: Any | None = solution
    self.allowed_root: str | None = allowed_root
    self.marker_scale: float = float(marker_scale)
    self.fixed_frame: str = fixed_frame or "root"
    if isinstance(fixed_orientation, bool):
      self.default_fixed_orientation: bool = fixed_orientation
      self.fixed_orientation: dict[str, bool] = {}
    elif isinstance(fixed_orientation, dict):
      self.default_fixed_orientation = bool(default_fixed_orientation)
      self.fixed_orientation = dict(fixed_orientation)
    else:
      self.default_fixed_orientation = bool(default_fixed_orientation)
      self.fixed_orientation = {}

    self.drag_throttle_sec: float = float(drag_throttle_sec)
    self.reset_sim_on_release: bool = bool(reset_sim_on_release)
    self.tool_object: str = tool_object
    self.tool_frame: str = tool_frame
    self.entries_by_marker: dict[str, PoseEntry] = {}
    self.last_drag_update_time: dict[str, float] = {}
    self.lock = threading.RLock()

    initial_entries = (
      list(entries)
      if entries is not None
      else load_pose_entries(self.files, allowed_root=self.allowed_root)
    )
    for entry in initial_entries:
      marker_key = (
        entry.marker_name
        or f"{entry.parent_name or 'root'}/{entry.target_name}"
      )
      if not entry.marker_name:
        entry = dataclasses.replace(entry, marker_name=marker_key)
      self.entries_by_marker[marker_key] = entry
      self.fixed_orientation.setdefault(
        marker_key, self.default_fixed_orientation
      )

  def get_parent_pose_in_fixed_frame(self, parent_name: str) -> Pose3Tuple:
    """Resolves `T_fixed_parent` (`((x, y, z), (qx, qy, qz, qw))`).

    Chains loaded `PoseEntry` transforms up to `self.fixed_frame`, or queries
    the live SBL `ObjectWorld` when `parent_name` is not managed by loaded
    entries. Unfound top-level scene objects (such as `'enclosure'`, which is
    fixed at the world origin) default to identity.

    Args:
      parent_name: Parent object or `object/frame` name.

    Returns:
      Transform `(position, orientation)` of `parent_name` in
      `self.fixed_frame`.
    """
    if not parent_name or parent_name == self.fixed_frame:
      return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    accum = data_types.Pose3.identity()
    curr_name = parent_name
    visited: set[str] = set()
    matched_any = False
    while (
      curr_name and curr_name != self.fixed_frame and curr_name not in visited
    ):
      visited.add(curr_name)
      matched: PoseEntry | None = None
      for candidate in self.entries_by_marker.values():
        if (
          candidate.target_name == curr_name
          or f"{candidate.parent_name or 'root'}/{candidate.target_name}"
          == curr_name
        ):
          matched = candidate
          break
      if matched is None and "/" in curr_name:
        base_obj = curr_name.split("/", 1)[0]
        for candidate in self.entries_by_marker.values():
          if candidate.target_name == base_obj:
            matched = candidate
            break
      if matched is None:
        break
      matched_any = True
      matched_pose = _pose_tuple_to_pose3(
        (matched.position, matched.orientation)
      )
      accum = matched_pose.multiply(accum)
      curr_name = matched.parent_name or self.fixed_frame

    if matched_any:
      return _extract_pose_from_transform(accum)

    if self.world is not None:
      try:
        fixed_node = _resolve_world_node(self.world, self.fixed_frame)
        parent_node = _resolve_world_node(self.world, parent_name)
        tf = self.world.get_transform(fixed_node, parent_node)
        if tf is not None:
          return _extract_pose_from_transform(tf)
      except Exception:  # pylint: disable=broad-exception-caught
        pass

    return _extract_pose_from_transform(accum)

  def get_entry_pose_in_fixed_frame(self, entry: PoseEntry) -> Pose3Tuple:
    """Computes `T_fixed_target = T_fixed_parent * T_parent_target`."""
    pose_fixed_parent = _pose_tuple_to_pose3(
      self.get_parent_pose_in_fixed_frame(entry.parent_name)
    )
    pose_parent_target = _pose_tuple_to_pose3(
      (entry.position, entry.orientation)
    )
    return _extract_pose_from_transform(
      pose_fixed_parent.multiply(pose_parent_target)
    )

  def _marker_pose_dict(self, entry: PoseEntry) -> dict[str, list[float]]:
    """Returns `entry`'s pose in `self.fixed_frame` as a JSON-compatible dict."""
    m_pos, m_ori = self.get_entry_pose_in_fixed_frame(entry)
    return {
      "position": [float(m_pos[0]), float(m_pos[1]), float(m_pos[2])],
      "orientation": [
        float(m_ori[0]),
        float(m_ori[1]),
        float(m_ori[2]),
        float(m_ori[3]),
      ],
    }

  def _is_descendant_of(
    self, candidate: PoseEntry, ancestor: PoseEntry
  ) -> bool:
    """Returns True if `candidate`'s parent chain passes through `ancestor`."""
    if not ancestor.target_name:
      return False
    ancestor_keys = {
      ancestor.target_name,
      f"{ancestor.parent_name or 'root'}/{ancestor.target_name}",
    }
    if ancestor.marker_name:
      ancestor_keys.add(ancestor.marker_name)

    curr_name = candidate.parent_name
    visited: set[str] = set()
    while (
      curr_name and curr_name != self.fixed_frame and curr_name not in visited
    ):
      visited.add(curr_name)
      if curr_name in ancestor_keys:
        return True
      if "/" in curr_name and curr_name.split("/", 1)[0] in ancestor_keys:
        return True

      matched: PoseEntry | None = None
      for entry in self.entries_by_marker.values():
        if (
          entry.target_name == curr_name
          or f"{entry.parent_name or 'root'}/{entry.target_name}" == curr_name
        ):
          matched = entry
          break
      if matched is None and "/" in curr_name:
        base_obj = curr_name.split("/", 1)[0]
        for entry in self.entries_by_marker.values():
          if entry.target_name == base_obj:
            matched = entry
            break
      if matched is None:
        break
      if matched.marker_name == ancestor.marker_name:
        return True
      curr_name = matched.parent_name or self.fixed_frame

    return False

  def get_descendant_marker_poses(
    self, parent_marker_name: str
  ) -> dict[str, dict[str, list[float]]]:
    """Computes updated `fixed_frame` poses for all descendants of a marker.

    Args:
      parent_marker_name: Marker name of the parent entry that moved.

    Returns:
      Mapping from descendant `marker_name` to its `{"position": [...],
      "orientation": [...]}` dictionary in `self.fixed_frame`.
    """
    with self.lock:
      parent_entry = self.entries_by_marker.get(parent_marker_name)
      if parent_entry is None:
        return {}
      descendant_poses: dict[str, dict[str, list[float]]] = {}
      for marker_name, entry in self.entries_by_marker.items():
        if marker_name == parent_marker_name:
          continue
        if self._is_descendant_of(entry, parent_entry):
          descendant_poses[marker_name] = self._marker_pose_dict(entry)
      return descendant_poses

  def get_entries(self) -> list[PoseEntry]:
    """Returns all managed `PoseEntry` items in insertion order."""
    with self.lock:
      return list(self.entries_by_marker.values())

  def reload_entries(self) -> list[PoseEntry]:
    """Reloads all `PoseEntry` items from `self.files`."""
    with self.lock:
      loaded = load_pose_entries(self.files, allowed_root=self.allowed_root)
      self.entries_by_marker.clear()
      for entry in loaded:
        self.entries_by_marker[entry.marker_name] = entry
        self.fixed_orientation.setdefault(
          entry.marker_name, self.default_fixed_orientation
        )
      return list(self.entries_by_marker.values())

  def build_markers(self, ros_msgs: Any | None = None) -> dict[str, Any]:
    """Builds 6-DoF `InteractiveMarker` messages for all loaded entries."""
    with self.lock:
      markers: dict[str, Any] = {}
      for marker_name, entry in self.entries_by_marker.items():
        is_fixed = self.fixed_orientation.get(
          marker_name, self.default_fixed_orientation
        )
        markers[marker_name] = build_6dof_interactive_marker(
          entry=entry,
          scale=self.marker_scale,
          fixed_orientation=is_fixed,
          ros_msgs=ros_msgs,
          fixed_frame=self.fixed_frame,
          marker_pose=self.get_entry_pose_in_fixed_frame(entry),
        )
      return markers

  def reset_simulation(self) -> bool:
    """Resets the Gazebo simulation if connected to a simulated solution."""
    if self.solution is None:
      return False
    if not getattr(self.solution, "is_simulated", True):
      return False
    simulator = getattr(self.solution, "simulator", None)
    if simulator is None or not hasattr(simulator, "reset"):
      return False
    try:
      if self.world is not None:
        joint_updates = object_world_updates_pb2.ObjectWorldUpdates()
        for fpath in self.files:
          try:
            resolved = resolve_safe_pbtxt_path(
              fpath, allowed_root=self.allowed_root
            )
            if os.path.exists(resolved):
              with open(resolved, encoding="utf-8") as f:
                raw = object_world_updates_pb2.ObjectWorldUpdates()
                text_format.Parse(f.read(), raw)
              extracted = extract_joint_updates(raw)
              joint_updates.updates.extend(extracted.updates)
          except Exception:  # pylint: disable=broad-exception-caught
            continue
        if joint_updates.updates:
          self.world.batch_update(joint_updates)
      simulator.reset()
      return True
    except Exception as e:  # pylint: disable=broad-exception-caught
      logging.warning("Failed to reset simulation: %s", e)
      return False

  def handle_feedback(
    self,
    marker_name: str,
    event_type: int,
    position: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    menu_entry_id: int | None = None,
    now: float | None = None,
  ) -> dict[str, Any]:
    """Processes an RViz `InteractiveMarkerFeedback` event for `marker_name`.

    Args:
      marker_name: Interactive marker identifier (`entry.marker_name`).
      event_type: `FeedbackEventType` integer code.
      position: Optional `(x, y, z)` feedback position in `self.fixed_frame`.
      orientation: Optional `(qx, qy, qz, qw)` feedback quaternion in
        `self.fixed_frame`.
      menu_entry_id: Optional `MenuEntryId` when `event_type == MENU_SELECT`.
      now: Optional monotonic timestamp in seconds for drag throttling tests.

    Returns:
      Dictionary describing the action taken, resulting `PoseEntry` dict,
      `marker_pose` in `self.fixed_frame`, and any `descendant_marker_poses`.

    Raises:
      KeyError: If `marker_name` is not registered.
      ValueError: If `menu_entry_id` is unrecognized on `MENU_SELECT`.
    """
    with self.lock:
      if marker_name not in self.entries_by_marker:
        raise KeyError(f"Unknown interactive marker: {marker_name!r}")

      current = self.entries_by_marker[marker_name]
      current_time = time.monotonic() if now is None else float(now)

      if position is not None or orientation is not None:
        pose_root_parent = _pose_tuple_to_pose3(
          self.get_parent_pose_in_fixed_frame(current.parent_name)
        )
        pose_parent_curr = _pose_tuple_to_pose3(
          (current.position, current.orientation)
        )
        pose_root_curr = pose_root_parent.multiply(pose_parent_curr)
        curr_pos_fixed, curr_ori_fixed = _extract_pose_from_transform(
          pose_root_curr
        )
        pos_fixed = (
          (float(position[0]), float(position[1]), float(position[2]))
          if position is not None
          else curr_pos_fixed
        )
        ori_fixed = (
          _normalize_quaternion(orientation)
          if orientation is not None
          else curr_ori_fixed
        )
        pose_root_target = _pose_tuple_to_pose3((pos_fixed, ori_fixed))
        pose_parent_target = pose_root_parent.multiply_by_inverse(
          pose_root_target
        )
        new_pos, new_ori = _extract_pose_from_transform(pose_parent_target)
      else:
        new_pos = current.position
        new_ori = _normalize_quaternion(current.orientation)

      if event_type == FeedbackEventType.POSE_UPDATE:
        rpy = _quaternion_to_rpy_deg(new_ori)
        updated_entry = dataclasses.replace(
          current,
          position=new_pos,
          orientation=new_ori,
          rpy_deg=rpy,
        )
        self.entries_by_marker[marker_name] = updated_entry

        last_time = self.last_drag_update_time.get(marker_name)
        applied = False
        if (
          last_time is None
          or (current_time - last_time) >= self.drag_throttle_sec
        ):
          self.last_drag_update_time[marker_name] = current_time
          if self.world is not None:
            applied = apply_pose_entry_to_world(
              self.world, updated_entry, init_world=None
            )

        return {
          "status": "pose_updated",
          "entry": updated_entry.to_dict(),
          "marker_pose": self._marker_pose_dict(updated_entry),
          "descendant_marker_poses": self.get_descendant_marker_poses(
            marker_name
          ),
          "applied_to_world": applied,
          "saved_to_file": False,
        }

      if event_type == FeedbackEventType.MOUSE_UP:
        updated_entry = update_pose_in_pbtxt(
          filepath=current.file_path,
          update_index=current.update_index,
          position=new_pos,
          orientation=new_ori,
          allowed_root=self.allowed_root,
        )
        updated_entry = dataclasses.replace(
          updated_entry, marker_name=marker_name
        )
        self.entries_by_marker[marker_name] = updated_entry

        applied = False
        if self.world is not None:
          applied = apply_pose_entry_to_world(
            self.world, updated_entry, init_world=self.init_world
          )
        sim_reset = False
        if self.reset_sim_on_release:
          sim_reset = self.reset_simulation()

        return {
          "status": "saved_on_mouse_up",
          "entry": updated_entry.to_dict(),
          "marker_pose": self._marker_pose_dict(updated_entry),
          "descendant_marker_poses": self.get_descendant_marker_poses(
            marker_name
          ),
          "applied_to_world": applied,
          "saved_to_file": True,
          "sim_reset": sim_reset,
        }

      if event_type == FeedbackEventType.MENU_SELECT:
        if menu_entry_id == MenuEntryId.SAVE_NOW:
          updated_entry = update_pose_in_pbtxt(
            filepath=current.file_path,
            update_index=current.update_index,
            position=new_pos,
            orientation=new_ori,
            allowed_root=self.allowed_root,
          )
          updated_entry = dataclasses.replace(
            updated_entry, marker_name=marker_name
          )
          self.entries_by_marker[marker_name] = updated_entry
          applied = False
          if self.world is not None:
            applied = apply_pose_entry_to_world(
              self.world, updated_entry, init_world=self.init_world
            )
          return {
            "status": "saved_now",
            "entry": updated_entry.to_dict(),
            "marker_pose": self._marker_pose_dict(updated_entry),
            "descendant_marker_poses": self.get_descendant_marker_poses(
              marker_name
            ),
            "applied_to_world": applied,
            "saved_to_file": True,
          }

        if menu_entry_id == MenuEntryId.SNAP_TO_TOOL:
          snapped_entry = snap_entry_to_tool_pose(
            world=self.world,
            filepath=current.file_path,
            update_index=current.update_index,
            tool_object=self.tool_object,
            tool_frame=self.tool_frame,
            init_world=self.init_world,
            allowed_root=self.allowed_root,
          )
          snapped_entry = dataclasses.replace(
            snapped_entry, marker_name=marker_name
          )
          self.entries_by_marker[marker_name] = snapped_entry
          return {
            "status": "snapped_to_tool",
            "entry": snapped_entry.to_dict(),
            "marker_pose": self._marker_pose_dict(snapped_entry),
            "descendant_marker_poses": self.get_descendant_marker_poses(
              marker_name
            ),
            "applied_to_world": True,
            "saved_to_file": True,
          }

        if menu_entry_id == MenuEntryId.TOGGLE_AXES_MODE:
          new_mode = not self.fixed_orientation.get(
            marker_name, self.default_fixed_orientation
          )
          self.fixed_orientation[marker_name] = new_mode
          return {
            "status": "axes_mode_toggled",
            "fixed_orientation": new_mode,
            "entry": current.to_dict(),
            "marker_pose": self._marker_pose_dict(current),
          }

        if menu_entry_id == MenuEntryId.REVERT_FROM_FILE:
          disk_entries = load_pose_entries_from_file(
            current.file_path, allowed_root=self.allowed_root
          )
          reverted: PoseEntry | None = None
          for disk_entry in disk_entries:
            if disk_entry.update_index == current.update_index:
              reverted = dataclasses.replace(
                disk_entry, marker_name=marker_name
              )
              break
          if reverted is None:
            raise IndexError(
              f"Entry at index {current.update_index} no longer exists in "
              f"{current.file_path}"
            )
          self.entries_by_marker[marker_name] = reverted
          applied = False
          if self.world is not None:
            applied = apply_pose_entry_to_world(
              self.world, reverted, init_world=self.init_world
            )
          return {
            "status": "reverted_from_file",
            "entry": reverted.to_dict(),
            "marker_pose": self._marker_pose_dict(reverted),
            "descendant_marker_poses": self.get_descendant_marker_poses(
              marker_name
            ),
            "applied_to_world": applied,
            "saved_to_file": False,
          }

        if menu_entry_id == MenuEntryId.RESET_SIM:
          reset_ok = self.reset_simulation()
          return {
            "status": "sim_reset",
            "reset": reset_ok,
            "entry": current.to_dict(),
            "marker_pose": self._marker_pose_dict(current),
          }

        raise ValueError(f"Unsupported menu_entry_id: {menu_entry_id!r}")

      return {
        "status": "ignored",
        "entry": current.to_dict(),
        "marker_pose": self._marker_pose_dict(current),
        "applied_to_world": False,
        "saved_to_file": False,
      }
