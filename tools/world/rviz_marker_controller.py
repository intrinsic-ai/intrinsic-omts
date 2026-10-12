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

"""RViz 6-DoF InteractiveMarker builder and feedback controller."""

import dataclasses
import enum
import logging
import os
import threading
import time
from collections.abc import Sequence
from typing import Any

from google.protobuf import text_format
from intrinsic.world.proto import object_world_updates_pb2

from tools.world.apply_scene_updates import (
  DEFAULT_UPDATE_FILES,
  extract_joint_updates,
)
from tools.world.pbtxt_pose_store import (
  PoseEntry,
  _extract_pose_from_transform,
  _resolve_world_node,
  apply_pose_entry_to_world,
  load_pose_entries,
  load_pose_entries_from_file,
  resolve_safe_pbtxt_path,
  snap_entry_to_tool_pose,
  update_pose_in_pbtxt,
)
from tools.world.pose_math import (
  Pose3Tuple,
  compose_transforms,
  normalize_quaternion,
  quaternion_to_rpy_deg,
  relative_transform,
)


class FeedbackEventType(enum.IntEnum):
  """Standard `visualization_msgs/msg/InteractiveMarkerFeedback` event types."""

  KEEP_ALIVE = 0
  POSE_UPDATE = 1
  MENU_SELECT = 2
  BUTTON_CLICK = 3
  MOUSE_DOWN = 4
  MOUSE_UP = 5


class ControlInteractionMode(enum.IntEnum):
  """Standard `visualization_msgs/msg/InteractiveMarkerControl` modes."""

  NONE = 0
  MENU = 1
  BUTTON = 2
  MOVE_AXIS = 3
  MOVE_PLANE = 4
  ROTATE_AXIS = 5
  MOVE_ROTATE = 6
  MOVE_3D = 7
  ROTATE_3D = 8
  MOVE_ROTATE_3D = 9


class ControlOrientationMode(enum.IntEnum):
  """Standard `visualization_msgs/msg/InteractiveMarkerControl` orientations."""

  INHERIT = 0
  FIXED = 1
  VIEW_FACING = 2


class MenuEntryId(enum.IntEnum):
  """Right-click context menu entry IDs for each RViz interactive marker."""

  SAVE_NOW = 1
  SNAP_TO_TOOL = 2
  TOGGLE_AXES_MODE = 3
  REVERT_FROM_FILE = 4
  RESET_SIM = 5


MENU_LABELS: tuple[tuple[MenuEntryId, str], ...] = (
  (MenuEntryId.SAVE_NOW, "Save Pose to .pbtxt Now"),
  (MenuEntryId.SNAP_TO_TOOL, "Snap to Gripper TCP (tool_frame)"),
  (
    MenuEntryId.TOGGLE_AXES_MODE,
    "Toggle Control Axes (Local INHERIT / Parent FIXED)",
  ),
  (MenuEntryId.REVERT_FROM_FILE, "Revert Pose from .pbtxt"),
  (MenuEntryId.RESET_SIM, "Reset Gazebo Simulation"),
)

AXIS_CONTROL_SPECS: tuple[
  tuple[str, tuple[float, float, float, float], int], ...
] = (
  (
    "rotate_x",
    (0.70710678, 0.0, 0.0, 0.70710678),
    ControlInteractionMode.ROTATE_AXIS,
  ),
  (
    "move_x",
    (0.70710678, 0.0, 0.0, 0.70710678),
    ControlInteractionMode.MOVE_AXIS,
  ),
  (
    "rotate_z",
    (0.0, 0.70710678, 0.0, 0.70710678),
    ControlInteractionMode.ROTATE_AXIS,
  ),
  (
    "move_z",
    (0.0, 0.70710678, 0.0, 0.70710678),
    ControlInteractionMode.MOVE_AXIS,
  ),
  (
    "rotate_y",
    (0.0, 0.0, 0.70710678, 0.70710678),
    ControlInteractionMode.ROTATE_AXIS,
  ),
  (
    "move_y",
    (0.0, 0.0, 0.70710678, 0.70710678),
    ControlInteractionMode.MOVE_AXIS,
  ),
)


@dataclasses.dataclass
class _FallbackHeader:
  """Duck-typed `std_msgs.msg.Header` for unit tests without `rclpy`."""

  frame_id: str = ""


@dataclasses.dataclass
class _FallbackPoint:
  """Duck-typed `geometry_msgs.msg.Point`."""

  x: float = 0.0
  y: float = 0.0
  z: float = 0.0


@dataclasses.dataclass
class _FallbackQuaternion:
  """Duck-typed `geometry_msgs.msg.Quaternion`."""

  x: float = 0.0
  y: float = 0.0
  z: float = 0.0
  w: float = 1.0


@dataclasses.dataclass
class _FallbackPose:
  """Duck-typed `geometry_msgs.msg.Pose`."""

  position: _FallbackPoint = dataclasses.field(default_factory=_FallbackPoint)
  orientation: _FallbackQuaternion = dataclasses.field(
    default_factory=_FallbackQuaternion
  )


@dataclasses.dataclass
class _FallbackVector3:
  """Duck-typed `geometry_msgs.msg.Vector3`."""

  x: float = 0.0
  y: float = 0.0
  z: float = 0.0


@dataclasses.dataclass
class _FallbackColorRGBA:
  """Duck-typed `std_msgs.msg.ColorRGBA`."""

  r: float = 0.0
  g: float = 0.0
  b: float = 0.0
  a: float = 1.0


@dataclasses.dataclass
class _FallbackMarker:
  """Duck-typed `visualization_msgs.msg.Marker`."""

  CUBE = 1
  SPHERE = 2

  header: _FallbackHeader = dataclasses.field(default_factory=_FallbackHeader)
  type: int = 1
  scale: _FallbackVector3 = dataclasses.field(default_factory=_FallbackVector3)
  color: _FallbackColorRGBA = dataclasses.field(
    default_factory=_FallbackColorRGBA
  )


@dataclasses.dataclass
class _FallbackInteractiveMarkerControl:
  """Duck-typed `visualization_msgs.msg.InteractiveMarkerControl`."""

  name: str = ""
  orientation: _FallbackQuaternion = dataclasses.field(
    default_factory=_FallbackQuaternion
  )
  orientation_mode: int = int(ControlOrientationMode.INHERIT)
  interaction_mode: int = int(ControlInteractionMode.NONE)
  always_visible: bool = False
  markers: list[Any] = dataclasses.field(default_factory=list)


@dataclasses.dataclass
class _FallbackInteractiveMarker:
  """Duck-typed `visualization_msgs.msg.InteractiveMarker`."""

  header: _FallbackHeader = dataclasses.field(default_factory=_FallbackHeader)
  pose: _FallbackPose = dataclasses.field(default_factory=_FallbackPose)
  name: str = ""
  description: str = ""
  scale: float = 0.18
  controls: list[Any] = dataclasses.field(default_factory=list)


class _FallbackRosMsgs:
  """Factory namespace used when `visualization_msgs.msg` is not installed."""

  InteractiveMarker = _FallbackInteractiveMarker
  InteractiveMarkerControl = _FallbackInteractiveMarkerControl
  Marker = _FallbackMarker


def _get_ros_msgs(ros_msgs: Any | None = None) -> Any:
  """Returns `ros_msgs`, `visualization_msgs.msg` if importable, or fallback."""
  if ros_msgs is not None:
    return ros_msgs
  try:
    from visualization_msgs import msg as vis_msg  # type: ignore[import-not-found]

    return vis_msg
  except Exception:  # pylint: disable=broad-exception-caught
    return _FallbackRosMsgs


def build_6dof_interactive_marker(
  entry: PoseEntry,
  scale: float = 0.18,
  fixed_orientation: bool = False,
  ros_msgs: Any | None = None,
  fixed_frame: str = "root",
  marker_pose: Pose3Tuple | None = None,
) -> Any:
  """Builds a 6-DoF `InteractiveMarker` with a context menu for `entry`.

  Always anchors `int_marker.header.frame_id` in `fixed_frame` (default `"root"`)
  so RViz's `InteractiveMarkerClient` never attempts a `/tf` lookup on bare
  SBL object names (such as `"enclosure"` or `"cnc_enclosure"`), which only
  exist in `/tf` with `/base_link` suffixes.

  Args:
    entry: `PoseEntry` to represent as an interactive marker in RViz.
    scale: Marker scale in meters (default: `0.18`).
    fixed_orientation: If `True`, sets axis controls to `FIXED` (parent frame
      axes); if `False`, sets them to `INHERIT` (local frame axes).
    ros_msgs: Optional module/factory providing `InteractiveMarker`,
      `InteractiveMarkerControl`, and `Marker`.
    fixed_frame: RViz Fixed Frame name (default: `"root"`).
    marker_pose: Optional `(position, orientation)` tuple of `entry` expressed
      in `fixed_frame`. Defaults to `(entry.position, entry.orientation)`.

  Returns:
    Constructed `InteractiveMarker` message.
  """
  msgs = _get_ros_msgs(ros_msgs)
  int_marker = msgs.InteractiveMarker()
  int_marker.header.frame_id = fixed_frame or "root"
  int_marker.name = (
    entry.marker_name or f"{entry.parent_name or 'root'}/{entry.target_name}"
  )
  int_marker.description = f"{entry.target_name} ({entry.entry_type})"
  int_marker.scale = float(scale)

  pos, ori = (
    marker_pose
    if marker_pose is not None
    else (entry.position, entry.orientation)
  )
  int_marker.pose.position.x = float(pos[0])
  int_marker.pose.position.y = float(pos[1])
  int_marker.pose.position.z = float(pos[2])
  ox, oy, oz, ow = normalize_quaternion(*ori)
  int_marker.pose.orientation.x = float(ox)
  int_marker.pose.orientation.y = float(oy)
  int_marker.pose.orientation.z = float(oz)
  int_marker.pose.orientation.w = float(ow)

  # Central clickable MENU visual marker so right-clicking opens the RViz menu.
  menu_control = msgs.InteractiveMarkerControl()
  menu_control.name = "menu"
  menu_control.interaction_mode = int(ControlInteractionMode.MENU)
  menu_control.always_visible = True

  center_marker = msgs.Marker()
  if hasattr(center_marker, "header"):
    center_marker.header.frame_id = ""
  center_marker.type = getattr(msgs.Marker, "CUBE", 1)
  box_size = float(scale) * 0.25
  center_marker.scale.x = box_size
  center_marker.scale.y = box_size
  center_marker.scale.z = box_size
  if entry.entry_type == "create_frame":
    center_marker.color.r = 0.2
    center_marker.color.g = 0.75
    center_marker.color.b = 0.95
  else:
    center_marker.color.r = 0.95
    center_marker.color.g = 0.65
    center_marker.color.b = 0.2
  center_marker.color.a = 0.85
  menu_control.markers.append(center_marker)
  int_marker.controls.append(menu_control)

  # 6-DoF translation arrows and rotation rings along X, Y, Z.
  orientation_mode = int(
    ControlOrientationMode.FIXED
    if fixed_orientation
    else ControlOrientationMode.INHERIT
  )
  for ctrl_name, (qx, qy, qz, qw), interaction_mode in AXIS_CONTROL_SPECS:
    ctrl = msgs.InteractiveMarkerControl()
    ctrl.name = ctrl_name
    nq_x, nq_y, nq_z, nq_w = normalize_quaternion(qx, qy, qz, qw)
    ctrl.orientation.x = float(nq_x)
    ctrl.orientation.y = float(nq_y)
    ctrl.orientation.z = float(nq_z)
    ctrl.orientation.w = float(nq_w)
    ctrl.interaction_mode = int(interaction_mode)
    ctrl.orientation_mode = orientation_mode
    int_marker.controls.append(ctrl)

  return int_marker


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

    Queries the live SBL `ObjectWorld` when available and configured, or falls
    back to chaining loaded `PoseEntry` transforms up to `self.fixed_frame`.
    Unfound top-level scene objects (such as `'enclosure'`, which is fixed at
    the world origin) default to identity.

    Args:
      parent_name: Parent object or `object/frame` name.

    Returns:
      Transform `(position, orientation)` of `parent_name` in `self.fixed_frame`.
    """
    if not parent_name or parent_name == self.fixed_frame:
      return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    if self.world is not None:
      try:
        fixed_node = _resolve_world_node(self.world, self.fixed_frame)
        parent_node = _resolve_world_node(self.world, parent_name)
        tf = self.world.get_transform(fixed_node, parent_node)
        if tf is not None and not type(tf).__module__.startswith(
          "unittest.mock"
        ):
          return _extract_pose_from_transform(tf)
      except Exception:  # pylint: disable=broad-exception-caught
        pass

    # Fallback: walk loaded entries_by_marker chain up to self.fixed_frame.
    accum: Pose3Tuple = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    curr_name = parent_name
    visited: set[str] = set()
    while (
      curr_name and curr_name != self.fixed_frame and curr_name not in visited
    ):
      visited.add(curr_name)
      matched: PoseEntry | None = None
      for candidate in self.entries_by_marker.values():
        if candidate.target_name == curr_name:
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
      accum = compose_transforms(
        (matched.position, matched.orientation),
        accum,
      )
      curr_name = matched.parent_name or self.fixed_frame

    return accum

  def get_entry_pose_in_fixed_frame(self, entry: PoseEntry) -> Pose3Tuple:
    """Computes `T_fixed_target = T_fixed_parent * T_parent_target` for `entry`."""
    t_fixed_parent = self.get_parent_pose_in_fixed_frame(entry.parent_name)
    return compose_transforms(
      t_fixed_parent, (entry.position, entry.orientation)
    )

  def _marker_pose_dict(self, entry: PoseEntry) -> dict[str, list[float]]:
    """Returns `entry`'s pose in `self.fixed_frame` as a JSON-serializable dict."""
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
      Dictionary describing the action taken, resulting `PoseEntry` dict, and
      `marker_pose` in `self.fixed_frame`.

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
        t_root_parent = self.get_parent_pose_in_fixed_frame(current.parent_name)
        t_root_curr = compose_transforms(
          t_root_parent,
          (current.position, normalize_quaternion(*current.orientation)),
        )
        pos_fixed = (
          (float(position[0]), float(position[1]), float(position[2]))
          if position is not None
          else t_root_curr[0]
        )
        ori_fixed = (
          normalize_quaternion(
            float(orientation[0]),
            float(orientation[1]),
            float(orientation[2]),
            float(orientation[3]),
          )
          if orientation is not None
          else t_root_curr[1]
        )
        new_pos, new_ori = relative_transform(
          t_root_parent, (pos_fixed, ori_fixed)
        )
      else:
        new_pos = current.position
        new_ori = normalize_quaternion(*current.orientation)

      if event_type == FeedbackEventType.POSE_UPDATE:
        rpy = tuple(round(v, 4) for v in quaternion_to_rpy_deg(*new_ori))
        updated_entry = dataclasses.replace(
          current,
          position=new_pos,
          orientation=new_ori,
          rpy_deg=(rpy[0], rpy[1], rpy[2]),
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
