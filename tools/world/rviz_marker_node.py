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

"""Standalone ROS 2 InteractiveMarkerServer node for RViz frame editing.

Can run either in-process (`run_inprocess_ros2_node`) when `rclpy` and
`interactive_markers` are installed in the active Python interpreter, or via a
JSON-lines stdio child process (`run_subprocess_ros2_node` -> `main()`) invoked
by `bazel run //tools/world:edit_frames_rviz` using the active ROS 2 `python3`
on `PATH`.

Note: Top-level imports intentionally use only the Python standard library so a
standard ROS 2 `python3` interpreter can execute this module directly without
requiring Bazel's SBL protobuf dependencies.
"""

import dataclasses
import enum
import json
import logging
import math
import subprocess
import sys
import threading
from collections.abc import Sequence
from typing import Any


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


def _normalize_quat_xyzw(
  qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float, float, float]:
  """Normalizes `(qx, qy, qz, qw)` using stdlib `math.sqrt`."""
  norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
  if norm < 1e-12:
    return (0.0, 0.0, 0.0, 1.0)
  return (qx / norm, qy / norm, qz / norm, qw / norm)


def build_6dof_interactive_marker(
  entry: Any,
  scale: float = 0.18,
  fixed_orientation: bool = False,
  ros_msgs: Any | None = None,
  fixed_frame: str = "root",
  marker_pose: Any | None = None,
) -> Any:
  """Builds a 6-DoF `InteractiveMarker` with a context menu for `entry`.

  Always anchors `int_marker.header.frame_id` in `fixed_frame` (default `"root"`)
  so RViz's `InteractiveMarkerClient` never attempts a `/tf` lookup on bare
  SBL object names (such as `"enclosure"` or `"cnc_enclosure"`), which only
  exist in `/tf` with `/base_link` suffixes.

  Args:
    entry: `PoseEntry` object or serialized `dict[str, Any]` to represent as an
      interactive marker in RViz.
    scale: Marker scale in meters (default: `0.18`).
    fixed_orientation: If `True`, sets axis controls to `FIXED` (parent frame
      axes); if `False`, sets them to `INHERIT` (local frame axes).
    ros_msgs: Optional module/factory providing `InteractiveMarker`,
      `InteractiveMarkerControl`, and `Marker`.
    fixed_frame: RViz Fixed Frame name (default: `"root"`).
    marker_pose: Optional `(position, orientation)` tuple or `{"position": ...,
      "orientation": ...}` dict of `entry` expressed in `fixed_frame`. Defaults
      to `entry`'s own `(position, orientation)`.

  Returns:
    Constructed `InteractiveMarker` message.
  """
  if isinstance(entry, dict):
    marker_name = str(
      entry.get("marker_name")
      or f"{entry.get('parent_name') or 'root'}/{entry.get('target_name', '')}"
    )
    target_name = str(entry.get("target_name", ""))
    entry_type = str(entry.get("entry_type", "create_frame"))
    entry_pos = entry["position"]
    entry_ori = entry["orientation"]
  else:
    marker_name = (
      entry.marker_name or f"{entry.parent_name or 'root'}/{entry.target_name}"
    )
    target_name = entry.target_name
    entry_type = entry.entry_type
    entry_pos = entry.position
    entry_ori = entry.orientation

  if marker_pose is None:
    pos, ori = entry_pos, entry_ori
  elif isinstance(marker_pose, dict):
    pos = marker_pose["position"]
    ori = marker_pose["orientation"]
  else:
    pos, ori = marker_pose

  msgs = _get_ros_msgs(ros_msgs)
  int_marker = msgs.InteractiveMarker()
  int_marker.header.frame_id = fixed_frame or "root"
  int_marker.name = marker_name
  int_marker.description = f"{target_name} ({entry_type})"
  int_marker.scale = float(scale)

  int_marker.pose.position.x = float(pos[0])
  int_marker.pose.position.y = float(pos[1])
  int_marker.pose.position.z = float(pos[2])
  ox, oy, oz, ow = _normalize_quat_xyzw(
    float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])
  )
  int_marker.pose.orientation.x = float(ox)
  int_marker.pose.orientation.y = float(oy)
  int_marker.pose.orientation.z = float(oz)
  int_marker.pose.orientation.w = float(ow)

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
  if entry_type == "create_frame":
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

  orientation_mode = int(
    ControlOrientationMode.FIXED
    if fixed_orientation
    else ControlOrientationMode.INHERIT
  )
  for ctrl_name, (qx, qy, qz, qw), interaction_mode in AXIS_CONTROL_SPECS:
    ctrl = msgs.InteractiveMarkerControl()
    ctrl.name = ctrl_name
    nq_x, nq_y, nq_z, nq_w = _normalize_quat_xyzw(qx, qy, qz, qw)
    ctrl.orientation.x = float(nq_x)
    ctrl.orientation.y = float(nq_y)
    ctrl.orientation.z = float(nq_z)
    ctrl.orientation.w = float(nq_w)
    ctrl.interaction_mode = int(interaction_mode)
    ctrl.orientation_mode = orientation_mode
    int_marker.controls.append(ctrl)

  return int_marker


def build_marker_from_dict(
  item: dict[str, Any],
  scale: float,
  fixed_orientation: bool,
  fixed_frame: str = "root",
  marker_pose: dict[str, Any] | None = None,
  ros_msgs: Any | None = None,
) -> Any:
  """Constructs a `visualization_msgs.msg.InteractiveMarker` from a dict.

  Args:
    item: Serialized `PoseEntry` dictionary (`marker_name`, `target_name`,
      `entry_type`, `position`, `orientation`).
    scale: Marker scale in meters.
    fixed_orientation: Whether control axes use `FIXED` (`True`) or `INHERIT`
      (`False`) orientation mode.
    fixed_frame: RViz Fixed Frame to anchor `header.frame_id` in.
    marker_pose: Optional pose dictionary expressed in `fixed_frame`.
    ros_msgs: Optional ROS message factory module for unit tests.

  Returns:
    Constructed `InteractiveMarker` instance.
  """
  return build_6dof_interactive_marker(
    entry=item,
    scale=scale,
    fixed_orientation=fixed_orientation,
    ros_msgs=ros_msgs,
    fixed_frame=fixed_frame,
    marker_pose=marker_pose,
  )


def _dict_to_ros_pose(pose_cls: Any, pose_dict: dict[str, Any]) -> Any:
  """Converts a `{"position": [...], "orientation": [...]}` dict to `Pose`."""
  pose = pose_cls()
  pose.position.x = float(pose_dict["position"][0])
  pose.position.y = float(pose_dict["position"][1])
  pose.position.z = float(pose_dict["position"][2])
  pose.orientation.x = float(pose_dict["orientation"][0])
  pose.orientation.y = float(pose_dict["orientation"][1])
  pose.orientation.z = float(pose_dict["orientation"][2])
  pose.orientation.w = float(pose_dict["orientation"][3])
  return pose


def run_inprocess_ros2_node(controller: Any, topic_ns: str) -> None:
  """Runs the ROS 2 `InteractiveMarkerServer` directly in the current process.

  Args:
    controller: `RvizFrameEditorController` instance managing pose entries.
    topic_ns: ROS 2 topic namespace for the `InteractiveMarkerServer`.
  """
  import rclpy  # type: ignore[import-not-found]
  from geometry_msgs.msg import Pose  # type: ignore[import-not-found]
  from interactive_markers.interactive_marker_server import (  # type: ignore[import-not-found]
    InteractiveMarkerServer,
  )
  from interactive_markers.menu_handler import (  # type: ignore[import-not-found]
    MenuHandler,
  )
  from visualization_msgs import msg as vis_msg  # type: ignore[import-not-found]

  rclpy.init()
  node = rclpy.create_node("omts_frame_editor_rviz")
  server = InteractiveMarkerServer(node, topic_ns)
  menu_handler = MenuHandler()

  def _on_feedback(feedback: Any) -> None:
    pos = (
      float(feedback.pose.position.x),
      float(feedback.pose.position.y),
      float(feedback.pose.position.z),
    )
    ori = (
      float(feedback.pose.orientation.x),
      float(feedback.pose.orientation.y),
      float(feedback.pose.orientation.z),
      float(feedback.pose.orientation.w),
    )
    menu_id = (
      int(feedback.menu_entry_id)
      if int(feedback.event_type) == int(FeedbackEventType.MENU_SELECT)
      else None
    )
    try:
      result = controller.handle_feedback(
        marker_name=str(feedback.marker_name),
        event_type=int(feedback.event_type),
        position=pos,
        orientation=ori,
        menu_entry_id=menu_id,
      )
      status = result.get("status")
      entry_dict = result.get("entry")
      marker_pose_dict = result.get("marker_pose") or entry_dict
      descendant_poses = result.get("descendant_marker_poses") or {}
      if (
        status
        in (
          "saved_on_mouse_up",
          "saved_now",
          "snapped_to_tool",
          "reverted_from_file",
        )
        and isinstance(entry_dict, dict)
        and isinstance(marker_pose_dict, dict)
      ):
        server.setPose(
          str(feedback.marker_name), _dict_to_ros_pose(Pose, marker_pose_dict)
        )
        for desc_name, desc_pose_dict in descendant_poses.items():
          server.setPose(
            str(desc_name), _dict_to_ros_pose(Pose, desc_pose_dict)
          )
        server.applyChanges()
        print(
          f"[{status}] {feedback.marker_name} -> "
          f"pos={entry_dict['position']} rpy={entry_dict['rpy_deg']}"
        )
      elif status == "pose_updated" and descendant_poses:
        for desc_name, desc_pose_dict in descendant_poses.items():
          server.setPose(
            str(desc_name), _dict_to_ros_pose(Pose, desc_pose_dict)
          )
        server.applyChanges()
      elif status == "axes_mode_toggled":
        marker_name = str(feedback.marker_name)
        entry = controller.entries_by_marker[marker_name]
        is_fixed = controller.fixed_orientation.get(marker_name, False)
        rebuilt = build_6dof_interactive_marker(
          entry=entry,
          scale=controller.marker_scale,
          fixed_orientation=is_fixed,
          ros_msgs=vis_msg,
          fixed_frame=controller.fixed_frame,
          marker_pose=controller.get_entry_pose_in_fixed_frame(entry),
        )
        server.insert(rebuilt, feedback_callback=_on_feedback)
        menu_handler.apply(server, marker_name)
        server.applyChanges()
        mode_str = "FIXED (Parent)" if is_fixed else "INHERIT (Local)"
        print(f"[axes_mode_toggled] {marker_name} -> {mode_str}")
    except Exception as e:  # pylint: disable=broad-exception-caught
      node.get_logger().error(
        f"Error processing feedback for {feedback.marker_name}: {e}"
      )

  for menu_id, label in MENU_LABELS:
    handle = menu_handler.insert(label, callback=_on_feedback)
    if handle != int(menu_id):
      logging.debug("Menu handle %d mapped for %s", handle, label)

  for marker_name, marker_msg in controller.build_markers(
    ros_msgs=vis_msg
  ).items():
    server.insert(marker_msg, feedback_callback=_on_feedback)
    menu_handler.apply(server, marker_name)

  server.applyChanges()
  print(
    f"\n[✓] RViz InteractiveMarkerServer active on /{topic_ns}/update "
    f"({len(controller.entries_by_marker)} editable frames/transforms)."
  )
  print(
    "    In RViz: Add -> InteractiveMarkers -> Update Topic: "
    f"/{topic_ns}/update"
  )
  print("    Select 'Interact' in the RViz toolbar, then drag arrows/rings.")
  print("    Releasing the mouse saves to .updates.pbtxt automatically.")
  print("    Press Ctrl+C to stop.\n")

  try:
    rclpy.spin(node)
  except BaseException:  # pylint: disable=broad-exception-caught
    print("\nShutting down RViz interactive marker server...")
  finally:
    try:
      if rclpy.ok():
        server.clear()
        server.applyChanges()
        node.destroy_node()
        rclpy.shutdown()
    except Exception:  # pylint: disable=broad-exception-caught
      pass


def run_subprocess_ros2_node(
  controller: Any,
  topic_ns: str,
  cmd: Sequence[str],
) -> None:
  """Runs `rviz_marker_node.py` via the active ROS 2 Python subprocess.

  Args:
    controller: `RvizFrameEditorController` instance managing pose entries.
    topic_ns: ROS 2 topic namespace for the `InteractiveMarkerServer`.
    cmd: Command list `[python_bin, "-u", node_script_path]`.

  Raises:
    RuntimeError: If the ROS 2 marker subprocess exits before becoming ready.
  """
  proc = subprocess.Popen(
    list(cmd),
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
  )
  assert proc.stdin is not None
  assert proc.stdout is not None

  init_payload = {
    "topic_ns": topic_ns,
    "marker_scale": controller.marker_scale,
    "fixed_frame": controller.fixed_frame,
    "markers": [
      {
        "entry": entry.to_dict(),
        "marker_pose": controller._marker_pose_dict(entry),
        "fixed_orientation": controller.fixed_orientation.get(
          entry.marker_name, controller.default_fixed_orientation
        ),
      }
      for entry in controller.get_entries()
    ],
  }
  try:
    proc.stdin.write(json.dumps(init_payload) + "\n")
    proc.stdin.flush()
  except BrokenPipeError:
    pass

  stdout_iter = iter(proc.stdout)
  ready = False
  for raw_ready_line in stdout_iter:
    line = raw_ready_line.strip()
    if not line:
      continue
    try:
      ready_msg = json.loads(line)
    except ValueError:
      print(line)
      continue
    if isinstance(ready_msg, dict) and ready_msg.get("status") == "ready":
      ready = True
      break

  if not ready:
    try:
      returncode = proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
      proc.kill()
      returncode = proc.wait()
    stderr_output = ""
    if proc.stderr is not None:
      stderr_output = (
        proc.stderr.read()
        if hasattr(proc.stderr, "read")
        else "".join(proc.stderr)
      ).strip()
    detail = f": {stderr_output}" if stderr_output else ""
    raise RuntimeError(
      "ROS 2 marker node subprocess exited before becoming ready "
      f"(exit code {returncode}){detail}"
    )

  def _forward_stderr() -> None:
    if proc.stderr is None:
      return
    try:
      for err_line in proc.stderr:
        stripped = err_line.rstrip()
        if stripped:
          logging.warning("[rviz_marker_node] %s", stripped)
          sys.stderr.write(stripped + "\n")
          sys.stderr.flush()
    except Exception:  # pylint: disable=broad-exception-caught
      pass

  stderr_thread = threading.Thread(target=_forward_stderr, daemon=True)
  stderr_thread.start()

  print(
    f"\n[✓] RViz InteractiveMarkerServer active on /{topic_ns}/update "
    f"({len(controller.entries_by_marker)} editable frames/transforms)."
  )
  print(
    "    In RViz: Add -> InteractiveMarkers -> Update Topic: "
    f"/{topic_ns}/update"
  )
  print("    Select 'Interact' in the RViz toolbar, then drag arrows/rings.")
  print("    Releasing the mouse saves to .updates.pbtxt automatically.")
  print("    Press Ctrl+C to stop.\n")

  try:
    for raw_line in stdout_iter:
      line = raw_line.strip()
      if not line:
        continue
      try:
        event = json.loads(line)
      except ValueError:
        print(line)
        continue

      marker_name = str(event["marker_name"])
      event_type = int(event["event_type"])
      pos_list = event.get("position")
      ori_list = event.get("orientation")
      pos = (
        (float(pos_list[0]), float(pos_list[1]), float(pos_list[2]))
        if pos_list
        else None
      )
      ori = (
        (
          float(ori_list[0]),
          float(ori_list[1]),
          float(ori_list[2]),
          float(ori_list[3]),
        )
        if ori_list
        else None
      )
      menu_id = (
        int(event["menu_entry_id"])
        if event_type == int(FeedbackEventType.MENU_SELECT)
        else None
      )

      try:
        result = controller.handle_feedback(
          marker_name=marker_name,
          event_type=event_type,
          position=pos,
          orientation=ori,
          menu_entry_id=menu_id,
        )
        status = result.get("status")
        entry_dict = result.get("entry")
        marker_pose_dict = result.get("marker_pose") or entry_dict
        descendant_poses = result.get("descendant_marker_poses") or {}
        if status in (
          "saved_on_mouse_up",
          "saved_now",
          "snapped_to_tool",
          "reverted_from_file",
        ) and isinstance(entry_dict, dict):
          reply = {
            "cmd": "set_pose",
            "marker_name": marker_name,
            "entry": entry_dict,
            "marker_pose": marker_pose_dict,
            "descendant_marker_poses": descendant_poses,
          }
          proc.stdin.write(json.dumps(reply) + "\n")
          proc.stdin.flush()
          print(
            f"[{status}] {marker_name} -> "
            f"pos={entry_dict['position']} rpy={entry_dict['rpy_deg']}"
          )
        elif status == "pose_updated" and descendant_poses:
          reply = {
            "cmd": "set_pose",
            "descendant_marker_poses": descendant_poses,
          }
          proc.stdin.write(json.dumps(reply) + "\n")
          proc.stdin.flush()
        elif status == "axes_mode_toggled" and isinstance(entry_dict, dict):
          is_fixed = bool(result.get("fixed_orientation", False))
          reply = {
            "cmd": "rebuild_marker",
            "marker_name": marker_name,
            "entry": entry_dict,
            "marker_pose": marker_pose_dict,
            "fixed_orientation": is_fixed,
          }
          proc.stdin.write(json.dumps(reply) + "\n")
          proc.stdin.flush()
          mode_str = "FIXED (Parent)" if is_fixed else "INHERIT (Local)"
          print(f"[axes_mode_toggled] {marker_name} -> {mode_str}")
      except Exception as e:  # pylint: disable=broad-exception-caught
        logging.warning(
          "Error handling marker feedback for %s: %s", marker_name, e
        )
  except KeyboardInterrupt:
    print("\nShutting down RViz interactive marker bridge...")
  finally:
    try:
      proc.stdin.close()
    except Exception:  # pylint: disable=broad-exception-caught
      pass
    proc.terminate()
    try:
      proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
      proc.kill()


def main() -> None:
  """Runs the standalone JSON-lines stdio ROS 2 `InteractiveMarkerServer` node."""
  import rclpy  # type: ignore[import-not-found]
  from geometry_msgs.msg import Pose  # type: ignore[import-not-found]
  from interactive_markers.interactive_marker_server import (  # type: ignore[import-not-found]
    InteractiveMarkerServer,
  )
  from interactive_markers.menu_handler import (  # type: ignore[import-not-found]
    MenuHandler,
  )

  init_line = sys.stdin.readline()
  if not init_line:
    return
  cfg = json.loads(init_line)
  topic_ns = str(cfg["topic_ns"])
  scale = float(cfg["marker_scale"])
  fixed_frame = str(cfg.get("fixed_frame", "root") or "root")

  rclpy.init()
  node = rclpy.create_node("omts_frame_editor_rviz")
  server = InteractiveMarkerServer(node, topic_ns)
  menu_handler = MenuHandler()

  def _on_feedback(fb: Any) -> None:
    event = {
      "marker_name": str(fb.marker_name),
      "event_type": int(fb.event_type),
      "position": [
        float(fb.pose.position.x),
        float(fb.pose.position.y),
        float(fb.pose.position.z),
      ],
      "orientation": [
        float(fb.pose.orientation.x),
        float(fb.pose.orientation.y),
        float(fb.pose.orientation.z),
        float(fb.pose.orientation.w),
      ],
      "menu_entry_id": int(fb.menu_entry_id),
    }
    sys.stdout.write(json.dumps(event) + "\n")
    sys.stdout.flush()

  for _, label in MENU_LABELS:
    menu_handler.insert(label, callback=_on_feedback)

  for m_cfg in cfg["markers"]:
    marker = build_marker_from_dict(
      item=m_cfg["entry"],
      scale=scale,
      fixed_orientation=bool(m_cfg["fixed_orientation"]),
      fixed_frame=fixed_frame,
      marker_pose=m_cfg.get("marker_pose"),
    )
    server.insert(marker, feedback_callback=_on_feedback)
    menu_handler.apply(server, str(m_cfg["entry"]["marker_name"]))
  server.applyChanges()
  sys.stdout.write(
    json.dumps({"status": "ready", "count": len(cfg["markers"])}) + "\n"
  )
  sys.stdout.flush()

  def _stdin_reader() -> None:
    for raw_line in sys.stdin:
      line = raw_line.strip()
      if not line:
        continue
      cmd = json.loads(line)
      if cmd.get("cmd") == "set_pose":
        pose_src = cmd.get("marker_pose") or cmd.get("entry")
        if cmd.get("marker_name") and isinstance(pose_src, dict):
          server.setPose(
            str(cmd["marker_name"]), _dict_to_ros_pose(Pose, pose_src)
          )
        for desc_name, desc_pose_src in (
          cmd.get("descendant_marker_poses") or {}
        ).items():
          server.setPose(str(desc_name), _dict_to_ros_pose(Pose, desc_pose_src))
        server.applyChanges()
      elif cmd.get("cmd") == "rebuild_marker":
        ent = cmd["entry"]
        rebuilt = build_marker_from_dict(
          item=ent,
          scale=scale,
          fixed_orientation=bool(cmd["fixed_orientation"]),
          fixed_frame=fixed_frame,
          marker_pose=cmd.get("marker_pose"),
        )
        server.insert(rebuilt, feedback_callback=_on_feedback)
        menu_handler.apply(server, str(cmd["marker_name"]))
        server.applyChanges()
    try:
      if rclpy.ok():
        rclpy.shutdown()
    except Exception:  # pylint: disable=broad-exception-caught
      pass

  reader_thread = threading.Thread(target=_stdin_reader, daemon=True)
  reader_thread.start()
  try:
    rclpy.spin(node)
  except BaseException:  # pylint: disable=broad-exception-caught
    pass
  finally:
    try:
      if rclpy.ok():
        server.clear()
        server.applyChanges()
        node.destroy_node()
        rclpy.shutdown()
    except Exception:  # pylint: disable=broad-exception-caught
      pass


if __name__ == "__main__":
  main()
