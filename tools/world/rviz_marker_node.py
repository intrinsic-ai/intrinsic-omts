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

import json
import logging
import subprocess
import sys
import threading
from collections.abc import Sequence
from typing import Any

NODE_AXIS_SPECS: tuple[
  tuple[str, tuple[float, float, float, float], int], ...
] = (
  ("rotate_x", (0.70710678, 0.0, 0.0, 0.70710678), 5),
  ("move_x", (0.70710678, 0.0, 0.0, 0.70710678), 3),
  ("rotate_z", (0.0, 0.70710678, 0.0, 0.70710678), 5),
  ("move_z", (0.0, 0.70710678, 0.0, 0.70710678), 3),
  ("rotate_y", (0.0, 0.0, 0.70710678, 0.70710678), 5),
  ("move_y", (0.0, 0.0, 0.70710678, 0.70710678), 3),
)

NODE_MENU_LABELS: tuple[str, ...] = (
  "Save Pose to .pbtxt Now",
  "Snap to Gripper TCP (tool_frame)",
  "Toggle Control Axes (Local INHERIT / Parent FIXED)",
  "Revert Pose from .pbtxt",
  "Reset Gazebo Simulation",
)

MENU_SELECT_EVENT_TYPE = 2


def build_marker_from_dict(
  item: dict[str, Any],
  scale: float,
  fixed_orientation: bool,
  fixed_frame: str = "root",
  marker_pose: dict[str, Any] | None = None,
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

  Returns:
    Constructed `visualization_msgs.msg.InteractiveMarker` instance.
  """
  from visualization_msgs.msg import (  # type: ignore[import-not-found]
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
  )

  int_marker = InteractiveMarker()
  int_marker.header.frame_id = fixed_frame or "root"
  int_marker.name = str(item["marker_name"])
  int_marker.description = f"{item['target_name']} ({item['entry_type']})"
  int_marker.scale = float(scale)

  pose_src = marker_pose if marker_pose is not None else item
  int_marker.pose.position.x = float(pose_src["position"][0])
  int_marker.pose.position.y = float(pose_src["position"][1])
  int_marker.pose.position.z = float(pose_src["position"][2])
  int_marker.pose.orientation.x = float(pose_src["orientation"][0])
  int_marker.pose.orientation.y = float(pose_src["orientation"][1])
  int_marker.pose.orientation.z = float(pose_src["orientation"][2])
  int_marker.pose.orientation.w = float(pose_src["orientation"][3])

  menu_ctrl = InteractiveMarkerControl()
  menu_ctrl.name = "menu"
  menu_ctrl.interaction_mode = InteractiveMarkerControl.MENU
  menu_ctrl.always_visible = True

  box = Marker()
  box.header.frame_id = ""
  box.type = Marker.CUBE
  box_size = float(scale) * 0.25
  box.scale.x = box_size
  box.scale.y = box_size
  box.scale.z = box_size
  is_create_frame = item.get("entry_type") == "create_frame"
  box.color.r = 0.2 if is_create_frame else 0.95
  box.color.g = 0.75 if is_create_frame else 0.65
  box.color.b = 0.95 if is_create_frame else 0.2
  box.color.a = 0.85
  menu_ctrl.markers.append(box)
  int_marker.controls.append(menu_ctrl)

  orientation_mode = (
    InteractiveMarkerControl.FIXED
    if fixed_orientation
    else InteractiveMarkerControl.INHERIT
  )
  for ctrl_name, (qx, qy, qz, qw), interaction_mode in NODE_AXIS_SPECS:
    ctrl = InteractiveMarkerControl()
    ctrl.name = ctrl_name
    ctrl.orientation.x = float(qx)
    ctrl.orientation.y = float(qy)
    ctrl.orientation.z = float(qz)
    ctrl.orientation.w = float(qw)
    ctrl.interaction_mode = int(interaction_mode)
    ctrl.orientation_mode = int(orientation_mode)
    int_marker.controls.append(ctrl)

  return int_marker


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

  from tools.world.rviz_marker_controller import (
    MENU_LABELS,
    FeedbackEventType,
    build_6dof_interactive_marker,
  )

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
        new_pose = Pose()
        new_pose.position.x = float(marker_pose_dict["position"][0])
        new_pose.position.y = float(marker_pose_dict["position"][1])
        new_pose.position.z = float(marker_pose_dict["position"][2])
        new_pose.orientation.x = float(marker_pose_dict["orientation"][0])
        new_pose.orientation.y = float(marker_pose_dict["orientation"][1])
        new_pose.orientation.z = float(marker_pose_dict["orientation"][2])
        new_pose.orientation.w = float(marker_pose_dict["orientation"][3])
        server.setPose(str(feedback.marker_name), new_pose)
        server.applyChanges()
        print(
          f"[{status}] {feedback.marker_name} -> "
          f"pos={entry_dict['position']} rpy={entry_dict['rpy_deg']}"
        )
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
  """
  proc = subprocess.Popen(
    list(cmd),
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
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
  proc.stdin.write(json.dumps(init_payload) + "\n")
  proc.stdin.flush()

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
    for raw_line in proc.stdout:
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
        if event_type == MENU_SELECT_EVENT_TYPE
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
          }
          proc.stdin.write(json.dumps(reply) + "\n")
          proc.stdin.flush()
          print(
            f"[{status}] {marker_name} -> "
            f"pos={entry_dict['position']} rpy={entry_dict['rpy_deg']}"
          )
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

  for label in NODE_MENU_LABELS:
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

  def _stdin_reader() -> None:
    for raw_line in sys.stdin:
      line = raw_line.strip()
      if not line:
        continue
      cmd = json.loads(line)
      if cmd.get("cmd") == "set_pose":
        pose_src = cmd.get("marker_pose") or cmd["entry"]
        pose = Pose()
        pose.position.x = float(pose_src["position"][0])
        pose.position.y = float(pose_src["position"][1])
        pose.position.z = float(pose_src["position"][2])
        pose.orientation.x = float(pose_src["orientation"][0])
        pose.orientation.y = float(pose_src["orientation"][1])
        pose.orientation.z = float(pose_src["orientation"][2])
        pose.orientation.w = float(pose_src["orientation"][3])
        server.setPose(str(cmd["marker_name"]), pose)
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
