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

"""RViz 6-DoF InteractiveMarker frame & transform editor for ObjectWorldUpdates.

Publishes an InteractiveMarkerServer (default namespace `/omts_frame_editor`)
with 6-DoF translation/rotation controls and a right-click context menu for
every `create_frame` and `update_transform` entry in the configured `.pbtxt`
files. Dragging a marker in RViz streams live pose updates to the SBL
`ObjectWorld` (so `/tf` updates in real time in RViz), and releasing the mouse
automatically persists the updated pose to the source `.updates.pbtxt` file.
"""

import argparse
import os
import shutil
import subprocess
from collections.abc import Sequence

from intrinsic.solutions import deployments

from src.utils.world_utils import (
  connect_initial_world,
  find_file,
)
from tools.world.apply_scene_updates import (
  DEFAULT_UPDATE_FILES,
  apply_pbtxt_file,
)
from tools.world.pbtxt_pose_store import (
  DEFAULT_PBTXT_HEADER,
  Pose3Tuple,
  PoseEntry,
  add_frame_to_pbtxt,
  apply_pose_entry_to_world,
  load_pose_entries,
  load_pose_entries_from_file,
  resolve_safe_pbtxt_path,
  snap_entry_to_tool_pose,
  update_pose_in_pbtxt,
)
from tools.world.rviz_marker_controller import RvizFrameEditorController
from tools.world.rviz_marker_node import (
  AXIS_CONTROL_SPECS,
  MENU_LABELS,
  ControlInteractionMode,
  ControlOrientationMode,
  FeedbackEventType,
  MenuEntryId,
  build_6dof_interactive_marker,
  build_marker_from_dict,
  run_inprocess_ros2_node,
  run_subprocess_ros2_node,
)

_connect_initial_world = connect_initial_world

__all__ = [
  "AXIS_CONTROL_SPECS",
  "DEFAULT_PBTXT_HEADER",
  "MENU_LABELS",
  "ControlInteractionMode",
  "ControlOrientationMode",
  "FeedbackEventType",
  "MenuEntryId",
  "Pose3Tuple",
  "PoseEntry",
  "RvizFrameEditorController",
  "_configure_zenoh_env",
  "_resolve_ros2_python_command",
  "add_frame_to_pbtxt",
  "apply_pose_entry_to_world",
  "build_6dof_interactive_marker",
  "build_marker_from_dict",
  "connect_initial_world",
  "load_pose_entries",
  "load_pose_entries_from_file",
  "main",
  "parse_args",
  "resolve_safe_pbtxt_path",
  "run_ros2_interactive_marker_loop",
  "snap_entry_to_tool_pose",
  "update_pose_in_pbtxt",
]


def _configure_zenoh_env(zenoh_endpoint: str) -> None:
  """Configures `ZENOH_CONFIG_OVERRIDE` for ROS 2 `rmw_zenoh_cpp` if unset."""
  if not zenoh_endpoint:
    return
  os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
  os.environ.setdefault(
    "ZENOH_CONFIG_OVERRIDE",
    f'mode="client";connect/endpoints=["{zenoh_endpoint}"]',
  )


def _resolve_ros2_python_command(ros_python: str = "python3") -> list[str]:
  """Resolves the command to run `rviz_marker_node.py` in the active ROS 2 env.

  Args:
    ros_python: Python executable name or path from the active ROS 2
      environment (default: `"python3"` on `PATH`).

  Returns:
    Command list `[python_bin, "-u", node_script_path]`.

  Raises:
    RuntimeError: If `ros_python` cannot be found or cannot import `rclpy` and
      `interactive_markers`.
  """
  python_bin = shutil.which(ros_python)
  if python_bin is None and os.path.exists(ros_python):
    python_bin = os.path.abspath(ros_python)

  if not python_bin:
    raise RuntimeError(
      f"ROS 2 Python executable {ros_python!r} was not found on PATH. "
      "Ensure ROS 2 is properly installed and sourced in your shell before "
      "running edit_frames_rviz."
    )

  check_proc = subprocess.run(
    [python_bin, "-c", "import rclpy, interactive_markers"],
    check=False,
    capture_output=True,
    text=True,
  )
  if check_proc.returncode != 0:
    stderr_msg = (check_proc.stderr or "").strip()
    detail = f" ({stderr_msg})" if stderr_msg else ""
    raise RuntimeError(
      f"Python interpreter {python_bin!r} could not import `rclpy` and "
      f"`interactive_markers`{detail}. Ensure ROS 2 with `rclpy` and "
      "`interactive_markers` is properly installed and sourced before running "
      "edit_frames_rviz."
    )

  node_script_path = find_file("tools/world/rviz_marker_node.py")
  if not os.path.exists(node_script_path):
    node_script_path = os.path.join(
      os.path.dirname(os.path.abspath(__file__)), "rviz_marker_node.py"
    )

  return [python_bin, "-u", os.path.abspath(node_script_path)]


def run_ros2_interactive_marker_loop(
  controller: RvizFrameEditorController,
  topic_ns: str = "omts_frame_editor",
  zenoh_endpoint: str = "tcp/127.0.0.1:7447",
  ros_python: str = "python3",
) -> None:
  """Runs the ROS 2 `InteractiveMarkerServer` event loop.

  Runs `run_inprocess_ros2_node` directly when `rclpy` and `interactive_markers`
  are importable in the current interpreter, or spawns `rviz_marker_node.py`
  using `ros_python` from the active ROS 2 shell environment when invoked via
  Bazel's hermetic Python toolchain.

  Args:
    controller: `RvizFrameEditorController` instance managing pose entries.
    topic_ns: InteractiveMarkerServer topic namespace.
    zenoh_endpoint: Zenoh router endpoint when using `rmw_zenoh_cpp`.
    ros_python: Python executable from the active ROS 2 environment.
  """
  _configure_zenoh_env(zenoh_endpoint)
  try:
    import interactive_markers.interactive_marker_server  # noqa: F401 # type: ignore[import-not-found]
    import rclpy  # noqa: F401 # type: ignore[import-not-found]

    in_process_available = True
  except Exception:  # pylint: disable=broad-exception-caught
    in_process_available = False

  if in_process_available:
    run_inprocess_ros2_node(controller=controller, topic_ns=topic_ns)
  else:
    cmd = _resolve_ros2_python_command(ros_python=ros_python)
    run_subprocess_ros2_node(
      controller=controller,
      topic_ns=topic_ns,
      cmd=cmd,
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments for `edit_frames_rviz`."""
  parser = argparse.ArgumentParser(
    description=(
      "Launch an RViz 6-DoF InteractiveMarker server for editing "
      "ObjectWorldUpdates (.pbtxt) frames and transforms directly in RViz."
    )
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution gRPC address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
    "--files",
    nargs="*",
    default=DEFAULT_UPDATE_FILES,
    help="Path(s) to .pbtxt ObjectWorldUpdates files to load and edit.",
  )
  parser.add_argument(
    "--topic_ns",
    type=str,
    default="omts_frame_editor",
    help=(
      "InteractiveMarkerServer topic namespace (subscribe RViz "
      "InteractiveMarkers to /<topic_ns>/update)."
    ),
  )
  parser.add_argument(
    "--fixed_frame",
    type=str,
    default="root",
    help="RViz Fixed Frame to anchor all InteractiveMarkers in (default: root).",
  )
  parser.add_argument(
    "--marker_scale",
    type=float,
    default=0.18,
    help="Scale of the 6-DoF interactive markers in meters (default: 0.18).",
  )
  parser.add_argument(
    "--fixed_orientation",
    action=argparse.BooleanOptionalAction,
    default=False,
    help=(
      "Use parent-fixed control axes (FIXED) instead of local frame axes "
      "(INHERIT)."
    ),
  )
  parser.add_argument(
    "--zenoh_endpoint",
    type=str,
    default="tcp/127.0.0.1:7447",
    help="Zenoh router endpoint when using rmw_zenoh_cpp.",
  )
  parser.add_argument(
    "--ros_python",
    type=str,
    default="python3",
    help=(
      "Python executable from the active ROS 2 environment "
      "(default: python3 on PATH)."
    ),
  )
  parser.add_argument(
    "--offline",
    action=argparse.BooleanOptionalAction,
    default=False,
    help=(
      "Run in offline mode (.pbtxt editing only, without connecting to a "
      "solution)."
    ),
  )
  parser.add_argument(
    "--update_init_world",
    action=argparse.BooleanOptionalAction,
    default=True,
    help=(
      "Also apply updates to 'init_world' so they survive world resets "
      "(default: True)."
    ),
  )
  parser.add_argument(
    "--apply_on_start",
    action=argparse.BooleanOptionalAction,
    default=True,
    help=(
      "Apply all configured .pbtxt files to the live world on startup "
      "(default: True)."
    ),
  )
  parser.add_argument(
    "--reset_sim_on_release",
    action=argparse.BooleanOptionalAction,
    default=False,
    help=(
      "Automatically reset the Gazebo simulation on mouse release when "
      "editing transforms."
    ),
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  """Entrypoint for the RViz 6-DoF InteractiveMarker frame editor."""
  args = parse_args(argv)
  solution = None
  world = None
  init_world = None

  if not args.offline:
    try:
      print(f"Connecting to solution at {args.address}...")
      solution = deployments.connect(address=args.address)
      world = solution.world
      if args.update_init_world:
        init_world = connect_initial_world(solution)
      if args.apply_on_start:
        for fpath in args.files:
          apply_pbtxt_file(world=world, filepath=fpath, init_world=init_world)
    except Exception as e:  # pylint: disable=broad-exception-caught
      print(
        f"[-] Warning: Could not connect to {args.address} ({e}). "
        "Continuing in offline (.pbtxt only) mode."
      )
      solution = None
      world = None
      init_world = None

  controller = RvizFrameEditorController(
    files=args.files,
    world=world,
    init_world=init_world,
    solution=solution,
    marker_scale=args.marker_scale,
    default_fixed_orientation=args.fixed_orientation,
    reset_sim_on_release=args.reset_sim_on_release,
    fixed_frame=args.fixed_frame,
  )
  run_ros2_interactive_marker_loop(
    controller=controller,
    topic_ns=args.topic_ns,
    zenoh_endpoint=args.zenoh_endpoint,
    ros_python=args.ros_python,
  )


if __name__ == "__main__":
  main()
