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
import re
import shutil
import subprocess
import sys
from collections.abc import Sequence

_PYTHON_VERSION_RE = re.compile(r"(?:^|[/\\])python(\d+\.\d+)(?:$|[/\\])")


def _sanitize_sys_path() -> None:
  """Filters mismatched `pythonX.Y` paths and demotes external `PYTHONPATH` entries.

  When `bazel run //tools/world:edit_frames_rviz` is invoked from an activated
  ROS 2 / Pixi / Conda shell, `PYTHONPATH` entries from that shell (often built
  for a different Python minor version such as 3.12) can be placed at the front
  of `sys.path` ahead of Bazel's `.runfiles` site-packages by the `.venv`
  interpreter startup, causing C-extension ABI errors (e.g. when importing
  `numpy`). This function removes `sys.path` entries targeting a different
  `pythonX.Y` version and moves external `PYTHONPATH` entries after internal
  interpreter / `.runfiles` paths, while leaving `os.environ["PYTHONPATH"]`
  intact for the child `rviz_marker_node.py` ROS 2 process.
  """
  current_ver = f"{sys.version_info.major}.{sys.version_info.minor}"
  raw_pythonpath = os.environ.get("PYTHONPATH", "")
  pythonpath_entries = {
    norm_fn(p)
    for p in raw_pythonpath.split(os.pathsep)
    if p
    for norm_fn in (os.path.normpath, os.path.realpath)
  }
  internal_prefixes = tuple(
    norm_fn(p)
    for p in (sys.prefix, sys.base_prefix)
    if p
    for norm_fn in (os.path.normpath, os.path.realpath)
  )
  in_bazel_runfiles = ".runfiles" in os.path.abspath(__file__)

  primary_paths: list[str] = []
  demoted_paths: list[str] = []
  for entry in sys.path:
    match = _PYTHON_VERSION_RE.search(entry)
    if match and match.group(1) != current_ver:
      continue
    norm_entry = os.path.normpath(entry) if entry else ""
    real_entry = os.path.realpath(entry) if entry else ""
    is_internal = (
      ".runfiles" in norm_entry
      or ".runfiles" in real_entry
      or (
        bool(norm_entry)
        and (
          norm_entry.startswith(internal_prefixes)
          or real_entry.startswith(internal_prefixes)
        )
      )
    )
    is_external_pythonpath = (
      norm_entry in pythonpath_entries
      or real_entry in pythonpath_entries
      or (in_bazel_runfiles and not is_internal)
    )
    if is_external_pythonpath and not is_internal:
      demoted_paths.append(entry)
    else:
      primary_paths.append(entry)

  sys.path[:] = primary_paths + demoted_paths


_sanitize_sys_path()

from intrinsic.solutions import deployments  # noqa: E402

from src.utils.world_utils import (  # noqa: E402
  connect_initial_world,
  find_file,
)
from tools.world.apply_scene_updates import (  # noqa: E402
  DEFAULT_UPDATE_FILES,
  apply_pbtxt_file,
  discover_setup_update_files,
)
from tools.world.pbtxt_pose_store import (  # noqa: E402
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
from tools.world.rviz_marker_controller import (  # noqa: E402
  RvizFrameEditorController,
)
from tools.world.rviz_marker_node import (  # noqa: E402
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
  "DEFAULT_UPDATE_FILES",
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
  "_sanitize_sys_path",
  "add_frame_to_pbtxt",
  "apply_pose_entry_to_world",
  "build_6dof_interactive_marker",
  "build_marker_from_dict",
  "connect_initial_world",
  "discover_setup_update_files",
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
    "--setup",
    type=str,
    default="omts",
    help=(
      "Cell setup directory under configs/<setup>/ whose *.updates.pbtxt "
      "files to load when --files is omitted (default: omts)."
    ),
  )
  parser.add_argument(
    "--files",
    nargs="*",
    default=None,
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
  args = parser.parse_args(argv)
  if args.files is None:
    args.files = discover_setup_update_files(args.setup)
  return args


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
    except Exception as e:  # pylint: disable=broad-exception-caught
      print(
        f"[-] Warning: Could not connect to {args.address} ({e}). "
        "Continuing in offline (.pbtxt only) mode."
      )
      solution = None
      world = None
      init_world = None

    if world is not None and args.apply_on_start:
      for fpath in args.files:
        try:
          apply_pbtxt_file(world=world, filepath=fpath, init_world=init_world)
        except Exception as e:  # pylint: disable=broad-exception-caught
          print(f"[-] Warning: Failed to apply {fpath} on startup ({e}).")

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
