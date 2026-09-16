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

"""Stores the robot tool frame pose as a frame in scene.updates.pbtxt."""

import argparse
import os
from collections.abc import Sequence
from typing import Any

from google.protobuf import text_format
from intrinsic.math.python import data_types
from intrinsic.solutions import deployments
from intrinsic.world.proto import object_world_updates_pb2

from src.core.types import Pose3D


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description=(
      "Store current robot tool frame pose as a named frame in"
      " scene.updates.pbtxt."
    )
  )
  parser.add_argument(
    "name",
    nargs="?",
    default=None,
    type=str,
    help=(
      "Name of the frame to store/overwrite (e.g. 'view', 'infeed_grasp',"
      " 'infeed_pre_grasp')."
    ),
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution gRPC address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
    "--parent_object",
    type=str,
    default="root",
    help=(
      "Parent object name in SBL world for the stored frame (default: 'root')."
    ),
  )
  parser.add_argument(
    "--tool_object",
    type=str,
    default="gripper",
    help="Tool object name in SBL world (default: 'gripper').",
  )
  parser.add_argument(
    "--tool_frame",
    type=str,
    default="tool_frame",
    help=(
      "Tool frame name on tool_object in SBL world (default: 'tool_frame')."
    ),
  )
  parser.add_argument(
    "--scene_updates_file",
    type=str,
    default="configs/omts/scene.updates.pbtxt",
    help="Path to scene.updates.pbtxt file to store frames in.",
  )
  return parser.parse_args(argv)


def find_scene_updates_file(filepath: str) -> str:
  """Resolves path to the scene updates file in workspace or runfiles."""
  if os.path.isabs(filepath) and os.path.exists(filepath):
    return filepath

  for env_key in ("BUILD_WORKING_DIRECTORY", "BUILD_WORKSPACE_DIRECTORY"):
    dir_path = os.environ.get(env_key)
    if dir_path:
      ws_file = os.path.join(dir_path, filepath)
      if os.path.exists(ws_file) or os.path.exists(os.path.dirname(ws_file)):
        return ws_file

  runfiles_dir = os.environ.get("PYTHON_RUNFILES") or os.environ.get(
    "TEST_SRCDIR"
  )
  if runfiles_dir:
    r_path = os.path.join(runfiles_dir, "_main", filepath)
    if os.path.exists(r_path):
      return r_path

  return filepath


def get_current_tool_pose(
  world: Any,
  parent_object_name: str = "root",
  tool_object_name: str = "gripper",
  tool_frame_name: str = "tool_frame",
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
  """Retrieves the current tool frame pose relative to parent object."""
  parent_obj = getattr(world, parent_object_name)
  tool_obj = getattr(world, tool_object_name)
  tool_frame = getattr(tool_obj, tool_frame_name)
  transform = world.get_transform(parent_obj, tool_frame)
  pose = Pose3D.from_proto(transform)
  return pose.position, pose.orientation


def save_frame_to_scene_updates(
  frame_name: str,
  position: tuple[float, float, float],
  orientation: tuple[float, float, float, float],
  parent_object_name: str = "root",
  filepath: str = "configs/omts/scene.updates.pbtxt",
) -> str:
  """Saves or updates a frame's pose in the specified scene.updates.pbtxt file."""
  resolved_path = find_scene_updates_file(filepath)

  updates = object_world_updates_pb2.ObjectWorldUpdates()
  if os.path.exists(resolved_path):
    with open(resolved_path, encoding="utf-8") as f:
      content = f.read()
      if content.strip():
        text_format.Parse(content, updates)

  rounded_pose = Pose3D(
    x=round(position[0], 4),
    y=round(position[1], 4),
    z=round(position[2], 4),
    qx=round(orientation[0], 4),
    qy=round(orientation[1], 4),
    qz=round(orientation[2], 4),
    qw=round(orientation[3], 4),
  ).to_proto()

  found = False
  for update in updates.updates:
    if update.HasField("create_frame"):
      cf = update.create_frame
      parent_name = (
        cf.parent_object_with_filter.reference.by_name.object_name or "root"
      )
      if cf.new_frame_name == frame_name and parent_name == parent_object_name:
        cf.parent_t_new_frame.CopyFrom(rounded_pose)
        found = True
        break

  if not found:
    new_update = updates.updates.add()
    cf = new_update.create_frame
    cf.parent_object_with_filter.reference.by_name.object_name = (
      parent_object_name
    )
    cf.new_frame_name = frame_name
    cf.parent_t_new_frame.CopyFrom(rounded_pose)

  header = (
    "# proto-file: intrinsic/world/public/proto/object_world_updates.proto\n"
    "# proto-message: intrinsic_proto.world.ObjectWorldUpdates\n\n"
  )
  body = text_format.MessageToString(updates)

  with open(resolved_path, "w", encoding="utf-8") as f:
    f.write(header + body)

  return resolved_path


def update_live_world_frame(
  world: Any,
  frame_name: str,
  position: tuple[float, float, float],
  orientation: tuple[float, float, float, float],
  parent_object_name: str = "root",
) -> None:
  """Updates or creates the frame in the live connected SBL ObjectWorld."""
  try:
    parent_obj = getattr(world, parent_object_name, None)
    if parent_obj is None:
      return

    pose = data_types.Pose3(
      data_types.Rotation3(
        data_types.Quaternion(
          [
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
          ]
        )
      ),
      [position[0], position[1], position[2]],
    )

    frame_exists = False
    if hasattr(parent_obj, "list_frames"):
      frame_exists = frame_name in parent_obj.list_frames()
    elif hasattr(parent_obj, frame_name):
      frame_exists = True

    if frame_exists:
      frame_node = getattr(parent_obj, frame_name)
      world.update_transform(node_a=parent_obj, node_b=frame_node, a_t_b=pose)
      print(
        f"Updated live frame '{parent_object_name}.{frame_name}' in solution"
        " world."
      )
    else:
      new_updates = object_world_updates_pb2.ObjectWorldUpdates()
      up = new_updates.updates.add()
      cf = up.create_frame
      cf.parent_object_with_filter.reference.by_name.object_name = (
        parent_object_name
      )
      cf.new_frame_name = frame_name
      cf.parent_t_new_frame.CopyFrom(
        Pose3D(
          x=position[0],
          y=position[1],
          z=position[2],
          qx=orientation[0],
          qy=orientation[1],
          qz=orientation[2],
          qw=orientation[3],
        ).to_proto()
      )
      world.batch_update(new_updates)
      print(
        f"Created live frame '{parent_object_name}.{frame_name}' in solution"
        " world."
      )
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f"Note: Could not update live world directly: {e}")


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)

  frame_name = args.name
  if not frame_name:
    frame_name = input(
      "Enter frame name to store/overwrite"
      " (e.g. 'view', 'infeed_grasp', 'infeed_pre_grasp'): "
    ).strip()
    if not frame_name:
      print("Error: A frame name must be specified.")
      return

  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  print(
    f"Fetching transform for '{args.tool_object}.{args.tool_frame}' in"
    f" '{args.parent_object}'..."
  )
  pos, ori = get_current_tool_pose(
    world=world,
    parent_object_name=args.parent_object,
    tool_object_name=args.tool_object,
    tool_frame_name=args.tool_frame,
  )

  print(
    f"Captured tool pose in '{args.parent_object}':\n"
    f"  Position (xyz):    [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]\n"
    f"  Orientation (xyzw): [{ori[0]:.4f}, {ori[1]:.4f}, {ori[2]:.4f},"
    f" {ori[3]:.4f}]"
  )

  saved_path = save_frame_to_scene_updates(
    frame_name=frame_name,
    position=pos,
    orientation=ori,
    parent_object_name=args.parent_object,
    filepath=args.scene_updates_file,
  )
  print(f"[✓] Successfully saved frame '{frame_name}' to {saved_path}")

  update_live_world_frame(
    world=world,
    frame_name=frame_name,
    position=pos,
    orientation=ori,
    parent_object_name=args.parent_object,
  )


if __name__ == "__main__":
  main()
