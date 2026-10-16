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

"""Script to store the current robot tool frame pose in scene.updates.pbtxt."""

import argparse
import os
from collections.abc import Sequence
from typing import Any

from google.protobuf import text_format
from intrinsic.math.python import data_types, proto_conversion
from intrinsic.solutions import deployments
from intrinsic.world.proto import object_world_updates_pb2

from src.utils.world_utils import find_file
from tools.world.pbtxt_pose_store import (
  DEFAULT_PBTXT_HEADER,
  Pose3Tuple,
  _extract_header_comments,
  _extract_pose_from_transform,
  _replace_pose_in_pbtxt_content,
)

find_scene_updates_file = find_file


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
      "Name of the frame to store/overwrite (e.g. 'view', 'grasp',"
      " 'pre_grasp')."
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
    help="Tool frame name on tool_object in SBL world (default: 'tool_frame').",
  )
  parser.add_argument(
    "--scene_updates_file",
    type=str,
    default="configs/omts/scene.updates.pbtxt",
    help="Path to scene.updates.pbtxt file to store frames in.",
  )
  return parser.parse_args(argv)


def get_current_tool_pose(
  world: Any,
  parent_object_name: str = "root",
  tool_object_name: str = "gripper",
  tool_frame_name: str = "tool_frame",
) -> Pose3Tuple:
  """Retrieves (position, orientation) of tool frame relative to parent object.

  Args:
    world: SBL ObjectWorld instance.
    parent_object_name: Parent object name in world (default: 'root').
    tool_object_name: Tool object name in world (default: 'gripper').
    tool_frame_name: Tool frame name in world (default: 'tool_frame').

  Returns:
    Tuple of ((x, y, z), (qx, qy, qz, qw)).
  """
  parent_obj = getattr(world, parent_object_name)
  tool_obj = getattr(world, tool_object_name)
  tool_frame = getattr(tool_obj, tool_frame_name)
  transform = world.get_transform(parent_obj, tool_frame)
  return _extract_pose_from_transform(transform)


def save_frame_to_scene_updates(
  frame_name: str,
  position: tuple[float, float, float],
  orientation: tuple[float, float, float, float],
  parent_object_name: str = "root",
  filepath: str = "configs/scene.updates.pbtxt",
) -> str:
  """Saves or updates a frame's pose in the specified scene.updates.pbtxt file.

  Args:
    frame_name: Name of the frame (e.g. 'view', 'grasp').
    position: (x, y, z) position in meters.
    orientation: (x, y, z, w) quaternion orientation.
    parent_object_name: Parent object name (default: 'root').
    filepath: Relative or absolute path to the .pbtxt file.

  Returns:
    The resolved path where the updates were written.
  """
  resolved_path = find_scene_updates_file(filepath)

  content = ""
  updates = object_world_updates_pb2.ObjectWorldUpdates()
  if os.path.exists(resolved_path):
    with open(resolved_path, encoding="utf-8") as f:
      content = f.read()
      if content.strip():
        text_format.Parse(content, updates)

  rounded_pos = (
    round(float(position[0]), 4),
    round(float(position[1]), 4),
    round(float(position[2]), 4),
  )
  ori_tuple = (
    float(orientation[0]),
    float(orientation[1]),
    float(orientation[2]),
    float(orientation[3]),
  )
  quat_proto = proto_conversion.quaternion_to_proto(
    data_types.Quaternion(list(ori_tuple))
  )

  found_index: int | None = None
  for idx, update in enumerate(updates.updates):
    if update.HasField("create_frame"):
      cf = update.create_frame
      parent_name = (
        cf.parent_object_with_filter.reference.by_name.object_name or "root"
      )
      if cf.new_frame_name == frame_name and parent_name == parent_object_name:
        cf.parent_t_new_frame.position.x = rounded_pos[0]
        cf.parent_t_new_frame.position.y = rounded_pos[1]
        cf.parent_t_new_frame.position.z = rounded_pos[2]
        cf.parent_t_new_frame.orientation.CopyFrom(quat_proto)
        found_index = idx
        break

  new_content: str | None = None
  if found_index is not None:
    new_content = _replace_pose_in_pbtxt_content(
      content=content,
      update_index=found_index,
      total_updates=len(updates.updates),
      position=rounded_pos,
      orientation=ori_tuple,
    )
  else:
    new_update = updates.updates.add()
    cf = new_update.create_frame
    cf.parent_object_with_filter.reference.by_name.object_name = (
      parent_object_name
    )
    cf.new_frame_name = frame_name
    cf.parent_t_new_frame.position.x = rounded_pos[0]
    cf.parent_t_new_frame.position.y = rounded_pos[1]
    cf.parent_t_new_frame.position.z = rounded_pos[2]
    cf.parent_t_new_frame.orientation.CopyFrom(quat_proto)

  if new_content is None:
    header = (
      _extract_header_comments(content)
      if content.strip()
      else DEFAULT_PBTXT_HEADER
    )
    new_content = header + text_format.MessageToString(updates)

  with open(resolved_path, "w", encoding="utf-8") as f:
    f.write(new_content)

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
      rotation=data_types.Rotation3.from_xyzw(
        [
          float(orientation[0]),
          float(orientation[1]),
          float(orientation[2]),
          float(orientation[3]),
        ],
        normalize=True,
      ),
      translation=[float(position[0]), float(position[1]), float(position[2])],
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
      cf.parent_t_new_frame.CopyFrom(proto_conversion.pose_to_proto(pose))
      world.batch_update(new_updates)
      print(
        f"Created live frame '{parent_object_name}.{frame_name}' in solution"
        " world."
      )
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f"Note: Could not update live world directly: {e}")


def main(argv: Sequence[str] | None = None) -> None:
  """Entrypoint for the store_frame CLI utility."""
  args = parse_args(argv)

  frame_name = args.name
  if not frame_name:
    frame_name = input(
      "Enter frame name to store/overwrite (e.g. 'view', 'grasp', "
      "'pre_grasp'): "
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
    f"  Orientation (xyzw): [{ori[0]:.6f}, {ori[1]:.6f}, {ori[2]:.6f},"
    f" {ori[3]:.6f}]"
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
