"""Stores the robot tool frame pose as a frame in scene.updates.pbtxt."""

import argparse
import os
from typing import Any, Sequence

from google.protobuf import text_format
from intrinsic.math.python import data_types
from intrinsic.solutions import deployments
from intrinsic.world.public.proto import object_world_updates_pb2


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
          "Parent object name in SBL world for the stored frame (default:"
          " 'root')."
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
      default="configs/scene.updates.pbtxt",
      help="Path to scene.updates.pbtxt file to store frames in.",
  )
  return parser.parse_args(argv)


def find_scene_updates_file(filepath: str) -> str:
  """Resolves path to the scene updates file in workspace or runfiles."""
  if os.path.isabs(filepath) and os.path.exists(filepath):
    return filepath

  # Check BUILD_WORKING_DIRECTORY first if invoked via 'bazel run'
  working_dir = os.environ.get("BUILD_WORKING_DIRECTORY")
  if working_dir:
    ws_file = os.path.join(working_dir, filepath)
    if os.path.exists(ws_file) or os.path.exists(os.path.dirname(ws_file)):
      return ws_file

  # Check BUILD_WORKSPACE_DIRECTORY
  workspace_dir = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
  if workspace_dir:
    ws_file = os.path.join(workspace_dir, filepath)
    if os.path.exists(ws_file) or os.path.exists(os.path.dirname(ws_file)):
      return ws_file

  # Check runfiles
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
  """Retrieves the current tool frame pose relative to parent object.

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

  # Extract translation
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

  # Extract quaternion
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

  return (pos_x, pos_y, pos_z), (ori_x, ori_y, ori_z, ori_w)


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

  updates = object_world_updates_pb2.ObjectWorldUpdates()
  if os.path.exists(resolved_path):
    with open(resolved_path, "r", encoding="utf-8") as f:
      content = f.read()
      if content.strip():
        text_format.Parse(content, updates)

  # Check if frame already exists in updates
  found = False
  for update in updates.updates:
    if update.HasField("create_frame"):
      cf = update.create_frame
      parent_name = "root"
      if cf.parent_object_with_filter.reference.by_name.object_name:
        parent_name = (
            cf.parent_object_with_filter.reference.by_name.object_name
        )
      if cf.new_frame_name == frame_name and parent_name == parent_object_name:
        cf.parent_t_new_frame.position.x = round(position[0], 4)
        cf.parent_t_new_frame.position.y = round(position[1], 4)
        cf.parent_t_new_frame.position.z = round(position[2], 4)
        cf.parent_t_new_frame.orientation.x = round(orientation[0], 4)
        cf.parent_t_new_frame.orientation.y = round(orientation[1], 4)
        cf.parent_t_new_frame.orientation.z = round(orientation[2], 4)
        cf.parent_t_new_frame.orientation.w = round(orientation[3], 4)
        found = True
        break

  if not found:
    new_update = updates.updates.add()
    cf = new_update.create_frame
    cf.parent_object_with_filter.reference.by_name.object_name = (
        parent_object_name
    )
    cf.new_frame_name = frame_name
    cf.parent_t_new_frame.position.x = round(position[0], 4)
    cf.parent_t_new_frame.position.y = round(position[1], 4)
    cf.parent_t_new_frame.position.z = round(position[2], 4)
    cf.parent_t_new_frame.orientation.x = round(orientation[0], 4)
    cf.parent_t_new_frame.orientation.y = round(orientation[1], 4)
    cf.parent_t_new_frame.orientation.z = round(orientation[2], 4)
    cf.parent_t_new_frame.orientation.w = round(orientation[3], 4)

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
            data_types.Quaternion([
                orientation[0],
                orientation[1],
                orientation[2],
                orientation[3],
            ])
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
      cf.parent_t_new_frame.position.x = position[0]
      cf.parent_t_new_frame.position.y = position[1]
      cf.parent_t_new_frame.position.z = position[2]
      cf.parent_t_new_frame.orientation.x = orientation[0]
      cf.parent_t_new_frame.orientation.y = orientation[1]
      cf.parent_t_new_frame.orientation.z = orientation[2]
      cf.parent_t_new_frame.orientation.w = orientation[3]
      world.batch_update(new_updates)
      print(
          f"Created live frame '{parent_object_name}.{frame_name}' in solution"
          " world."
      )
  except Exception as e:
    print(f"Note: Could not update live world directly: {e}")


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)

  frame_name = args.name
  if not frame_name:
    frame_name = input(
        "Enter frame name to store/overwrite"
        " (e.g. 'view', 'grasp', 'pre_grasp'): "
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
