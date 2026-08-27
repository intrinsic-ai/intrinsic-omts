"""CLI utility to load and apply ObjectWorldUpdates (.pbtxt) live to a running solution."""

import argparse
import os
from typing import Any, Sequence

from google.protobuf import text_format
from intrinsic.solutions import deployments
from intrinsic.world.public.proto import object_world_refs_pb2
from intrinsic.world.public.proto import object_world_updates_pb2

DEFAULT_UPDATE_FILES = [
    "configs/ur_module.attachments.updates.pbtxt",
    "configs/scene.updates.pbtxt",
    "configs/align_robot.updates.pbtxt",
]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
      description="Apply ObjectWorldUpdates (.pbtxt) live to a running solution deployment."
  )
  parser.add_argument(
      "--address",
      type=str,
      default="localhost:17080",
      help="Solution address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
      "--files",
      nargs="*",
      default=DEFAULT_UPDATE_FILES,
      help="Path(s) to .pbtxt ObjectWorldUpdates files to apply in order.",
  )
  return parser.parse_args(argv)


def find_file(filepath: str) -> str:
  """Resolves file path in direct directory, workspace, or runfiles."""
  if os.path.exists(filepath):
    return filepath
  ws_path = os.path.join("/usr/local/google/home/mschweiger/workspaces/omts", filepath)
  if os.path.exists(ws_path):
    return ws_path
  runfiles_dir = os.environ.get("PYTHON_RUNFILES") or os.environ.get("TEST_SRCDIR")
  if runfiles_dir:
    r_path = os.path.join(runfiles_dir, "_main", filepath)
    if os.path.exists(r_path):
      return r_path
  return filepath


def adapt_updates_for_live_world(
    world: Any, updates: object_world_updates_pb2.ObjectWorldUpdates
) -> object_world_updates_pb2.ObjectWorldUpdates:
  """Converts create_frame requests into update_transform requests if frames already exist."""
  adapted = object_world_updates_pb2.ObjectWorldUpdates()

  for update in updates.updates:
    if update.HasField("create_frame"):
      cf = update.create_frame
      parent_name = "root"
      if cf.parent_object_with_filter.reference.by_name.object_name:
        parent_name = cf.parent_object_with_filter.reference.by_name.object_name

      frame_name = cf.new_frame_name

      # Check if parent object and frame already exist in world
      parent_obj = getattr(world, parent_name, None)
      frame_exists = False
      if parent_obj is not None:
        if hasattr(parent_obj, "list_frames") and frame_name in parent_obj.list_frames():
          frame_exists = True
        elif hasattr(parent_obj, frame_name):
          frame_exists = True

      if frame_exists:
        # Frame already exists; convert create_frame to update_transform
        new_up = adapted.updates.add()
        ut = new_up.update_transform
        ut.node_a.by_name.object.object_name = parent_name
        ut.node_b.by_name.frame.object_name = parent_name
        ut.node_b.by_name.frame.frame_name = frame_name
        ut.node_to_update.by_name.frame.object_name = parent_name
        ut.node_to_update.by_name.frame.frame_name = frame_name
        ut.a_t_b.CopyFrom(cf.parent_t_new_frame)
      else:
        # New frame; keep create_frame
        adapted.updates.add().CopyFrom(update)
    else:
      adapted.updates.add().CopyFrom(update)

  return adapted


def apply_pbtxt_file(world: Any, filepath: str) -> None:
  """Loads a .pbtxt file and pushes its updates to the active ObjectWorld."""
  resolved_path = find_file(filepath)
  if not os.path.exists(resolved_path):
    print(f"[-] Warning: File not found: {filepath} (resolved: {resolved_path})")
    return

  print(f"[+] Reading update file: {filepath}")
  with open(resolved_path, "r", encoding="utf-8") as f:
    pbtxt_content = f.read()

  raw_updates = object_world_updates_pb2.ObjectWorldUpdates()
  text_format.Parse(pbtxt_content, raw_updates)

  adapted_updates = adapt_updates_for_live_world(world=world, updates=raw_updates)

  print(f"    Applying {len(adapted_updates.updates)} update rule(s) to live world...")
  world.batch_update(adapted_updates)
  print(f"[✓] Successfully applied: {filepath}")


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  print(f"\n=== Applying {len(args.files)} World Update File(s) Live ===")
  for fpath in args.files:
    apply_pbtxt_file(world=world, filepath=fpath)

  print("\n=== Current Active World State Verification ===")
  try:
    if hasattr(world, "root"):
      print("Frames on 'root':")
      if hasattr(world.root, "list_frames"):
        for f in world.root.list_frames():
          print(f"  - {f}: {world.get_transform(world.root, getattr(world.root, f))}")

    if hasattr(world, "ur_module") and hasattr(world.ur_module, "flange"):
      print(
          f"Flange in root: {world.get_transform(world.root, world.ur_module.flange)}"
      )
    if hasattr(world, "gripper") and hasattr(world.gripper, "tool_frame"):
      print(
          "Tool Frame in root:"
          f" {world.get_transform(world.root, world.gripper.tool_frame)}"
      )
  except Exception as e:
    print(f"Transform query error: {e}")

  print("\n[✓] Live world updates complete.")


if __name__ == "__main__":
  main()
