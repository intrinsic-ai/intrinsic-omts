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

"""CLI utility to load and apply ObjectWorldUpdates (.pbtxt) live."""

import argparse
import logging
import os
from collections.abc import Sequence
from typing import Any

from google.protobuf import text_format
from intrinsic.solutions import deployments
from intrinsic.world.proto import (
  object_world_updates_pb2,
)

DEFAULT_UPDATE_FILES = [
  "configs/common/ur_module.attachments.updates.pbtxt",
  "configs/omts/scene.updates.pbtxt",
  "configs/omts/align_robot.updates.pbtxt",
  "configs/omts/cnc_enclosure.updates.pbtxt",
  "configs/omts/schunk.updates.pbtxt",
  "configs/omts/camera_mount.updates.pbtxt",
  "configs/omts/orbbec_gemini.updates.pbtxt",
]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description=(
      "Apply ObjectWorldUpdates (.pbtxt) live to a running solution deployment."
    )
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
  parser.add_argument(
    "--reset_sim",
    action=argparse.BooleanOptionalAction,
    default=True,
    help=(
      "Whether to reset simulation to synchronize sim_world and reload Gazebo"
      " when connected to a simulated solution (default: True)."
    ),
  )
  return parser.parse_args(argv)


def find_file(filepath: str) -> str:
  """Resolves file path in direct directory, workspace, or runfiles."""
  if os.path.isabs(filepath) and os.path.exists(filepath):
    return filepath

  # Check BUILD_WORKING_DIRECTORY first if invoked via 'bazel run'
  working_dir = os.environ.get("BUILD_WORKING_DIRECTORY")
  if working_dir:
    w_path = os.path.join(working_dir, filepath)
    if os.path.exists(w_path):
      return w_path

  # Check BUILD_WORKSPACE_DIRECTORY
  workspace_dir = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
  if workspace_dir:
    ws_path = os.path.join(workspace_dir, filepath)
    if os.path.exists(ws_path):
      return ws_path

  if os.path.exists(filepath):
    return filepath

  runfiles_dir = os.environ.get("PYTHON_RUNFILES") or os.environ.get(
    "TEST_SRCDIR"
  )
  if runfiles_dir:
    r_path = os.path.join(runfiles_dir, "_main", filepath)
    if os.path.exists(r_path):
      return r_path
    r_path2 = os.path.join(runfiles_dir, filepath)
    if os.path.exists(r_path2):
      return r_path2

  return filepath


def adapt_updates_for_live_world(
  world: Any, updates: object_world_updates_pb2.ObjectWorldUpdates
) -> object_world_updates_pb2.ObjectWorldUpdates:
  """Converts create_frame into update_transform if frames exist."""
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
        if hasattr(parent_obj, "list_frames"):
          frame_exists = frame_name in parent_obj.list_frames()
        else:
          frame_exists = hasattr(parent_obj, frame_name)

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
    elif update.HasField("update_transform"):
      ut = update.update_transform
      target_obj_name = None
      if (
        ut.node_to_update.HasField("by_name")
        and ut.node_to_update.by_name.HasField("object")
        and ut.node_to_update.by_name.object.object_name
      ):
        target_obj_name = ut.node_to_update.by_name.object.object_name
      elif (
        ut.node_b.HasField("by_name")
        and ut.node_b.by_name.HasField("object")
        and ut.node_b.by_name.object.object_name
      ):
        target_obj_name = ut.node_b.by_name.object.object_name

      if target_obj_name is not None and target_obj_name != "root":
        obj_exists = False
        if hasattr(world, "list_objects"):
          try:
            objs = world.list_objects()
            existing = {
              o
              if isinstance(o, str)
              else (getattr(o, "name", None) or getattr(o, "id", None))
              for o in objs
            }
            obj_exists = target_obj_name in existing
          except Exception:  # pylint: disable=broad-exception-caught
            obj_exists = hasattr(world, target_obj_name)
        else:
          obj_exists = hasattr(world, target_obj_name)

        if not obj_exists:
          logging.info(
            "Skipping update_transform for missing object: %s", target_obj_name
          )
          continue

      adapted.updates.add().CopyFrom(update)
    else:
      adapted.updates.add().CopyFrom(update)

  return adapted


def apply_pbtxt_file(world: Any, filepath: str) -> None:
  """Loads a .pbtxt file and pushes its updates to the active ObjectWorld."""
  resolved_path = find_file(filepath)
  if not os.path.exists(resolved_path):
    print(
      f"[-] Warning: File not found: {filepath} (resolved: {resolved_path})"
    )
    return

  print(f"[+] Reading update file: {filepath}")
  with open(resolved_path, encoding="utf-8") as f:
    pbtxt_content = f.read()

  raw_updates = object_world_updates_pb2.ObjectWorldUpdates()
  text_format.Parse(pbtxt_content, raw_updates)

  adapted_updates = adapt_updates_for_live_world(
    world=world, updates=raw_updates
  )

  print(
    f"    Applying {len(adapted_updates.updates)} update rule(s) to live"
    " world..."
  )
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
          print(
            f"  - {f}:"
            f" {world.get_transform(world.root, getattr(world.root, f))}"
          )

    if hasattr(world, "ur_module") and hasattr(world.ur_module, "flange"):
      print(
        "Flange in root:"
        f" {world.get_transform(world.root, world.ur_module.flange)}"
      )
    if hasattr(world, "gripper") and hasattr(world.gripper, "tool_frame"):
      print(
        "Tool Frame in root:"
        f" {world.get_transform(world.root, world.gripper.tool_frame)}"
      )
  except Exception as e:
    print(f"Transform query error: {e}")

  if (
    args.reset_sim and solution.is_simulated and solution.simulator is not None
  ):
    print(
      "\n[+] Solution is simulated. Resetting simulation to synchronize"
      " Gazebo with updated Belief World..."
    )
    try:
      solution.simulator.reset()
      print("[✓] Simulation reset successfully executed.")
    except Exception as e:
      print(f"[-] Warning: Failed to reset simulation: {e}")

  print("\n[✓] Live world updates complete.")


if __name__ == "__main__":
  main()
