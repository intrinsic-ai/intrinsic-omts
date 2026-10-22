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

from src.utils.world_utils import (
  connect_initial_world,
  find_file,
)


def discover_setup_update_files(setup: str = "omts") -> list[str]:
  """Discovers all `configs/<setup>/*.updates.pbtxt` files in sorted order."""
  rel_dir = os.path.join("configs", setup)
  resolved_dir = find_file(rel_dir)
  if not os.path.isdir(resolved_dir):
    return []
  filenames = sorted(
    fname
    for fname in os.listdir(resolved_dir)
    if fname.endswith(".updates.pbtxt")
  )
  return [os.path.join("configs", setup, fname) for fname in filenames]


DEFAULT_UPDATE_FILES = discover_setup_update_files("omts")


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
    "--setup",
    type=str,
    default="omts",
    help=(
      "Cell setup directory under configs/<setup>/ whose *.updates.pbtxt"
      " files to apply when --files is omitted (default: omts)."
    ),
  )
  parser.add_argument(
    "--files",
    nargs="*",
    default=None,
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
  parser.add_argument(
    "--update_init_world",
    action=argparse.BooleanOptionalAction,
    default=False,
    help=(
      "Whether to also apply updates to the initial world ('init_world') so"
      " they persist across world resets (default: False)."
    ),
  )
  args = parser.parse_args(argv)
  if args.files is None:
    args.files = discover_setup_update_files(args.setup)
  return args


def get_world_object_names(world: Any) -> set[str] | None:
  """Returns the set of existing object names in `world`, or `None` if unknown.

  Queries `world.list_object_names()` or `world.list_objects()` without calling
  `hasattr(world, <object_name>)`, which would invoke `ObjectWorld.__getattr__`
  and raise `grpc.RpcError` (`StatusCode.NOT_FOUND`) on missing objects.
  """
  if world is None:
    return None
  try:
    list_names_fn = getattr(world, "list_object_names", None)
    if callable(list_names_fn):
      names = list_names_fn()
      if isinstance(names, (list, tuple, set)):
        valid_names = {str(n) for n in names if isinstance(n, str) and n}
        return valid_names | {"root"}
  except Exception:  # pylint: disable=broad-exception-caught
    pass

  try:
    list_objs_fn = getattr(world, "list_objects", None)
    if callable(list_objs_fn):
      objs = list_objs_fn()
      if isinstance(objs, (list, tuple, set)):
        extracted: set[str] = {"root"}
        for obj in objs:
          if isinstance(obj, str) and obj:
            extracted.add(obj)
          else:
            name = getattr(obj, "name", None) or getattr(obj, "id", None)
            if isinstance(name, str) and name:
              extracted.add(name)
        return extracted
  except Exception:  # pylint: disable=broad-exception-caught
    pass

  return None


def world_object_exists(
  world: Any,
  obj_name: str | None,
  known_objects: set[str] | None = None,
) -> bool:
  """Returns `True` if `obj_name` (or the object prefix of `obj/frame`) exists in `world`."""
  if not obj_name:
    return False
  base_obj = obj_name.split("/", 1)[0]
  if base_obj == "root":
    return True
  if known_objects is None:
    known_objects = get_world_object_names(world)
  if known_objects is not None:
    return base_obj in known_objects
  try:
    return getattr(world, base_obj, None) is not None
  except Exception:  # pylint: disable=broad-exception-caught
    return False


def _extract_node_object_name(node: Any) -> str | None:
  """Extracts the referenced object name from a `TransformNodeReference`."""
  if not node.HasField("by_name"):
    return None
  if node.by_name.HasField("object") and node.by_name.object.object_name:
    return node.by_name.object.object_name
  if node.by_name.HasField("frame") and node.by_name.frame.object_name:
    return node.by_name.frame.object_name
  return None


def adapt_updates_for_live_world(
  world: Any,
  updates: object_world_updates_pb2.ObjectWorldUpdates,
  is_simulated: bool = True,
) -> object_world_updates_pb2.ObjectWorldUpdates:
  """Converts `create_frame` into `update_transform` if frames exist and skips rules for missing objects.

  Args:
    world: Target SBL `ObjectWorld` instance.
    updates: Raw `ObjectWorldUpdates` message to adapt.
    is_simulated: Whether the connected solution is simulated. When `False`
      (real hardware), `update_object_joints` and `update_object_joint` rules
      are skipped so live robot joint states from ICON are preserved.

  Returns:
    Adapted `ObjectWorldUpdates` containing only rules valid for `world`.
  """
  adapted = object_world_updates_pb2.ObjectWorldUpdates()
  known_objects = get_world_object_names(world)

  for update in updates.updates:
    if update.HasField("create_frame"):
      cf = update.create_frame
      parent_name = (
        cf.parent_object_with_filter.reference.by_name.object_name or "root"
      )
      frame_name = cf.new_frame_name

      if not world_object_exists(world, parent_name, known_objects):
        logging.info(
          "Skipping create_frame %s for missing parent object: %s",
          frame_name,
          parent_name,
        )
        continue

      try:
        parent_obj = getattr(world, parent_name, None)
      except Exception:  # pylint: disable=broad-exception-caught
        parent_obj = None

      frame_exists = False
      if parent_obj is not None:
        try:
          if hasattr(parent_obj, "list_frames"):
            frames = parent_obj.list_frames()
            if isinstance(frames, (list, tuple, set)):
              frame_names = {
                f if isinstance(f, str) else getattr(f, "name", None)
                for f in frames
              }
              frame_exists = frame_name in frame_names
            else:
              frame_exists = getattr(parent_obj, frame_name, None) is not None
          else:
            frame_exists = getattr(parent_obj, frame_name, None) is not None
        except Exception:  # pylint: disable=broad-exception-caught
          frame_exists = False

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
      referenced_objs = [
        obj_name
        for obj_name in (
          _extract_node_object_name(ut.node_a),
          _extract_node_object_name(ut.node_b),
          _extract_node_object_name(ut.node_to_update),
        )
        if obj_name is not None
      ]
      missing_obj = next(
        (
          obj_name
          for obj_name in referenced_objs
          if not world_object_exists(world, obj_name, known_objects)
        ),
        None,
      )
      if missing_obj is not None:
        logging.info(
          "Skipping update_transform for missing object: %s", missing_obj
        )
        continue

      adapted.updates.add().CopyFrom(update)
    elif update.HasField("update_object_joints"):
      if not is_simulated:
        logging.info(
          "Skipping update_object_joints on real hardware deployment."
        )
        continue
      obj_name = update.update_object_joints.object.by_name.object_name
      if obj_name and not world_object_exists(world, obj_name, known_objects):
        logging.info(
          "Skipping update_object_joints for missing object: %s", obj_name
        )
        continue
      adapted.updates.add().CopyFrom(update)
    elif update.HasField("update_object_joint"):
      if not is_simulated:
        logging.info(
          "Skipping update_object_joint on real hardware deployment."
        )
        continue
      obj_name = update.update_object_joint.object.by_name.object_name
      if obj_name and not world_object_exists(world, obj_name, known_objects):
        logging.info(
          "Skipping update_object_joint for missing object: %s", obj_name
        )
        continue
      adapted.updates.add().CopyFrom(update)
    elif update.HasField("reparent_object"):
      child_obj = update.reparent_object.object.by_name.object_name
      parent_obj_name = (
        update.reparent_object.new_parent.reference.by_name.object_name
      )
      if (
        child_obj and not world_object_exists(world, child_obj, known_objects)
      ) or (
        parent_obj_name
        and not world_object_exists(world, parent_obj_name, known_objects)
      ):
        logging.info(
          "Skipping reparent_object for missing object: %s -> %s",
          child_obj,
          parent_obj_name,
        )
        continue
      adapted.updates.add().CopyFrom(update)
    elif update.HasField("update_object_properties"):
      obj_name = update.update_object_properties.object.by_name.object_name
      if obj_name and not world_object_exists(world, obj_name, known_objects):
        logging.info(
          "Skipping update_object_properties for missing object: %s", obj_name
        )
        continue
      adapted.updates.add().CopyFrom(update)
    else:
      adapted.updates.add().CopyFrom(update)

  return adapted


def extract_joint_updates(
  updates: object_world_updates_pb2.ObjectWorldUpdates,
) -> object_world_updates_pb2.ObjectWorldUpdates:
  """Extracts only joint update rules (`update_object_joints` / `update_object_joint`).

  Because `WorldUpdater` continuously streams `position_sensed` from ICON/Gazebo
  into the Belief World (`"world"`) at ~50 Hz while the solution is running,
  any joint position updates applied to `"world"` early in a multi-file batch
  will be overwritten by `WorldUpdater` before `solution.simulator.reset()`
  is reached. Re-applying these joint updates immediately before
  `solution.simulator.reset()` ensures `"world"` holds the target joint
  configuration at the exact instant `SimulatorWorldManager` pauses
  `WorldUpdater` and clones `"world"` into `"sim_world"`.
  """
  joint_updates = object_world_updates_pb2.ObjectWorldUpdates()
  for update in updates.updates:
    if update.HasField("update_object_joints") or update.HasField(
      "update_object_joint"
    ):
      joint_updates.updates.add().CopyFrom(update)
  return joint_updates


_connect_initial_world = connect_initial_world


def apply_pbtxt_file(
  world: Any,
  filepath: str,
  init_world: Any | None = None,
  is_simulated: bool = True,
) -> object_world_updates_pb2.ObjectWorldUpdates:
  """Loads a .pbtxt file and pushes its updates to the active and initial worlds.

  Args:
    world: Active belief `ObjectWorld` instance.
    filepath: Path to the `.pbtxt` `ObjectWorldUpdates` file.
    init_world: Optional initial `ObjectWorld` (`EditWorldId.INITIAL`).
    is_simulated: Whether the connected solution is simulated. When `False`,
      joint updates are skipped for both `world` and `init_world`.

  Returns:
    The adapted `ObjectWorldUpdates` applied to `world`.
  """
  resolved_path = find_file(filepath)
  if not os.path.exists(resolved_path):
    print(
      f"[-] Warning: File not found: {filepath} (resolved: {resolved_path})"
    )
    return object_world_updates_pb2.ObjectWorldUpdates()

  print(f"[+] Reading update file: {filepath}")
  with open(resolved_path, encoding="utf-8") as f:
    pbtxt_content = f.read()

  raw_updates = object_world_updates_pb2.ObjectWorldUpdates()
  text_format.Parse(pbtxt_content, raw_updates)

  adapted_updates = adapt_updates_for_live_world(
    world=world, updates=raw_updates, is_simulated=is_simulated
  )

  if not adapted_updates.updates:
    print(
      f"    Skipping {filepath}: no update rules apply to objects in the active"
      " world."
    )
    return adapted_updates

  print(
    f"    Applying {len(adapted_updates.updates)} update rule(s) to live"
    " world..."
  )
  world.batch_update(adapted_updates)

  if init_world is not None:
    try:
      adapted_init_updates = adapt_updates_for_live_world(
        world=init_world, updates=raw_updates, is_simulated=is_simulated
      )
      if adapted_init_updates.updates:
        init_world.batch_update(adapted_init_updates)
    except Exception as e:  # pylint: disable=broad-exception-caught
      logging.warning(
        "Failed to apply %s to initial world ('init_world'): %s", filepath, e
      )

  print(f"[✓] Successfully applied: {filepath}")
  return adapted_updates


def main(argv: Sequence[str] | None = None) -> None:
  """Main entrypoint for applying scene updates live."""
  args = parse_args(argv)
  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world
  is_simulated = bool(getattr(solution, "is_simulated", False))
  init_world = (
    connect_initial_world(solution) if args.update_init_world else None
  )

  print(f"\n=== Applying {len(args.files)} World Update File(s) Live ===")
  joint_updates = object_world_updates_pb2.ObjectWorldUpdates()
  for fpath in args.files:
    applied = apply_pbtxt_file(
      world=world,
      filepath=fpath,
      init_world=init_world,
      is_simulated=is_simulated,
    )
    if is_simulated and isinstance(
      applied, object_world_updates_pb2.ObjectWorldUpdates
    ):
      extracted = extract_joint_updates(applied)
      joint_updates.updates.extend(extracted.updates)

  if (
    args.reset_sim and solution.is_simulated and solution.simulator is not None
  ):
    print(
      "\n[+] Solution is simulated. Resetting simulation to synchronize"
      " Gazebo with updated Belief World..."
    )
    try:
      if joint_updates.updates:
        world.batch_update(joint_updates)
      solution.simulator.reset()
      print("[✓] Simulation reset successfully executed.")
    except Exception as e:
      print(f"[-] Warning: Failed to reset simulation: {e}")

  print("\n=== Current Active World State Verification ===")
  try:
    known_objects = get_world_object_names(world)
    if world_object_exists(world, "root", known_objects):
      print("Frames on 'root':")
      if hasattr(world.root, "list_frames"):
        for f in world.root.list_frames():
          print(
            f"  - {f}:"
            f" {world.get_transform(world.root, getattr(world.root, f))}"
          )

    if world_object_exists(world, "ur_module", known_objects) and hasattr(
      world.ur_module, "flange"
    ):
      print(
        "Flange in root:"
        f" {world.get_transform(world.root, world.ur_module.flange)}"
      )
    if world_object_exists(world, "gripper", known_objects) and hasattr(
      world.gripper, "tool_frame"
    ):
      print(
        "Tool Frame in root:"
        f" {world.get_transform(world.root, world.gripper.tool_frame)}"
      )
  except Exception as e:
    print(f"Transform query error: {e}")

  print("\n[✓] Live world updates complete.")


if __name__ == "__main__":
  main()
