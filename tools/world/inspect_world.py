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

"""Utility to inspect and list scene objects, frames, and joint configs in SBL world."""

import argparse
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments


def get_object_frames(obj: Any) -> list[tuple[str, Any]]:
  """Returns sorted ``(frame_name, frame_node)`` pairs for a world object.

  Supports real SBL ``WorldObject`` instances (``.frames`` / ``.frame_names``)
  as well as mock/legacy objects exposing ``.list_frames()``.

  Args:
    obj: A world object (e.g., ``world.root`` or an entry from
      ``world.list_objects()``).

  Returns:
    Sorted list of ``(frame_name, frame_node)`` tuples.
  """
  frames_attr = getattr(obj, "frames", None)
  if isinstance(frames_attr, (list, tuple)):
    resolved: list[tuple[str, Any]] = []
    for frame in frames_attr:
      name = str(
        frame if isinstance(frame, str) else getattr(frame, "name", frame)
      )
      node = getattr(obj, name, None) if isinstance(frame, str) else frame
      resolved.append((name, node))
    return sorted(resolved, key=lambda item: item[0])

  frame_names_attr = getattr(obj, "frame_names", None)
  if isinstance(frame_names_attr, (list, tuple)):
    return sorted(
      ((str(name), getattr(obj, str(name), None)) for name in frame_names_attr),
      key=lambda item: item[0],
    )

  list_frames_fn = getattr(obj, "list_frames", None)
  if callable(list_frames_fn):
    listed = list_frames_fn()
    if isinstance(listed, (list, tuple, set)):
      resolved = []
      for frame in listed:
        name = str(
          frame if isinstance(frame, str) else getattr(frame, "name", frame)
        )
        node = getattr(obj, name, None) if isinstance(frame, str) else frame
        resolved.append((name, node))
      return sorted(resolved, key=lambda item: item[0])

  return []


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description="Inspect objects and joint configurations in the solution world."
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution address to connect to (default: localhost:17080).",
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  print("\n=== Objects in Active Belief World ===")
  objects = world.list_objects()
  for item in objects:
    obj_name = (
      item if isinstance(item, str) else getattr(item, "name", str(item))
    )
    print(f" - {obj_name}")
    try:
      obj = item if not isinstance(item, str) else getattr(world, obj_name)
      obj_frames = get_object_frames(obj)
      if obj_frames:
        print("   Frames:")
        for frame_name, _ in obj_frames:
          print(f"     * {frame_name}")
      if not hasattr(obj, "joint_configurations"):
        obj = getattr(world, obj_name, obj)
      if hasattr(obj, "joint_configurations") and list(
        obj.joint_configurations.keys()
      ):
        print("   Joint Configurations:")
        for cfg_name in obj.joint_configurations.keys():
          cfg = obj.joint_configurations[cfg_name]
          joint_pos = getattr(cfg, "joint_position", cfg)
          print(f"     * {cfg_name}: {list(joint_pos)}")
    except Exception:
      pass

  print("\n=== Solution Skills ===")
  try:
    if hasattr(solution.skills, "ai") and hasattr(
      solution.skills.ai, "intrinsic"
    ):
      for skill_name in dir(solution.skills.ai.intrinsic):
        if not skill_name.startswith("_"):
          print(f" - ai.intrinsic.{skill_name}")
  except Exception as e:
    print(f"   Error listing skills: {e}")

  print("\n=== Solution Resources ===")
  try:
    if callable(getattr(type(solution.resources), "update", None)):
      solution.resources.update()
    for res_name in dir(solution.resources):
      if not res_name.startswith("_"):
        try:
          res = solution.resources[res_name]
        except (KeyError, TypeError):
          res = getattr(solution.resources, res_name, None)
        if hasattr(res, "name"):
          print(f" - {res.name}")
  except Exception as e:
    print(f"   Error listing resources: {e}")

  print("\n=== Transforms Inspection ===")
  try:
    if hasattr(world, "ur_module") and hasattr(world.ur_module, "flange"):
      print(
        f"Flange in root: {world.get_transform(world.root, world.ur_module.flange)}"
      )
    if hasattr(world, "gripper") and hasattr(world.gripper, "tool_frame"):
      print(
        "Tool Frame in root:"
        f" {world.get_transform(world.root, world.gripper.tool_frame)}"
      )
    root_frames = get_object_frames(world.root)
    if root_frames:
      print("Root frames:")
      for frame_name, frame_node in root_frames:
        if frame_node is None:
          continue
        try:
          print(
            f" - root.{frame_name}: {world.get_transform(world.root, frame_node)}"
          )
        except Exception as e:
          print(f" - root.{frame_name}: <error: {e}>")
  except Exception as e:
    print(f"   Error checking transforms: {e}")


if __name__ == "__main__":
  main()
