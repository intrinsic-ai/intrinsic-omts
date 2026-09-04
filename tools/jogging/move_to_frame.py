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

"""Interactive CLI tool to move the robot tool frame to a target frame in the scene."""

import argparse
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments, execution

from src.hardware.robot import UrRobot


def list_available_frames(world: Any) -> list[tuple[str, str]]:
  """Discovers and returns all available (parent_object, frame_name) pairs in the world.

  Args:
    world: The connected SBL ObjectWorld instance.

  Returns:
    A list of (parent_object_name, frame_name) tuples found in the world.
  """
  frames: list[tuple[str, str]] = []

  # Inspect root frames first
  if hasattr(world, "root"):
    root_obj = world.root
    if hasattr(root_obj, "list_frames"):
      for f in root_obj.list_frames():
        frames.append(("root", f))
    elif hasattr(root_obj, "frames"):
      for f in root_obj.frames:
        frame_name = f if isinstance(f, str) else getattr(f, "name", str(f))
        frames.append(("root", frame_name))

  # Inspect other scene objects
  if hasattr(world, "list_objects"):
    for obj_item in world.list_objects():
      obj_name = getattr(obj_item, "name", str(obj_item))
      if obj_name in ("root", "ur_module"):
        continue
      obj = getattr(world, obj_name, None)
      if obj is not None:
        if hasattr(obj, "list_frames"):
          for f in obj.list_frames():
            frames.append((obj_name, getattr(f, "name", str(f))))
        elif hasattr(obj, "frames"):
          for f in obj.frames:
            frame_name = getattr(f, "name", str(f))
            frames.append((obj_name, frame_name))

  # Fallback / scene defaults if dynamic discovery returned nothing
  if not frames:
    default_scene_frames = [
      ("root", "view"),
      ("root", "pre_grasp"),
      ("root", "grasp"),
      ("root", "machine_approach"),
      ("root", "pre_place_vise"),
      ("root", "place_vise"),
    ]
    for parent, frame in default_scene_frames:
      frames.append((parent, frame))

  # Deduplicate while preserving insertion order
  seen = set()
  unique_frames: list[tuple[str, str]] = []
  for item in frames:
    if item not in seen:
      seen.add(item)
      unique_frames.append(item)

  return unique_frames


def prompt_for_frame(
  available_frames: list[tuple[str, str]],
) -> tuple[str, str] | None:
  """Prompts the user to select a target frame from the available list.

  Args:
    available_frames: List of (parent_object_name, frame_name) tuples.

  Returns:
    The selected (parent_object_name, frame_name) tuple, or None if the user
    chose to quit.
  """
  if not available_frames:
    print("No frames available in the scene.")
    return None

  print("\nAvailable Frames in Scene:")
  for i, (parent, frame) in enumerate(available_frames, start=1):
    print(f"  [{i}] {parent} / {frame}")

  while True:
    try:
      choice = input(
        f"\nEnter frame index (1-{len(available_frames)}) to move robot, or"
        " 'q' to quit: "
      ).strip()
      if choice.lower() in ("q", "quit", "exit"):
        return None

      idx = int(choice)
      if 1 <= idx <= len(available_frames):
        return available_frames[idx - 1]
      print(
        "Invalid choice. Please enter a number between 1 and"
        f" {len(available_frames)}."
      )
    except (ValueError, EOFError):
      print("Invalid input. Please enter a valid number or 'q'.")


def move_robot_to_frame(
  solution: Any,
  target_frame_name: str,
  target_object_name: str = "root",
  motion_type: str = "ANY",
  allow_tool_z_rotation: bool = False,
  arm_part_name: str = "ur_module",
  tool_object_name: str = "gripper",
  tool_frame_name: str = "tool_frame",
) -> None:
  """Plans and executes a Cartesian motion moving the robot tool to the target frame.

  Args:
    solution: Connected SBL deployment instance.
    target_frame_name: Frame name to move to (e.g. 'view', 'grasp').
    target_object_name: Parent object of target frame in world (default:
      'root').
    motion_type: Motion segment type ('ANY', 'LINEAR', 'JOINT').
    allow_tool_z_rotation: Whether to allow rotation around tool Z approach axis.
    arm_part_name: Robot arm part name in solution.world (default: 'ur_module').
    tool_object_name: End-effector tool object name (default: 'gripper').
    tool_frame_name: Frame name on tool object to align (default: 'tool_frame').
  """
  print(
    f"\nPlanning motion for '{arm_part_name}' moving tool"
    f" '{tool_object_name}/{tool_frame_name}' ->"
    f" '{target_object_name}/{target_frame_name}' (motion_type:"
    f" {motion_type}, allow_tool_z_rot: {allow_tool_z_rotation})..."
  )
  try:
    from intrinsic.world.public.proto import object_world_refs_pb2

    current_target_t = solution.world.get_transform(
      solution.world.root,
      solution.world.get_transform_node(
        object_world_refs_pb2.TransformNodeReference(
          by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            frame=object_world_refs_pb2.FrameReferenceByName(
              object_name=target_object_name, frame_name=target_frame_name
            )
          )
        )
      ),
    )
    print(
      f"Current pose of '{target_object_name}/{target_frame_name}' in root: {current_target_t}"
    )
  except Exception as te:
    print(f"Could not query target frame: {te}")

  robot = UrRobot(
    solution=solution,
    arm_part_name=arm_part_name,
    tool_object_name=tool_object_name,
    tool_frame_name=tool_frame_name,
  )

  task = robot.build_move_cartesian_task(
    target_frame_name=target_frame_name,
    target_object_name=target_object_name,
    motion_type=motion_type,
    allow_tool_z_rotation=allow_tool_z_rotation,
    name=(
      f"Move {tool_object_name}.{tool_frame_name} to"
      f" {target_object_name}.{target_frame_name} ({motion_type})"
    ),
  )

  try:
    solution.executive.run(task)
    print(
      f"[✓] Successfully moved robot to"
      f" '{target_object_name}/{target_frame_name}'."
    )
  except execution.ExecutionFailedError as e:
    print(f"\n[!] Motion execution failed: {e}")
    if hasattr(solution.executive, "get_errors"):
      print(f"Executive errors: {solution.executive.get_errors()}")
  except Exception as e:
    print(f"\n[!] Error during motion execution: {e}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description="Developer CLI tool to move robot to a scene frame."
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution gRPC address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
    "--frame",
    type=str,
    default=None,
    help="Target frame name to move to. If omitted, prompts interactively.",
  )
  parser.add_argument(
    "--parent_object",
    type=str,
    default="root",
    help="Parent object of target frame in world (default: 'root').",
  )
  parser.add_argument(
    "--motion_type",
    type=str,
    default="ANY",
    choices=["ANY", "LINEAR", "JOINT"],
    help="Motion trajectory type: ANY (default), LINEAR, or JOINT.",
  )
  parser.add_argument(
    "--arm_part_name",
    type=str,
    default="ur_module",
    help="Robot arm part name in solution.world (default: 'ur_module').",
  )
  parser.add_argument(
    "--tool_object_name",
    type=str,
    default="gripper",
    help="End-effector tool object name (default: 'gripper').",
  )
  parser.add_argument(
    "--tool_frame_name",
    type=str,
    default="tool_frame",
    help="Tool frame name on tool_object_name (default: 'tool_frame').",
  )
  parser.add_argument(
    "--allow_tool_z_rotation",
    action="store_true",
    default=False,
    help="Allow free rotation around tool Z approach axis using PositionEquality + RotationCone.",
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)

  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  if args.frame:
    move_robot_to_frame(
      solution=solution,
      target_frame_name=args.frame,
      target_object_name=args.parent_object,
      motion_type=args.motion_type,
      allow_tool_z_rotation=args.allow_tool_z_rotation,
      arm_part_name=args.arm_part_name,
      tool_object_name=args.tool_object_name,
      tool_frame_name=args.tool_frame_name,
    )
    return

  available_frames = list_available_frames(world)
  if not available_frames:
    print("No target frames discovered in the active world model.")
    return

  while True:
    selected = prompt_for_frame(available_frames)
    if selected is None:
      print("Exiting move_to_frame tool.")
      break

    parent_obj, frame_name = selected
    move_robot_to_frame(
      solution=solution,
      target_frame_name=frame_name,
      target_object_name=parent_obj,
      motion_type=args.motion_type,
      allow_tool_z_rotation=args.allow_tool_z_rotation,
      arm_part_name=args.arm_part_name,
      tool_object_name=args.tool_object_name,
      tool_frame_name=args.tool_frame_name,
    )


if __name__ == "__main__":
  main()
