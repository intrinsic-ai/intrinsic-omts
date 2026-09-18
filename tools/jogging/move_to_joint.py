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

"""Interactive and CLI tool to move the robot to a target joint configuration or positions."""

import argparse
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments, execution

from src.core.config import RobotConfig
from src.core.types import JointPosition
from src.hardware.robot import UrRobot


def list_available_joint_configs(
  world: Any,
  arm_part_name: str = "ur_module",
) -> list[tuple[str, list[float]]]:
  """Discovers and returns all available named joint configurations."""
  configs: list[tuple[str, list[float]]] = []
  try:
    robot = getattr(world, arm_part_name)
  except AttributeError:
    try:
      robot = world.get_kinematic_object(arm_part_name)
    except Exception:  # pylint: disable=broad-exception-caught
      return configs

  if hasattr(robot, "joint_configurations"):
    jc = robot.joint_configurations
    if hasattr(jc, "keys"):
      for name in jc.keys():
        cfg = jc[name]
        pos = (
          list(cfg.joint_position)
          if hasattr(cfg, "joint_position")
          else list(cfg)
        )
        configs.append((name, pos))

  return configs


def parse_joint_values(joint_str: str) -> list[float] | None:
  """Parses a string of comma- or space-separated float joint angles."""
  cleaned = (
    joint_str.replace(",", " ").replace("[", "").replace("]", "").strip()
  )
  if not cleaned:
    return None
  try:
    values = [float(x) for x in cleaned.split()]
    return values if values else None
  except ValueError:
    return None


def prompt_for_joint_target(
  available_configs: list[tuple[str, list[float]]],
) -> str | list[float] | None:
  """Prompts the user to select a named joint config or enter joint positions."""
  print("\nAvailable Joint Configurations:")
  if available_configs:
    for i, (name, positions) in enumerate(available_configs, start=1):
      formatted_pos = ", ".join(f"{p:.4f}" for p in positions)
      print(f"  [{i}] {name} -> [{formatted_pos}]")
  else:
    print("  (No named joint configurations stored in active world model)")

  while True:
    try:
      choice = input(
        f"\nEnter config index (1-{len(available_configs)}), config name,"
        " joint angles (e.g. 0.0 -1.57 1.57 -1.57 -1.57 0.0), or 'q' to"
        " quit: "
      ).strip()
      if choice.lower() in ("q", "quit", "exit"):
        return None

      if choice.isdigit() and available_configs:
        idx = int(choice)
        if 1 <= idx <= len(available_configs):
          return available_configs[idx - 1][0]
        print(
          "Invalid index. Please enter a number between 1 and"
          f" {len(available_configs)}."
        )
        continue

      for name, _ in available_configs:
        if choice == name:
          return name

      parsed_joints = parse_joint_values(choice)
      if parsed_joints is not None and len(parsed_joints) >= 1:
        return parsed_joints

      print("Invalid input. Please enter a valid index, name, numbers, or 'q'.")
    except (ValueError, EOFError):
      print("Invalid input.")


def move_robot_to_joint(
  solution: Any,
  joint_target: str | JointPosition | Sequence[float],
  arm_part_name: str = "ur_module",
) -> None:
  """Plans and executes a joint motion moving the robot to the target pose."""
  target_pos: str | JointPosition
  if isinstance(joint_target, (str, JointPosition)):
    target_pos = joint_target
  else:
    target_pos = JointPosition(tuple(joint_target))

  target_desc = (
    target_pos
    if isinstance(target_pos, str)
    else [round(x, 4) for x in target_pos.to_list()]
  )
  print(f"\nPlanning joint motion for '{arm_part_name}' -> {target_desc}...")

  config = RobotConfig(
    arm_part_name=arm_part_name,
    tool_object_name="gripper",
    tool_frame_name="tool_frame",
  )
  robot = UrRobot(
    solution=solution,
    config=config,
  )

  task = (
    robot.build_move_to_joint_position_task(
      joint_position=target_pos,
      name=f"Move {arm_part_name} to {target_desc}",
    )
    if isinstance(target_pos, JointPosition)
    else robot.build_move_joint_task(
      joint_target=target_pos,
      name=f"Move {arm_part_name} to {target_desc}",
    )
  )

  try:
    solution.executive.run(task)
    print(f"[✓] Successfully moved robot to '{target_desc}'.")
  except execution.ExecutionFailedError as e:
    print(f"\n[!] Motion execution failed: {e}")
    if hasattr(solution.executive, "get_errors"):
      print(f"Executive errors: {solution.executive.get_errors()}")
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f"\n[!] Error during motion execution: {e}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description=(
      "Developer CLI tool to move robot to a joint configuration or"
      " explicit joint positions."
    )
  )
  parser.add_argument(
    "target",
    nargs="?",
    default=None,
    type=str,
    help=(
      "Named joint configuration (e.g. 'home', 'view_joints') or joint angles."
    ),
  )
  parser.add_argument(
    "--joints",
    "-j",
    nargs="+",
    type=float,
    default=None,
    help=(
      "Explicit joint positions in radians (e.g. --joints 0.0 -1.57 1.57"
      " -1.57 -1.57 0.0)."
    ),
  )
  parser.add_argument(
    "--name",
    "-n",
    type=str,
    default=None,
    help="Name of stored joint configuration to move to.",
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution gRPC address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
    "--robot_name",
    "--arm_part_name",
    dest="robot_name",
    type=str,
    default="ur_module",
    help="Robot object name in the world (default: 'ur_module').",
  )
  return parser.parse_args(argv)


def resolve_joint_target(
  args: argparse.Namespace,
) -> str | list[float] | None:
  """Resolves the target joint positions or configuration name from arguments."""
  if args.joints:
    return list(args.joints)
  if args.name:
    return args.name
  if args.target:
    parsed = parse_joint_values(args.target)
    if parsed is not None and len(parsed) > 1:
      return parsed
    return args.target
  return None


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)

  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  target = resolve_joint_target(args)
  if target is not None:
    move_robot_to_joint(
      solution=solution,
      joint_target=target,
      arm_part_name=args.robot_name,
    )
    return

  available_configs = list_available_joint_configs(
    world=world, arm_part_name=args.robot_name
  )
  while True:
    selected = prompt_for_joint_target(available_configs)
    if selected is None:
      print("Exiting move_to_joint tool.")
      break

    move_robot_to_joint(
      solution=solution,
      joint_target=selected,
      arm_part_name=args.robot_name,
    )


if __name__ == "__main__":
  main()
