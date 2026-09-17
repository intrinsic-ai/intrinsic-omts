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

"""Interactive and CLI tool to command robotic grippers (Robotiq, DIO)."""

import argparse
import sys
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments, execution

from src.core.config import GripperConfig
from src.hardware.gripper import (
  DioGripper,
  GripperInterface,
  RobotiqGripper,
)

VALID_ACTIONS: tuple[str, ...] = ("open", "close")

MENU_OPTIONS: list[tuple[str, str, str]] = [
  ("1", "Open Gripper", "open"),
  ("2", "Close Gripper", "close"),
]


def create_gripper(
  args: argparse.Namespace, solution: Any = None
) -> GripperInterface:
  """Creates a GripperInterface instance based on command-line arguments.

  Args:
    args: Parsed command-line arguments.
    solution: Connected solution deployment.

  Returns:
    An instance of GripperInterface.

  Raises:
    ValueError: If gripper_type is unsupported.
  """
  gripper_type = getattr(args, "gripper_type", "robotiq")
  if gripper_type == "dio":
    config = GripperConfig(
      type="dio",
      dio_open_pin=getattr(args, "open_pin", 0),
      dio_close_pin=getattr(args, "close_pin", 1),
      dio_device_name=getattr(args, "device_name", "ur_module"),
      dio_output_block_name=getattr(args, "output_block_name", "standard_out"),
    )
    return DioGripper(
      solution=solution,
      config=config,
    )
  if gripper_type in ("robotiq", "sideloaded"):
    config = GripperConfig(
      type="robotiq",
      joint_name=getattr(args, "joint_name", "robotiq_hande_left_finger_joint"),
      open_position=getattr(args, "open_position", 0.025),
      close_position=getattr(args, "close_position", 0.0),
      action_name=getattr(args, "action_name", None),
    )
    return RobotiqGripper(solution=solution, config=config)
  raise ValueError(f"Unsupported gripper type: {gripper_type}")


def execute_action(
  gripper: GripperInterface,
  action: str,
  solution: Any = None,
) -> bool:
  """Builds and runs the appropriate task from gripper.

  Args:
    gripper: Gripper interface implementation.
    action: Action key matching one of VALID_ACTIONS ("open", "close").
    solution: Connected solution deployment.

  Returns:
    True on success, False on error.
  """
  if action == "open":
    task = gripper.build_open_task()
  elif action == "close":
    task = gripper.build_close_task()
  else:
    print(f"Unknown action: {action}")
    return False

  if solution is None:
    print(
      f"Cannot execute '{action}': solution deployment is required for live"
      " hardware."
    )
    return False

  try:
    solution.executive.run(task)
    print(f"Successfully executed action: {action}")
    return True
  except execution.ExecutionFailedError as e:
    print(f"Action '{action}' execution failed: {e}")
    if hasattr(solution.executive, "get_errors"):
      print(f"Executive errors: {solution.executive.get_errors()}")
    return False
  except Exception as e:
    print(f"Unexpected error executing '{action}': {e}")
    return False


def prompt_menu() -> str | None:
  """Displays interactive menu and prompts operator for choice.

  Returns:
    Action key string if a valid choice is made, or None to quit.
  """
  print("\nGripper Control Menu:")
  for key, label, _ in MENU_OPTIONS:
    print(f"  [{key}] {label}")
  print("  [q] Quit")

  while True:
    try:
      choice = input("\nEnter choice: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
      print("\nExiting.")
      return None

    if choice in ("q", "quit", "exit"):
      return None

    for key, _, action in MENU_OPTIONS:
      if choice == key:
        return action

    print(f"Invalid choice: {choice}. Enter 1-{len(MENU_OPTIONS)} or 'q'.")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command-line arguments."""
  parser = argparse.ArgumentParser(
    description="Command-line and interactive tool to control robotic grippers."
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution gRPC address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
    "--action",
    type=str,
    choices=VALID_ACTIONS,
    default=None,
    help="Specific action to execute. If omitted, runs in interactive menu.",
  )
  parser.add_argument(
    "--gripper_type",
    type=str,
    default="robotiq",
    choices=["robotiq", "dio", "sideloaded"],
    help="Gripper type: 'robotiq' or 'dio' (default: 'robotiq').",
  )
  parser.add_argument(
    "--joint_name",
    type=str,
    default="robotiq_hande_left_finger_joint",
    help=(
      "Name of gripper finger joint (default: robotiq_hande_left_finger_joint)."
    ),
  )
  parser.add_argument(
    "--open_position",
    type=float,
    default=0.025,
    help="Joint position for open state in meters (default: 0.025).",
  )
  parser.add_argument(
    "--close_position",
    type=float,
    default=0.0,
    help="Joint position for close state in meters (default: 0.0).",
  )
  parser.add_argument(
    "--action_name",
    type=str,
    default="/gripper/gripper_action_controller/gripper_cmd",
    help="ROS action controller name for gripper_cmd_skill.",
  )
  parser.add_argument(
    "--open_pin",
    type=int,
    default=0,
    help="DIO pin index to command gripper open (default: 0).",
  )
  parser.add_argument(
    "--close_pin",
    type=int,
    default=1,
    help="DIO pin index to command gripper close (default: 1).",
  )
  parser.add_argument(
    "--output_block_name",
    type=str,
    default="standard_out",
    help="DIO output block name (default: standard_out).",
  )
  parser.add_argument(
    "--device_name",
    type=str,
    default="ur_module",
    help="Optional device name for DIO skills.",
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  """Main entry point for gripper control tool."""
  args = parse_args(argv)

  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  gripper = create_gripper(args, solution=solution)

  if args.action:
    success = execute_action(
      gripper=gripper,
      action=args.action,
      solution=solution,
    )
    if not success:
      sys.exit(1)
    return

  while True:
    action = prompt_menu()
    if action is None:
      break
    execute_action(
      gripper=gripper,
      action=action,
      solution=solution,
    )


if __name__ == "__main__":
  main()
