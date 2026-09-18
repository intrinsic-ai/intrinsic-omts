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

"""Command-line and interactive tool to control CNC machine door, vise, and cycle signals."""

import argparse
import sys
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments, execution

from src.core.config import MachineConfig
from src.hardware.machine import CncMachineInterface, DioCncMachine

VALID_ACTIONS = (
  "open_door",
  "close_door",
  "open_vise",
  "close_vise",
  "trigger_cycle",
  "wait_cycle",
)


def execute_action(
  machine: CncMachineInterface,
  action: str,
  solution: Any,
  timeout_seconds: float = 30.0,
) -> bool:
  """Builds and runs the requested CNC machine task."""
  if action == "open_door":
    task = machine.build_open_door_task()
  elif action == "close_door":
    task = machine.build_close_door_task()
  elif action == "open_vise":
    task = machine.build_open_vise_task()
  elif action == "close_vise":
    task = machine.build_close_vise_task()
  elif action == "trigger_cycle":
    task = machine.build_trigger_cycle_task()
  elif action == "wait_cycle":
    task = machine.build_wait_cycle_complete_task(
      timeout_seconds=timeout_seconds
    )
  else:
    print(f"Unknown action: {action}")
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
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f"Unexpected error executing '{action}': {e}")
    return False


def prompt_menu() -> str | None:
  """Displays interactive menu and prompts operator for choice."""
  print("\nCNC Machine Control Menu:")
  for idx, action in enumerate(VALID_ACTIONS, start=1):
    print(f"  [{idx}] {action}")
  print("  [q] Quit")

  while True:
    try:
      choice = input("\nSelect action: ").strip()
    except EOFError:
      return None

    if choice.lower() in ("q", "quit", "exit"):
      return None
    if choice.isdigit():
      idx = int(choice)
      if 1 <= idx <= len(VALID_ACTIONS):
        return VALID_ACTIONS[idx - 1]
    if choice in VALID_ACTIONS:
      return choice
    print("Invalid selection. Enter a menu number, action name, or 'q'.")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command-line arguments."""
  parser = argparse.ArgumentParser(
    description="Command-line and interactive tool to control CNC machine."
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
    "--timeout_seconds",
    type=float,
    default=30.0,
    help="Timeout in seconds for waiting operations (default: 30.0).",
  )
  parser.add_argument(
    "--door_open_pin",
    type=int,
    default=2,
    help="DIO output pin for opening CNC door (default: 2).",
  )
  parser.add_argument(
    "--door_close_pin",
    type=int,
    default=3,
    help="DIO output pin for closing CNC door (default: 3).",
  )
  parser.add_argument(
    "--vise_open_pin",
    type=int,
    default=4,
    help="DIO output pin for opening CNC vise (default: 4).",
  )
  parser.add_argument(
    "--vise_close_pin",
    type=int,
    default=5,
    help="DIO output pin for clamping CNC vise (default: 5).",
  )
  parser.add_argument(
    "--cycle_start_pin",
    type=int,
    default=6,
    help="DIO output pin for triggering CNC cycle start (default: 6).",
  )
  parser.add_argument(
    "--cycle_done_input_pin",
    type=int,
    default=0,
    help="DIO input pin for CNC cycle complete signal (default: 0).",
  )
  parser.add_argument(
    "--device_name",
    type=str,
    default="ur_module",
    help="DIO device resource name (default: ur_module).",
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  """Main entry point for CNC machine control tool."""
  args = parse_args(argv)

  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  config = MachineConfig(
    door_open_pin=args.door_open_pin,
    door_close_pin=args.door_close_pin,
    vise_open_pin=args.vise_open_pin,
    vise_close_pin=args.vise_close_pin,
    cycle_start_pin=args.cycle_start_pin,
    cycle_complete_input_pin=args.cycle_done_input_pin,
    device_name=args.device_name,
    enclosure_object_name="cnc_enclosure",
    vise_object_name="schunk_egp_64nnb",
    door_open_joints=(0.4,),
    door_closed_joints=(0.0,),
    vise_open_joints=(0.01, 0.01),
    vise_closed_joints=(0.0, 0.0),
    output_block_name="standard_out",
    input_block_name="standard_in",
  )
  machine = DioCncMachine(
    solution=solution,
    config=config,
  )

  if args.action:
    success = execute_action(
      machine=machine,
      action=args.action,
      solution=solution,
      timeout_seconds=args.timeout_seconds,
    )
    if not success:
      sys.exit(1)
    return

  while True:
    action = prompt_menu()
    if action is None:
      break
    execute_action(
      machine=machine,
      action=action,
      solution=solution,
      timeout_seconds=args.timeout_seconds,
    )


if __name__ == "__main__":
  main()
