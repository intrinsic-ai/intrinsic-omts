"""Interactive and CLI tool to command CNC machine door, vise, and cycle."""

import argparse
import sys
from typing import Any, Sequence

from intrinsic.solutions import deployments
from intrinsic.solutions import execution
from src.hardware.machine import CncMachineInterface
from src.hardware.machine import DioCncMachine
from src.hardware.machine import MockCncMachine

VALID_ACTIONS: tuple[str, ...] = (
    "open_door",
    "close_door",
    "open_vise",
    "close_vise",
    "trigger_cycle",
    "wait_cycle",
)

MENU_OPTIONS: list[tuple[str, str, str]] = [
    ("1", "Open CNC Door", "open_door"),
    ("2", "Close CNC Door", "close_door"),
    ("3", "Open CNC Vise", "open_vise"),
    ("4", "Close CNC Vise", "close_vise"),
    ("5", "Trigger CNC Machining Cycle", "trigger_cycle"),
    ("6", "Wait for CNC Cycle Complete", "wait_cycle"),
]


def execute_action(
    machine: CncMachineInterface,
    action: str,
    solution: Any = None,
    timeout_seconds: float = 30.0,
) -> bool:
  """Builds and runs the appropriate task from machine.

  Args:
    machine: CNC machine interface implementation.
    action: Action key matching one of VALID_ACTIONS.
    solution: Connected solution deployment, or None if in mock mode.
    timeout_seconds: Timeout in seconds for wait operations.

  Returns:
    True on success, False on error.
  """
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

  if isinstance(machine, MockCncMachine):
    print(f"Successfully executed action: {action}")
    return True

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
  print("\nCNC Machine Control Menu:")
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
      "--mock",
      action="store_true",
      default=False,
      help="Use mock CNC machine without connecting to a deployment.",
  )
  parser.add_argument(
      "--device_name",
      type=str,
      default=None,
      help="Optional device name for DIO skills.",
  )
  parser.add_argument(
      "--output_block_name",
      type=str,
      default="standard_out",
      help="Block name for DIO output signals (default: standard_out).",
  )
  parser.add_argument(
      "--input_block_name",
      type=str,
      default="standard_in",
      help="Block name for DIO input signals (default: standard_in).",
  )
  parser.add_argument(
      "--door_open_pin",
      type=int,
      default=2,
      help="Output pin index to command door open (default: 2).",
  )
  parser.add_argument(
      "--door_close_pin",
      type=int,
      default=3,
      help="Output pin index to command door close (default: 3).",
  )
  parser.add_argument(
      "--vise_open_pin",
      type=int,
      default=4,
      help="Output pin index to command vise open (default: 4).",
  )
  parser.add_argument(
      "--vise_close_pin",
      type=int,
      default=5,
      help="Output pin index to command vise close (default: 5).",
  )
  parser.add_argument(
      "--cycle_start_pin",
      type=int,
      default=6,
      help="Output pin index to command cycle start (default: 6).",
  )
  parser.add_argument(
      "--cycle_done_input_pin",
      type=int,
      default=0,
      help="Input pin index to wait for cycle complete (default: 0).",
  )
  parser.add_argument(
      "--timeout_seconds",
      type=float,
      default=30.0,
      help="Timeout in seconds for waiting operations (default: 30.0).",
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  """Main entry point for CNC machine control tool."""
  args = parse_args(argv)

  if args.mock:
    machine: CncMachineInterface = MockCncMachine()
    solution = None
  else:
    print(f"Connecting to solution at {args.address}...")
    solution = deployments.connect(address=args.address)
    machine = DioCncMachine(
        solution=solution,
        door_open_pin=args.door_open_pin,
        door_close_pin=args.door_close_pin,
        vise_open_pin=args.vise_open_pin,
        vise_close_pin=args.vise_close_pin,
        cycle_start_pin=args.cycle_start_pin,
        cycle_done_input_pin=args.cycle_done_input_pin,
        output_block_name=args.output_block_name,
        input_block_name=args.input_block_name,
        device_name=args.device_name,
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
