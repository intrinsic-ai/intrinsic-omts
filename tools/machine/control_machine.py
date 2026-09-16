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

"""Interactive and CLI tool to command CNC machine door, vise, cycle, and prep."""

import argparse
import sys
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments, execution

from src.core.world import World
from src.hardware.machine import (
  CncMachineInterface,
  DioCncMachine,
  MachineConfig,
  MockCncMachine,
  run_initial_machine_prep,
)
from src.hardware.robot import MotionConfig, RobotInterface
from tools.common.cli import (
  add_dio_machine_arguments,
)
from tools.common.cli import (
  prompt_menu as common_prompt_menu,
)

VALID_ACTIONS: tuple[str, ...] = (
  "open_door",
  "close_door",
  "open_vise",
  "close_vise",
  "trigger_cycle",
  "wait_cycle",
  "sync_open",
  "sync_closed",
  "prep",
)

MENU_OPTIONS: list[tuple[str, str, str]] = [
  ("1", "Open CNC Door", "open_door"),
  ("2", "Close CNC Door", "close_door"),
  ("3", "Open CNC Vise", "open_vise"),
  ("4", "Close CNC Vise", "close_vise"),
  ("5", "Trigger CNC Machining Cycle", "trigger_cycle"),
  ("6", "Wait for CNC Cycle Complete", "wait_cycle"),
  ("7", "Sync Digital Twin Joints (Open)", "sync_open"),
  ("8", "Sync Digital Twin Joints (Closed)", "sync_closed"),
  (
    "9",
    "Prep Cell & Machine (Reset World, Retract Arm, Close Door/Vise)",
    "prep",
  ),
]


def _sync_twin_joints(
  solution: Any, object_name: str, joints: list[float]
) -> None:
  """Synchronizes digital twin joint positions in ObjectWorld."""
  if solution is None or getattr(solution, "world", None) is None:
    return
  try:
    w = World(solution.world, solution=solution)
    task = w.build_joint_update_task(object_name, joints)
    solution.executive.run(task)
  except Exception as sync_err:  # pylint: disable=broad-exception-caught
    print(
      f"Warning: Failed to synchronize digital twin joints for '{object_name}':"
      f" {sync_err}"
    )


def execute_action(
  machine: CncMachineInterface,
  action: str,
  solution: Any = None,
  timeout_seconds: float = 30.0,
  args: argparse.Namespace | None = None,
) -> bool:
  """Builds and runs the appropriate task from machine."""
  if action == "sync_open":
    if isinstance(machine, MockCncMachine):
      machine.command_log.append("sync_open")
      print("Successfully executed action: sync_open")
      return True
    _sync_twin_joints(solution, "cnc_enclosure", [0.4])
    _sync_twin_joints(solution, "schunk_egp_64nnb", [0.01, 0.01])
    print("Successfully synchronized digital twin joints to OPEN.")
    return True

  if action == "sync_closed":
    if isinstance(machine, MockCncMachine):
      machine.command_log.append("sync_closed")
      print("Successfully executed action: sync_closed")
      return True
    _sync_twin_joints(solution, "cnc_enclosure", [0.0])
    _sync_twin_joints(solution, "schunk_egp_64nnb", [0.00, 0.00])
    print("Successfully synchronized digital twin joints to CLOSED.")
    return True

  if action == "prep":
    if isinstance(machine, MockCncMachine):
      machine.command_log.append("prep")
      print("Successfully executed action: prep")
      return True
    if solution is None:
      print("Cannot execute 'prep': solution deployment is required.")
      return False
    reset_world = getattr(args, "reset_world", True) if args else True
    close_door_and_vise = (
      getattr(args, "close_door_and_vise", True) if args else True
    )
    view_frame = getattr(args, "view_frame", "view") if args else "view"
    arm_part = (
      getattr(args, "arm_part_name", "ur_module") if args else "ur_module"
    )

    robot = RobotInterface.from_config(
      solution=solution,
      config=MotionConfig(arm_part_name=arm_part),
      mock_hardware=False,
    )
    if reset_world:
      World(solution.world, solution=solution).reset(
        robot=robot,
        solution=solution,
      )

    run_initial_machine_prep(
      solution=solution,
      machine=machine,
      robot=robot,
      view_frame=view_frame,
      close_door_and_vise=close_door_and_vise,
    )
    print("Successfully executed action: prep")
    return True

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
    if action == "open_door":
      _sync_twin_joints(solution, "cnc_enclosure", [0.4])
    elif action == "close_door":
      _sync_twin_joints(solution, "cnc_enclosure", [0.0])
    elif action == "open_vise":
      _sync_twin_joints(solution, "schunk_egp_64nnb", [0.01, 0.01])
    elif action == "close_vise":
      _sync_twin_joints(solution, "schunk_egp_64nnb", [0.00, 0.00])
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
  return common_prompt_menu("CNC Machine Control Menu", MENU_OPTIONS)


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
    "--mock_hardware",
    action="store_true",
    default=False,
    help="Alias for --mock.",
  )
  parser.add_argument(
    "--timeout_seconds",
    type=float,
    default=30.0,
    help="Timeout in seconds for waiting operations (default: 30.0).",
  )
  parser.add_argument(
    "--view_frame",
    type=str,
    default="view",
    help="View frame for safe retract during prep (default: view).",
  )
  parser.add_argument(
    "--arm_part_name",
    type=str,
    default="ur_module",
    help="Arm component name for prep retract (default: ur_module).",
  )
  parser.add_argument(
    "--machine_type",
    type=str,
    default="dio",
    choices=["none", "mock", "dio"],
    help="CNC machine adapter type (default: dio).",
  )
  parser.add_argument(
    "--reset_world",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="Purge stale infeed objects and reset world state during prep.",
  )
  parser.add_argument(
    "--close_door_and_vise",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="Retract arm and close CNC door/vise during prep.",
  )
  add_dio_machine_arguments(parser)
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  """Main entry point for CNC machine control tool."""
  args = parse_args(argv)
  use_mock = args.mock or args.mock_hardware or args.machine_type == "mock"

  if use_mock:
    machine: CncMachineInterface = MockCncMachine()
    solution = None
  else:
    print(f"Connecting to solution at {args.address}...")
    solution = deployments.connect(address=args.address)
    machine = DioCncMachine(
      solution=solution,
      config=MachineConfig(
        machine_type=args.machine_type,
        door_open_pin=args.door_open_pin,
        door_close_pin=args.door_close_pin,
        vise_open_pin=args.vise_open_pin,
        vise_close_pin=args.vise_close_pin,
        cycle_start_pin=args.cycle_start_pin,
        output_block_name=args.output_block_name,
        input_block_name=args.input_block_name,
        device_name=args.device_name,
      ),
    )

  if args.action:
    success = execute_action(
      machine=machine,
      action=args.action,
      solution=solution,
      timeout_seconds=args.timeout_seconds,
      args=args,
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
      args=args,
    )


if __name__ == "__main__":
  main()
