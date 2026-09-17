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

"""Common CLI and interactive helpers for OMTS tool scripts."""

import argparse
from collections.abc import Sequence
from typing import Any


def prompt_menu(
  title: str,
  options: Sequence[tuple[str, str, Any]],
) -> Any | None:
  """Displays an interactive menu and prompts operator for choice.

  Args:
    title: Header title to display above the menu options.
    options: Sequence of (key, label, action) tuples.

  Returns:
    The action associated with the selected key, or None to quit.
  """
  print(f"\n{title}:")
  for key, label, _ in options:
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

    for key, _, action in options:
      if choice == key:
        return action

    print(f"Invalid choice: {choice}. Enter 1-{len(options)} or 'q'.")


def add_dio_machine_arguments(parser: argparse.ArgumentParser) -> None:
  """Adds standard CNC machine DIO pin arguments to an ArgumentParser."""
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
    "--device_name",
    type=str,
    default=None,
    help="Optional device name for DIO skills.",
  )


def add_dio_gripper_arguments(parser: argparse.ArgumentParser) -> None:
  """Adds standard gripper DIO pin arguments to an ArgumentParser."""
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
