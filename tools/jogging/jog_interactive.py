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

"""Interactive CLI keyboard teleoperation for robot joints via ICON."""

import sys
import termios
import tty
from collections.abc import Sequence

import grpc
from absl import app, flags
from intrinsic.icon.python import create_action_utils, errors, icon_api
from intrinsic.util.grpc import connection

_HOST = flags.DEFINE_string(
  "host", "localhost", "ICON server gRPC connection host."
)
_PORT = flags.DEFINE_integer("port", 17080, "ICON server gRPC connection port.")
_INSTANCE = flags.DEFINE_string(
  "instance", "icon", "The instance of ICON if behind an ingress."
)
_PART_NAME = flags.DEFINE_string(
  "part_name",
  "",
  "Specific part name to control in ICON. If empty, uses the first controllable part.",
)


def get_key() -> str:
  """Reads a single keypress or escape sequence from standard input."""
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    if ch == "\x1b":
      ch += sys.stdin.read(2)
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  print(
    f"Connecting to ICON at {_HOST.value}:{_PORT.value} (instance: '{_INSTANCE.value}')..."
  )
  icon_client = icon_api.Client.connect_with_params(
    connection.ConnectionParams(f"{_HOST.value}:{_PORT.value}", _INSTANCE.value)
  )

  parts = icon_client.list_parts()
  if not parts:
    raise LookupError("No parts found on the ICON server.")

  print("Available parts on ICON server:")
  for p in parts:
    print(f"  - {p}")

  if _PART_NAME.value and _PART_NAME.value in parts:
    part_name = _PART_NAME.value
  elif len(parts) > 1 and parts[0] == "icon" and "arm" in parts:
    part_name = "arm"
  else:
    part_name = parts[1] if len(parts) > 1 else parts[0]

  # Get part config to determine number of DoFs
  ndof = None
  part_configs = icon_client.get_config().part_configs
  for config in part_configs:
    if config.name == part_name:
      if config.HasField("generic_config"):
        ndof = config.generic_config.joint_position_config.num_joints
        print(f"Part '{part_name}' has {ndof} DoFs.")
      break

  if ndof is None:
    raise ValueError(
      f"Could not retrieve configuration for part '{part_name}'."
    )

  print(f"\nConnected! Controlling part '{part_name}' with {ndof} joints.\n")

  action_id_counter = 0

  try:
    with icon_client.start_session(parts) as session:
      while True:
        try:
          print("-" * 50)
          joint_input = input(
            f"Enter joint index to rotate (0 to {ndof - 1}) or 'q' to quit: "
          ).strip()
          if joint_input.lower() == "q":
            print("Exiting jogging...")
            break

          joint_idx = int(joint_input)
          if not (0 <= joint_idx < ndof):
            print(
              f"Error: Invalid joint index. Must be between 0 and {ndof - 1}."
            )
            continue

          print(f"\n--- Live control for joint {joint_idx} ---")
          print(
            "Use Left/Right arrow keys to jog by 0.01 rad. Press 'q' or 'x' to return."
          )

          while True:
            key = get_key()
            if key.lower() in ("q", "x") or key == "\x03":
              print("\nExiting live control for this joint.\n")
              break

            delta_value = 0.0
            if key == "\x1b[C":  # Right arrow
              delta_value = 0.01
            elif key == "\x1b[D":  # Left arrow
              delta_value = -0.01
            else:
              continue

            # Fetch current status
            status = icon_client.get_status()
            part_status = status.part_status[part_name]
            current_positions = [
              j.position_sensed for j in part_status.joint_states
            ]

            if not current_positions or len(current_positions) != ndof:
              sys.stdout.write(
                "\rError: Could not retrieve current joint positions from robot."
              )
              sys.stdout.flush()
              continue

            goal_position = list(current_positions)
            goal_position[joint_idx] += delta_value
            goal_velocity = [0.0] * ndof

            action = session.add_action(
              create_action_utils.create_point_to_point_move_action(
                action_id=action_id_counter,
                joint_position_part_name=part_name,
                goal_position=goal_position,
                goal_velocity=goal_velocity,
              )
            )
            action_id_counter += 1

            session.start_action(action_id=action.id)
            sys.stdout.write(
              f"\rJogged joint {joint_idx} by {delta_value:+.2f} rad. Current: {goal_position[joint_idx]:.3f}   "
            )
            sys.stdout.flush()

        except ValueError:
          print("Error: Invalid input. Please enter a valid number.")
        except errors.Session.ActionError as e:
          print(f"\nAction error: {e}")

  except grpc.RpcError as e:
    print(f"\nICON RPC error: {e}")
  except KeyboardInterrupt:
    print("\nExiting.")


if __name__ == "__main__":
  app.run(main)
