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

import atexit
import contextlib
import enum
import os
import select
import sys
import termios
import time
import tty
from collections.abc import Sequence
from typing import Self

import grpc
from absl import app, flags, logging
from intrinsic.icon.actions import joint_jogging_pb2
from intrinsic.icon.proto import joint_space_pb2
from intrinsic.icon.python import actions, errors, icon_api
from intrinsic.kinematics.types import joint_limits_pb2
from intrinsic.util.grpc import connection

_HOST = flags.DEFINE_string(
  "host",
  "localhost",
  "ICON server gRPC connection host.",
)
_PORT = flags.DEFINE_integer(
  "port",
  17080,
  "ICON server gRPC connection port.",
)
_INSTANCE = flags.DEFINE_string(
  "instance",
  None,
  "The instance of ICON if behind an ingress.",
  required=True,
)
_PART_NAME = flags.DEFINE_string(
  "part_name",
  "",
  "Specific part name to control in ICON. If empty, uses the first controllable part.",
)
_MAX_VELOCITY = flags.DEFINE_float(
  "max_velocity",
  0.1,
  "Joint jogging velocity in rad/s (default 0.1 rad/s ≈ 5.7 deg/s).",
)
_DEADMAN_TIMEOUT_SEC = flags.DEFINE_float(
  "deadman_timeout_sec",
  0.2,
  "Time window in seconds after which velocity resets to zero if no key is received.",
)
_POLLING_RATE_SEC = 0.05


class Key(enum.StrEnum):
  """Semantic representation of keyboard input keys with ANSI sequence values."""

  # Cursor keys in CSI
  RIGHT = "\x1b[C"
  LEFT = "\x1b[D"
  # Standard control keys
  SPACE = " "
  ESCAPE = "\x1b"
  QUIT = "q"
  UNKNOWN = ""

  @classmethod
  def _missing_(cls, value: object) -> Self:
    """Maps alternative sequences to existing Key members."""
    if not isinstance(value, str):
      return cls.UNKNOWN

    aliases = {
      # Cursor keys in SS3
      "\x1bOC": cls.RIGHT,
      "\x1bOD": cls.LEFT,
      # Quit aliases
      "Q": cls.QUIT,
      "\x03": cls.QUIT,  # Ctrl+C
    }
    if value in aliases:
      return aliases[value]

    # If keys are held down, multiple sequences may be buffered together.
    for member in (cls.RIGHT, cls.LEFT, cls.SPACE, cls.QUIT):
      if value.startswith(member.value):
        return member
    for seq, member in aliases.items():
      if value.startswith(seq):
        return member

    return cls.UNKNOWN


@contextlib.contextmanager
def raw_terminal_mode(fd: int = sys.stdin.fileno()):
  """Sets terminal to cbreak mode and guarantees restoration upon exit.

  cbreak (rare) mode provides:
    - Unbuffered character-at-a-time input: Keys register immediately without
      requiring the user to press `Enter`.
    - Echo suppression: Suppresses ANSI escape sequences (^[[C, etc.) from
      cluttering the live display.
    - Unlike fully raw mode (tty.setraw), cbreak preserves OS signal handling
      for `Ctrl+C` and `Ctrl+Z`, ensuring emergency stops and
      `KeyboardInterrupt` remain fully functional.

  Registers an atexit fallback hook to guarantee terminal settings are restored
  even if the process exits unexpectedly.
  """
  old_settings = termios.tcgetattr(fd)

  def restore_terminal() -> None:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

  atexit.register(restore_terminal)
  try:
    tty.setcbreak(fd)
    yield
  finally:
    restore_terminal()
    atexit.unregister(restore_terminal)


def read_key(timeout: float | None = _POLLING_RATE_SEC) -> Key | None:
  """Reads a single keypress or escape sequence with a non-blocking timeout.

  Args:
    timeout: Maximum time to wait in seconds (None = block until input arrives).

  Returns:
    A Key enum member, or None if the timeout expired without input.
  """
  _ESCAPE_SEQUENCE_TIMEOUT_SEC = 0.015

  fd = sys.stdin.fileno()
  rlist, _, _ = select.select([fd], [], [], timeout)
  if not rlist:
    return None

  data = os.read(fd, 1024).decode("utf-8", errors="replace")
  if not data:
    return None

  # Distinguish standalone `Escape` from multi-byte ANSI sequences (e.g., arrow
  # keys). Over network connections like SSH, trailing bytes may arrive across
  # packet boundaries a few milliseconds after the leading `\x1b`.
  if data == Key.ESCAPE:
    if select.select([fd], [], [], _ESCAPE_SEQUENCE_TIMEOUT_SEC)[0]:
      data += os.read(fd, 1024).decode("utf-8", errors="replace")

  return Key(data)


def resolve_part_name(
  icon_client: icon_api.Client, requested_part: str | None = None
) -> str:
  """Resolves the controllable robot part to use from available parts."""
  parts = icon_client.list_parts()
  if not parts:
    raise LookupError("No parts found on the ICON server.")

  logging.info("Available parts on ICON server:")
  for p in parts:
    logging.info("  - %s", p)

  controllable_parts = [p for p in parts if p != "icon"]
  if not controllable_parts:
    raise LookupError(
      f"No controllable robot parts found on ICON server (available parts: {parts})."
    )

  if requested_part:
    if requested_part not in controllable_parts:
      raise ValueError(
        f"Requested part '{requested_part}' is not a controllable part on the"
        f" server (controllable parts: {controllable_parts})."
      )
    return requested_part

  # Default to 'arm' if available, otherwise pick the first controllable part.
  _DEFAULT_PART = "arm"
  if _DEFAULT_PART in controllable_parts:
    return _DEFAULT_PART
  return controllable_parts[0]


def get_part_joint_info(
  icon_client: icon_api.Client, part_name: str
) -> tuple[int, joint_limits_pb2.JointLimits]:
  """Retrieves the number of DoFs and application joint limits for a part."""
  ndof = None
  part_config = None
  for config in icon_client.get_config().part_configs:
    if config.name == part_name:
      if config.HasField("generic_config"):
        part_config = config
        ndof = config.generic_config.joint_position_config.num_joints
        logging.info("Part '%s' has %d DoFs.", part_name, ndof)
      break

  if ndof is None or part_config is None:
    raise ValueError(
      f"Could not retrieve configuration for part '{part_name}'."
    )

  if not part_config.generic_config.HasField("joint_limits_config"):
    raise ValueError(f"Part '{part_name}' is missing joint_limits_config.")

  return ndof, part_config.generic_config.joint_limits_config.application_limits


def send_velocity_command(
  stream: icon_api.Stream, velocities: Sequence[float]
) -> None:
  """Sends a streaming joint velocity command to the action stream.

  Velocity streaming is preferred over position-based streaming due to:
    - Responsiveness: Position streaming would require client-side numerical
      integration, where network jitter or latency causes target lag, tracking
      error, and overshoot when stopping.
    - Server-side real-time interpolation: The ICON server executes an online
      trajectory generator at the hardware control frequency to generate smooth
      motion profiles toward the commanded velocities while respecting limits.
    - Built-in watchdog: If packets drop or network latency spikes, the server's
      internal watchdog counter expires and automatically commands a smooth
      deceleration to zero velocity, preventing runaway motions without hard
      emergency-stop faults.
  """
  cmd = joint_jogging_pb2.JointJoggingStreamingParams(
    goal_velocity=joint_space_pb2.JointVec(joints=velocities)
  )
  stream.write(cmd)


def jog_joint_loop(
  stream: icon_api.Stream,
  joint_idx: int,
  ndof: int,
  max_velocity: float,
  deadman_timeout_sec: float,
  polling_rate_sec: float = _POLLING_RATE_SEC,
) -> None:
  """Runs the real-time interactive jogging loop for a single joint."""
  logging.info("--- Live control for joint %d ---", joint_idx)
  logging.info(
    "Hold Left/Right arrow keys to jog (±%.3f rad/s)."
    " Release or press Space to stop, 'q' to return.",
    max_velocity,
  )

  with raw_terminal_mode():
    current_direction = 0.0
    last_keypress_time = time.monotonic()

    while True:
      key = read_key(timeout=polling_rate_sec)

      if key in (Key.QUIT, Key.ESCAPE):
        send_velocity_command(stream, [0.0] * ndof)
        # Finalize the in-place status line before standard logging resumes.
        sys.stdout.write("\n")
        logging.info("Exiting live control for this joint.")
        break

      if key == Key.RIGHT:
        current_direction = 1.0
        last_keypress_time = time.monotonic()
      elif key == Key.LEFT:
        current_direction = -1.0
        last_keypress_time = time.monotonic()
      elif key == Key.SPACE:
        current_direction = 0.0
        last_keypress_time = time.monotonic()
      elif key not in (None, Key.UNKNOWN):
        # Any other key immediately stops the motion.
        current_direction = 0.0

      if time.monotonic() - last_keypress_time > deadman_timeout_sec:
        current_direction = 0.0

      # Stream updated velocity to ICON.
      goal_velocity = [0.0] * ndof
      goal_velocity[joint_idx] = current_direction * max_velocity
      send_velocity_command(stream, goal_velocity)

      # Use `sys.stdout.write` with carriage return (`\r`) to rewrite the status
      # in-place. Standard logging would append newlines and timestamps,
      # flooding the terminal scrollback.
      status = "JOGGING" if current_direction != 0.0 else "IDLE"
      sys.stdout.write(
        f"\r[{status}] Joint {joint_idx}: {goal_velocity[joint_idx]:+.3f} rad/s   "
      )
      sys.stdout.flush()


def run_jogging_session(
  icon_client: icon_api.Client,
  part_name: str,
  ndof: int,
  app_limits: joint_limits_pb2.JointLimits,
) -> None:
  """Starts an ICON session and manages the interactive joint selection loop."""
  fixed_params = joint_jogging_pb2.JointJoggingFixedParams(
    joint_limits=app_limits
  )
  action_desc = actions.Action(
    action_id=0,
    action_type="intrinsic.joint_jogging",
    part_name_or_slot_part_map={"arm": part_name},
    params=fixed_params,
  )

  with icon_client.start_session([part_name]) as session:
    action = session.add_action(action_desc)
    session.start_action(action.id)
    stream = session.open_stream(action.id, "joint_jogging_command")

    while True:
      try:
        logging.info("-" * 50)
        joint_input = input(
          f"Enter joint index to rotate (0 to {ndof - 1}) or '{Key.QUIT}' to quit: "
        ).strip()
        if joint_input.lower() == Key.QUIT:
          logging.info("Exiting jogging...")
          break

        joint_idx = int(joint_input)
        if not (0 <= joint_idx < ndof):
          logging.error(
            "Invalid joint index. Must be between 0 and %d.", ndof - 1
          )
          continue

        jog_joint_loop(
          stream=stream,
          joint_idx=joint_idx,
          ndof=ndof,
          max_velocity=_MAX_VELOCITY.value,
          deadman_timeout_sec=_DEADMAN_TIMEOUT_SEC.value,
        )

      except ValueError:
        logging.error("Invalid input. Please enter a valid number.")
      except errors.Session.ActionError as e:
        logging.error("Action error: %s", e)

    stream.end()


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  logging.info(
    "Connecting to ICON at %s:%d (instance: '%s')...",
    _HOST.value,
    _PORT.value,
    _INSTANCE.value,
  )
  icon_client = icon_api.Client.connect_with_params(
    connection.ConnectionParams(f"{_HOST.value}:{_PORT.value}", _INSTANCE.value)
  )

  try:
    part_name = resolve_part_name(icon_client, _PART_NAME.value or None)
    ndof, app_limits = get_part_joint_info(icon_client, part_name)
    logging.info(
      "Connected! Controlling part '%s' with %d joints.", part_name, ndof
    )
    run_jogging_session(icon_client, part_name, ndof, app_limits)
  except grpc.RpcError as e:
    logging.error("ICON RPC error: %s", e)
  except KeyboardInterrupt:
    logging.info("Exiting.")


if __name__ == "__main__":
  app.run(main)
