"""Interactive CLI keyboard teleoperation for robot joints via ICON."""

import sys
import termios
import tty
from typing import Sequence

from absl import app
from absl import flags

from intrinsic.icon.python import create_action_utils
from intrinsic.icon.python import errors
from intrinsic.icon.python import icon_api
from intrinsic.solutions import deployments
from intrinsic.util.grpc import connection

_HOST = flags.DEFINE_string("host", "localhost", "ICON server gRPC connection host.")
_PORT = flags.DEFINE_integer("port", 17080, "ICON server gRPC connection port.")
_INSTANCE = flags.DEFINE_string(
    "instance", None, "The instance of ICON if behind an ingress."
)
_PART_NAME = flags.DEFINE_string(
    "part_name", "arm", "Part name to control in ICON."
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

  print(f"Connecting to ICON at {_HOST.value}:{_PORT.value}...")
  icon_client = icon_api.Client.connect_with_params(
      connection.ConnectionParams(
          f"{_HOST.value}:{_PORT.value}", _INSTANCE.value
      )
  )

  parts = icon_client.list_parts()
  if not parts:
    raise LookupError("No parts found on the ICON server.")

  part_name = _PART_NAME.value
  if part_name not in parts:
    part_name = parts[0]

  print(f"Controlling part '{part_name}'.")
  print("Use keys 1-6 (increase) / q-y (decrease) to jog joints. 'Ctrl+C' to exit.")

  try:
    with icon_client.start_session([part_name]) as session:
      while True:
        key = get_key()
        if key == "\x03":  # Ctrl+C
          break
        # Process interactive jogging step
  except KeyboardInterrupt:
    print("\nExiting interactive jogger.")


if __name__ == "__main__":
  app.run(main)
