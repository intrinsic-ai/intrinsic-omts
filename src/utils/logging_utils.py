"""Logging and telemetry formatting utilities for OMTS."""

import datetime
import sys
from typing import Any


def format_step_header(step_number: int, step_name: str) -> str:
  """Formats a standardized console header for an execution step."""
  timestamp = datetime.datetime.now().strftime("%H:%M:%S")
  divider = "=" * 60
  return f"\n{divider}\n[{timestamp}] Step {step_number:02d}: {step_name}\n{divider}"


def log_step(step_number: int, step_name: str) -> None:
  """Prints a standardized step execution message to stdout."""
  print(format_step_header(step_number, step_name), flush=True)


def log_info(message: str) -> None:
  """Logs an informational message."""
  timestamp = datetime.datetime.now().strftime("%H:%M:%S")
  print(f"[{timestamp}] [INFO] {message}", flush=True)


def log_error(message: str, error: Any = None) -> None:
  """Logs an error message to stderr."""
  timestamp = datetime.datetime.now().strftime("%H:%M:%S")
  if error is not None:
    print(f"[{timestamp}] [ERROR] {message}: {error}", file=sys.stderr, flush=True)
  else:
    print(f"[{timestamp}] [ERROR] {message}", file=sys.stderr, flush=True)
