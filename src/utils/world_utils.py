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

"""Utilities for resolving workspace/runfile paths and connecting to SBL worlds."""

import logging
import os
from typing import Any

from intrinsic.solutions import worlds


def find_file(filepath: str) -> str:
  """Resolves `filepath` across direct path, Bazel workspace/working dir, or runfiles.

  Args:
    filepath: Relative or absolute file path to resolve.

  Returns:
    Resolved file path string if found (or whose parent directory exists in
    `BUILD_WORKSPACE_DIRECTORY` or `BUILD_WORKING_DIRECTORY`), or the original
    `filepath` if no candidate location exists.
  """
  if os.path.isabs(filepath):
    return filepath

  working_dir = os.environ.get("BUILD_WORKING_DIRECTORY")
  if working_dir:
    w_path = os.path.join(working_dir, filepath)
    if os.path.exists(w_path):
      return w_path

  workspace_dir = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
  if workspace_dir:
    ws_path = os.path.join(workspace_dir, filepath)
    if os.path.exists(ws_path):
      return ws_path

  if os.path.dirname(filepath):
    if workspace_dir:
      ws_path = os.path.join(workspace_dir, filepath)
      if os.path.exists(os.path.dirname(ws_path)):
        return ws_path
    if working_dir:
      w_path = os.path.join(working_dir, filepath)
      if os.path.exists(os.path.dirname(w_path)):
        return w_path

  runfiles_dir = os.environ.get("PYTHON_RUNFILES") or os.environ.get(
    "TEST_SRCDIR"
  )
  if runfiles_dir:
    r_path = os.path.join(runfiles_dir, "_main", filepath)
    if os.path.exists(r_path):
      return r_path
    r_path2 = os.path.join(runfiles_dir, filepath)
    if os.path.exists(r_path2):
      return r_path2

  if os.path.exists(filepath):
    return filepath

  return filepath


def connect_initial_world(solution: Any) -> Any | None:
  """Connects to `init_world` (`worlds.EditWorldId.INITIAL`) if available.

  Args:
    solution: Connected SBL solution deployment handle.

  Returns:
    Connected initial `ObjectWorld` instance, or `None` if `grpc_channel` is
    unavailable or connection fails.
  """
  grpc_channel = getattr(solution, "grpc_channel", None)
  if grpc_channel is None:
    return None
  try:
    return worlds.ObjectWorld.connect(worlds.EditWorldId.INITIAL, grpc_channel)
  except Exception as e:  # pylint: disable=broad-exception-caught
    logging.warning("Could not connect to initial world ('init_world'): %s", e)
    return None
