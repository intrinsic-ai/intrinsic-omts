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

"""Utilities for extracting and injecting Python script code into SBL BT tasks."""

import importlib
import inspect
import os
import sys
import types
from collections.abc import Callable
from typing import Any


def _read_source_from_file(filepath: str) -> str:
  """Reads file content, checking direct path and Bazel runfiles."""
  if os.path.exists(filepath):
    with open(filepath, encoding="utf-8") as f:
      return f.read()

  # Check Bazel runfiles
  runfiles_dir = os.environ.get("PYTHON_RUNFILES") or os.environ.get(
    "TEST_SRCDIR"
  )
  if runfiles_dir:
    r_path = os.path.join(runfiles_dir, "_main", filepath)
    if os.path.exists(r_path):
      with open(r_path, encoding="utf-8") as f:
        return f.read()

  raise FileNotFoundError(f"Could not locate source file: {filepath}")


def load_python_script(
  source: types.ModuleType | Callable[..., Any] | str,
  function_name: str | None = None,
  call_args: str | None = "context, params",
) -> str:
  """Extracts executable Python source code from a module, function, or file for SBL bt.PythonScript.

  This helper enables writing SBL Behavior Tree PythonScript actions as standard,
  type-checked, lintable Python modules instead of large raw string literals.

  Args:
      source: A Python module object, callable function within a module, or module
        dot-path / filepath string.
      function_name: Optional name of the entrypoint function to invoke. If `source`
        is a function, its name is automatically used by default.
      call_args: Arguments string to pass when appending the invocation call, e.g.
        "context, params". If None or empty, no invocation statement is appended.

  Returns:
      Source code string suitable for bt.PythonScript(function_body=...).

  Raises:
      ValueError: If the source cannot be resolved or is invalid.
  """
  target_func_name = function_name
  module_source = ""

  if callable(source) and not isinstance(source, types.ModuleType):
    target_func_name = target_func_name or getattr(source, "__name__", None)
    mod = inspect.getmodule(source) or sys.modules.get(
      getattr(source, "__module__", "")
    )
    if mod is not None:
      try:
        module_source = inspect.getsource(mod)
      except (OSError, TypeError):
        mod_file = getattr(mod, "__file__", None)
        if mod_file:
          module_source = _read_source_from_file(mod_file)
    if not module_source:
      try:
        module_source = inspect.getsource(source)
      except (OSError, TypeError) as err:
        raise ValueError(
          f"Unable to extract source for function {source}: {err}"
        ) from err

  elif isinstance(source, types.ModuleType):
    try:
      module_source = inspect.getsource(source)
    except (OSError, TypeError):
      mod_file = getattr(source, "__file__", None)
      if mod_file:
        module_source = _read_source_from_file(mod_file)
      else:
        raise ValueError(f"Unable to extract source for module {source}")

  elif isinstance(source, str):
    if source.endswith(".py") or os.path.exists(source):
      module_source = _read_source_from_file(source)
    else:
      # Treat as module dot-path
      mod = importlib.import_module(source)
      return load_python_script(
        source=mod,
        function_name=function_name,
        call_args=call_args,
      )
  else:
    raise ValueError(f"Unsupported source type: {type(source)}")

  code = module_source.rstrip()
  if target_func_name and call_args:
    call_stmt = f"{target_func_name}({call_args})"
    code = f"{code}\n\n{call_stmt}\n"

  return code
