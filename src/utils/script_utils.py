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

"""Utilities for extracting and injecting Python code into SBL BT tasks."""

import importlib
import inspect
import os
import re
import sys
import types
from collections.abc import Callable, Sequence
from typing import Any

_FIRST_PARTY_IMPORT = re.compile(r"^\s*(?:from|import)\s+src\b")


def _read_source_from_file(filepath: str) -> str:
  """Reads file content, checking direct path and Bazel runfiles."""
  if os.path.exists(filepath):
    with open(filepath, encoding="utf-8") as f:
      return f.read()

  runfiles_dir = os.environ.get("PYTHON_RUNFILES") or os.environ.get(
    "TEST_SRCDIR"
  )
  if runfiles_dir:
    r_path = os.path.join(runfiles_dir, "_main", filepath)
    if os.path.exists(r_path):
      with open(r_path, encoding="utf-8") as f:
        return f.read()

  raise FileNotFoundError(f"Could not locate source file: {filepath}")


def _strip_first_party_imports(source: str) -> str:
  """Drops `import src...` lines from a module's source."""
  return "\n".join(
    line for line in source.splitlines() if not _FIRST_PARTY_IMPORT.match(line)
  )


def _module_source(source: types.ModuleType) -> str:
  """Returns a module's source text, falling back to reading its file."""
  try:
    return inspect.getsource(source)
  except (OSError, TypeError):
    mod_file = getattr(source, "__file__", None)
    if not mod_file:
      raise ValueError(
        f"Unable to extract source for module {source}"
      ) from None
    return _read_source_from_file(mod_file)


def load_python_script(
  source: types.ModuleType | Callable[..., Any] | str,
  function_name: str | None = None,
  call_args: str | None = "context, params",
  preludes: Sequence[types.ModuleType] = (),
) -> str:
  """Extracts executable Python source code for SBL bt.PythonScript.

  Args:
    source: A Python module object, callable function within a module, or module
      dot-path / filepath string.
    function_name: Optional name of the entrypoint function to invoke. If
      `source` is a function, its name is automatically used by default.
    call_args: Arguments string to pass when appending the invocation call, e.g.
      "context, params". If None or empty, no invocation statement is appended.
    preludes: Explicit modules whose source is prepended first.

  Returns:
    Source code string suitable for bt.PythonScript(function_body=...).
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
      module_source = inspect.getsource(source)
  elif isinstance(source, types.ModuleType):
    module_source = _module_source(source)
  elif isinstance(source, str):
    if source.endswith(".py") or os.path.exists(source):
      module_source = _read_source_from_file(source)
    else:
      mod = importlib.import_module(source)
      return load_python_script(
        source=mod,
        function_name=function_name,
        call_args=call_args,
        preludes=preludes,
      )
  else:
    raise ValueError(f"Unsupported source type: {type(source)}")

  parts = [_module_source(prelude) for prelude in preludes]
  parts.append(module_source)
  code = "\n\n".join(
    _strip_first_party_imports(part).rstrip() for part in parts
  )
  if target_func_name and call_args:
    code = f"{code}\n\n{target_func_name}({call_args})\n"

  compile(code, "<injected script>", "exec")
  return code


def create_dwell_task(
  dwell_time_sec: float,
  solution: Any | None = None,
  task_name: str | None = None,
) -> Any:
  """Creates a Task node that pauses execution for a specified duration."""
  from intrinsic.solutions import behavior_tree as bt

  del solution
  name = task_name or f"Dwell ({dwell_time_sec}s)"
  action = bt.PythonScript(
    function_body=f"import time\ntime.sleep({float(dwell_time_sec)})\n",
  )
  task = bt.Task(action=action, name=name)
  task.root = task
  return task
