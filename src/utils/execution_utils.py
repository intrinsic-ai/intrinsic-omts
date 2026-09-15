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

"""Helpers for mapping OMTS options onto Intrinsic executive settings."""

from intrinsic.solutions import execution

from src.core.types import SimulationMode

_SIMULATION_MODE_MAP: dict[
  SimulationMode, execution.Executive.SimulationMode
] = {
  SimulationMode.REALITY: execution.Executive.SimulationMode.REALITY,
  SimulationMode.PREVIEW: execution.Executive.SimulationMode.PREVIEW,
  SimulationMode.FAST_PREVIEW: execution.Executive.SimulationMode.FAST_PREVIEW,
}


def to_executive_simulation_mode(
  mode: SimulationMode | None,
) -> execution.Executive.SimulationMode | None:
  """Converts an OMTS simulation mode to the executive SDK enum.

  Args:
      mode: OMTS simulation mode, or None to keep whatever mode is currently
        set in the executive.

  Returns:
      The matching `Executive.SimulationMode`, or None if `mode` is None.

  Raises:
      ValueError: If the mode has no executive counterpart.
  """
  if mode is None:
    return None

  try:
    return _SIMULATION_MODE_MAP[mode]
  except KeyError as err:
    raise ValueError(f"Unsupported simulation mode: {mode}") from err
