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

"""CNC Machine & Vise hardware interfaces and Digital I/O implementation."""

import abc
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import behavior_tree as bt
from intrinsic.world.proto import (
  object_world_refs_pb2,
  object_world_updates_pb2,
)

from src.core.config import MachineConfig
from src.utils.execution_utils import (
  object_exists_in_world,
  resolve_adio_resource,
)
from src.utils.script_utils import create_dwell_task


class CncMachineInterface(abc.ABC):
  """Abstract interface for CNC machine door, vise clamping, and cycle signals."""

  @abc.abstractmethod
  def build_open_door_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to command the CNC enclosure door open."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_close_door_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to command the CNC enclosure door closed."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_open_vise_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to open the CNC pneumatic/hydraulic vise."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_close_vise_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to clamp the CNC pneumatic/hydraulic vise."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_trigger_cycle_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to trigger CNC machining cycle start."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_wait_cycle_complete_task(
    self, timeout_seconds: float, name: str | None = None
  ) -> bt.Node:
    """Builds a task to wait for the CNC cycle complete signal."""
    raise NotImplementedError


class DioCncMachine(CncMachineInterface):
  """CNC machine controller using Discrete I/O pins and world joint sync."""

  def __init__(
    self,
    solution: Any,
    config: MachineConfig,
  ) -> None:
    """Initializes the DIO CNC machine adapter.

    Args:
        solution: Connected SBL deployment instance.
        config: Scoped MachineConfig defining DIO pins, block names, and
          belief-world joint targets for the enclosure door and vise.
    """
    self._solution = solution
    self._door_open_pin = config.door_open_pin
    self._door_close_pin = config.door_close_pin
    self._vise_open_pin = config.vise_open_pin
    self._vise_close_pin = config.vise_close_pin
    self._cycle_start_pin = config.cycle_start_pin
    self._cycle_done_input_pin = config.cycle_complete_input_pin
    self._output_block_name = config.output_block_name
    self._input_block_name = config.input_block_name
    self._device_name = config.device_name
    self._enclosure_object_name = config.enclosure_object_name
    self._vise_object_name = config.vise_object_name
    self._door_open_joints = tuple(config.door_open_joints)
    self._door_closed_joints = tuple(config.door_closed_joints)
    self._vise_open_joints = tuple(config.vise_open_joints)
    self._vise_closed_joints = tuple(config.vise_closed_joints)

    self._dio_set_skill = solution.skills.ai.intrinsic.dio_set_output
    self._dio_wait_skill = getattr(
      solution.skills.ai.intrinsic, "dio_wait_for_input", None
    )
    self._dio_read_skill = getattr(
      solution.skills.ai.intrinsic, "dio_read_input", None
    )
    self._update_world_skill = getattr(
      solution.skills.ai.intrinsic, "update_world", None
    )

  def _build_dio_set_task(
    self,
    pins: Sequence[int] | int,
    states: Sequence[bool] | bool,
    task_name: str,
  ) -> bt.Node:
    pin_list = [pins] if isinstance(pins, int) else [int(p) for p in pins]
    state_list = (
      [bool(states)] * len(pin_list)
      if isinstance(states, bool)
      else [bool(s) for s in states]
    )

    kwargs: dict[str, Any] = {}
    adio_resource = resolve_adio_resource(self._solution, self._device_name)
    if adio_resource is not None:
      kwargs["adio"] = adio_resource

    block_cls = getattr(
      getattr(
        getattr(self._dio_set_skill, "intrinsic_proto", None), "skills", None
      ),
      "DioOutputBlock",
      None,
    ) or getattr(self._dio_set_skill, "DioOutputBlock", None)

    if block_cls is not None:
      kwargs["dio_output_blocks"] = [
        block_cls(
          block_name=self._output_block_name,
          indices=pin_list,
          values=state_list,
        )
      ]
    else:
      kwargs["dio_output_blocks"] = [
        {
          "block_name": self._output_block_name,
          "indices": pin_list,
          "values": state_list,
        }
      ]
    return bt.Task(action=self._dio_set_skill(**kwargs), name=task_name)

  def _build_world_joint_update_task(
    self,
    object_name: str | None,
    joints: Sequence[float],
    task_name: str,
  ) -> bt.Node | None:
    if (
      self._update_world_skill is None
      or not object_name
      or not object_exists_in_world(self._solution, object_name)
    ):
      return None
    update_proto = object_world_updates_pb2.ObjectWorldUpdate(
      update_object_joints=object_world_updates_pb2.UpdateObjectJointsRequest(
        object=object_world_refs_pb2.ObjectReference(
          by_name=object_world_refs_pb2.ObjectReferenceByName(
            object_name=object_name
          )
        ),
        joint_positions=[float(v) for v in joints],
      )
    )
    return bt.Task(
      action=self._update_world_skill(update=update_proto),
      name=task_name,
    )

  def _build_actuation_task(
    self,
    active_pin: int,
    inactive_pin: int,
    task_name: str,
    object_name: str | None,
    joints: Sequence[float],
    update_name: str,
    dwell_time_sec: float = 0.0,
    dwell_name: str | None = None,
  ) -> bt.Node:
    dio_task = self._build_dio_set_task(
      pins=[active_pin, inactive_pin],
      states=[True, False],
      task_name=task_name,
    )
    children: list[bt.Node] = [dio_task]
    if dwell_time_sec > 0.0:
      children.append(
        create_dwell_task(
          dwell_time_sec=dwell_time_sec,
          solution=self._solution,
          task_name=dwell_name or f"{task_name} Dwell ({dwell_time_sec}s)",
        )
      )
    world_task = self._build_world_joint_update_task(
      object_name=object_name,
      joints=joints,
      task_name=update_name,
    )
    if world_task is not None:
      children.append(world_task)
    if len(children) == 1:
      return children[0]
    return bt.Sequence(
      name=task_name,
      children=children,
    )

  def build_open_door_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to open the CNC door via DIO, wait 10s, and sync belief-world joints."""
    task_name = name or "Open CNC Door (DIO)"
    return self._build_actuation_task(
      active_pin=self._door_open_pin,
      inactive_pin=self._door_close_pin,
      task_name=task_name,
      object_name=self._enclosure_object_name,
      joints=self._door_open_joints,
      update_name="Update CNC Door Joint (Open)",
      dwell_time_sec=10.0,
      dwell_name="Wait for CNC Door Open (10.0s)",
    )

  def build_close_door_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to close the CNC door via DIO, wait 10s, and sync belief-world joints."""
    task_name = name or "Close CNC Door (DIO)"
    return self._build_actuation_task(
      active_pin=self._door_close_pin,
      inactive_pin=self._door_open_pin,
      task_name=task_name,
      object_name=self._enclosure_object_name,
      joints=self._door_closed_joints,
      update_name="Update CNC Door Joint (Closed)",
      dwell_time_sec=10.0,
      dwell_name="Wait for CNC Door Closed (10.0s)",
    )

  def build_open_vise_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to open the CNC vise via DIO and sync belief-world joints."""
    task_name = name or "Open CNC Vise (DIO)"
    return self._build_actuation_task(
      active_pin=self._vise_open_pin,
      inactive_pin=self._vise_close_pin,
      task_name=task_name,
      object_name=self._vise_object_name,
      joints=self._vise_open_joints,
      update_name="Update CNC Vise Joint (Open)",
    )

  def build_close_vise_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to clamp the CNC vise via DIO and sync belief-world joints."""
    task_name = name or "Clamp CNC Vise (DIO)"
    return self._build_actuation_task(
      active_pin=self._vise_close_pin,
      inactive_pin=self._vise_open_pin,
      task_name=task_name,
      object_name=self._vise_object_name,
      joints=self._vise_closed_joints,
      update_name="Update CNC Vise Joint (Clamped)",
    )

  def build_trigger_cycle_task(self, name: str | None = None) -> bt.Node:
    """Builds a sequence pulsing the CNC cycle start digital output (0.5s high)."""
    task_name = name or "Trigger CNC Machining Cycle (DIO)"
    pulse_high = self._build_dio_set_task(
      pins=self._cycle_start_pin,
      states=True,
      task_name="Set Cycle Start Pin High",
    )
    dwell = create_dwell_task(
      dwell_time_sec=0.5,
      solution=self._solution,
      task_name="Cycle Start Pulse Dwell (0.5s)",
    )
    pulse_low = self._build_dio_set_task(
      pins=self._cycle_start_pin,
      states=False,
      task_name="Reset Cycle Start Pin Low",
    )
    return bt.Sequence(
      name=task_name,
      children=[pulse_high, dwell, pulse_low],
    )

  def _build_dio_read_task(self, task_name: str) -> bt.Node:
    kwargs: dict[str, Any] = {
      "block_name": self._input_block_name,
    }
    adio_resource = resolve_adio_resource(self._solution, self._device_name)
    if adio_resource is not None:
      kwargs["adio"] = adio_resource
    return bt.Task(action=self._dio_read_skill(**kwargs), name=task_name)

  def build_wait_cycle_complete_task(
    self, timeout_seconds: float, name: str | None = None
  ) -> bt.Node:
    """Builds a task waiting for CNC cycle completion via DIO input or dwell.

    Args:
        timeout_seconds: Maximum wait duration in seconds.
        name: Optional custom name for the Behavior Tree node.

    Returns:
        Configured SBL Task or Sequence node waiting for cycle completion.
    """
    task_name = name or "Wait for CNC Cycle Complete"
    if self._cycle_done_input_pin is None:
      return create_dwell_task(
        dwell_time_sec=timeout_seconds,
        solution=self._solution,
        task_name=f"{task_name} (Timed {timeout_seconds}s)",
      )

    if self._dio_wait_skill is not None:
      kwargs: dict[str, Any] = {
        "block_name": self._input_block_name,
        "indices": [self._cycle_done_input_pin],
        "values": [True],
        "timeout": timeout_seconds,
      }
      adio_resource = resolve_adio_resource(self._solution, self._device_name)
      if adio_resource is not None:
        kwargs["adio"] = adio_resource
      return bt.Task(action=self._dio_wait_skill(**kwargs), name=task_name)

    if self._dio_read_skill is not None:
      dwell_task = create_dwell_task(
        dwell_time_sec=timeout_seconds,
        solution=self._solution,
        task_name=f"{task_name} (Dwell {timeout_seconds}s)",
      )
      read_task = self._build_dio_read_task(
        task_name=f"{task_name} (Read DIO Input)",
      )
      return bt.Sequence(
        name=task_name,
        children=[dwell_task, read_task],
      )

    return create_dwell_task(
      dwell_time_sec=timeout_seconds,
      solution=self._solution,
      task_name=f"{task_name} (Fallback Dwell {timeout_seconds}s)",
    )
