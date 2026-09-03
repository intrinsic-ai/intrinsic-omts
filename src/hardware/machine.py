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

"""CNC Machine & Vise hardware interfaces and implementations."""

import abc
from typing import Any, Optional
from intrinsic.solutions import behavior_tree as bt


class CncMachineInterface(abc.ABC):
  """Abstract interface for CNC machine door, vise clamping, and cycle signals."""

  @abc.abstractmethod
  def build_open_door_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a task to command the CNC enclosure door open."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_close_door_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a task to command the CNC enclosure door closed."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_open_vise_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a task to open the CNC pneumatic/hydraulic vise."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_close_vise_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a task to clamp the CNC pneumatic/hydraulic vise."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_trigger_cycle_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a task to trigger CNC machining cycle start."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_wait_cycle_complete_task(
      self, timeout_seconds: float = 30.0, name: Optional[str] = None
  ) -> bt.Node:
    """Builds a task to wait for the CNC cycle complete signal."""
    raise NotImplementedError


class DioCncMachine(CncMachineInterface):
  """CNC machine controller using Discrete I/O pins via SBL dio skills."""

  def __init__(
      self,
      solution: Any,
      door_open_pin: int = 2,
      door_close_pin: int = 3,
      vise_open_pin: int = 4,
      vise_close_pin: int = 5,
      cycle_start_pin: int = 6,
      cycle_done_input_pin: int = 0,
      device_name: str = "ur_module",
      is_mock: bool = False,
  ) -> None:
    self._solution = solution
    self._door_open_pin = door_open_pin
    self._door_close_pin = door_close_pin
    self._vise_open_pin = vise_open_pin
    self._vise_close_pin = vise_close_pin
    self._cycle_start_pin = cycle_start_pin
    self._cycle_done_input_pin = cycle_done_input_pin
    self._device_name = device_name
    self._is_mock = is_mock
    self._dio_set_skill = solution.skills.ai.intrinsic.dio_set_output
    self._dio_read_skill = solution.skills.ai.intrinsic.dio_read_input

  def build_open_door_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Open CNC Door (DIO)"
    action = self._dio_set_skill(
        pin=self._door_open_pin, state=True, device_name=self._device_name
    )
    return bt.Task(action=action, name=task_name)

  def build_close_door_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Close CNC Door (DIO)"
    action = self._dio_set_skill(
        pin=self._door_close_pin, state=True, device_name=self._device_name
    )
    return bt.Task(action=action, name=task_name)

  def build_open_vise_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Open CNC Vise (DIO)"
    action = self._dio_set_skill(
        pin=self._vise_open_pin, state=True, device_name=self._device_name
    )
    return bt.Task(action=action, name=task_name)

  def build_close_vise_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Clamp CNC Vise (DIO)"
    action = self._dio_set_skill(
        pin=self._vise_close_pin, state=True, device_name=self._device_name
    )
    return bt.Task(action=action, name=task_name)

  def build_trigger_cycle_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Trigger CNC Machining Cycle (DIO)"
    action = self._dio_set_skill(
        pin=self._cycle_start_pin, state=True, device_name=self._device_name
    )
    return bt.Task(action=action, name=task_name)

  def build_wait_cycle_complete_task(
      self, timeout_seconds: float = 30.0, name: Optional[str] = None
  ) -> bt.Node:
    task_name = name or "Wait for CNC Cycle Complete"
    if self._is_mock:
      return bt.Task(
          action=bt.PythonScript(
              function_body='print("[MockCNC] Wait for cycle complete (bypassed)")'
          ),
          name=f"{task_name} (Mock Bypassed)",
      )

    read_action = self._dio_read_skill(
        pin=self._cycle_done_input_pin,
        device_name=self._device_name,
    )
    return bt.Task(action=read_action, name=task_name)


class MockCncMachine(CncMachineInterface):
  """Mock CNC machine for testing when CNC hardware signals are not deployed."""

  def __init__(self) -> None:
    self.door_open: bool = False
    self.vise_open: bool = True
    self.cycle_triggered: bool = False
    self.command_log: list[str] = []

  def build_open_door_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Mock Open CNC Door"
    self.door_open = True
    self.command_log.append("open_door")
    return bt.Task(
        action=bt.PythonScript(
            function_body='print("[MockCNC] Open CNC Door")'
        ),
        name=task_name,
    )

  def build_close_door_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Mock Close CNC Door"
    self.door_open = False
    self.command_log.append("close_door")
    return bt.Task(
        action=bt.PythonScript(
            function_body='print("[MockCNC] Close CNC Door")'
        ),
        name=task_name,
    )

  def build_open_vise_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Mock Open CNC Vise"
    self.vise_open = True
    self.command_log.append("open_vise")
    return bt.Task(
        action=bt.PythonScript(
            function_body='print("[MockCNC] Open CNC Vise")'
        ),
        name=task_name,
    )

  def build_close_vise_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Mock Clamp CNC Vise"
    self.vise_open = False
    self.command_log.append("close_vise")
    return bt.Task(
        action=bt.PythonScript(
            function_body='print("[MockCNC] Clamp CNC Vise")'
        ),
        name=task_name,
    )

  def build_trigger_cycle_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Mock Trigger CNC Machining Cycle"
    self.cycle_triggered = True
    self.command_log.append("trigger_cycle")
    return bt.Task(
        action=bt.PythonScript(
            function_body='print("[MockCNC] Trigger Machining Cycle Start")'
        ),
        name=task_name,
    )

  def build_wait_cycle_complete_task(
      self, timeout_seconds: float = 30.0, name: Optional[str] = None
  ) -> bt.Node:
    task_name = name or "Mock Wait for CNC Cycle Complete"
    self.command_log.append("wait_cycle_complete")
    return bt.Task(
        action=bt.PythonScript(
            function_body='print("[MockCNC] Machining Cycle Complete")'
        ),
        name=task_name,
    )
