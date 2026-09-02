"""CNC Machine & Vise hardware interfaces and implementations."""

import abc
from typing import Any, Optional, Sequence, Union
from intrinsic.solutions import behavior_tree as bt


class CncMachineInterface(abc.ABC):
  """Abstract interface for CNC door, vise, and cycle signals."""

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
      door_close_pin: Optional[int] = 3,
      vise_open_pin: int = 4,
      vise_close_pin: Optional[int] = 5,
      cycle_start_pin: int = 6,
      cycle_done_input_pin: int = 0,
      output_block_name: str = "standard_out",
      input_block_name: str = "standard_in",
      device_name: Optional[str] = None,
      is_mock: bool = False,
  ) -> None:
    self._solution = solution
    self._door_open_pin = door_open_pin
    self._door_close_pin = door_close_pin
    self._vise_open_pin = vise_open_pin
    self._vise_close_pin = vise_close_pin
    self._cycle_start_pin = cycle_start_pin
    self._cycle_done_input_pin = cycle_done_input_pin
    self._output_block_name = output_block_name
    self._input_block_name = input_block_name
    self._device_name = device_name
    self._is_mock = is_mock
    self._dio_set_skill = solution.skills.ai.intrinsic.dio_set_output
    self._dio_read_skill = solution.skills.ai.intrinsic.dio_read_input

  def _resolve_adio_kwarg(self) -> dict[str, Any]:
    """Resolves optional adio equipment keyword argument for DIO skills."""
    if not self._device_name or self._device_name == "ur_module":
      return {}
    if hasattr(self._solution, "resources") and hasattr(
        self._solution.resources, self._device_name
    ):
      return {"adio": getattr(self._solution.resources, self._device_name)}
    return {}

  def _build_dio_set_task(
      self,
      pins: Union[Sequence[int], int],
      states: Union[Sequence[bool], bool],
      task_name: str,
  ) -> bt.Node:
    if isinstance(pins, int):
      pin_list = [pins]
      state_list = [bool(states)]
    else:
      pin_list = [int(p) for p in pins]
      if isinstance(states, bool):
        state_list = [states] * len(pin_list)
      else:
        state_list = [bool(s) for s in states]

    if hasattr(self._dio_set_skill, "intrinsic_proto") and hasattr(
        self._dio_set_skill.intrinsic_proto, "skills"
    ):
      block_cls = self._dio_set_skill.intrinsic_proto.skills.DioOutputBlock
    elif hasattr(self._dio_set_skill, "DioOutputBlock"):
      block_cls = self._dio_set_skill.DioOutputBlock
    else:
      block_cls = getattr(
          getattr(self._dio_set_skill, "ai", None), "intrinsic", None
      )
      if block_cls and hasattr(block_cls, "DioOutputBlock"):
        block_cls = block_cls.DioOutputBlock
      else:
        block_cls = None

    kwargs: dict[str, Any] = self._resolve_adio_kwarg()
    if block_cls is not None:
      block = block_cls(
          block_name=self._output_block_name,
          indices=pin_list,
          values=state_list,
      )
      kwargs["dio_output_blocks"] = [block]
    else:
      kwargs["dio_output_blocks"] = [{
          "block_name": self._output_block_name,
          "indices": pin_list,
          "values": state_list,
      }]
    action = self._dio_set_skill(**kwargs)
    return bt.Task(action=action, name=task_name)

  def build_open_door_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Open CNC Door (DIO)"
    if self._door_close_pin is not None:
      return self._build_dio_set_task(
          pins=[self._door_open_pin, self._door_close_pin],
          states=[True, False],
          task_name=task_name,
      )
    return self._build_dio_set_task(
        pins=self._door_open_pin, states=True, task_name=task_name
    )

  def build_close_door_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Close CNC Door (DIO)"
    if self._door_close_pin is not None:
      return self._build_dio_set_task(
          pins=[self._door_open_pin, self._door_close_pin],
          states=[False, True],
          task_name=task_name,
      )
    return self._build_dio_set_task(
        pins=self._door_open_pin, states=False, task_name=task_name
    )

  def build_open_vise_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Open CNC Vise (DIO)"
    if self._vise_close_pin is not None:
      return self._build_dio_set_task(
          pins=[self._vise_open_pin, self._vise_close_pin],
          states=[True, False],
          task_name=task_name,
      )
    return self._build_dio_set_task(
        pins=self._vise_open_pin, states=True, task_name=task_name
    )

  def build_close_vise_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Clamp CNC Vise (DIO)"
    if self._vise_close_pin is not None:
      return self._build_dio_set_task(
          pins=[self._vise_open_pin, self._vise_close_pin],
          states=[False, True],
          task_name=task_name,
      )
    return self._build_dio_set_task(
        pins=self._vise_open_pin, states=False, task_name=task_name
    )

  def build_trigger_cycle_task(self, name: Optional[str] = None) -> bt.Node:
    task_name = name or "Trigger CNC Machining Cycle (DIO)"
    return self._build_dio_set_task(
        pins=self._cycle_start_pin, states=True, task_name=task_name
    )

  def build_wait_cycle_complete_task(
      self, timeout_seconds: float = 30.0, name: Optional[str] = None
  ) -> bt.Node:
    task_name = name or "Wait for CNC Cycle Complete"
    if self._is_mock:
      return bt.Task(
          action=bt.PythonScript(
              function_body=(
                  'print("[MockCNC] Wait for cycle complete (bypassed)")'
              )
          ),
          name=f"{task_name} (Mock Bypassed)",
      )

    kwargs: dict[str, Any] = {
        "block_name": self._input_block_name,
        "timeout": timeout_seconds,
    }
    kwargs.update(self._resolve_adio_kwarg())
    read_action = self._dio_read_skill(**kwargs)
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
