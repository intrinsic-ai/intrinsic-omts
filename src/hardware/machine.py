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
import dataclasses
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.core.types import FixtureState, MachineDoorState
from src.core.world import World
from src.hardware.robot import RobotInterface


@dataclasses.dataclass(frozen=True)
class MachineConfig:
  """Configuration for CNC machine I/O pins, blocks, and digital twin joints."""

  machine_type: str = "dio"
  door_open_pin: int = 2
  door_close_pin: int | None = 3
  vise_open_pin: int = 4
  vise_close_pin: int | None = 5
  cycle_start_pin: int = 6
  cycle_done_input_pin: int = 7
  output_block_name: str = "standard_out"
  input_block_name: str = "standard_in"
  device_name: str | None = None
  enclosure_object_name: str = "cnc_enclosure"
  door_open_joints: tuple[float, ...] = (0.4,)
  door_closed_joints: tuple[float, ...] = (0.0,)
  vise_object_name: str = "schunk_egp_64nnb"
  vise_open_joints: tuple[float, ...] = (0.01, 0.01)
  vise_closed_joints: tuple[float, ...] = (0.00, 0.00)
  machining_timeout_seconds: float = 30.0
  initial_close_door_and_vise: bool = True


class CncMachineInterface(abc.ABC):
  """Abstract interface for CNC door, vise, and cycle signals.

  Each command records the state it drives the cell into, so tree builders can
  read it back instead of re-deriving it. Implementations that move real
  hardware also emit the matching digital twin joint update.
  """

  _door: MachineDoorState = MachineDoorState.CLOSED
  _vise: FixtureState = FixtureState.OPEN

  @classmethod
  def from_config(
    cls,
    solution: Any,
    config: MachineConfig | None = None,
    mock_hardware: bool = False,
  ) -> "CncMachineInterface | None":
    """Creates and initializes the CNC machine hardware adapter from config."""
    cfg = config or MachineConfig()
    if cfg.machine_type == "none" and not mock_hardware:
      return None
    if mock_hardware or cfg.machine_type == "mock":
      return MockCncMachine()
    if cfg.machine_type == "dio":
      return DioCncMachine(solution=solution, config=cfg)
    raise ValueError(f"Unsupported machine_type: {cfg.machine_type}")

  @property
  def door_state(self) -> MachineDoorState:
    """Returns the door state this adapter has most recently commanded."""
    return self._door

  @property
  def vise_state(self) -> FixtureState:
    """Returns the vise state this adapter has most recently commanded."""
    return self._vise

  def build_open_door_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to command the CNC enclosure door open."""
    self._door = MachineDoorState.OPEN
    return self._build_open_door_task(name)

  def build_close_door_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to command the CNC enclosure door closed."""
    self._door = MachineDoorState.CLOSED
    return self._build_close_door_task(name)

  def build_open_vise_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to open the CNC pneumatic/hydraulic vise."""
    self._vise = FixtureState.OPEN
    return self._build_open_vise_task(name)

  def build_close_vise_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to clamp the CNC pneumatic/hydraulic vise."""
    self._vise = FixtureState.CLAMPED
    return self._build_close_vise_task(name)

  @abc.abstractmethod
  def _build_open_door_task(self, name: str | None) -> bt.Node:
    raise NotImplementedError

  @abc.abstractmethod
  def _build_close_door_task(self, name: str | None) -> bt.Node:
    raise NotImplementedError

  @abc.abstractmethod
  def _build_open_vise_task(self, name: str | None) -> bt.Node:
    raise NotImplementedError

  @abc.abstractmethod
  def _build_close_vise_task(self, name: str | None) -> bt.Node:
    raise NotImplementedError

  @abc.abstractmethod
  def build_trigger_cycle_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to trigger CNC machining cycle start."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_wait_cycle_complete_task(
    self, timeout_seconds: float = 30.0, name: str | None = None
  ) -> bt.Node:
    """Builds a task to wait for the CNC cycle complete signal."""
    raise NotImplementedError


class DioCncMachine(CncMachineInterface):
  """CNC machine controller using Discrete I/O pins via SBL dio skills."""

  def __init__(
    self,
    solution: Any,
    door_open_pin: int = 2,
    door_close_pin: int | None = 3,
    vise_open_pin: int = 4,
    vise_close_pin: int | None = 5,
    cycle_start_pin: int = 6,
    cycle_done_input_pin: int = 7,
    output_block_name: str = "standard_out",
    input_block_name: str = "standard_in",
    device_name: str | None = None,
    config: MachineConfig | None = None,
  ) -> None:
    self._solution = solution
    self._config = config or MachineConfig(
      door_open_pin=door_open_pin,
      door_close_pin=door_close_pin,
      vise_open_pin=vise_open_pin,
      vise_close_pin=vise_close_pin,
      cycle_start_pin=cycle_start_pin,
      cycle_done_input_pin=cycle_done_input_pin,
      output_block_name=output_block_name,
      input_block_name=input_block_name,
      device_name=device_name,
    )
    self._dio_set_skill = solution.skills.ai.intrinsic.dio_set_output
    self._dio_wait_skill = solution.skills.ai.intrinsic.dio_wait_for_input

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
    dev = self._config.device_name
    if (
      dev
      and dev != "ur_module"
      and hasattr(getattr(self._solution, "resources", None), dev)
    ):
      kwargs["adio"] = getattr(self._solution.resources, dev)

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
          block_name=self._config.output_block_name,
          indices=pin_list,
          values=state_list,
        )
      ]
    else:
      kwargs["dio_output_blocks"] = [
        {
          "block_name": self._config.output_block_name,
          "indices": pin_list,
          "values": state_list,
        }
      ]
    return bt.Task(action=self._dio_set_skill(**kwargs), name=task_name)

  def _build_command_task(
    self,
    on_pin: int,
    off_pin: int | None,
    activate: bool,
    task_name: str,
    obj_name: str,
    joints: Sequence[float],
    update_name: str,
  ) -> bt.Node:
    """Drives the pins and mirrors the result onto the digital twin."""
    pins = [on_pin, off_pin] if off_pin is not None else on_pin
    states = [activate, not activate] if off_pin is not None else activate
    world = getattr(self._solution, "world", None)
    if not hasattr(world, "build_joint_update_task"):
      world = World(world, solution=self._solution)
    return bt.Sequence(
      name=task_name,
      children=[
        self._build_dio_set_task(pins=pins, states=states, task_name=task_name),
        world.build_joint_update_task(obj_name, joints, task_name=update_name),
      ],
    )

  def _build_open_door_task(self, name: str | None) -> bt.Node:
    return self._build_command_task(
      self._config.door_open_pin,
      self._config.door_close_pin,
      True,
      name or "Open CNC Door (DIO)",
      self._config.enclosure_object_name,
      self._config.door_open_joints,
      "Update Door Joint (Open)",
    )

  def _build_close_door_task(self, name: str | None) -> bt.Node:
    return self._build_command_task(
      self._config.door_open_pin,
      self._config.door_close_pin,
      False,
      name or "Close CNC Door (DIO)",
      self._config.enclosure_object_name,
      self._config.door_closed_joints,
      "Update Door Joint (Closed)",
    )

  def _build_open_vise_task(self, name: str | None) -> bt.Node:
    return self._build_command_task(
      self._config.vise_open_pin,
      self._config.vise_close_pin,
      True,
      name or "Open CNC Vise (DIO)",
      self._config.vise_object_name,
      self._config.vise_open_joints,
      "Update Vise Joint (Open)",
    )

  def _build_close_vise_task(self, name: str | None) -> bt.Node:
    return self._build_command_task(
      self._config.vise_open_pin,
      self._config.vise_close_pin,
      False,
      name or "Clamp CNC Vise (DIO)",
      self._config.vise_object_name,
      self._config.vise_closed_joints,
      "Update Vise Joint (Clamped)",
    )

  def build_trigger_cycle_task(self, name: str | None = None) -> bt.Node:
    task_name = name or "Trigger CNC Machining Cycle (DIO)"
    return self._build_dio_set_task(
      pins=self._config.cycle_start_pin, states=True, task_name=task_name
    )

  def build_wait_cycle_complete_task(
    self, timeout_seconds: float = 30.0, name: str | None = None
  ) -> bt.Node:
    kwargs: dict[str, Any] = {
      "block_name": self._config.input_block_name,
      "indices": [self._config.cycle_done_input_pin],
      "values": [True],
      "timeout": timeout_seconds,
    }
    dev = self._config.device_name
    if (
      dev
      and dev != "ur_module"
      and hasattr(getattr(self._solution, "resources", None), dev)
    ):
      kwargs["adio"] = getattr(self._solution.resources, dev)
    return bt.Task(
      action=self._dio_wait_skill(**kwargs),
      name=name or "Wait for CNC Cycle Complete",
    )


class MockCncMachine(CncMachineInterface):
  """Mock CNC machine for testing when CNC hardware signals are not deployed."""

  def __init__(self) -> None:
    self.command_log: list[str] = []

  def _log_task(self, command: str, task_name: str, message: str) -> bt.Node:
    self.command_log.append(command)
    return bt.Task(
      action=bt.PythonScript(function_body=f'print("[MockCNC] {message}")'),
      name=task_name,
    )

  def _build_open_door_task(self, name: str | None) -> bt.Node:
    return self._log_task(
      "open_door", name or "Mock Open CNC Door", "Open CNC Door"
    )

  def _build_close_door_task(self, name: str | None) -> bt.Node:
    return self._log_task(
      "close_door", name or "Mock Close CNC Door", "Close CNC Door"
    )

  def _build_open_vise_task(self, name: str | None) -> bt.Node:
    return self._log_task(
      "open_vise", name or "Mock Open CNC Vise", "Open CNC Vise"
    )

  def _build_close_vise_task(self, name: str | None) -> bt.Node:
    return self._log_task(
      "close_vise", name or "Mock Clamp CNC Vise", "Clamp CNC Vise"
    )

  def build_trigger_cycle_task(self, name: str | None = None) -> bt.Node:
    return self._log_task(
      "trigger_cycle",
      name or "Mock Trigger CNC Machining Cycle",
      "Trigger Machining Cycle Start",
    )

  def build_wait_cycle_complete_task(
    self, timeout_seconds: float = 30.0, name: str | None = None
  ) -> bt.Node:
    del timeout_seconds
    return self._log_task(
      "wait_cycle_complete",
      name or "Mock Wait for CNC Cycle Complete",
      "Machining Cycle Complete",
    )


def run_initial_machine_prep(
  solution: Any,
  machine: CncMachineInterface | None,
  robot: RobotInterface | None = None,
  view_frame: str = "view",
  parent_object: str = "root",
  close_door_and_vise: bool = True,
  name: str = "Pre-Flight Machine Prep (Close Door & Vise)",
) -> None:
  """Executes initial machine state preparation, retracting robot first."""
  tasks = []
  if robot is not None:
    tasks.append(
      robot.build_move_cartesian_task(
        target_frame_name=view_frame,
        target_object_name=parent_object,
        motion_type="ANY",
        name=f"Retract Arm to {view_frame} Before Closing Door",
      )
    )
  if machine is not None and close_door_and_vise:
    tasks.extend(
      [
        machine.build_close_door_task(name="Initial Close CNC Door"),
        machine.build_close_vise_task(name="Initial Close CNC Vise"),
      ]
    )
  if tasks:
    prep_tree = bt.Sequence(name=name, children=tasks)
    if hasattr(solution, "run") and callable(solution.run):
      solution.run(prep_tree)
    elif hasattr(solution, "executive") and hasattr(solution.executive, "run"):
      solution.executive.run(prep_tree)
