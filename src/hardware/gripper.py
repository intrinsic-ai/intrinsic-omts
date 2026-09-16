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

"""Stateless gripper hardware interfaces and implementations."""

import abc
import dataclasses
from typing import Any

from intrinsic.solutions import behavior_tree as bt

DEFAULT_GRIPPER_ACTION_NAME = "/gripper/gripper_action_controller/gripper_cmd"


@dataclasses.dataclass(frozen=True)
class GripperConfig:
  """Configuration for gripper hardware backend and commanded travel."""

  hardware_type: str = "robotiq"
  open_position: float = 0.024
  close_position: float = 0.010
  action_name: str | None = DEFAULT_GRIPPER_ACTION_NAME
  dio_open_pin: int = 0
  dio_close_pin: int = 1
  output_block_name: str = "standard_out"
  joint_name: str = "robotiq_hande_left_finger_joint"


class GripperInterface(abc.ABC):
  """Stateless abstract interface for robotic end-effector gripping actions."""

  @classmethod
  def from_config(
    cls,
    solution: Any,
    config: GripperConfig | None = None,
    mock_hardware: bool = False,
  ) -> "GripperInterface":
    """Creates and initializes the gripper hardware adapter from config."""
    cfg = config or GripperConfig()
    if mock_hardware or cfg.hardware_type == "mock":
      return MockGripper()
    if cfg.hardware_type == "robotiq":
      return RobotiqGripper(
        solution=solution,
        joint_name=cfg.joint_name,
        open_position=cfg.open_position,
        close_position=cfg.close_position,
        action_name=cfg.action_name,
      )
    if cfg.hardware_type == "dio":
      return DioGripper(
        solution=solution,
        open_pin=cfg.dio_open_pin,
        close_pin=cfg.dio_close_pin,
        output_block_name=cfg.output_block_name,
      )
    raise ValueError(f"Unsupported gripper hardware_type: {cfg.hardware_type}")

  @abc.abstractmethod
  def build_open_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to open the gripper fingers."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_close_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to close/grasp with the gripper."""
    raise NotImplementedError


class RobotiqGripper(GripperInterface):
  """Robotiq adaptive gripper controlled via ai.intrinsic.gripper_cmd_skill."""

  def __init__(
    self,
    solution: Any,
    joint_name: str = "robotiq_hande_left_finger_joint",
    open_position: float = 0.024,
    close_position: float = 0.01,
    action_name: str | None = DEFAULT_GRIPPER_ACTION_NAME,
  ) -> None:
    """Initializes the Robotiq gripper adapter."""
    self._solution = solution
    self._joint_name = joint_name
    self._open_position = open_position
    self._close_position = close_position
    self._action_name = action_name
    self._gripper_cmd_skill = solution.skills.ai.intrinsic.gripper_cmd_skill

  def _build_command_task(self, position: float, task_name: str) -> bt.Node:
    command = self._gripper_cmd_skill.ai.intrinsic.JointState(
      name=[self._joint_name],
      position=[position],
    )
    kwargs: dict[str, Any] = {"command": command}
    if self._action_name is not None:
      kwargs["action_name"] = self._action_name
    return bt.Task(action=self._gripper_cmd_skill(**kwargs), name=task_name)

  def build_open_task(self, name: str | None = None) -> bt.Node:
    return self._build_command_task(
      self._open_position, name or "Open Robotiq Gripper"
    )

  def build_close_task(self, name: str | None = None) -> bt.Node:
    return self._build_command_task(
      self._close_position, name or "Close Robotiq Gripper"
    )


class DioGripper(GripperInterface):
  """Digital I/O pneumatic or electrical gripper implementation."""

  def __init__(
    self,
    solution: Any,
    open_pin: int = 0,
    close_pin: int = 1,
    output_block_name: str = "standard_out",
  ) -> None:
    self._solution = solution
    self._open_pin = open_pin
    self._close_pin = close_pin
    self._output_block_name = output_block_name
    self._dio_set_skill = solution.skills.ai.intrinsic.dio_set_output

  def _build_dio_set_task(
    self, pin: int, state: bool, task_name: str
  ) -> bt.Node:
    block = self._dio_set_skill.intrinsic_proto.skills.DioOutputBlock(
      block_name=self._output_block_name,
      indices=[pin],
      values=[state],
    )
    action = self._dio_set_skill(dio_output_blocks=[block])
    return bt.Task(action=action, name=task_name)

  def build_open_task(self, name: str | None = None) -> bt.Node:
    return self._build_dio_set_task(
      pin=self._open_pin, state=True, task_name=name or "Open Gripper (DIO)"
    )

  def build_close_task(self, name: str | None = None) -> bt.Node:
    return self._build_dio_set_task(
      pin=self._close_pin, state=True, task_name=name or "Close Gripper (DIO)"
    )


class MockGripper(GripperInterface):
  """Mock gripper adapter for unit tests and offline tree generation."""

  def __init__(self, solution: Any = None, **kwargs: Any) -> None:
    del solution, kwargs
    self.command_log: list[str] = []

  def build_open_task(self, name: str | None = None) -> bt.Node:
    self.command_log.append("open")
    return bt.Task(
      action=bt.PythonScript(function_body="pass"),
      name=name or "Mock Open Gripper",
    )

  def build_close_task(self, name: str | None = None) -> bt.Node:
    self.command_log.append("close")
    return bt.Task(
      action=bt.PythonScript(function_body="pass"),
      name=name or "Mock Close Gripper",
    )


__all__ = [
  "DEFAULT_GRIPPER_ACTION_NAME",
  "DioGripper",
  "GripperConfig",
  "GripperInterface",
  "MockGripper",
  "RobotiqGripper",
]
