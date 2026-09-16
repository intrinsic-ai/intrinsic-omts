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

"""Gripper hardware interfaces and implementations."""

import abc
import dataclasses
from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.core.types import GripperState

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
  """Abstract interface for robotic end-effector gripping actions.

  Implementations provide `_build_open_task` and `_build_close_task`; the
  interface records the commanded aperture so tree builders can tell what the
  previous subtree already asked for.
  """

  _state: GripperState = GripperState.UNKNOWN

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

  @property
  def commanded_state(self) -> GripperState:
    """Returns the aperture this adapter has most recently commanded."""
    return self._state

  def build_open_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to open the gripper fingers."""
    self._state = GripperState.OPEN
    return self._build_open_task(name)

  def build_close_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to close/grasp with the gripper."""
    self._state = GripperState.CLOSED
    return self._build_close_task(name)

  @abc.abstractmethod
  def _build_open_task(self, name: str | None) -> bt.Node:
    raise NotImplementedError

  @abc.abstractmethod
  def _build_close_task(self, name: str | None) -> bt.Node:
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

  def _build_open_task(self, name: str | None) -> bt.Node:
    return self._build_command_task(
      self._open_position, name or "Open Robotiq Gripper"
    )

  def _build_close_task(self, name: str | None) -> bt.Node:
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
    block_cls = getattr(
      getattr(
        getattr(self._dio_set_skill, "intrinsic_proto", None), "skills", None
      ),
      "DioOutputBlock",
      None,
    ) or getattr(self._dio_set_skill, "DioOutputBlock", None)

    if block_cls is not None:
      block = block_cls(
        block_name=self._output_block_name,
        indices=[pin],
        values=[state],
      )
      action = self._dio_set_skill(dio_output_blocks=[block])
    else:
      action = self._dio_set_skill(
        dio_output_blocks=[
          {
            "block_name": self._output_block_name,
            "indices": [pin],
            "values": [state],
          }
        ]
      )
    return bt.Task(action=action, name=task_name)

  def _build_open_task(self, name: str | None) -> bt.Node:
    return self._build_dio_set_task(
      pin=self._open_pin, state=True, task_name=name or "Open Gripper (DIO)"
    )

  def _build_close_task(self, name: str | None) -> bt.Node:
    return self._build_dio_set_task(
      pin=self._close_pin, state=True, task_name=name or "Close Gripper (DIO)"
    )


class MockGripper(GripperInterface):
  """Mock gripper for testing when gripper hardware service is not deployed."""

  def __init__(self) -> None:
    self._state = GripperState.UNKNOWN
    self.command_log: list[str] = []

  def _build_open_task(self, name: str | None) -> bt.Node:
    self.command_log.append("open")
    return bt.Task(
      action=bt.PythonScript(
        function_body='print("[MockGripper] Gripper Opened")'
      ),
      name=name or "Mock Open Gripper",
    )

  def _build_close_task(self, name: str | None) -> bt.Node:
    self.command_log.append("close")
    return bt.Task(
      action=bt.PythonScript(
        function_body='print("[MockGripper] Gripper Closed (Part Grasped)")'
      ),
      name=name or "Mock Close Gripper",
    )
