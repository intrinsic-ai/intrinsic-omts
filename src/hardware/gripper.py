"""Gripper hardware interfaces and implementations."""

import abc
from typing import Any

from intrinsic.solutions import behavior_tree as bt


class GripperInterface(abc.ABC):
  """Abstract interface for robotic end-effector gripping actions."""

  @abc.abstractmethod
  def build_open_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to open the gripper fingers."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_close_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to close/grasp with the gripper."""
    raise NotImplementedError


class DioGripper(GripperInterface):
  """Digital I/O pneumatic or electrical gripper implementation."""

  def __init__(
    self,
    solution: Any,
    open_pin: int = 0,
    close_pin: int = 1,
    device_name: str = "ur_module",
  ) -> None:
    self._solution = solution
    self._open_pin = open_pin
    self._close_pin = close_pin
    self._device_name = device_name
    self._dio_set_skill = solution.skills.ai.intrinsic.dio_set_output

  def build_open_task(self, name: str | None = None) -> bt.Node:
    task_name = name or "Open Gripper (DIO)"
    action = self._dio_set_skill(
      pin=self._open_pin,
      state=True,
      device_name=self._device_name,
    )
    return bt.Task(action=action, name=task_name)

  def build_close_task(self, name: str | None = None) -> bt.Node:
    task_name = name or "Close Gripper (DIO)"
    action = self._dio_set_skill(
      pin=self._close_pin,
      state=True,
      device_name=self._device_name,
    )
    return bt.Task(action=action, name=task_name)


class RobotiqGripper(GripperInterface):
  """Robotiq adaptive gripper controlled via gripper_cmd_skill."""

  def __init__(
    self,
    solution: Any,
    joint_name: str = "robotiq_hande_left_finger_joint",
    open_position: float = 0.025,
    close_position: float = 0.0,
    action_name: str | None = None,
  ) -> None:
    """Initializes the Robotiq gripper adapter.

    Args:
        solution: Live SBL solution handle containing deployed skills.
        joint_name: Name of the active finger joint (default:
          "robotiq_hande_left_finger_joint").
        open_position: Finger joint position in meters for open state (default:
          0.025).
        close_position: Finger joint position in meters for closed state
          (default: 0.0).
        action_name: Optional ROS action controller name for gripper_cmd_skill.
    """
    self._solution = solution
    self._joint_name = joint_name
    self._open_position = open_position
    self._close_position = close_position
    self._action_name = action_name
    self._gripper_cmd_skill = solution.skills.ai.intrinsic.gripper_cmd_skill

  def build_open_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to open the gripper."""
    task_name = name or "Open Robotiq Gripper"
    command = self._gripper_cmd_skill.ai.intrinsic.JointState(
      name=[self._joint_name],
      position=[self._open_position],
    )
    kwargs: dict[str, Any] = {"command": command}
    if self._action_name is not None:
      kwargs["action_name"] = self._action_name
    action = self._gripper_cmd_skill(**kwargs)
    return bt.Task(action=action, name=task_name)

  def build_close_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to close/grasp with the gripper."""
    task_name = name or "Close Robotiq Gripper"
    command = self._gripper_cmd_skill.ai.intrinsic.JointState(
      name=[self._joint_name],
      position=[self._close_position],
    )
    kwargs: dict[str, Any] = {"command": command}
    if self._action_name is not None:
      kwargs["action_name"] = self._action_name
    action = self._gripper_cmd_skill(**kwargs)
    return bt.Task(action=action, name=task_name)


class MockGripper(GripperInterface):
  """Mock gripper for testing when gripper hardware service is not deployed."""

  def __init__(self) -> None:
    self.state: str = "open"
    self.command_log: list[str] = []

  def build_open_task(self, name: str | None = None) -> bt.Node:
    task_name = name or "Mock Open Gripper"
    self.state = "open"
    self.command_log.append("open")
    return bt.Task(
      action=bt.PythonScript(
        function_body='print("[MockGripper] Gripper Opened")'
      ),
      name=task_name,
    )

  def build_close_task(self, name: str | None = None) -> bt.Node:
    task_name = name or "Mock Close Gripper"
    self.state = "closed"
    self.command_log.append("close")
    return bt.Task(
      action=bt.PythonScript(
        function_body='print("[MockGripper] Gripper Closed (Part Grasped)")'
      ),
      name=task_name,
    )
