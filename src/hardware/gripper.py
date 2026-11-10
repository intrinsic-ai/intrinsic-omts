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
import logging
from typing import Any

from intrinsic.solutions import behavior_tree as bt
from intrinsic.world.proto import (
  object_world_refs_pb2,
  object_world_updates_pb2,
)

from src.core.config import GripperConfig
from src.utils.math_utils import resolve_adio_resource

# Belief-world object whose finger joints the update_world fallback sets.
_WORLD_FALLBACK_GRIPPER_OBJECT = "gripper"


def _find_skill(solution: Any, skill_name: str) -> Any | None:
  """Returns the `ai.intrinsic.<skill_name>` skill class, or None if absent.

  Args:
      solution: Connected SBL deployment instance.
      skill_name: Skill name within the `ai.intrinsic` package.

  Returns:
      The skill class, or None if the solution does not provide it.
  """
  try:
    return getattr(solution.skills.ai.intrinsic, skill_name, None)
  except (AttributeError, KeyError):
    return None


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
    config: GripperConfig,
  ) -> None:
    """Initializes the DIO gripper adapter from GripperConfig.

    Args:
        solution: Connected SBL deployment instance.
        config: Scoped GripperConfig with DIO pin and output block settings.

    Raises:
        ValueError: If required DIO fields are missing from `config`.
    """
    if (
      config.dio_open_pin is None
      or config.dio_close_pin is None
      or config.dio_output_block_name is None
    ):
      raise ValueError(
        "DioGripper requires dio_open_pin, dio_close_pin, and "
        "dio_output_block_name in GripperConfig."
      )
    self._solution = solution
    self._open_pin: int = config.dio_open_pin
    self._close_pin: int = config.dio_close_pin
    self._output_block_name: str = config.dio_output_block_name
    self._device_name: str | None = config.dio_device_name
    self._dio_set_skill = solution.skills.ai.intrinsic.dio_set_output

  def _build_dio_task(
    self, active_pin: int, inactive_pin: int, task_name: str
  ) -> bt.Node:
    block_cls = getattr(
      getattr(
        getattr(self._dio_set_skill, "intrinsic_proto", None), "skills", None
      ),
      "DioOutputBlock",
      None,
    ) or getattr(self._dio_set_skill, "DioOutputBlock", None)

    kwargs: dict[str, Any] = {}
    adio_resource = resolve_adio_resource(self._solution, self._device_name)
    if adio_resource is not None:
      kwargs["adio"] = adio_resource

    if block_cls is not None:
      kwargs["dio_output_blocks"] = [
        block_cls(
          block_name=self._output_block_name,
          indices=[active_pin, inactive_pin],
          values=[True, False],
        )
      ]
    else:
      kwargs["dio_output_blocks"] = [
        {
          "block_name": self._output_block_name,
          "indices": [active_pin, inactive_pin],
          "values": [True, False],
        }
      ]
    return bt.Task(action=self._dio_set_skill(**kwargs), name=task_name)

  def build_open_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to open the DIO gripper."""
    task_name = name or "Open Gripper (DIO)"
    return self._build_dio_task(
      active_pin=self._open_pin,
      inactive_pin=self._close_pin,
      task_name=task_name,
    )

  def build_close_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to close the DIO gripper."""
    task_name = name or "Close Gripper (DIO)"
    return self._build_dio_task(
      active_pin=self._close_pin,
      inactive_pin=self._open_pin,
      task_name=task_name,
    )


class RobotiqGripper(GripperInterface):
  """Robotiq adaptive gripper controlled via gripper_cmd_skill.

  If `GripperConfig.allow_world_joint_fallback` is `True`, the solution is
  simulated, and it does not provide `gripper_cmd_skill`, finger motion is
  emulated by setting the gripper joints in the belief world through the
  `update_world` skill. This is a simulation-only fallback for platforms
  without a gripper service.
  """

  def __init__(
    self,
    solution: Any,
    config: GripperConfig,
  ) -> None:
    """Initializes the Robotiq gripper adapter from GripperConfig.

    Args:
        solution: Connected SBL deployment instance.
        config: Scoped GripperConfig with finger joint and stroke positions.

    Raises:
        ValueError: If required Robotiq fields are missing from `config`.
        RuntimeError: If `gripper_cmd_skill` is not installed and the
          `update_world` fallback is disabled, the solution is not simulated,
          or `update_world` is unavailable.
    """
    if (
      config.joint_name is None
      or config.open_position is None
      or config.close_position is None
    ):
      raise ValueError(
        "RobotiqGripper requires joint_name, open_position, and "
        "close_position in GripperConfig."
      )
    self._solution = solution
    self._joint_name: str = config.joint_name
    self._open_position: float = config.open_position
    self._close_position: float = config.close_position
    self._action_name: str | None = config.action_name
    self._gripper_cmd_skill = _find_skill(solution, "gripper_cmd_skill")
    self._update_world_skill = None
    if self._gripper_cmd_skill is not None:
      return

    if not config.allow_world_joint_fallback:
      raise RuntimeError(
        "Skill 'ai.intrinsic.gripper_cmd_skill' is not installed in the "
        "solution. Install the gripper service, or set "
        "'gripper.allow_world_joint_fallback: true' to emulate the gripper "
        "via update_world in simulation."
      )
    if getattr(solution, "is_simulated", False) is not True:
      raise RuntimeError(
        "Skill 'ai.intrinsic.gripper_cmd_skill' is not installed and the "
        "solution is not simulated. 'gripper.allow_world_joint_fallback' only "
        "applies to simulated solutions; install the gripper service."
      )
    self._update_world_skill = _find_skill(solution, "update_world")
    if self._update_world_skill is None:
      raise RuntimeError(
        "'gripper.allow_world_joint_fallback' is enabled, but neither "
        "'ai.intrinsic.gripper_cmd_skill' nor 'ai.intrinsic.update_world' is "
        "installed in the solution."
      )
    logging.warning(
      "Skill 'ai.intrinsic.gripper_cmd_skill' is not installed; "
      "'gripper.allow_world_joint_fallback' is enabled, so the gripper is "
      "emulated by setting joints of '%s' through update_world. No gripper "
      "hardware or gripper service is commanded.",
      _WORLD_FALLBACK_GRIPPER_OBJECT,
    )

  def _build_joint_task(self, position: float, task_name: str) -> bt.Node:
    if self._gripper_cmd_skill is not None:
      command = self._gripper_cmd_skill.ai.intrinsic.JointState(
        name=[self._joint_name],
        position=[position],
      )
      kwargs: dict[str, Any] = {"command": command}
      if self._action_name is not None:
        kwargs["action_name"] = self._action_name
      action = self._gripper_cmd_skill(**kwargs)
      return bt.Task(action=action, name=task_name)

    update_proto = object_world_updates_pb2.ObjectWorldUpdate(
      update_object_joints=object_world_updates_pb2.UpdateObjectJointsRequest(
        object=object_world_refs_pb2.ObjectReference(
          by_name=object_world_refs_pb2.ObjectReferenceByName(
            object_name=_WORLD_FALLBACK_GRIPPER_OBJECT
          )
        ),
        joint_positions=[float(position), float(position)],
      )
    )
    return bt.Task(
      action=self._update_world_skill(update=update_proto),
      name=task_name,
    )

  def build_open_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to open the gripper."""
    task_name = name or "Open Robotiq Gripper"
    return self._build_joint_task(self._open_position, task_name)

  def build_close_task(self, name: str | None = None) -> bt.Node:
    """Builds a behavior tree task to close/grasp with the gripper."""
    task_name = name or "Close Robotiq Gripper"
    return self._build_joint_task(self._close_position, task_name)
