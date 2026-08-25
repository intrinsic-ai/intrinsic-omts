"""Robot hardware interface and implementations for Universal Robots and Mocks."""

import abc
from typing import Any, Optional, Sequence
from intrinsic.solutions import behavior_tree as bt
from src.core.types import Pose3D


class RobotInterface(abc.ABC):
  """Abstract interface for robot motion and compliant contact control."""

  @abc.abstractmethod
  def build_move_joint_task(
      self, joint_configuration_name: str, name: Optional[str] = None
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot arm to a named joint pose."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_to_contact_task(
      self,
      direction: tuple[float, float, float] = (0.0, 0.0, -1.0),
      contact_force_newtons: float = 5.0,
      timeout_seconds: float = 15.0,
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds a compliant move_to_contact behavior tree task."""
    raise NotImplementedError


class UrRobot(RobotInterface):
  """Universal Robots controller wrapper targeting Intrinsic SBL skills."""

  def __init__(self, solution: Any, arm_part_name: str = "ur_module") -> None:
    """Initializes UR robot adapter.

    Args:
        solution: Connected SBL deployment instance (from deployments.connect).
        arm_part_name: Attribute name of robot arm in solution.world.
    """
    self._solution = solution
    self._arm_part_name = arm_part_name
    self._move_robot_skill = solution.skills.ai.intrinsic.move_robot
    self._move_to_contact_skill = solution.skills.ai.intrinsic.move_to_contact

  @property
  def arm_part(self) -> Any:
    """Retrieves the robot arm part from the solution world."""
    return getattr(self._solution.world, self._arm_part_name)

  def build_move_joint_task(
      self, joint_configuration_name: str, name: Optional[str] = None
  ) -> bt.Node:
    """Builds an SBL move_robot joint motion task."""
    task_name = name or f"Move to {joint_configuration_name}"
    joint_target = getattr(self.arm_part.joint_configurations, joint_configuration_name)

    skill_action = self._move_robot_skill(
        motion_segments=[
            self._move_robot_skill.intrinsic_proto.skills.MotionSegment(
                joint_position=joint_target,
                motion_type=self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.JOINT,
            )
        ],
        arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_to_contact_task(
      self,
      direction: tuple[float, float, float] = (0.0, 0.0, -1.0),
      contact_force_newtons: float = 5.0,
      timeout_seconds: float = 15.0,
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds an SBL move_to_contact compliance task."""
    task_name = name or "Compliant Move to Contact"
    flange_frame = self.arm_part.get_single_iso_flange_frame()

    skill_action = self._move_to_contact_skill(
        tool=flange_frame,
        fixed_vector=self._move_to_contact_skill.intrinsic_proto.manipulation.skills.FixedVector(
            direction=self._move_to_contact_skill.intrinsic_proto.Vector3(
                x=direction[0], y=direction[1], z=direction[2]
            )
        ),
        contact_force=float(contact_force_newtons),
        timeout_sec=float(timeout_seconds),
    )
    return bt.Task(action=skill_action, name=task_name)


class MockRobot(RobotInterface):
  """Mock robot adapter for offline simulation and unit testing."""

  def __init__(self) -> None:
    self.executed_commands: list[str] = []

  def build_move_joint_task(
      self, joint_configuration_name: str, name: Optional[str] = None
  ) -> bt.Node:
    task_name = name or f"Mock Move to {joint_configuration_name}"
    self.executed_commands.append(f"move_joint:{joint_configuration_name}")
    return bt.Sequence([])

  def build_move_to_contact_task(
      self,
      direction: tuple[float, float, float] = (0.0, 0.0, -1.0),
      contact_force_newtons: float = 5.0,
      timeout_seconds: float = 15.0,
      name: Optional[str] = None,
  ) -> bt.Node:
    task_name = name or "Mock Move to Contact"
    self.executed_commands.append(
        f"move_to_contact:dir={direction},force={contact_force_newtons}"
    )
    return bt.Sequence([])
