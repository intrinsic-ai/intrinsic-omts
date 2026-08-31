"""Robot hardware interface and implementations for Universal Robots and Mocks."""

import abc
from typing import Any, Optional

from intrinsic.manipulation.skills.force import move_to_contact_pb2
from intrinsic.math.proto import vector3_pb2
from intrinsic.motion_planning.public.proto.v1 import geometric_constraints_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.world.public.proto import object_world_refs_pb2


class RobotInterface(abc.ABC):
  """Abstract interface for robot motion and compliant contact control."""

  @abc.abstractmethod
  def build_move_joint_task(
      self, joint_configuration_name: str, name: Optional[str] = None
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot arm to a named joint pose."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_cartesian_task(
      self,
      target_frame_name: str,
      target_object_name: str = "root",
      motion_type: str = "ANY",
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot tool to a target frame."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_to_contact_task(
      self,
      direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
      contact_force_newtons: float = 5.0,
      timeout_seconds: float = 15.0,
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds a compliant move_to_contact behavior tree task."""
    raise NotImplementedError


class UrRobot(RobotInterface):
  """Universal Robots controller wrapper targeting Intrinsic SBL skills."""

  def __init__(
      self,
      solution: Any,
      arm_part_name: str = "ur_module",
      tool_object_name: str = "gripper",
      tool_frame_name: str = "tool_frame",
  ) -> None:
    """Initializes UR robot adapter.

    Args:
        solution: Connected SBL deployment instance (from deployments.connect).
        arm_part_name: Attribute name of robot arm in solution.world.
        tool_object_name: Object name for moving tool frame (default: 'gripper').
        tool_frame_name: Frame name under tool_object_name (default: 'tool_frame').
    """
    self._solution = solution
    self._arm_part_name = arm_part_name
    self._tool_object_name = tool_object_name
    self._tool_frame_name = tool_frame_name
    self._move_robot_skill = solution.skills.ai.intrinsic.move_robot
    self._move_to_contact_skill = solution.skills.ai.intrinsic.move_to_contact

  @property
  def arm_part(self) -> Any:
    """Retrieves the robot arm part from the solution world."""
    return getattr(self._solution.world, self._arm_part_name)

  @property
  def tool_frame_reference(self) -> object_world_refs_pb2.TransformNodeReference:
    """Constructs the moving tool frame reference (gripper TCP)."""
    return object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            frame=object_world_refs_pb2.FrameReferenceByName(
                object_name=self._tool_object_name,
                frame_name=self._tool_frame_name,
            )
        )
    )

  def build_move_joint_task(
      self, joint_configuration_name: str, name: Optional[str] = None
  ) -> bt.Node:
    """Builds an SBL move_robot joint motion task."""
    task_name = name or f"Move to {joint_configuration_name}"
    joint_target = getattr(
        self.arm_part.joint_configurations, joint_configuration_name
    )

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

  def build_move_cartesian_task(
      self,
      target_frame_name: str,
      target_object_name: str = "root",
      motion_type: str = "ANY",
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds an SBL move_robot Cartesian motion task aligning tool to target frame."""
    task_name = name or f"Move to {target_frame_name} ({motion_type})"

    target_node_ref = object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            frame=object_world_refs_pb2.FrameReferenceByName(
                object_name=target_object_name,
                frame_name=target_frame_name,
            )
        )
    )

    cartesian_pose = geometric_constraints_pb2.PoseEquality(
        moving_frame=self.tool_frame_reference,
        target_frame=target_node_ref,
    )

    if motion_type.upper() == "LINEAR":
      motion_type_enum = (
          self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.LINEAR
      )
    elif motion_type.upper() == "JOINT":
      motion_type_enum = (
          self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.JOINT
      )
    else:
      motion_type_enum = (
          self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.ANY
      )

    skill_action = self._move_robot_skill(
        motion_segments=[
            self._move_robot_skill.intrinsic_proto.skills.MotionSegment(
                cartesian_pose=cartesian_pose,
                motion_type=motion_type_enum,
            )
        ],
        arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_to_contact_task(
      self,
      direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
      contact_force_newtons: float = 5.0,
      timeout_seconds: float = 15.0,
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds an SBL move_to_contact compliance task."""
    task_name = name or "Compliant Move to Contact"

    fixed_vector = move_to_contact_pb2.FixedVector(
        direction=vector3_pb2.Vector3(
            x=direction[0], y=direction[1], z=direction[2]
        )
    )

    skill_action = self._move_to_contact_skill(
        tool=self.tool_frame_reference,
        fixed_vector=fixed_vector,
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

  def build_move_cartesian_task(
      self,
      target_frame_name: str,
      target_object_name: str = "root",
      motion_type: str = "ANY",
      name: Optional[str] = None,
  ) -> bt.Node:
    task_name = name or f"Mock Move to {target_frame_name} ({motion_type})"
    self.executed_commands.append(
        f"move_cartesian:{target_object_name}/{target_frame_name}:{motion_type}"
    )
    return bt.Sequence([])

  def build_move_to_contact_task(
      self,
      direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
      contact_force_newtons: float = 5.0,
      timeout_seconds: float = 15.0,
      name: Optional[str] = None,
  ) -> bt.Node:
    task_name = name or "Mock Move to Contact"
    self.executed_commands.append(
        f"move_to_contact:dir={direction},force={contact_force_newtons}"
    )
    return bt.Sequence([])
