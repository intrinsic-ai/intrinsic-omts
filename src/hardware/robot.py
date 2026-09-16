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

"""Robot hardware interface and implementations for Universal Robots and Mocks."""

import abc
import dataclasses
from collections.abc import Sequence
from typing import Any

from intrinsic.manipulation.skills.force import move_to_contact_pb2
from intrinsic.math.proto import (
  point_pb2,
  pose_pb2,
  quaternion_pb2,
  vector3_pb2,
)
from intrinsic.motion_planning.proto.v1 import geometric_constraints_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.world.proto import object_world_refs_pb2

from src.core.types import JointPosition
from src.utils.math_utils import (
  create_transform_node_ref,
  describe_motion_types,
  normalize_motion_types,
)


@dataclasses.dataclass(frozen=True)
class MotionConfig:
  """Cartesian robot kinematics, margins, and tool definitions."""

  min_safe_z: float = 0.95
  approach_height_m: float = 0.08
  grasp_offset_z: float = 0.005
  arm_part_name: str = "ur_module"
  tool_object_name: str = "gripper"
  tool_frame_name: str = "tool_frame"
  disable_collision_checking: bool = False
  contact_timeout_seconds: float = 40.0


class RobotInterface(abc.ABC):
  """Abstract interface for robot motion and compliant contact control."""

  @classmethod
  def from_config(
    cls,
    solution: Any,
    config: MotionConfig | None = None,
    mock_hardware: bool = False,
  ) -> "RobotInterface":
    """Creates and initializes the robot hardware adapter from configuration."""
    if mock_hardware:
      return MockRobot()
    cfg = config or MotionConfig()
    return UrRobot(
      solution=solution,
      arm_part_name=cfg.arm_part_name,
      tool_object_name=cfg.tool_object_name,
      tool_frame_name=cfg.tool_frame_name,
      disable_collision_checking=cfg.disable_collision_checking,
    )

  @abc.abstractmethod
  def clear_faults(self) -> bool:
    """Clears robot hardware faults via realtime control skill."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_joint_task(
    self,
    joint_target: str | JointPosition | Sequence[float] | Any,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to move the robot arm to a named joint pose or configuration."""
    raise NotImplementedError

  def build_move_to_joint_position_task(
    self,
    joint_position: JointPosition,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to move the robot arm to an explicit JointPosition."""
    return self.build_move_joint_task(
      joint_target=joint_position,
      name=name,
    )

  @abc.abstractmethod
  def build_move_cartesian_task(
    self,
    target_frame_name: str | None = None,
    target_object_name: str = "root",
    motion_type: str = "ANY",
    target_frame_offset: (
      tuple[tuple[float, float, float], tuple[float, float, float, float]]
      | None
    ) = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to move the robot tool to a target frame or object."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_blended_cartesian_task(
    self,
    target_frames: Sequence[tuple[str, str]],
    motion_type: str | Sequence[str] = "ANY",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to move through multiple target frames in a blended trajectory."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_relative_cartesian_task(
    self,
    translation: tuple[float, float, float],
    motion_type: str = "LINEAR",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to move the robot tool relative to its current pose."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_to_contact_task(
    self,
    direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
    contact_force_newtons: float = 5.0,
    timeout_seconds: float = 15.0,
    name: str | None = None,
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
    disable_collision_checking: bool = False,
  ) -> None:
    """Initializes UR robot adapter."""
    self._solution = solution
    self._arm_part_name = arm_part_name
    self._tool_object_name = tool_object_name
    self._tool_frame_name = tool_frame_name
    self._disable_collision_checking = disable_collision_checking
    self._move_robot_skill = solution.skills.ai.intrinsic.move_robot
    self._move_to_contact_skill = solution.skills.ai.intrinsic.move_to_contact

  @property
  def arm_part(self) -> Any:
    """Retrieves the robot arm part from the solution world."""
    return getattr(self._solution.world, self._arm_part_name)

  @property
  def tool_frame_reference(
    self,
  ) -> object_world_refs_pb2.TransformNodeReference:
    """Constructs the moving tool frame reference (gripper TCP)."""
    return create_transform_node_ref(
      self._tool_object_name, self._tool_frame_name
    )

  def clear_faults(self) -> bool:
    """Clears faults with ai.intrinsic.enable_realtime_control."""
    skills = getattr(self._solution, "skills", None)
    ai_skills = getattr(skills, "ai", None) if skills else None
    intrinsic_skills = (
      getattr(ai_skills, "intrinsic", None) if ai_skills else None
    )
    if not intrinsic_skills or not hasattr(
      intrinsic_skills, "enable_realtime_control"
    ):
      return False

    action = intrinsic_skills.enable_realtime_control(clear_faults=True)
    tree = bt.Sequence(
      name="Clear Robot Faults Sequence",
      children=[
        bt.Task(action=action, name="Enable Realtime Control and Clear Faults")
      ],
    )
    if hasattr(self._solution, "executive") and hasattr(
      self._solution.executive, "run"
    ):
      self._solution.executive.run(tree)
    elif hasattr(self._solution, "execute"):
      self._solution.execute(tree)
    elif hasattr(self._solution, "run"):
      self._solution.run(tree)
    return True

  def _motion_type_enum(self, motion_type: str) -> Any:
    """Maps a motion type name onto MotionSegment.MotionType, defaulting to ANY."""
    motion_proto = (
      self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType
    )
    return {
      "LINEAR": motion_proto.LINEAR,
      "JOINT": motion_proto.JOINT,
    }.get(motion_type.upper(), motion_proto.ANY)

  def _get_collision_settings(self) -> Any | None:
    """Returns CollisionSettings with disabled collision checking."""
    if not self._disable_collision_checking:
      return None
    try:
      if hasattr(self._move_robot_skill, "intrinsic_proto") and hasattr(
        self._move_robot_skill.intrinsic_proto, "world"
      ):
        return self._move_robot_skill.intrinsic_proto.world.CollisionSettings(
          disable_collision_checking=True
        )
    except (AttributeError, TypeError, ValueError):
      pass
    return None

  def _build_trajectory_segment(
    self, motion_type_enum: Any, **segment_kwargs: Any
  ) -> Any:
    """Builds a MotionSegment proto with optional collision settings."""
    col_settings = self._get_collision_settings()
    if col_settings is not None:
      segment_kwargs["collision_settings"] = col_settings
    return self._move_robot_skill.intrinsic_proto.skills.MotionSegment(
      motion_type=motion_type_enum,
      **segment_kwargs,
    )

  def build_move_joint_task(
    self,
    joint_target: str | JointPosition | Sequence[float] | Any,
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL move_robot joint motion task."""
    if isinstance(joint_target, str):
      task_name = name or f"Move to {joint_target}"
      try:
        target_pos = getattr(self.arm_part.joint_configurations, joint_target)
      except AttributeError:
        if (
          hasattr(self.arm_part, "joint_configurations")
          and joint_target in self.arm_part.joint_configurations
        ):
          target_pos = self.arm_part.joint_configurations[joint_target]
        else:
          raise
    elif isinstance(joint_target, JointPosition):
      target_list = joint_target.to_list()
      task_name = name or f"Move to joint positions {target_list}"
      if hasattr(self._move_robot_skill, "intrinsic_proto") and hasattr(
        self._move_robot_skill.intrinsic_proto, "icon"
      ):
        target_pos = self._move_robot_skill.intrinsic_proto.icon.JointVec(
          joints=target_list
        )
      else:
        target_pos = target_list
    elif hasattr(joint_target, "joint_position") or hasattr(
      joint_target, "joints"
    ):
      task_name = name or "Move to Joint Configuration"
      target_pos = joint_target
    elif isinstance(joint_target, (list, tuple, Sequence)):
      target_list = list(joint_target)
      task_name = name or f"Move to joint positions {target_list}"
      if hasattr(self._move_robot_skill, "intrinsic_proto") and hasattr(
        self._move_robot_skill.intrinsic_proto, "icon"
      ):
        target_pos = self._move_robot_skill.intrinsic_proto.icon.JointVec(
          joints=target_list
        )
      else:
        target_pos = target_list
    else:
      raise ValueError(f"Unsupported joint target type: {type(joint_target)}")

    motion_type_joint = self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.JOINT
    segment = self._build_trajectory_segment(
      motion_type_enum=motion_type_joint,
      joint_position=target_pos,
    )
    skill_action = self._move_robot_skill(
      motion_segments=[segment],
      arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_cartesian_task(
    self,
    target_frame_name: str | None = None,
    target_object_name: str = "root",
    motion_type: str = "ANY",
    target_frame_offset: (
      tuple[tuple[float, float, float], tuple[float, float, float, float]]
      | None
    ) = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL move_robot Cartesian motion task aligning tool to frame."""
    target_desc = (
      f"{target_object_name}/{target_frame_name}"
      if target_frame_name
      else target_object_name
    )
    task_name = name or f"Move to {target_desc} ({motion_type})"

    target_node_ref = create_transform_node_ref(
      target_object_name, target_frame_name
    )
    cartesian_pose = geometric_constraints_pb2.PoseEquality(
      moving_frame=self.tool_frame_reference,
      target_frame=target_node_ref,
    )
    if target_frame_offset is not None:
      pos, quat = target_frame_offset
      cartesian_pose.target_frame_offset.CopyFrom(
        pose_pb2.Pose(
          position=point_pb2.Point(x=pos[0], y=pos[1], z=pos[2]),
          orientation=quaternion_pb2.Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
          ),
        )
      )

    motion_segment = self._build_trajectory_segment(
      motion_type_enum=self._motion_type_enum(motion_type),
      cartesian_pose=cartesian_pose,
    )
    skill_action = self._move_robot_skill(
      motion_segments=[motion_segment],
      arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_blended_cartesian_task(
    self,
    target_frames: Sequence[tuple[str, str]],
    motion_type: str | Sequence[str] = "ANY",
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL move_robot task executing a blended trajectory."""
    if not target_frames:
      raise ValueError("target_frames must contain at least one target frame.")

    motion_types = normalize_motion_types(motion_type, len(target_frames))
    path_desc = " -> ".join(f"{obj}/{frame}" for obj, frame in target_frames)
    task_name = name or (
      f"Blended Move through {path_desc} ({describe_motion_types(motion_types)})"
    )

    motion_segments = []
    for (obj_name, frame_name), segment_type in zip(
      target_frames, motion_types, strict=True
    ):
      target_node_ref = create_transform_node_ref(obj_name, frame_name)
      cartesian_pose = geometric_constraints_pb2.PoseEquality(
        moving_frame=self.tool_frame_reference,
        target_frame=target_node_ref,
      )
      motion_segments.append(
        self._build_trajectory_segment(
          motion_type_enum=self._motion_type_enum(segment_type),
          cartesian_pose=cartesian_pose,
        )
      )

    skill_action = self._move_robot_skill(
      motion_segments=motion_segments,
      arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_relative_cartesian_task(
    self,
    translation: tuple[float, float, float],
    motion_type: str = "LINEAR",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a relative Cartesian motion task along tool frames."""
    task_name = (
      name
      or f"Move relative ({translation[0]:.3f}, {translation[1]:.3f},"
      f" {translation[2]:.3f}) [{motion_type}]"
    )

    relative_cartesian_pose = geometric_constraints_pb2.RelativePoseEquality(
      moving_frame=self.tool_frame_reference,
      relative_pose=pose_pb2.Pose(
        position=point_pb2.Point(
          x=translation[0], y=translation[1], z=translation[2]
        ),
        orientation=quaternion_pb2.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
      ),
    )
    motion_segment = self._build_trajectory_segment(
      motion_type_enum=self._motion_type_enum(motion_type),
      relative_cartesian_pose=relative_cartesian_pose,
    )
    skill_action = self._move_robot_skill(
      motion_segments=[motion_segment],
      arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_to_contact_task(
    self,
    direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
    contact_force_newtons: float = 5.0,
    timeout_seconds: float = 15.0,
    name: str | None = None,
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

  def clear_faults(self) -> bool:
    self.executed_commands.append("clear_faults")
    return True

  def build_move_joint_task(
    self,
    joint_target: str | JointPosition | Sequence[float] | Any,
    name: str | None = None,
  ) -> bt.Node:
    if isinstance(joint_target, JointPosition):
      target_desc = str(joint_target.to_list())
    elif isinstance(joint_target, str):
      target_desc = joint_target
    else:
      target_desc = str(list(joint_target))
    task_name = name or f"Mock Move to {target_desc}"
    self.executed_commands.append(f"move_joint:{target_desc}")
    return bt.Sequence(name=task_name, children=[])

  def build_move_cartesian_task(
    self,
    target_frame_name: str | None = None,
    target_object_name: str = "root",
    motion_type: str = "ANY",
    target_frame_offset: (
      tuple[tuple[float, float, float], tuple[float, float, float, float]]
      | None
    ) = None,
    name: str | None = None,
  ) -> bt.Node:
    target_desc = (
      f"{target_object_name}/{target_frame_name}"
      if target_frame_name
      else target_object_name
    )
    task_name = name or f"Mock Move to {target_desc} ({motion_type})"
    self.executed_commands.append(f"move_cartesian:{target_desc}:{motion_type}")
    return bt.Sequence(name=task_name, children=[])

  def build_move_blended_cartesian_task(
    self,
    target_frames: Sequence[tuple[str, str]],
    motion_type: str | Sequence[str] = "ANY",
    name: str | None = None,
  ) -> bt.Node:
    motion_types = normalize_motion_types(motion_type, len(target_frames))
    type_desc = describe_motion_types(motion_types)
    path_desc = "->".join(f"{obj}/{frame}" for obj, frame in target_frames)
    task_name = name or f"Mock Blended Move to {path_desc} ({type_desc})"
    self.executed_commands.append(
      f"move_blended_cartesian:{path_desc}:{type_desc}"
    )
    return bt.Sequence(name=task_name, children=[])

  def build_move_relative_cartesian_task(
    self,
    translation: tuple[float, float, float],
    motion_type: str = "LINEAR",
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or f"Mock Move Relative ({translation}) [{motion_type}]"
    self.executed_commands.append(
      f"move_relative_cartesian:{translation}:{motion_type}"
    )
    return bt.Sequence(name=task_name, children=[])

  def build_move_to_contact_task(
    self,
    direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
    contact_force_newtons: float = 5.0,
    timeout_seconds: float = 15.0,
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or "Mock Move to Contact"
    self.executed_commands.append(
      f"move_to_contact:dir={direction},force={contact_force_newtons}"
    )
    return bt.Sequence(name=task_name, children=[])
