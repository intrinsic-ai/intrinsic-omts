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


class RobotInterface(abc.ABC):
  """Abstract interface for robot motion and compliant contact control."""

  @abc.abstractmethod
  def build_move_joint_task(
    self, joint_configuration_name: str, name: str | None = None
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot arm to a named joint pose."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_cartesian_task(
    self,
    target_frame_name: str | None = None,
    target_object_name: str = "root",
    motion_type: str = "ANY",
    allow_tool_z_rotation: bool = False,
    cone_opening_half_angle: float = 0.0,
    moving_frame_offset: tuple[float, float, float] | None = None,
    target_frame_offset: tuple[
      tuple[float, float, float], tuple[float, float, float, float]
    ]
    | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot tool to a target frame or object."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_relative_cartesian_task(
    self,
    translation: tuple[float, float, float],
    motion_type: str = "LINEAR",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot tool relative to its current pose."""
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
    disable_collision_checking: bool = True,
  ) -> None:
    """Initializes UR robot adapter.

    Args:
        solution: Connected SBL deployment instance (from deployments.connect).
        arm_part_name: Attribute name of robot arm in solution.world.
        tool_object_name: Object name for moving tool frame (default: 'gripper').
        tool_frame_name: Frame name under tool_object_name (default: 'tool_frame').
        disable_collision_checking: Whether to disable collision checking in motion planning.
    """
    self._solution = solution
    self._arm_part_name = arm_part_name
    self._tool_object_name = tool_object_name
    self._tool_frame_name = tool_frame_name
    self._disable_collision_checking = disable_collision_checking
    self._move_robot_skill = solution.skills.ai.intrinsic.move_robot
    self._move_to_contact_skill = solution.skills.ai.intrinsic.move_to_contact

  def _get_collision_settings(self) -> Any | None:
    """Returns CollisionSettings with disabled collision checking if configured."""
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

  @property
  def arm_part(self) -> Any:
    """Retrieves the robot arm part from the solution world."""
    return getattr(self._solution.world, self._arm_part_name)

  @property
  def tool_frame_reference(
    self,
  ) -> object_world_refs_pb2.TransformNodeReference:
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
    self, joint_configuration_name: str, name: str | None = None
  ) -> bt.Node:
    """Builds an SBL move_robot joint motion task."""
    task_name = name or f"Move to {joint_configuration_name}"
    joint_target = getattr(
      self.arm_part.joint_configurations, joint_configuration_name
    )

    segment_kwargs: dict[str, Any] = {
      "joint_position": joint_target,
      "motion_type": (
        self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.JOINT
      ),
    }
    col_settings = self._get_collision_settings()
    if col_settings is not None:
      segment_kwargs["collision_settings"] = col_settings

    skill_action = self._move_robot_skill(
      motion_segments=[
        self._move_robot_skill.intrinsic_proto.skills.MotionSegment(
          **segment_kwargs
        )
      ],
      arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_cartesian_task(
    self,
    target_frame_name: str | None = None,
    target_object_name: str = "root",
    motion_type: str = "ANY",
    allow_tool_z_rotation: bool = False,
    cone_opening_half_angle: float = 0.0,
    moving_frame_offset: tuple[float, float, float] | None = None,
    target_frame_offset: tuple[
      tuple[float, float, float], tuple[float, float, float, float]
    ]
    | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL move_robot Cartesian motion task aligning tool to target frame or object.

    When `allow_tool_z_rotation=True`, replaces strict 6-DOF PoseEquality with a
    ConstraintIntersection of PositionEquality and RotationCone along tool +Z.
    This frees the wrist rotation around the approach axis, significantly
    expanding the feasible IK solution space.
    """
    target_desc = (
      f"{target_object_name}/{target_frame_name}"
      if target_frame_name
      else target_object_name
    )
    task_name = name or f"Move to {target_desc} ({motion_type})"

    if target_frame_name:
      target_node_ref = object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
          frame=object_world_refs_pb2.FrameReferenceByName(
            object_name=target_object_name,
            frame_name=target_frame_name,
          )
        )
      )
    else:
      target_node_ref = object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
          object=object_world_refs_pb2.ObjectReferenceByName(
            object_name=target_object_name,
          )
        )
      )

    if motion_type.upper() == "LINEAR":
      motion_type_enum = self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.LINEAR
    elif motion_type.upper() == "JOINT":
      motion_type_enum = self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.JOINT
    else:
      motion_type_enum = self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.ANY

    if allow_tool_z_rotation:
      pos_equality = geometric_constraints_pb2.PositionEquality(
        moving_frame=self.tool_frame_reference,
        target_frame=target_node_ref,
      )
      if moving_frame_offset is not None:
        pos_equality.moving_frame_offset.CopyFrom(
          point_pb2.Point(
            x=moving_frame_offset[0],
            y=moving_frame_offset[1],
            z=moving_frame_offset[2],
          )
        )
      if target_frame_offset is not None:
        pos, _ = target_frame_offset
        pos_equality.target_frame_offset.CopyFrom(
          point_pb2.Point(x=pos[0], y=pos[1], z=pos[2])
        )

      # Constrain tool +Z to point vertically downwards (-Z in root) into the table,
      # freeing rotation around the tool approach axis.
      rot_cone_target_frame = object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
          object=object_world_refs_pb2.ObjectReferenceByName(
            object_name="root",
          )
        )
      )
      rot_cone_target_axis = vector3_pb2.Vector3(x=0.0, y=0.0, z=-1.0)

      rot_cone = geometric_constraints_pb2.RotationCone(
        moving_frame=self.tool_frame_reference,
        target_frame=rot_cone_target_frame,
        moving_axis=vector3_pb2.Vector3(x=0.0, y=0.0, z=1.0),
        target_axis=rot_cone_target_axis,
        cone_opening_half_angle=cone_opening_half_angle,
      )
      constraint_intersection = (
        geometric_constraints_pb2.ConstraintIntersection(
          constraints=[
            geometric_constraints_pb2.GeometricConstraint(
              position_equality=pos_equality
            ),
            geometric_constraints_pb2.GeometricConstraint(
              rotation_cone=rot_cone
            ),
          ]
        )
      )
      segment_kwargs = {
        "constraint_intersection": constraint_intersection,
        "motion_type": motion_type_enum,
      }
    else:
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
      segment_kwargs = {
        "cartesian_pose": cartesian_pose,
        "motion_type": motion_type_enum,
      }

    col_settings = self._get_collision_settings()
    if col_settings is not None:
      segment_kwargs["collision_settings"] = col_settings

    motion_segment = (
      self._move_robot_skill.intrinsic_proto.skills.MotionSegment(
        **segment_kwargs
      )
    )

    skill_action = self._move_robot_skill(
      motion_segments=[motion_segment],
      arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_relative_cartesian_task(
    self,
    translation: tuple[float, float, float],
    motion_type: str = "LINEAR",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a relative Cartesian motion task along tool frames using RelativePoseEquality."""
    task_name = (
      name
      or f"Move relative ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) [{motion_type}]"
    )

    if motion_type.upper() == "LINEAR":
      motion_type_enum = self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.LINEAR
    elif motion_type.upper() == "JOINT":
      motion_type_enum = self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.JOINT
    else:
      motion_type_enum = self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType.ANY

    relative_cartesian_pose = geometric_constraints_pb2.RelativePoseEquality(
      moving_frame=self.tool_frame_reference,
      relative_pose=pose_pb2.Pose(
        position=point_pb2.Point(
          x=translation[0], y=translation[1], z=translation[2]
        ),
        orientation=quaternion_pb2.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
      ),
    )
    segment_kwargs = {
      "relative_cartesian_pose": relative_cartesian_pose,
      "motion_type": motion_type_enum,
    }

    col_settings = self._get_collision_settings()
    if col_settings is not None:
      segment_kwargs["collision_settings"] = col_settings

    motion_segment = (
      self._move_robot_skill.intrinsic_proto.skills.MotionSegment(
        **segment_kwargs
      )
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

  def build_move_joint_task(
    self, joint_configuration_name: str, name: str | None = None
  ) -> bt.Node:
    task_name = name or f"Mock Move to {joint_configuration_name}"
    self.executed_commands.append(f"move_joint:{joint_configuration_name}")
    return bt.Sequence(name=task_name, children=[])

  def build_move_cartesian_task(
    self,
    target_frame_name: str | None = None,
    target_object_name: str = "root",
    motion_type: str = "ANY",
    allow_tool_z_rotation: bool = False,
    cone_opening_half_angle: float = 0.0,
    moving_frame_offset: tuple[float, float, float] | None = None,
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
    self.executed_commands.append(
      f"move_cartesian:{target_desc}:{motion_type}:z_rot={allow_tool_z_rotation}"
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
