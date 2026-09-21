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

"""Robot hardware interface and Universal Robots implementation for SBL."""

import abc
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
from intrinsic.world.proto import (
  collision_action_pb2,
  collision_settings_pb2,
  object_world_refs_pb2,
)

from src.core.config import RobotConfig
from src.core.types import JointPosition, Pose3D
from src.utils.execution_utils import (
  create_transform_node_ref,
  describe_motion_types,
  normalize_motion_types,
  object_exists_in_world,
)


class RobotInterface(abc.ABC):
  """Abstract interface for robot motion, compliant contact, and object attachment."""

  @abc.abstractmethod
  def build_move_joint_task(
    self,
    joint_target: str | JointPosition | Sequence[float] | Any,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot arm to a named joint pose or configuration."""
    raise NotImplementedError

  def build_move_to_joint_position_task(
    self,
    joint_positions: JointPosition | Sequence[float],
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot arm to an explicit JointPosition."""
    if isinstance(joint_positions, JointPosition):
      joint_positions = joint_positions.as_list()
    return self.build_move_joint_task(
      joint_target=joint_positions,
      name=name,
    )

  @abc.abstractmethod
  def build_move_cartesian_task(
    self,
    target_frame_name: str | None,
    target_object_name: str,
    motion_type: str,
    allow_tool_z_rotation: bool = False,
    cone_opening_half_angle: float = 0.0,
    moving_frame_offset: tuple[float, float, float] | None = None,
    target_frame_offset: (
      tuple[tuple[float, float, float], tuple[float, float, float, float]]
      | None
    ) = None,
    excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot tool to a target frame or object."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_blended_cartesian_task(
    self,
    target_frames: Sequence[tuple[str, str]],
    motion_type: str | Sequence[str] = "ANY",
    target_frame_offset: Pose3D | None = None,
    excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task to execute a blended trajectory through target frames."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_relative_cartesian_task(
    self,
    translation: tuple[float, float, float],
    motion_type: str = "LINEAR",
    excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
    exclude_collision: bool = False,
    excluded_collision_objects: Sequence[str] | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a behavior tree task to move the robot tool relative to its current pose."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_move_to_contact_task(
    self,
    direction: tuple[float, float, float],
    contact_force_newtons: float,
    timeout_seconds: float,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a compliant move_to_contact behavior tree task."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_attach_object_task(
    self,
    object_name: str,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to attach an object to the robot gripper in the object world."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_detach_object_task(
    self,
    object_name: str,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to detach an object from the robot gripper in the object world."""
    raise NotImplementedError


class UrRobot(RobotInterface):
  """Universal Robots controller wrapper targeting Intrinsic SBL skills."""

  def __init__(
    self,
    solution: Any,
    config: RobotConfig,
  ) -> None:
    """Initializes UR robot adapter with collision checking enabled.

    Args:
        solution: Connected SBL deployment instance (from deployments.connect).
        config: Scoped RobotConfig defining arm part and tool frame names.
    """
    self._solution = solution
    self._arm_part_name = config.arm_part_name
    self._tool_object_name = config.tool_object_name
    self._tool_frame_name = config.tool_frame_name
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

  def build_move_joint_task(
    self,
    joint_target: str | JointPosition | Sequence[float] | Any,
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL move_robot joint motion task."""
    if isinstance(joint_target, str):
      task_name = name or f"Move to {joint_target}"
      target_pos = getattr(self.arm_part.joint_configurations, joint_target)
    elif isinstance(joint_target, JointPosition):
      task_name = name or f"Move to JointPosition {joint_target.to_list()}"
      target_pos = joint_target.to_list()
    elif isinstance(joint_target, Sequence):
      target_pos = [float(v) for v in joint_target]
      task_name = name or f"Move to Joint {target_pos}"
    else:
      task_name = name or "Move Joint"
      target_pos = joint_target

    motion_segment = (
      self._move_robot_skill.intrinsic_proto.skills.MotionSegment(
        joint_position=target_pos,
        motion_type=self._motion_type_enum("JOINT"),
      )
    )
    skill_action = self._move_robot_skill(
      motion_segments=[motion_segment],
      arm_part=self.arm_part,
    )
    return bt.Task(action=skill_action, name=task_name)

  def build_move_cartesian_task(
    self,
    target_frame_name: str | None,
    target_object_name: str,
    motion_type: str,
    allow_tool_z_rotation: bool = False,
    cone_opening_half_angle: float = 0.0,
    moving_frame_offset: tuple[float, float, float] | None = None,
    target_frame_offset: (
      tuple[tuple[float, float, float], tuple[float, float, float, float]]
      | None
    ) = None,
    excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL move_robot Cartesian motion task aligning tool to target frame or object."""
    target_desc = (
      f"{target_object_name}/{target_frame_name}"
      if target_frame_name
      else target_object_name
    )
    task_name = name or f"Move to {target_desc} ({motion_type})"
    target_node_ref = create_transform_node_ref(
      target_object_name, target_frame_name
    )
    motion_type_enum = self._motion_type_enum(motion_type)

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

      rot_cone_target_frame = create_transform_node_ref("root")
      rot_cone = geometric_constraints_pb2.RotationCone(
        moving_frame=self.tool_frame_reference,
        target_frame=rot_cone_target_frame,
        moving_axis=vector3_pb2.Vector3(x=0.0, y=0.0, z=1.0),
        target_axis=vector3_pb2.Vector3(x=0.0, y=0.0, z=-1.0),
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
      segment_kwargs: dict[str, Any] = {
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

    collision_settings = self._build_collision_settings(
      excluded_collision_pairs
    )
    if collision_settings is not None:
      segment_kwargs["collision_settings"] = collision_settings

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

  def build_move_blended_cartesian_task(
    self,
    target_frames: Sequence[tuple[str, str]],
    motion_type: str | Sequence[str] = "ANY",
    target_frame_offset: Pose3D | None = None,
    excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL move_robot task executing a blended trajectory through target frames."""
    if not target_frames:
      raise ValueError("target_frames must contain at least one target frame.")

    motion_types = normalize_motion_types(motion_type, len(target_frames))
    path_desc = " -> ".join(f"{obj}/{frame}" for obj, frame in target_frames)
    task_name = name or (
      f"Blended Move through {path_desc} ({describe_motion_types(motion_types)})"
    )

    collision_settings = self._build_collision_settings(
      excluded_collision_pairs
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
      if target_frame_offset is not None:
        if isinstance(target_frame_offset, Pose3D):
          pos = (
            target_frame_offset.x,
            target_frame_offset.y,
            target_frame_offset.z,
          )
          quat = (
            target_frame_offset.qx,
            target_frame_offset.qy,
            target_frame_offset.qz,
            target_frame_offset.qw,
          )
        elif isinstance(target_frame_offset, tuple):
          pos, quat = target_frame_offset
        else:
          pos, quat = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        cartesian_pose.target_frame_offset.CopyFrom(
          pose_pb2.Pose(
            position=point_pb2.Point(x=pos[0], y=pos[1], z=pos[2]),
            orientation=quaternion_pb2.Quaternion(
              x=quat[0], y=quat[1], z=quat[2], w=quat[3]
            ),
          )
        )
      segment_kwargs: dict[str, Any] = {
        "cartesian_pose": cartesian_pose,
        "motion_type": self._motion_type_enum(segment_type),
      }
      if collision_settings is not None:
        segment_kwargs["collision_settings"] = collision_settings
      motion_segments.append(
        self._move_robot_skill.intrinsic_proto.skills.MotionSegment(
          **segment_kwargs
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
    excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
    exclude_collision: bool = False,
    excluded_collision_objects: Sequence[str] | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a relative Cartesian motion task along tool frames using RelativePoseEquality."""
    del exclude_collision
    task_name = (
      name
      or f"Move relative ({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) [{motion_type}]"
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
    segment_kwargs: dict[str, Any] = {
      "relative_cartesian_pose": relative_cartesian_pose,
      "motion_type": self._motion_type_enum(motion_type),
    }
    pairs = list(excluded_collision_pairs or ())
    if not pairs and excluded_collision_objects:
      pairs = [
        (self._tool_object_name, obj_name)
        for obj_name in excluded_collision_objects
      ]
    collision_settings = self._build_collision_settings(pairs or None)
    if collision_settings is not None:
      segment_kwargs["collision_settings"] = collision_settings

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
    direction: tuple[float, float, float],
    contact_force_newtons: float,
    timeout_seconds: float,
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

  def build_attach_object_task(
    self,
    object_name: str,
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL attach_object_to_robot task to attach an object to the gripper."""
    task_name = name or f"Attach {object_name} to {self._tool_object_name}"
    attach_skill = self._solution.skills.ai.intrinsic.attach_object_to_robot
    gripper_ref = object_world_refs_pb2.ObjectReference(
      by_name=object_world_refs_pb2.ObjectReferenceByName(
        object_name=self._tool_object_name
      )
    )
    object_ref = object_world_refs_pb2.ObjectReference(
      by_name=object_world_refs_pb2.ObjectReferenceByName(
        object_name=object_name
      )
    )
    action = attach_skill(
      gripper_entity=gripper_ref,
      object_entity=object_ref,
    )
    return bt.Task(action=action, name=task_name)

  def build_detach_object_task(
    self,
    object_name: str,
    name: str | None = None,
  ) -> bt.Node:
    """Builds an SBL detach_object task to detach an object from the gripper."""
    task_name = name or f"Detach {object_name} from {self._tool_object_name}"
    detach_skill = self._solution.skills.ai.intrinsic.detach_object
    gripper_ref = object_world_refs_pb2.ObjectReference(
      by_name=object_world_refs_pb2.ObjectReferenceByName(
        object_name=self._tool_object_name
      )
    )
    object_ref = object_world_refs_pb2.ObjectReference(
      by_name=object_world_refs_pb2.ObjectReferenceByName(
        object_name=object_name
      )
    )
    action = detach_skill(
      gripper_entity=gripper_ref,
      object_entity=object_ref,
    )
    return bt.Task(action=action, name=task_name)

  def _motion_type_enum(self, motion_type: str) -> Any:
    """Maps a motion type string to MotionSegment.MotionType enum."""
    motion_proto = (
      self._move_robot_skill.intrinsic_proto.skills.MotionSegment.MotionType
    )
    return {
      "LINEAR": motion_proto.LINEAR,
      "JOINT": motion_proto.JOINT,
    }.get(motion_type.upper(), motion_proto.ANY)

  def _build_collision_settings(
    self,
    excluded_collision_pairs: Sequence[tuple[str, str]] | None,
  ) -> collision_settings_pb2.CollisionSettings | None:
    """Builds scoped CollisionSettings for valid world object pairs."""
    if not excluded_collision_pairs:
      return None
    valid_pairs = [
      (left_obj, right_obj)
      for left_obj, right_obj in excluded_collision_pairs
      if object_exists_in_world(self._solution, left_obj)
      and object_exists_in_world(self._solution, right_obj)
    ]
    if not valid_pairs:
      return None
    collision_rules = [
      collision_settings_pb2.CollisionSettings.CollisionRule(
        left=[
          collision_settings_pb2.ObjectOrEntityReference(
            object=object_world_refs_pb2.ObjectReference(
              by_name=object_world_refs_pb2.ObjectReferenceByName(
                object_name=left_obj,
              )
            )
          )
        ],
        right=[
          collision_settings_pb2.ObjectOrEntityReference(
            object=object_world_refs_pb2.ObjectReference(
              by_name=object_world_refs_pb2.ObjectReferenceByName(
                object_name=right_obj,
              )
            )
          )
        ],
        collision_action=collision_action_pb2.CollisionAction(
          is_excluded=True,
        ),
      )
      for left_obj, right_obj in valid_pairs
    ]
    return collision_settings_pb2.CollisionSettings(
      disable_collision_checking=False,
      collision_rules=collision_rules,
    )
