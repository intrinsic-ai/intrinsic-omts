"""Reusable motion tasks and building blocks for robot arm movements."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.hardware.robot import RobotInterface


def create_move_to_named_pose_task(
    robot: RobotInterface,
    pose_name: str,
    task_name: Optional[str] = None,
) -> bt.Node:
  """Builds a joint motion task moving the arm to a named joint configuration."""
  return robot.build_move_joint_task(
      joint_configuration_name=pose_name,
      name=task_name or f"Move to {pose_name}",
  )


def create_move_to_frame_task(
    robot: RobotInterface,
    frame_name: str,
    parent_object: str = "root",
    motion_type: str = "ANY",
    task_name: Optional[str] = None,
) -> bt.Node:
  """Builds a Cartesian motion task moving the arm tool to a target frame."""
  return robot.build_move_cartesian_task(
      target_frame_name=frame_name,
      target_object_name=parent_object,
      motion_type=motion_type,
      name=task_name or f"Move to {parent_object}/{frame_name} ({motion_type})",
  )


def create_compliant_touchdown_task(
    robot: RobotInterface,
    direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
    contact_force_newtons: float = 10.0,
    timeout_seconds: float = 20.0,
    task_name: Optional[str] = None,
) -> bt.Node:
  """Builds a compliant move_to_contact task."""
  return robot.build_move_to_contact_task(
      direction=direction,
      contact_force_newtons=contact_force_newtons,
      timeout_seconds=timeout_seconds,
      name=task_name or "Compliant Touchdown",
  )
