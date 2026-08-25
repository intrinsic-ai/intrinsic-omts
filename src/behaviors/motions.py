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


def create_compliant_touchdown_task(
    robot: RobotInterface,
    direction: tuple[float, float, float] = (0.0, 0.0, -1.0),
    contact_force_newtons: float = 5.0,
    timeout_seconds: float = 15.0,
    task_name: Optional[str] = None,
) -> bt.Node:
  """Builds a compliant move_to_contact task."""
  return robot.build_move_to_contact_task(
      direction=direction,
      contact_force_newtons=contact_force_newtons,
      timeout_seconds=timeout_seconds,
      name=task_name or "Compliant Touchdown",
  )
