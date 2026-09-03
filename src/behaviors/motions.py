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
    frame_name: Optional[str] = None,
    parent_object: str = "root",
    motion_type: str = "ANY",
    allow_tool_z_rotation: bool = False,
    cone_opening_half_angle: float = 0.0,
    moving_frame_offset: Optional[tuple[float, float, float]] = None,
    target_frame_offset: Optional[
        tuple[tuple[float, float, float], tuple[float, float, float, float]]
    ] = None,
    task_name: Optional[str] = None,
) -> bt.Node:
  """Builds a Cartesian motion task moving the arm tool to a target frame or object."""
  target_desc = f"{parent_object}/{frame_name}" if frame_name else parent_object
  return robot.build_move_cartesian_task(
      target_frame_name=frame_name,
      target_object_name=parent_object,
      motion_type=motion_type,
      allow_tool_z_rotation=allow_tool_z_rotation,
      cone_opening_half_angle=cone_opening_half_angle,
      moving_frame_offset=moving_frame_offset,
      target_frame_offset=target_frame_offset,
      name=task_name or f"Move to {target_desc} ({motion_type})",
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


def create_relative_retract_task(
    robot: RobotInterface,
    distance_meters: float = 0.03,
    task_name: Optional[str] = None,
) -> bt.Node:
  """Builds a relative Cartesian linear retract task along the tool -Z axis.

  Args:
      robot: Robot controller adapter.
      distance_meters: Positive distance in meters to retract along tool -Z (default: 0.03m / 3cm).
      task_name: Optional custom descriptive name for the behavior tree task.

  Returns:
      Executable behavior tree Node commanding relative Cartesian LINEAR motion.
  """
  retract_distance = -abs(distance_meters)
  return robot.build_move_relative_cartesian_task(
      translation=(0.0, 0.0, retract_distance),
      motion_type="LINEAR",
      name=task_name or f"Relative Retract ({abs(distance_meters) * 100:.1f} cm, -Z Tool)",
  )

