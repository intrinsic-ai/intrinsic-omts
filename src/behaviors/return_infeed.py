"""Infeed return / outfeed placement subtree."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import create_compliant_touchdown_task, create_move_to_named_pose_task
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface


def build_return_to_infeed_subtree(
    robot: RobotInterface,
    gripper: GripperInterface,
    workpiece: Workpiece,
    infeed_approach_pose_name: str = "view_pose3",
    home_pose_name: str = "home",
) -> bt.Node:
  """Builds the Behavior Tree subtree for returning the finished part back to infeed.

  Steps (13 in OMTS pipeline):
  16. Navigate to infeed return approach pose.
  17. Perform compliant touchdown via move_to_contact to place part safely on table.
  18. Open gripper to release part.
  19. Retract arm and move to home pose.

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      workpiece: Workpiece instance being returned.
      infeed_approach_pose_name: Joint configuration name for infeed approach.
      home_pose_name: Joint configuration name for home / final idle position.

  Returns:
      Behavior tree sequence node executing infeed return.
  """
  tasks: list[bt.Node] = [
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=infeed_approach_pose_name,
          task_name=f"Step 13a: Approach Infeed Placement ({infeed_approach_pose_name})",
      ),
      create_compliant_touchdown_task(
          robot=robot,
          direction=(0.0, 0.0, -1.0),
          contact_force_newtons=5.0,
          task_name="Step 13b: Compliant Touchdown to Table Surface",
      ),
      gripper.build_open_task(name="Step 13c: Release Finished Part"),
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=infeed_approach_pose_name,
          task_name="Step 13d: Retract Arm from Table",
      ),
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=home_pose_name,
          task_name=f"Step 13e: Return to {home_pose_name} Pose",
      ),
  ]

  return bt.Sequence(name="5. Return to Infeed Subtree", children=tasks)
