"""Infeed part localization and picking subtree."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import create_compliant_touchdown_task, create_move_to_named_pose_task
from src.core.infeed import GridInfeedStrategy, InfeedStrategy, PerceptionInfeedStrategy
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def build_pick_from_infeed_subtree(
    robot: RobotInterface,
    gripper: GripperInterface,
    vision: VisionInterface,
    infeed_strategy: InfeedStrategy,
    workpiece: Workpiece,
    view_pose_name: str = "view_pose",
) -> bt.Node:
  """Builds the Behavior Tree subtree for locating and grasping a raw workpiece.

  Steps (1-2 in OMTS pipeline):
  1. Open gripper.
  2. Move robot to view pose.
  3. If perception infeed: trigger camera image capture & pose estimation.
  4. Perform compliant touchdown via move_to_contact into workpiece.
  5. Close gripper to grasp part.
  6. Retract arm with workpiece.

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      vision: 3D camera / perception adapter.
      infeed_strategy: Vision vs. grid infeed strategy.
      workpiece: Workpiece instance being picked.
      view_pose_name: Joint configuration name for perception view.

  Returns:
      Behavior tree sequence node executing infeed pick.
  """
  tasks: list[bt.Node] = [
      gripper.build_open_task(name="Step 01a: Open Gripper"),
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=view_pose_name,
          task_name=f"Step 01b: Move to {view_pose_name}",
      ),
  ]

  if isinstance(infeed_strategy, PerceptionInfeedStrategy):
    tasks.append(
        vision.build_capture_image_task(name="Step 01c: Capture Infeed Image")
    )

  tasks.extend([
      create_compliant_touchdown_task(
          robot=robot,
          direction=(0.0, 0.0, -1.0),
          contact_force_newtons=5.0,
          task_name="Step 02a: Compliant Touchdown to Raw Stock",
      ),
      gripper.build_close_task(name="Step 02b: Grasp Raw Stock"),
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=view_pose_name,
          task_name="Step 02c: Retract Part from Infeed",
      ),
  ])

  return bt.Sequence(name="1. Pick From Infeed Subtree", children=tasks)
