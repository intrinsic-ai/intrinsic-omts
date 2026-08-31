"""Infeed part localization and picking subtree."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import (
    create_compliant_touchdown_task,
    create_move_to_frame_task,
)
from src.core.infeed import InfeedStrategy
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
    parent_object: str = "root",
    view_frame_name: str = "view",
    pregrasp_frame_name: str = "pre_grasp",
    grasp_frame_name: str = "grasp",
) -> bt.Node:
  """Builds the Behavior Tree subtree for locating and grasping a raw workpiece.

  Sequence:
  1. Move robot to view frame (ANY Cartesian motion).
  2. Mock perception acquisition step.
  3. Move to pre_grasp frame (ANY Cartesian motion).
  4. Perform compliant touchdown (move_to_contact in -Z).
  5. Mock close gripper to grasp part.
  6. Retract arm linearly back up to pre_grasp (LINEAR Cartesian motion).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      vision: Vision/perception adapter.
      infeed_strategy: Infeed strategy model.
      workpiece: Workpiece model instance.
      parent_object: Name of parent object for target frames (default: 'root').
      view_frame_name: Name of perception view frame (default: 'view').
      pregrasp_frame_name: Name of pre-grasp approach frame (default: 'pre_grasp').
      grasp_frame_name: Name of grasp target frame (default: 'grasp').

  Returns:
      Behavior tree sequence executing the infeed pick pipeline.
  """
  tasks: list[bt.Node] = [
      create_move_to_frame_task(
          robot=robot,
          frame_name=view_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          task_name=f"Step 01: Move to View Frame ({parent_object}/{view_frame_name})",
      ),
      vision.build_capture_image_task(name="Step 02: Mock Perception Acquisition"),
      create_move_to_frame_task(
          robot=robot,
          frame_name=pregrasp_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          task_name=f"Step 03: Move to Pre-Grasp ({parent_object}/{pregrasp_frame_name})",
      ),
      create_compliant_touchdown_task(
          robot=robot,
          direction=(0.0, 0.0, 1.0),
          contact_force_newtons=10.0,
          task_name="Step 04: Compliant Touchdown to Part (+Z Tool)",
      ),
      gripper.build_close_task(name="Step 05: Mock Close Gripper (Grasp Part)"),
      create_move_to_frame_task(
          robot=robot,
          frame_name=pregrasp_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          task_name=f"Step 06: Linear Retract to Pre-Grasp ({parent_object}/{pregrasp_frame_name})",
      ),
  ]

  return bt.Sequence(name="1. Infeed Pick Subtree", children=tasks)
