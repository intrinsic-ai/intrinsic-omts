"""Behavior Tree builder for building block pick and place."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import create_move_to_frame_task
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def build_building_block_pick_place_tree(
    robot: RobotInterface,
    gripper: GripperInterface,
    vision: Optional[VisionInterface] = None,
    target_object: str = "raw_stock",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    parent_object: str = "root",
    pregrasp_frame_name: str = "dynamic_pregrasp",
    grasp_frame_name: str = "dynamic_grasp",
    preplace_frame_name: str = "dynamic_preplace",
    place_frame_name: str = "dynamic_place",
    view_frame_name: str = "view",
    settling_timeout_seconds: Optional[float] = None,
    tree_name: str = "Building Block Pick & Place Cycle",
) -> bt.BehaviorTree:
  """Assembles the pick-transit-place Behavior Tree."""
  children = [
      create_move_to_frame_task(
          robot=robot,
          frame_name=pregrasp_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              "1. Move to Pre-Grasp"
              f" ({parent_object}/{pregrasp_frame_name})"
          ),
      ),
  ]

  step_offset = 0
  if vision is not None:
    children.append(
        vision.build_estimate_and_update_pose_task(
            target_object=target_object,
            pose_estimator_id=pose_estimator_id,
            name=f"2. Estimate & Update Pose ({target_object})",
        )
    )
    step_offset = 1

  children.extend([
      gripper.build_open_task(
          name=f"{2 + step_offset}. Open Gripper (Prepare Grasp)"
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=grasp_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{3 + step_offset}. Move to Grasp"
              f" ({parent_object}/{grasp_frame_name})"
          ),
      ),
      gripper.build_close_task(
          name=f"{4 + step_offset}. Close Gripper (Grasp Block)"
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=pregrasp_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{5 + step_offset}. Linear Retract Up"
              f" ({parent_object}/{pregrasp_frame_name})"
          ),
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=preplace_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{6 + step_offset}. Transit to Pre-Place"
              f" ({parent_object}/{preplace_frame_name})"
          ),
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=place_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{7 + step_offset}. Move to Place"
              f" ({parent_object}/{place_frame_name})"
          ),
      ),
      gripper.build_open_task(
          name=f"{8 + step_offset}. Open Gripper (Release Block)"
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=preplace_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{9 + step_offset}. Linear Retract from Place"
              f" ({parent_object}/{preplace_frame_name})"
          ),
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=view_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{10 + step_offset}. Return to View"
              f" ({parent_object}/{view_frame_name})"
          ),
      ),
  ])

  sequence = bt.Sequence(
      name="Building Block Pick & Place Sequence",
      children=children,
  )
  return bt.BehaviorTree(name=tree_name, root=sequence)

