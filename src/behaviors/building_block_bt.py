"""Behavior Tree builder for building block pick and place."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import create_move_to_frame_task
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def build_building_block_pick_place_tree(
    robot: RobotInterface,
    gripper: GripperInterface,
    machine: Optional[CncMachineInterface] = None,
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

  step = 2
  if vision is not None:
    children.append(
        vision.build_estimate_and_update_pose_task(
            target_object=target_object,
            pose_estimator_id=pose_estimator_id,
            name=f"{step}. Estimate & Update Pose ({target_object})",
        )
    )
    step += 1

  children.append(
      gripper.build_open_task(
          name=f"{step}. Open Gripper (Prepare Grasp)"
      )
  )
  step += 1

  if machine is not None:
    children.append(
        machine.build_open_vise_task(
            name=f"{step}. Open CNC Vise (Prepare Vise)"
        )
    )
    step += 1

  children.extend([
      create_move_to_frame_task(
          robot=robot,
          frame_name=grasp_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{step}. Move to Grasp"
              f" ({parent_object}/{grasp_frame_name})"
          ),
      ),
      gripper.build_close_task(
          name=f"{step + 1}. Close Gripper (Grasp Block)"
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=pregrasp_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{step + 2}. Linear Retract Up"
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
              f"{step + 3}. Transit to Pre-Place"
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
              f"{step + 4}. Move to Place"
              f" ({parent_object}/{place_frame_name})"
          ),
      ),
      gripper.build_open_task(
          name=f"{step + 5}. Open Gripper (Release Block)"
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=preplace_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=(
              f"{step + 6}. Linear Retract from Place"
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
              f"{step + 7}. Return to View"
              f" ({parent_object}/{view_frame_name})"
          ),
      ),
  ])

  sequence = bt.Sequence(
      name="Building Block Pick & Place Sequence",
      children=children,
  )
  return bt.BehaviorTree(name=tree_name, root=sequence)

