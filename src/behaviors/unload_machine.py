"""CNC machine unloading and extraction subtree."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import (
    create_compliant_touchdown_task,
    create_move_to_frame_task,
)
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_unload_machine_subtree(
    robot: RobotInterface,
    gripper: GripperInterface,
    machine: CncMachineInterface,
    workpiece: Workpiece,
    parent_object: str = "root",
    machine_approach_frame_name: str = "machine_approach",
    preplace_vise_frame_name: str = "pre_place_vise",
    place_vise_frame_name: str = "place_vise",
) -> bt.Node:
  """Builds the Behavior Tree subtree for unloading a finished part from the CNC.

  Steps (10-12 in OMTS pipeline):
  10. Open CNC door (DIO / Mock).
  11. Open CNC vise (DIO / Mock).
  12a. Move robot arm to machine entry approach position (ANY Cartesian motion).
  12b. Move robot arm to vise grasp approach position (ANY Cartesian motion).
  12c. Compliantly align with part via move_to_contact (+Z compliant touchdown).
  12d. Close gripper to grasp part (Mock).
  12e. Retract arm to vise approach position (LINEAR Cartesian motion).
  12f. Retract arm to machine entry approach position (LINEAR Cartesian motion).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: CNC machine controller adapter.
      workpiece: Workpiece instance being unloaded.
      parent_object: Name of parent object for target frames (default: 'root').
      machine_approach_frame_name: Name of machine entry approach frame (default: 'machine_approach').
      preplace_vise_frame_name: Name of pre-place vise approach frame (default: 'pre_place_vise').
      place_vise_frame_name: Name of place vise frame (default: 'place_vise').

  Returns:
      Behavior tree sequence node executing machine unloading.
  """
  tasks: list[bt.Node] = [
      machine.build_open_door_task(name="Step 10: Open CNC Door"),
      machine.build_open_vise_task(name="Step 11: Open CNC Vise"),
      create_move_to_frame_task(
          robot=robot,
          frame_name=machine_approach_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          task_name=f"Step 12a: Approach Machine Entry ({parent_object}/{machine_approach_frame_name})",
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=preplace_vise_frame_name,
          parent_object=parent_object,
          motion_type="ANY",
          task_name=f"Step 12b: Approach Machined Part ({parent_object}/{preplace_vise_frame_name})",
      ),
      create_compliant_touchdown_task(
          robot=robot,
          direction=(0.0, 0.0, 1.0),
          contact_force_newtons=5.0,
          task_name="Step 12c: Compliant Touchdown to Machined Part (+Z Tool)",
      ),
      gripper.build_close_task(name="Step 12d: Grasp Machined Part"),
      create_move_to_frame_task(
          robot=robot,
          frame_name=preplace_vise_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          task_name=f"Step 12e: Retract Machined Part from Vise ({parent_object}/{preplace_vise_frame_name})",
      ),
      create_move_to_frame_task(
          robot=robot,
          frame_name=machine_approach_frame_name,
          parent_object=parent_object,
          motion_type="LINEAR",
          task_name=f"Step 12f: Retract Machined Part from Machine ({parent_object}/{machine_approach_frame_name})",
      ),
  ]

  return bt.Sequence(name="4. Unload Machine Subtree", children=tasks)
