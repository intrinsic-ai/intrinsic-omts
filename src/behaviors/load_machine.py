"""CNC machine loading and fixturing subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  create_compliant_touchdown_task,
  create_move_to_frame_task,
)
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_load_machine_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  machine: CncMachineInterface,
  workpiece: Workpiece,
  parent_object: str = "root",
  machine_approach_frame_name: str = "machine_approach",
  preplace_vise_frame_name: str = "pre_place_vise",
  place_vise_frame_name: str = "place_vise",
) -> bt.Node:
  """Builds the Behavior Tree subtree for loading raw stock into the CNC machine.

  Steps (3-7 in OMTS pipeline):
  3. Open CNC door (DIO / Mock).
  4. Open CNC vise (DIO / Mock).
  5a. Move arm to machine entry approach position (ANY Cartesian motion).
  5b. Move arm to CNC vise approach position (ANY Cartesian motion).
  5c. Seat part into vise using move_to_contact (+Z compliant touchdown).
  6. Clamp CNC vise (DIO / Mock).
  7a. Open gripper to release part in vise (Mock).
  7b. Retract arm to vise approach position (LINEAR Cartesian motion).
  7c. Retract arm to machine entry approach position (LINEAR Cartesian motion).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: CNC machine adapter.
      workpiece: Workpiece model instance.
      parent_object: Name of parent object for target frames (default: 'root').
      machine_approach_frame_name: Name of machine entry approach frame (default: 'machine_approach').
      preplace_vise_frame_name: Name of pre-place vise approach frame (default: 'pre_place_vise').
      place_vise_frame_name: Name of place vise frame (default: 'place_vise').

  Returns:
      Behavior tree sequence executing machine loading and fixturing.
  """
  tasks: list[bt.Node] = [
    machine.build_open_door_task(name="Step 03: Open CNC Door"),
    machine.build_open_vise_task(name="Step 04: Open CNC Vise"),
    create_move_to_frame_task(
      robot=robot,
      frame_name=machine_approach_frame_name,
      parent_object=parent_object,
      motion_type="ANY",
      task_name=f"Step 05a: Approach Machine Entry ({parent_object}/{machine_approach_frame_name})",
    ),
    create_move_to_frame_task(
      robot=robot,
      frame_name=preplace_vise_frame_name,
      parent_object=parent_object,
      motion_type="ANY",
      task_name=f"Step 05b: Approach CNC Vise ({parent_object}/{preplace_vise_frame_name})",
    ),
    create_compliant_touchdown_task(
      robot=robot,
      direction=(0.0, 0.0, 1.0),
      contact_force_newtons=8.0,
      task_name="Step 05c: Compliant Seat Part into Vise (+Z Tool)",
    ),
    machine.build_close_vise_task(name="Step 06: Clamp CNC Vise"),
    gripper.build_open_task(name="Step 07a: Release Part in Vise"),
    create_move_to_frame_task(
      robot=robot,
      frame_name=preplace_vise_frame_name,
      parent_object=parent_object,
      motion_type="LINEAR",
      task_name=f"Step 07b: Retract Arm to Vise Approach ({parent_object}/{preplace_vise_frame_name})",
    ),
    create_move_to_frame_task(
      robot=robot,
      frame_name=machine_approach_frame_name,
      parent_object=parent_object,
      motion_type="LINEAR",
      task_name=f"Step 07c: Retract Arm to Machine Entry ({parent_object}/{machine_approach_frame_name})",
    ),
  ]

  return bt.Sequence(name="2. Load Machine Subtree", children=tasks)
