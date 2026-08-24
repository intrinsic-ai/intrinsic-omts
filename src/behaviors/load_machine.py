"""CNC machine loading and fixturing subtree."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import create_compliant_touchdown_task, create_move_to_named_pose_task
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_load_machine_subtree(
    robot: RobotInterface,
    gripper: GripperInterface,
    machine: CncMachineInterface,
    workpiece: Workpiece,
    cnc_approach_pose_name: str = "view_pose2",
) -> bt.Node:
  """Builds the Behavior Tree subtree for loading raw stock into the CNC machine.

  Steps (3-6 in OMTS pipeline):
  3. Open CNC door (DIO).
  4. Open CNC vise (DIO).
  5. Move arm into CNC vise approach position and seat part using move_to_contact.
  6. Close CNC vise (DIO).
  7. Open gripper and retract arm outside CNC enclosure.

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: CNC machine controller adapter.
      workpiece: Workpiece instance being loaded.
      cnc_approach_pose_name: Joint configuration name for CNC vise entry approach.

  Returns:
      Behavior tree sequence node executing machine loading.
  """
  tasks: list[bt.Node] = [
      machine.build_open_door_task(name="Step 03: Open CNC Door"),
      machine.build_open_vise_task(name="Step 04: Open CNC Vise"),
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=cnc_approach_pose_name,
          task_name=f"Step 05a: Approach CNC Vise ({cnc_approach_pose_name})",
      ),
      create_compliant_touchdown_task(
          robot=robot,
          direction=(0.0, 0.0, -1.0),
          contact_force_newtons=8.0,
          task_name="Step 05b: Compliant Seat Part into Vise",
      ),
      machine.build_close_vise_task(name="Step 06: Clamp CNC Vise"),
      gripper.build_open_task(name="Step 07a: Release Part in Vise"),
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=cnc_approach_pose_name,
          task_name=f"Step 07b: Retract Arm to {cnc_approach_pose_name}",
      ),
  ]

  return bt.Sequence(name="2. Load Machine Subtree", children=tasks)
