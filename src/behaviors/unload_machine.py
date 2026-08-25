"""CNC machine unloading and extraction subtree."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import create_compliant_touchdown_task, create_move_to_named_pose_task
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_unload_machine_subtree(
    robot: RobotInterface,
    gripper: GripperInterface,
    machine: CncMachineInterface,
    workpiece: Workpiece,
    cnc_approach_pose_name: str = "view_pose2",
) -> bt.Node:
  """Builds the Behavior Tree subtree for unloading a finished part from the CNC.

  Steps (10-12 in OMTS pipeline):
  10. Open CNC door (DIO).
  11. Open CNC vise (DIO).
  12. Move robot arm into vise grasp approach position.
  13. Compliantly align with part via move_to_contact.
  14. Close gripper to grasp part.
  15. Retract arm with machined part outside CNC enclosure.

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: CNC machine controller adapter.
      workpiece: Workpiece instance being unloaded.
      cnc_approach_pose_name: Joint configuration name for CNC vise entry approach.

  Returns:
      Behavior tree sequence node executing machine unloading.
  """
  tasks: list[bt.Node] = [
      machine.build_open_door_task(name="Step 10: Open CNC Door"),
      machine.build_open_vise_task(name="Step 11: Open CNC Vise"),
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=cnc_approach_pose_name,
          task_name=f"Step 12a: Approach Machined Part ({cnc_approach_pose_name})",
      ),
      create_compliant_touchdown_task(
          robot=robot,
          direction=(0.0, 0.0, -1.0),
          contact_force_newtons=5.0,
          task_name="Step 12b: Compliant Touchdown to Machined Part",
      ),
      gripper.build_close_task(name="Step 12c: Grasp Machined Part"),
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=cnc_approach_pose_name,
          task_name=f"Step 12d: Retract Machined Part from CNC",
      ),
  ]

  return bt.Sequence(name="4. Unload Machine Subtree", children=tasks)
