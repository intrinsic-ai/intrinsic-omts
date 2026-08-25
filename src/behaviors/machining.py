"""CNC machining process cycle handshake subtree."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.motions import create_move_to_named_pose_task
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_machining_handshake_subtree(
    robot: RobotInterface,
    machine: CncMachineInterface,
    wait_pose_name: str = "home",
    machining_timeout_seconds: float = 30.0,
) -> bt.Node:
  """Builds the Behavior Tree subtree for executing the CNC machining cycle.

  Steps (7-9 in OMTS pipeline):
  7. Move robot arm to safe standby / wait pose outside enclosure.
  8. Close CNC door (DIO).
  9. Trigger CNC cycle start (DIO) and wait for cycle completion (DIO / signal).

  Args:
      robot: Robot controller adapter.
      machine: CNC machine controller adapter.
      wait_pose_name: Joint configuration name for safe standby.
      machining_timeout_seconds: Timeout waiting for cycle complete signal.

  Returns:
      Behavior tree sequence node executing machining cycle handshake.
  """
  tasks: list[bt.Node] = [
      create_move_to_named_pose_task(
          robot=robot,
          pose_name=wait_pose_name,
          task_name=f"Step 08: Move to Safe Standby ({wait_pose_name})",
      ),
      machine.build_close_door_task(name="Step 09a: Close CNC Door"),
      machine.build_trigger_cycle_task(name="Step 09b: Trigger CNC Cycle Start"),
      machine.build_wait_cycle_complete_task(
          timeout_seconds=machining_timeout_seconds,
          name="Step 09c: Wait for CNC Cycle Complete",
      ),
  ]

  return bt.Sequence(name="3. Machining Handshake Subtree", children=tasks)
