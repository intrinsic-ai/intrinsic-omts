"""CNC machining process cycle handshake subtree."""

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import create_move_to_frame_task
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface


def build_machining_handshake_subtree(
  robot: RobotInterface,
  machine: CncMachineInterface,
  parent_object: str = "root",
  standby_frame_name: str = "machine_approach",
  machining_timeout_seconds: float = 30.0,
) -> bt.Node:
  """Builds the Behavior Tree subtree for executing the CNC machining cycle.

  Steps (7-9 in OMTS pipeline):
  8. Move robot arm to safe standby position outside enclosure (ANY Cartesian motion).
  9a. Close CNC door (DIO / Mock).
  9b. Trigger CNC cycle start (DIO / Mock).
  9c. Wait for CNC cycle completion (DIO / Mock).

  Args:
      robot: Robot controller adapter.
      machine: CNC machine controller adapter.
      parent_object: Name of parent object for target frames (default: 'root').
      standby_frame_name: Target frame name for safe standby (default: 'machine_approach').
      machining_timeout_seconds: Timeout waiting for cycle complete signal.

  Returns:
      Behavior tree sequence node executing machining cycle handshake.
  """
  tasks: list[bt.Node] = [
    create_move_to_frame_task(
      robot=robot,
      frame_name=standby_frame_name,
      parent_object=parent_object,
      motion_type="ANY",
      task_name=f"Step 08: Move to Safe Standby ({parent_object}/{standby_frame_name})",
    ),
    machine.build_close_door_task(name="Step 09a: Close CNC Door"),
    machine.build_trigger_cycle_task(name="Step 09b: Trigger CNC Cycle Start"),
    machine.build_wait_cycle_complete_task(
      timeout_seconds=machining_timeout_seconds,
      name="Step 09c: Wait for CNC Cycle Complete",
    ),
  ]

  return bt.Sequence(name="3. Machining Handshake Subtree", children=tasks)
