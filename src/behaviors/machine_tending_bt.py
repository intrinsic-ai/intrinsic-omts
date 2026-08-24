"""Master Behavior Tree builder for the Open Machine Tending Solution."""

from typing import Optional
from intrinsic.solutions import behavior_tree as bt
from src.behaviors.load_machine import build_load_machine_subtree
from src.behaviors.machining import build_machining_handshake_subtree
from src.behaviors.pick import build_pick_from_infeed_subtree
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.infeed import InfeedStrategy
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def build_machine_tending_behavior_tree(
    robot: RobotInterface,
    gripper: GripperInterface,
    machine: CncMachineInterface,
    vision: VisionInterface,
    infeed_strategy: InfeedStrategy,
    workpiece: Workpiece,
    tree_name: str = "OMTS Machine Tending Master Cycle",
) -> bt.BehaviorTree:
  """Assembles the complete machine tending sequence into an SBL Behavior Tree.

  Sequence:
  1. Pick raw stock from infeed (perception or grid).
  2. Load part into CNC vise and clamp.
  3. Standby & execute CNC machining cycle handshake.
  4. Unclamp & extract finished part from CNC vise.
  5. Return finished part back to infeed and home robot.

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: CNC machine controller adapter.
      vision: 3D camera adapter.
      infeed_strategy: Infeed acquisition strategy (Perception vs. Grid).
      workpiece: Workpiece domain instance being processed.
      tree_name: Descriptive name for the Behavior Tree.

  Returns:
      Executable SBL BehaviorTree instance.
  """
  pick_subtree = build_pick_from_infeed_subtree(
      robot=robot,
      gripper=gripper,
      vision=vision,
      infeed_strategy=infeed_strategy,
      workpiece=workpiece,
  )

  load_subtree = build_load_machine_subtree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      workpiece=workpiece,
  )

  machining_subtree = build_machining_handshake_subtree(
      robot=robot,
      machine=machine,
  )

  unload_subtree = build_unload_machine_subtree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      workpiece=workpiece,
  )

  return_subtree = build_return_to_infeed_subtree(
      robot=robot,
      gripper=gripper,
      workpiece=workpiece,
  )

  root_sequence = bt.Sequence(
      name="OMTS Master Machine Tending Pipeline",
      children=[
          pick_subtree,
          load_subtree,
          machining_subtree,
          unload_subtree,
          return_subtree,
      ],
  )

  return bt.BehaviorTree(name=tree_name, root=root_sequence)
