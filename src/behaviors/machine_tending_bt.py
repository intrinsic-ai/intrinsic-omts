# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    parent_object: str = "root",
    view_frame_name: str = "view",
    pregrasp_frame_name: str = "pre_grasp",
    grasp_frame_name: str = "grasp",
    machine_approach_frame_name: str = "machine_approach",
    preplace_vise_frame_name: str = "pre_place_vise",
    place_vise_frame_name: str = "place_vise",
    tree_name: str = "OMTS Machine Tending Master Cycle",
) -> bt.BehaviorTree:
  """Assembles the complete machine tending sequence into an SBL Behavior Tree.

  Sequence:
  1. Pick raw stock from infeed (view -> perception -> open gripper -> pre_grasp -> touch -> linear retract 3cm -> grasp -> linear retract).
  2. Load part into CNC vise (machine_approach -> pre_place_vise -> touch into vise -> clamp -> release -> retract).
  3. Standby & execute CNC machining cycle handshake (machine_approach standby -> door close -> cycle start -> complete).
  4. Unclamp & extract finished part from CNC vise (machine_approach -> pre_place_vise -> touch -> linear retract 3cm -> grasp -> retract).
  5. Return finished part back to infeed (pre_grasp -> touch table -> release -> retract -> view).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: CNC machine controller adapter.
      vision: 3D camera adapter.
      infeed_strategy: Infeed acquisition strategy (Perception vs. Grid).
      workpiece: Workpiece domain instance being processed.
      parent_object: Name of parent object for target frames (default: 'root').
      view_frame_name: Name of perception view frame (default: 'view').
      pregrasp_frame_name: Name of pre-grasp frame (default: 'pre_grasp').
      grasp_frame_name: Name of grasp frame (default: 'grasp').
      machine_approach_frame_name: Name of machine entry approach frame (default: 'machine_approach').
      preplace_vise_frame_name: Name of pre-place vise frame (default: 'pre_place_vise').
      place_vise_frame_name: Name of place vise frame (default: 'place_vise').
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
      parent_object=parent_object,
      view_frame_name=view_frame_name,
      pregrasp_frame_name=pregrasp_frame_name,
      grasp_frame_name=grasp_frame_name,
  )

  load_subtree = build_load_machine_subtree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      workpiece=workpiece,
      parent_object=parent_object,
      machine_approach_frame_name=machine_approach_frame_name,
      preplace_vise_frame_name=preplace_vise_frame_name,
      place_vise_frame_name=place_vise_frame_name,
  )

  machining_subtree = build_machining_handshake_subtree(
      robot=robot,
      machine=machine,
      parent_object=parent_object,
      standby_frame_name=machine_approach_frame_name,
  )

  unload_subtree = build_unload_machine_subtree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      workpiece=workpiece,
      parent_object=parent_object,
      machine_approach_frame_name=machine_approach_frame_name,
      preplace_vise_frame_name=preplace_vise_frame_name,
      place_vise_frame_name=place_vise_frame_name,
  )

  return_subtree = build_return_to_infeed_subtree(
      robot=robot,
      gripper=gripper,
      workpiece=workpiece,
      parent_object=parent_object,
      pregrasp_frame_name=pregrasp_frame_name,
      grasp_frame_name=grasp_frame_name,
      view_frame_name=view_frame_name,
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
