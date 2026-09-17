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

import dataclasses
from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.load_machine import build_load_machine_subtree
from src.behaviors.machining import build_machining_handshake_subtree
from src.behaviors.motions import DEFAULT_TOUCHDOWN, Touchdown
from src.behaviors.pick import build_pick_from_infeed_subtree
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.infeed import InfeedStrategy
from src.core.types import DEFAULT_FRAMES, Frames, Phase
from src.core.workcell import WorkcellState
from src.core.workpiece import Workpiece
from src.core.world import WorldInterface
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import DEFAULT_MOTION, MotionConfig, RobotInterface
from src.hardware.vision import VisionInterface


def build_machine_tending_behavior_tree(
  robot: RobotInterface,
  gripper: GripperInterface,
  machine: CncMachineInterface,
  vision: VisionInterface,
  infeed_strategy: InfeedStrategy,
  workpiece: Workpiece,
  *,
  frames: Frames = DEFAULT_FRAMES,
  touchdown: Touchdown = DEFAULT_TOUCHDOWN,
  motion: MotionConfig = DEFAULT_MOTION,
  state: WorkcellState | None = None,
  machining_timeout_seconds: float = 30.0,
  solution: Any | None = None,
  world: WorldInterface | None = None,
  tree_name: str = "OMTS Machine Tending Master Cycle",
  enable_object_reparenting: bool = False,
  perception_max_retries: int = 3,
  perception_retry_delay_sec: float = 1.0,
  start_phase: Phase | str | None = None,
  num_cycles: int = 1,
) -> bt.BehaviorTree:
  """Assembles complete machine tending sequence into an SBL Behavior Tree."""
  grasp_td = dataclasses.replace(
    touchdown, retract_after_m=motion.grasp_offset_z
  )
  release_td = dataclasses.replace(touchdown, retract_after_m=0.0)

  raw_start_phase = (
    start_phase
    if start_phase is not None
    else (state.phase if state is not None else Phase.PICK)
  )
  phase = Phase(raw_start_phase)

  pick_subtree = build_pick_from_infeed_subtree(
    robot=robot,
    gripper=gripper,
    vision=vision,
    infeed_strategy=infeed_strategy,
    workpiece=workpiece,
    machine=machine,
    parent_object=frames.root,
    view_frame_name=frames.view,
    pregrasp_frame_name=frames.infeed_pre_grasp,
    grasp_frame_name=frames.infeed_grasp,
    min_safe_z=motion.min_safe_z,
    touchdown=grasp_td,
    approach_offset_z=motion.approach_height_m,
    enable_object_reparenting=enable_object_reparenting,
    perception_max_retries=perception_max_retries,
    perception_retry_delay_sec=perception_retry_delay_sec,
    solution=solution,
    world=world,
  )

  load_subtree = build_load_machine_subtree(
    robot=robot,
    gripper=gripper,
    machine=machine,
    workpiece=workpiece,
    parent_object=frames.root,
    entry_via_frame_name=frames.transit,
    machine_approach_frame_name=frames.machine_approach,
    vise_approach_frame_name=frames.vise_pre_place,
    vise_place_frame_name=frames.vise_place,
    touchdown=release_td,
    solution=solution,
    world=world,
    enable_object_reparenting=enable_object_reparenting,
  )

  machining_subtree = build_machining_handshake_subtree(
    robot=robot,
    machine=machine,
    parent_object=frames.root,
    standby_frame_name=frames.machine_approach,
    machining_timeout_seconds=machining_timeout_seconds,
    solution=solution,
  )

  unload_subtree = build_unload_machine_subtree(
    robot=robot,
    gripper=gripper,
    machine=machine,
    workpiece=workpiece,
    parent_object=frames.root,
    machine_approach_frame_name=frames.machine_approach,
    vise_approach_frame_name=frames.vise_pre_place,
    vise_place_frame_name=frames.vise_place,
    touchdown=grasp_td,
    solution=solution,
    world=world,
    enable_object_reparenting=enable_object_reparenting,
  )

  return_subtree = build_return_to_infeed_subtree(
    robot=robot,
    gripper=gripper,
    workpiece=workpiece,
    parent_object=frames.root,
    transit_frame_name=frames.transit,
    preplace_frame_name=frames.infeed_pre_grasp,
    place_frame_name=frames.infeed_grasp,
    view_frame_name=frames.view,
    touchdown=release_td,
    enable_object_reparenting=enable_object_reparenting,
    solution=solution,
    world=world,
  )

  subtrees = {
    Phase.PICK: pick_subtree,
    Phase.LOAD: load_subtree,
    Phase.MACHINING: machining_subtree,
    Phase.UNLOAD: unload_subtree,
    Phase.RETURN: return_subtree,
  }

  root_name = "OMTS Master Machine Tending Pipeline"
  if phase is not Phase.PICK:
    root_name += f" (from {phase})"
  root_sequence = bt.Sequence(
    name=root_name,
    children=[subtrees[p] for p in phase.remaining],
  )

  if num_cycles == 1:
    root_node: bt.Node = root_sequence
  elif phase is Phase.PICK:
    root_node = bt.Loop(
      max_times=max(0, num_cycles),
      do_child=root_sequence,
      name=f"{root_name} Loop ({num_cycles} cycles)",
    )
  else:
    full_cycle_seq = bt.Sequence(
      name="OMTS Master Machine Tending Pipeline",
      children=[subtrees[p] for p in Phase.PICK.remaining],
    )
    remaining_cycles = 0 if num_cycles <= 0 else num_cycles - 1
    root_node = bt.Sequence(
      name=f"{root_name} + {remaining_cycles} Cycles",
      children=[
        root_sequence,
        bt.Loop(
          max_times=remaining_cycles,
          do_child=full_cycle_seq,
          name=f"Subsequent Cycles Loop ({remaining_cycles} cycles)",
        ),
      ],
    )

  return bt.BehaviorTree(name=tree_name, root=root_node)
