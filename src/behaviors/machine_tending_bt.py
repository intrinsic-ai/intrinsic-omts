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
from src.behaviors.motions import Touchdown
from src.behaviors.pick import build_pick_from_infeed_subtree
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.infeed import InfeedStrategy
from src.core.types import Frames, Phase
from src.core.workcell import WorkcellState
from src.core.workpiece import Workpiece
from src.core.world import World
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
  *,
  frames: Frames | None = None,
  touchdown: Touchdown | None = None,
  pick_touchdown: Touchdown | None = None,
  load_touchdown: Touchdown | None = None,
  unload_touchdown: Touchdown | None = None,
  return_touchdown: Touchdown | None = None,
  state: WorkcellState | None = None,
  world: World | None = None,
  grasp_offset_z: float = 0.005,
  min_safe_z: float | None = 0.95,
  approach_offset_z: float = 0.08,
  return_to_view_frame: bool = False,
  close_gripper_before_perception: bool = False,
  close_cnc_door_on_return: bool = False,
  machining_timeout_seconds: float = 30.0,
  solution: Any | None = None,
  tree_name: str = "OMTS Machine Tending Master Cycle",
  enable_object_reparenting: bool = False,
  perception_max_retries: int = 3,
  perception_retry_delay_sec: float = 1.0,
  start_phase: Phase | str | None = None,
  **kwargs: Any,
) -> bt.BehaviorTree:
  """Assembles complete machine tending sequence into an SBL Behavior Tree."""
  del world

  approach_height_m = kwargs.get("approach_height_m")
  actual_approach_offset = (
    approach_height_m if approach_height_m is not None else approach_offset_z
  )
  move_to_view_first = kwargs.get("move_to_view_first", True)

  if frames is None:
    frames = Frames()
  if "parent_object" in kwargs:
    frames = dataclasses.replace(frames, root=kwargs["parent_object"])
  if "view_frame_name" in kwargs:
    frames = dataclasses.replace(frames, view=kwargs["view_frame_name"])
  if "pregrasp_frame_name" in kwargs:
    frames = dataclasses.replace(
      frames, infeed_pre_grasp=kwargs["pregrasp_frame_name"]
    )
  if "grasp_frame_name" in kwargs:
    frames = dataclasses.replace(
      frames, infeed_grasp=kwargs["grasp_frame_name"]
    )
  if "transit_frame_name" in kwargs:
    frames = dataclasses.replace(frames, transit=kwargs["transit_frame_name"])
  if "machine_approach_frame_name" in kwargs:
    frames = dataclasses.replace(
      frames, machine_approach=kwargs["machine_approach_frame_name"]
    )
  if "vise_approach_frame_name" in kwargs:
    frames = dataclasses.replace(
      frames, vise_pre_place=kwargs["vise_approach_frame_name"]
    )
  if "vise_place_frame_name" in kwargs:
    frames = dataclasses.replace(
      frames, vise_place=kwargs["vise_place_frame_name"]
    )
  if "preplace_frame_name" in kwargs:
    frames = dataclasses.replace(
      frames, infeed_pre_grasp=kwargs["preplace_frame_name"]
    )
  if "place_frame_name" in kwargs:
    frames = dataclasses.replace(
      frames, infeed_grasp=kwargs["place_frame_name"]
    )

  base_td = touchdown or Touchdown()
  if "contact_timeout_seconds" in kwargs:
    base_td = dataclasses.replace(
      base_td, timeout_s=kwargs["contact_timeout_seconds"]
    )

  if pick_touchdown is None:
    pick_td = dataclasses.replace(base_td, retract_after_m=grasp_offset_z)
    if "pick_contact_force_n" in kwargs:
      pick_td = dataclasses.replace(
        pick_td, force_n=kwargs["pick_contact_force_n"]
      )
    if "pick_standoff_distance_m" in kwargs:
      pick_td = dataclasses.replace(
        pick_td, standoff_m=kwargs["pick_standoff_distance_m"]
      )
  else:
    pick_td = pick_touchdown

  if load_touchdown is None:
    load_td = dataclasses.replace(base_td, retract_after_m=0.0)
    if "load_contact_force_n" in kwargs:
      load_td = dataclasses.replace(
        load_td, force_n=kwargs["load_contact_force_n"]
      )
    if "load_standoff_distance_m" in kwargs:
      load_td = dataclasses.replace(
        load_td, standoff_m=kwargs["load_standoff_distance_m"]
      )
  else:
    load_td = load_touchdown

  if unload_touchdown is None:
    unload_td = dataclasses.replace(base_td, retract_after_m=grasp_offset_z)
    if "unload_contact_force_n" in kwargs:
      unload_td = dataclasses.replace(
        unload_td, force_n=kwargs["unload_contact_force_n"]
      )
    if "unload_standoff_distance_m" in kwargs:
      unload_td = dataclasses.replace(
        unload_td, standoff_m=kwargs["unload_standoff_distance_m"]
      )
  else:
    unload_td = unload_touchdown

  if return_touchdown is None:
    return_td = dataclasses.replace(base_td, retract_after_m=0.0)
    if "return_contact_force_n" in kwargs:
      return_td = dataclasses.replace(
        return_td, force_n=kwargs["return_contact_force_n"]
      )
    if "return_standoff_distance_m" in kwargs:
      return_td = dataclasses.replace(
        return_td, standoff_m=kwargs["return_standoff_distance_m"]
      )
  else:
    return_td = return_touchdown

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
    min_safe_z=min_safe_z,
    grasp_offset_z=grasp_offset_z,
    touchdown=pick_td,
    close_gripper_before_perception=close_gripper_before_perception,
    approach_offset_z=actual_approach_offset,
    move_to_view_first=move_to_view_first,
    enable_object_reparenting=enable_object_reparenting,
    clear_motion_planner_cache=True,
    perception_max_retries=perception_max_retries,
    perception_retry_delay_sec=perception_retry_delay_sec,
    solution=solution,
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
    touchdown=load_td,
    solution=solution,
    clear_motion_planner_cache=True,
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
    touchdown=unload_td,
    grasp_offset_z=grasp_offset_z,
    solution=solution,
    clear_motion_planner_cache=True,
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
    machine=machine if close_cnc_door_on_return else None,
    touchdown=return_td,
    return_to_view_frame=return_to_view_frame,
    view_frame_name=frames.view,
    enable_object_reparenting=enable_object_reparenting,
    clear_motion_planner_cache=True,
    solution=solution,
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

  return bt.BehaviorTree(name=tree_name, root=root_sequence)
