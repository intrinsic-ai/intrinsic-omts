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

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.load_machine import build_load_machine_subtree
from src.behaviors.machining import build_machining_handshake_subtree
from src.behaviors.pick import build_pick_from_infeed_subtree
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.config import AppConfig
from src.core.infeed import InfeedStrategy
from src.core.types import Phase
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def build_machine_tending_behavior_tree(
    robot: RobotInterface,
    gripper: GripperInterface,
    machine: CncMachineInterface | None,
    vision: VisionInterface,
    infeed_strategy: InfeedStrategy,
    config: AppConfig,
    num_cycles_override: int | None = None,
    start_phase: Phase = Phase.PICK,
    tree_name: str = "OMTS Machine Tending Master Cycle",
) -> bt.BehaviorTree:
  """Assembles the complete machine tending sequence into an SBL Behavior Tree.

  Implements Pattern B (Pure Behavior Tree orchestration):
  1. CNC machine prep (open door & vise), infeed perception, and part pick with
     belief world object attachment.
  2. Blended transit & loading into CNC vise with belief world detachment.
  3. Standby & CNC machining cycle handshake.
  4. CNC vise unloading & extraction with belief world attachment.
  5. Blended transit & return to infeed table with belief world detachment.
  6. Wraps the cycle in `bt.Loop` (`num_cycles > 1` or `num_cycles <= 0`)
     for native executive-controlled multi-cycle or continuous execution.

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: CNC machine controller adapter.
      vision: 3D camera adapter.
      infeed_strategy: Infeed acquisition strategy (Perception vs. Grid).
      config: Application configuration dataclass.
      num_cycles_override: Optional override for number of cycles to execute (1
        = single, >1 = finite Loop, <=0 = infinite Loop).
      start_phase: Starting phase of the tending cycle for mid-cycle recovery.
      tree_name: Descriptive name for the Behavior Tree.

  Returns:
      Executable SBL BehaviorTree instance.
  """
  if isinstance(start_phase, str):
    start_phase = Phase(start_phase)

  num_cycles = (
      num_cycles_override
      if num_cycles_override is not None
      else config.cycle.num_cycles
  )
  loop_counter_key = "cycle_index" if num_cycles != 1 else None

  pick_subtree = build_pick_from_infeed_subtree(
      robot=robot,
      gripper=gripper,
      vision=vision,
      infeed_strategy=infeed_strategy,
      config=config,
      machine=machine,
      loop_counter_key=loop_counter_key,
  )

  load_subtree = build_load_machine_subtree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      config=config,
      include_entry_guard=(start_phase == Phase.LOAD),
  )

  machining_subtree = build_machining_handshake_subtree(
      robot=robot,
      machine=machine,
      config=config,
      include_entry_guard=(start_phase == Phase.MACHINING),
  )

  unload_subtree = build_unload_machine_subtree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      config=config,
      include_entry_guard=(start_phase == Phase.UNLOAD),
  )

  return_subtree = build_return_to_infeed_subtree(
      robot=robot,
      gripper=gripper,
      config=config,
  )

  phase_subtrees = {
      Phase.PICK: pick_subtree,
      Phase.LOAD: load_subtree,
      Phase.MACHINING: machining_subtree,
      Phase.UNLOAD: unload_subtree,
      Phase.RETURN: return_subtree,
  }

  cycle_sequence = bt.Sequence(
      name="Single Machine Tending Cycle",
      children=[phase_subtrees[p] for p in start_phase.remaining],
  )

  root_node: bt.Node
  if num_cycles > 1:
    root_node = bt.Loop(
        max_times=num_cycles,
        do_child=cycle_sequence,
        loop_counter_key=loop_counter_key,
        name=f"Machine Tending Loop ({num_cycles} cycles)",
    )
  elif num_cycles <= 0:
    root_node = bt.Loop(
        max_times=0,
        do_child=cycle_sequence,
        loop_counter_key=loop_counter_key,
        name="Continuous Machine Tending Loop",
    )
  else:
    root_node = cycle_sequence

  return bt.BehaviorTree(name=tree_name, root=root_node)
