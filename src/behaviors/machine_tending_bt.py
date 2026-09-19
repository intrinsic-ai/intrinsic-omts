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
  tree_name: str = "OMTS Machine Tending Master Cycle",
) -> bt.BehaviorTree:
  """Assembles the complete machine tending sequence into an SBL Behavior Tree.

  Orchestrates the entire multi-step cycle within a single Behavior Tree:
  1. Infeed pick: optional CNC door/vise prep, 6D vision pose estimation,
     compliant touchdown, 3 cm linear retract, grasp, and world attachment.
  2. Machine load: optional blended transit, compliant vise seating, clamping,
     release, world detachment, and linear retract.
  3. Machining handshake: standby outside enclosure, door close, cycle start
     pulse, and cycle completion wait.
  4. Machine unload: door/vise open, compliant touchdown to machined part,
     3 cm linear retract, grasp, world attachment, 3 cm linear lift clear of
     vise jaws, and linear extraction.
  5. Infeed return: optional blended transit, compliant placement on table,
     release, world detachment, linear retract, and return to view frame.
  6. Wraps the cycle sequence in `bt.Loop` when `num_cycles > 1` (finite) or
     `num_cycles <= 0` (continuous).

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      machine: Optional CNC machine controller adapter (`None` for cells without
        a CNC enclosure/vise).
      vision: 3D camera perception adapter.
      infeed_strategy: Infeed acquisition strategy (Perception vs. Grid).
      config: Validated application configuration dataclass.
      num_cycles_override: Optional override for number of cycles to execute
        (1 = single sequence, >1 = finite Loop, <=0 = continuous Loop).
      tree_name: Descriptive name for the Behavior Tree.

  Returns:
      Executable SBL BehaviorTree instance.
  """
  pick_subtree = build_pick_from_infeed_subtree(
    robot=robot,
    gripper=gripper,
    vision=vision,
    infeed_strategy=infeed_strategy,
    config=config,
    machine=machine,
  )

  load_subtree = build_load_machine_subtree(
    robot=robot,
    gripper=gripper,
    machine=machine,
    config=config,
  )

  machining_subtree = build_machining_handshake_subtree(
    robot=robot,
    machine=machine,
    config=config,
  )

  unload_subtree = build_unload_machine_subtree(
    robot=robot,
    gripper=gripper,
    machine=machine,
    config=config,
  )

  return_subtree = build_return_to_infeed_subtree(
    robot=robot,
    gripper=gripper,
    config=config,
  )

  cycle_sequence = bt.Sequence(
    name="Single Machine Tending Cycle",
    children=[
      pick_subtree,
      load_subtree,
      machining_subtree,
      unload_subtree,
      return_subtree,
    ],
  )

  num_cycles = (
    num_cycles_override
    if num_cycles_override is not None
    else config.cycle.num_cycles
  )

  root_node: bt.Node
  if num_cycles > 1:
    root_node = bt.Loop(
      max_times=num_cycles,
      do_child=cycle_sequence,
      name=f"Machine Tending Loop ({num_cycles} cycles)",
    )
  elif num_cycles <= 0:
    root_node = bt.Loop(
      max_times=0,
      do_child=cycle_sequence,
      name="Continuous Machine Tending Loop",
    )
  else:
    root_node = cycle_sequence

  return bt.BehaviorTree(name=tree_name, root=root_node)
