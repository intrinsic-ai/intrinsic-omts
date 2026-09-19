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

"""Main application entrypoint for the Open Machine Tending Solution."""

from collections.abc import Sequence

from absl import app, flags, logging
from intrinsic.solutions import deployments

from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.core.config import AppConfig, load_app_config
from src.core.infeed import (
  InfeedMode,
  PerceptionInfeedStrategy,
)
from src.core.types import SimulationMode
from src.hardware.gripper import DioGripper, GripperInterface, RobotiqGripper
from src.hardware.machine import DioCncMachine
from src.hardware.robot import UrRobot
from src.hardware.vision import OrbbecVision
from src.utils.execution_utils import to_executive_simulation_mode

_CONFIG = flags.DEFINE_string(
  "config",
  "configs/omts/app_config.yaml",
  "Path to the cell YAML configuration file.",
)
_ADDRESS = flags.DEFINE_string(
  "address",
  "localhost:17080",
  "gRPC address of the running SBL solution deployment.",
)
_SIMULATION_MODE = flags.DEFINE_enum_class(
  "simulation_mode",
  None,
  SimulationMode,
  "Executive execution mode override: 'reality', 'preview', or 'fast_preview'.",
)
_NUM_CYCLES = flags.DEFINE_integer(
  "num_cycles",
  None,
  "Optional override for number of cycles (1 = single, >1 = finite Loop, <=0 = continuous Loop).",
)


def run_machine_tending_pipeline(
  solution_address: str,
  config: AppConfig,
  simulation_mode: SimulationMode | None = None,
  num_cycles_override: int | None = None,
) -> None:
  """Connects to the solution deployment and executes the machine tending BT.

  Args:
      solution_address: gRPC endpoint of the running SBL solution deployment.
      config: Validated cell configuration dataclass.
      simulation_mode: Optional executive execution mode override.
      num_cycles_override: Optional cycle count override (1 = single cycle,
        >1 = finite bt.Loop, <=0 = continuous bt.Loop).
  """
  logging.info(
    "Connecting to Intrinsic solution at %s (cell: %s)...",
    solution_address,
    config.cell_name,
  )
  solution = deployments.connect(address=solution_address)

  gripper: GripperInterface
  if config.gripper.type == "dio":
    gripper = DioGripper(
      solution=solution,
      config=config.gripper,
    )
  else:
    gripper = RobotiqGripper(
      solution=solution,
      config=config.gripper,
    )

  robot = UrRobot(
    solution=solution,
    config=config.robot,
  )
  machine = (
    DioCncMachine(
      solution=solution,
      config=config.machine,
    )
    if config.machine is not None
    else None
  )
  vision = OrbbecVision(
    solution=solution,
    config=config.vision,
  )

  infeed_mode = InfeedMode(config.vision.infeed_mode)
  if infeed_mode != InfeedMode.PERCEPTION:
    raise ValueError(
      f"Unsupported infeed_mode '{config.vision.infeed_mode}'. Only "
      f"'{InfeedMode.PERCEPTION.value}' is currently supported."
    )
  infeed_strategy = PerceptionInfeedStrategy(
    config=config.vision,
    view_frame_name=config.frames.view_frame,
  )

  tree = build_machine_tending_behavior_tree(
    robot=robot,
    gripper=gripper,
    machine=machine,
    vision=vision,
    infeed_strategy=infeed_strategy,
    config=config,
    num_cycles_override=num_cycles_override,
  )

  exec_sim_mode = to_executive_simulation_mode(simulation_mode)
  num_cycles = (
    num_cycles_override
    if num_cycles_override is not None
    else config.cycle.num_cycles
  )
  logging.info(
    "Executing machine tending Behavior Tree (cycles=%d)...", num_cycles
  )
  solution.executive.run(tree, simulation_mode=exec_sim_mode)


def main(argv: Sequence[str]) -> None:
  """Parses CLI flags and launches the OMTS machine tending pipeline.

  Args:
      argv: Non-flag command-line arguments passed by `absl.app`.

  Raises:
      app.UsageError: If unexpected positional arguments are provided.
  """
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")
  config = load_app_config(_CONFIG.value)
  run_machine_tending_pipeline(
    solution_address=_ADDRESS.value,
    config=config,
    simulation_mode=_SIMULATION_MODE.value,
    num_cycles_override=_NUM_CYCLES.value,
  )


if __name__ == "__main__":
  app.run(main)
