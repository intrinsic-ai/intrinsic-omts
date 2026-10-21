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
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments

from src.core.config import AppConfig, load_app_config
from src.core.types import GraspPlannerType, SimulationMode
from src.hardware.robot import Robot
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
_GRASP_PLANNER = flags.DEFINE_enum_class(
  "grasp_planner",
  None,
  GraspPlannerType,
  "Optional override for the grasp planner backend, taking precedence over "
  "the 'grasp' section of the cell config.",
)


def run_machine_tending_pipeline(
  solution_address: str,
  config: AppConfig,
  simulation_mode: SimulationMode | None = None,
  num_cycles_override: int | None = None,
  grasp_planner_override: GraspPlannerType | None = None,
) -> None:
  """Connects to the solution deployment and executes the machine tending BT.

  Args:
      solution_address: gRPC endpoint of the running SBL solution deployment.
      config: Validated cell configuration dataclass.
      simulation_mode: Optional executive execution mode override.
      num_cycles_override: Optional cycle count override (1 = single cycle,
        >1 = finite bt.Loop, <=0 = continuous bt.Loop).
      grasp_planner_override: Optional grasp planner backend override, taking
        precedence over `config.grasp`.
  """
  logging.info(
    "Connecting to Intrinsic solution at %s (cell: %s)...",
    solution_address,
    config.cell_name,
  )
  solution = deployments.connect(address=solution_address)

  robot = Robot(
    solution=solution,
    config=config.robot,
  )

  timeout_seconds = 120.0
  touchdown_task = robot.build_move_to_contact_task(
    direction=config.pick_touchdown.direction,
    contact_force_newtons=config.pick_touchdown.force_n,
    timeout_seconds=timeout_seconds,
    name="Move to Contact",
  )
  tree = bt.BehaviorTree(
    name="Move to Contact",
    root=touchdown_task,
  )

  exec_sim_mode = to_executive_simulation_mode(simulation_mode)
  logging.info(
    "Executing Move to Contact Behavior Tree (force=%.1f N, timeout=%.1f s)...",
    config.pick_touchdown.force_n,
    timeout_seconds,
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
    grasp_planner_override=_GRASP_PLANNER.value,
  )


if __name__ == "__main__":
  app.run(main)
