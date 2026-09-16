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

import dataclasses
import os
import pathlib
import time
from collections.abc import Sequence
from typing import Any

import yaml
from absl import app, flags, logging
from intrinsic.solutions import deployments

from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.behaviors.motions import Touchdown
from src.core.infeed import InfeedMode, PerceptionInfeedStrategy
from src.core.types import Frames, Phase, SimulationMode
from src.core.workcell import WorkcellState
from src.core.world import MockSolution, resolve_world
from src.hardware.gripper import GripperConfig, GripperInterface
from src.hardware.machine import (
  CncMachineInterface,
  MachineConfig,
  run_initial_machine_prep,
)
from src.hardware.robot import MotionConfig, RobotInterface
from src.hardware.vision import PerceptionConfig, VisionInterface
from src.utils.execution_utils import to_executive_simulation_mode
from src.utils.tree_diagnostics import log_tree_failure_diagnostics


@dataclasses.dataclass(frozen=True)
class AppConfig:
  """Complete tending application configuration assembled from YAML and flags."""

  solution_address: str = "localhost:17080"
  mock_hardware: bool = False
  simulation_mode: SimulationMode | None = None
  export_dot: str | None = None
  infeed_mode: InfeedMode = InfeedMode.PERCEPTION
  inter_cycle_max_retries: int = 3
  enable_object_reparenting: bool = True
  reset_world_between_cycles: bool = False
  frames: Frames = dataclasses.field(default_factory=Frames)
  state: WorkcellState = dataclasses.field(default_factory=WorkcellState.create)
  perception: PerceptionConfig = dataclasses.field(
    default_factory=PerceptionConfig
  )
  motion: MotionConfig = dataclasses.field(default_factory=MotionConfig)
  gripper: GripperConfig = dataclasses.field(default_factory=GripperConfig)
  machine: MachineConfig = dataclasses.field(default_factory=MachineConfig)
  touchdown: Touchdown = dataclasses.field(default_factory=Touchdown)

  @property
  def machine_type(self) -> str:
    return self.machine.machine_type

  @property
  def machining_timeout_seconds(self) -> float:
    return self.machine.machining_timeout_seconds

  @property
  def initial_close_door_and_vise(self) -> bool:
    return self.machine.initial_close_door_and_vise

  @classmethod
  def from_yaml(cls, path: str | os.PathLike[str]) -> "AppConfig":
    """Loads an AppConfig instance from a YAML configuration file."""
    resolved = pathlib.Path(path)
    if not resolved.exists():
      repo_root = pathlib.Path(__file__).resolve().parent.parent
      candidate = repo_root / path
      if candidate.exists():
        resolved = candidate

    with open(resolved, encoding="utf-8") as f:
      data = yaml.safe_load(f) or {}

    perception_data = dict(data.get("perception", {}))
    if "sensor_ids" in perception_data:
      perception_data["sensor_ids"] = tuple(
        int(x) for x in perception_data["sensor_ids"]
      )

    machine_data = dict(data.get("machine", {}))
    for joint_key in (
      "door_open_joints",
      "door_closed_joints",
      "vise_open_joints",
      "vise_closed_joints",
    ):
      if joint_key in machine_data:
        machine_data[joint_key] = tuple(
          float(x) for x in machine_data[joint_key]
        )

    return cls(
      solution_address=data.get("solution_address", cls.solution_address),
      infeed_mode=InfeedMode(data.get("infeed_mode", InfeedMode.PERCEPTION)),
      inter_cycle_max_retries=data.get(
        "inter_cycle_max_retries", cls.inter_cycle_max_retries
      ),
      enable_object_reparenting=data.get(
        "enable_object_reparenting", cls.enable_object_reparenting
      ),
      reset_world_between_cycles=data.get(
        "reset_world_between_cycles", cls.reset_world_between_cycles
      ),
      frames=Frames(**data.get("frames", {})),
      touchdown=Touchdown(**data.get("touchdown", {})),
      motion=MotionConfig(**data.get("motion", {})),
      gripper=GripperConfig(**data.get("gripper", {})),
      machine=MachineConfig(**machine_data),
      perception=PerceptionConfig(**perception_data),
    )


_CONFIG = flags.DEFINE_string(
  "config",
  "configs/omts/app_config.yaml",
  "Path to YAML cell configuration file.",
)
_ADDRESS = flags.DEFINE_string(
  "address",
  None,
  "gRPC address of the running SBL solution deployment (overrides YAML).",
)
_NUM_CYCLES = flags.DEFINE_integer(
  "num_cycles",
  1,
  "Number of machine tending cycles to run (-1 for infinite).",
)
_START_PHASE = flags.DEFINE_enum_class(
  "start_phase",
  Phase.PICK,
  Phase,
  "Cycle phase to start or resume from.",
)
_SIMULATION_MODE = flags.DEFINE_enum_class(
  "simulation_mode",
  None,
  SimulationMode,
  "Executive execution mode: 'reality', 'preview', or 'fast_preview'.",
)
_MOCK_HARDWARE = flags.DEFINE_bool(
  "mock_hardware",
  False,
  "Use offline mock hardware adapters instead of live SBL skill stubs.",
)
_EXPORT_DOT = flags.DEFINE_string(
  "export_dot",
  None,
  "Optional file path to export Graphviz DOT representation of the BT.",
)


def config_from_flags() -> AppConfig:
  """Builds an AppConfig instance from the active YAML config and flags."""
  base = AppConfig.from_yaml(_CONFIG.value)
  return dataclasses.replace(
    base,
    solution_address=_ADDRESS.value or base.solution_address,
    mock_hardware=_MOCK_HARDWARE.value,
    simulation_mode=_SIMULATION_MODE.value,
    export_dot=_EXPORT_DOT.value,
    state=WorkcellState.create(
      total_cycles=_NUM_CYCLES.value,
      phase=_START_PHASE.value,
    ),
  )


def run_machine_tending_cycle(
  config: AppConfig | None = None,
  *,
  solution: Any | None = None,
  robot: RobotInterface | None = None,
  gripper: GripperInterface | None = None,
  machine: CncMachineInterface | None = None,
  vision: VisionInterface | None = None,
  start_phase: Phase | str | None = None,
  num_cycles: int | None = None,
  initial_close_door_and_vise: bool | None = None,
  simulation_mode: SimulationMode | None = None,
  **kwargs: Any,
) -> int:
  """Executes one or more machine tending cycles driven by AppConfig."""
  if config is None:
    config = config_from_flags()

  if config.infeed_mode != InfeedMode.PERCEPTION:
    raise NotImplementedError("Grid infeed is not implemented for this demo.")

  raw_phase = (
    start_phase
    if start_phase is not None
    else kwargs.get("start_phase", config.state.phase)
  )
  phase = Phase(raw_phase)

  total_cycles = (
    num_cycles
    if num_cycles is not None
    else kwargs.get("num_cycles", config.state.total_cycles)
  )
  close_prep = (
    initial_close_door_and_vise
    if initial_close_door_and_vise is not None
    else kwargs.get(
      "initial_close_door_and_vise", config.initial_close_door_and_vise
    )
  )
  sim_mode = (
    simulation_mode
    if simulation_mode is not None
    else kwargs.get("simulation_mode", config.simulation_mode)
  )

  if (
    phase != config.state.phase
    or total_cycles != config.state.total_cycles
    or close_prep != config.initial_close_door_and_vise
    or sim_mode != config.simulation_mode
  ):
    config = dataclasses.replace(
      config,
      simulation_mode=sim_mode,
      state=WorkcellState.create(total_cycles=total_cycles, phase=phase),
      machine=dataclasses.replace(
        config.machine, initial_close_door_and_vise=close_prep
      ),
    )

  if solution is None:
    if config.mock_hardware:
      solution = MockSolution()
    else:
      logging.info(
        "Connecting to Intrinsic solution at %s...", config.solution_address
      )
      solution = deployments.connect(address=config.solution_address)

  if robot is None:
    robot = RobotInterface.from_config(
      solution=solution,
      config=config.motion,
      mock_hardware=config.mock_hardware,
    )

  if gripper is None:
    gripper = GripperInterface.from_config(
      solution=solution,
      config=config.gripper,
      mock_hardware=config.mock_hardware,
    )

  if machine is None:
    machine_cfg = dataclasses.replace(
      config.machine, machine_type=config.machine_type
    )
    machine = CncMachineInterface.from_config(
      solution=solution,
      config=machine_cfg,
      mock_hardware=config.mock_hardware,
    )

  if vision is None:
    vision = VisionInterface.from_config(
      solution=solution,
      config=config.perception,
      mock_hardware=config.mock_hardware,
    )

  if config.initial_close_door_and_vise and machine is not None:
    run_initial_machine_prep(
      solution=solution,
      machine=machine,
      robot=robot,
      view_frame=config.frames.view,
      close_door_and_vise=True,
    )

  workcell_state = config.state

  if not config.mock_hardware:
    is_ready = False
    for prep_attempt in range(1, config.inter_cycle_max_retries + 1):
      try:
        if hasattr(solution, "cancel_all_operations") and callable(
          solution.cancel_all_operations
        ):
          solution.cancel_all_operations()
        elif hasattr(solution, "executive") and hasattr(
          solution.executive, "cancel"
        ):
          solution.executive.cancel()
        robot.clear_faults()
        if config.reset_world_between_cycles:
          resolve_world(solution).reset(
            robot=robot,
            solution=solution,
            workpiece_name=config.perception.scene_object_id,
          )
        is_ready = True
        break
      except Exception as prep_err:  # pylint: disable=broad-exception-caught
        logging.warning(
          "Cell preparation failed on attempt %d/%d: %s",
          prep_attempt,
          config.inter_cycle_max_retries,
          prep_err,
        )
        time.sleep(1.0)

    if not is_ready:
      raise RuntimeError(
        "Cell could not be prepared after "
        f"{config.inter_cycle_max_retries} attempts."
      )

  infeed_strategy = PerceptionInfeedStrategy(
    camera_name=config.perception.camera_name,
    pose_estimator_id=config.perception.estimator_id,
    scene_object_id=config.perception.scene_object_id,
    sensor_ids=config.perception.sensor_ids,
    min_num_instances=config.perception.min_instances,
    view_frame_name=config.frames.view,
  )
  workpiece = infeed_strategy.get_target_part()
  workcell_state.start_new_cycle(workpiece)

  logging.info(
    "Constructing SBL Behavior Tree (total_cycles=%d, start_phase=%s)...",
    workcell_state.total_cycles,
    workcell_state.phase.value,
  )
  tree = build_machine_tending_behavior_tree(
    robot=robot,
    gripper=gripper,
    machine=machine,
    vision=vision,
    infeed_strategy=infeed_strategy,
    workpiece=workpiece,
    frames=config.frames,
    touchdown=config.touchdown,
    motion=config.motion,
    state=workcell_state,
    machining_timeout_seconds=config.machining_timeout_seconds,
    solution=solution,
    enable_object_reparenting=config.enable_object_reparenting,
    perception_max_retries=config.perception.max_retries,
    perception_retry_delay_sec=config.perception.retry_delay_sec,
    start_phase=workcell_state.phase,
    num_cycles=workcell_state.total_cycles,
  )

  if config.export_dot:
    dot_content = tree.dot if hasattr(tree, "dot") else str(tree)
    pathlib.Path(config.export_dot).write_text(dot_content, encoding="utf-8")
    logging.info("Exported Behavior Tree DOT graph to %s.", config.export_dot)

  executive_simulation_mode = to_executive_simulation_mode(
    config.simulation_mode
  )
  logging.info(
    "Executing OMTS Machine Tending Pipeline (simulation mode: %s)...",
    config.simulation_mode.value
    if config.simulation_mode
    else "executive default",
  )
  cycle_start_time = time.perf_counter()
  try:
    if hasattr(solution, "run") and callable(solution.run):
      solution.run(tree, simulation_mode=executive_simulation_mode)
    else:
      kwargs_run = (
        {"simulation_mode": executive_simulation_mode}
        if executive_simulation_mode is not None
        else {}
      )
      solution.executive.run(tree, **kwargs_run)
    total_duration = time.perf_counter() - cycle_start_time
    logging.info(
      "Pipeline execution completed successfully in %.2fs.",
      total_duration,
    )
  except KeyboardInterrupt:
    logging.info("Machine tending execution interrupted by operator.")
    return 0
  except Exception as e:
    logging.error("Pipeline execution failed: %s", e)
    log_tree_failure_diagnostics(solution)
    if hasattr(solution, "executive") and hasattr(
      solution.executive, "get_errors"
    ):
      try:
        logging.error("Executive errors: %s", solution.executive.get_errors())
      except Exception as diag_err:  # pylint: disable=broad-exception-caught
        logging.warning("Failed to retrieve diagnostics: %s", diag_err)
    raise

  return workcell_state.total_cycles if workcell_state.total_cycles > 0 else 1


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  config = config_from_flags()
  run_machine_tending_cycle(config)


if __name__ == "__main__":
  app.run(main)
