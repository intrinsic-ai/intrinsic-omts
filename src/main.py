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
import time
from collections.abc import Sequence
from typing import Any

from absl import app, flags, logging
from intrinsic.solutions import deployments

from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.behaviors.motions import Touchdown
from src.core.infeed import InfeedMode, PerceptionInfeedStrategy
from src.core.solution import MockSolution, Solution, SolutionInterface
from src.core.types import Frames, Phase, SimulationMode
from src.core.workcell import WorkcellState
from src.core.workpiece import Workpiece
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
  """Complete tending application configuration assembled once from flags."""

  solution_address: str = "localhost:17080"
  mock_hardware: bool = False
  simulation_mode: SimulationMode | None = None
  infeed_mode: InfeedMode = InfeedMode.PERCEPTION
  inter_cycle_max_retries: int = 3
  return_to_view_frame: bool = False
  enable_object_reparenting: bool = False
  reset_world_between_cycles: bool = False
  state: WorkcellState = dataclasses.field(default_factory=WorkcellState.create)
  frames: Frames = dataclasses.field(default_factory=Frames)
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


# --- Flags reading defaults from configuration dataclasses ---

_ADDRESS = flags.DEFINE_string(
  "address",
  AppConfig.solution_address,
  "gRPC address of the running SBL solution deployment.",
)
_MOCK_HARDWARE = flags.DEFINE_bool(
  "mock_hardware",
  AppConfig.mock_hardware,
  "Use offline mock hardware adapters instead of live SBL skill stubs.",
)
_INFEED_MODE = flags.DEFINE_enum_class(
  "infeed_mode",
  InfeedMode.PERCEPTION,
  InfeedMode,
  "Infeed strategy mode ('perception' or 'grid').",
)
_SIMULATION_MODE = flags.DEFINE_enum_class(
  "simulation_mode",
  None,
  SimulationMode,
  "Executive execution mode: 'reality' (full physics), 'preview' "
  "(simulated with visualization), or 'fast_preview' (simulated without "
  "visualization). If unset, the mode currently configured in the executive "
  "is kept.",
)

_CAMERA_NAME = flags.DEFINE_string(
  "camera_name",
  PerceptionConfig.camera_name,
  "Attribute name of 3D camera in solution.world.",
)
_PERCEPTION_SERVICE_NAME = flags.DEFINE_string(
  "perception_service_name",
  PerceptionConfig.service_name,
  "Name of the IOC pose estimator service in solution resources.",
)
_POSE_ESTIMATOR_ID = flags.DEFINE_string(
  "pose_estimator_id",
  PerceptionConfig.estimator_id,
  "Asset ID of the registered pose estimator model.",
)
_SCENE_OBJECT_ID = flags.DEFINE_string(
  "scene_object_id",
  PerceptionConfig.scene_object_id,
  "Asset ID of the target scene object to spawn in belief world.",
)
_SENSOR_IDS = flags.DEFINE_list(
  "sensor_ids",
  [str(x) for x in PerceptionConfig.sensor_ids],
  "Sensor IDs to capture from the RGB-D camera (1=RGB, 4=Depth).",
)
_MIN_NUM_INSTANCES = flags.DEFINE_integer(
  "min_num_instances",
  PerceptionConfig.min_instances,
  "Minimum number of detected workpiece instances required.",
)
_PERCEPTION_MAX_RETRIES = flags.DEFINE_integer(
  "perception_max_retries",
  PerceptionConfig.max_retries,
  "Maximum retry attempts for perception capture and pose estimation.",
)
_PERCEPTION_RETRY_DELAY_SEC = flags.DEFINE_float(
  "perception_retry_delay_sec",
  PerceptionConfig.retry_delay_sec,
  "Dwell delay in seconds between perception retry attempts.",
)
_CLOSE_GRIPPER_BEFORE_PERCEPTION = flags.DEFINE_bool(
  "close_gripper_before_perception",
  PerceptionConfig.close_gripper_before_perception,
  "Close gripper fingers at view before camera capture to clear FOV.",
)

_GRIPPER_TYPE = flags.DEFINE_enum(
  "gripper_type",
  GripperConfig.hardware_type,
  ["mock", "dio", "robotiq"],
  "Gripper backend type: 'mock', 'dio', or 'robotiq'.",
)
_GRIPPER_JOINT_NAME = flags.DEFINE_string(
  "gripper_joint_name",
  GripperConfig.joint_name,
  "Robotiq finger joint name.",
)
_GRIPPER_OPEN_POSITION = flags.DEFINE_float(
  "gripper_open_position",
  GripperConfig.open_position,
  "Robotiq finger position in meters for open state.",
)
_GRIPPER_CLOSE_POSITION = flags.DEFINE_float(
  "gripper_close_position",
  GripperConfig.close_position,
  "Robotiq finger position in meters for close state.",
)
_GRIPPER_ACTION_NAME = flags.DEFINE_string(
  "gripper_action_name",
  GripperConfig.action_name,
  "Optional action name for gripper_cmd_skill.",
)
_GRIPPER_DIO_OPEN_PIN = flags.DEFINE_integer(
  "gripper_dio_open_pin",
  GripperConfig.dio_open_pin,
  "Digital output pin index to open gripper (DIO).",
)
_GRIPPER_DIO_CLOSE_PIN = flags.DEFINE_integer(
  "gripper_dio_close_pin",
  GripperConfig.dio_close_pin,
  "Digital output pin index to close gripper (DIO).",
)

_MACHINE_TYPE = flags.DEFINE_enum(
  "machine_type",
  MachineConfig.machine_type,
  ["none", "dio", "mock"],
  "CNC machine adapter type: 'none', 'dio', or 'mock'.",
)
_INITIAL_CLOSE_DOOR_AND_VISE = flags.DEFINE_bool(
  "initial_close_door_and_vise",
  MachineConfig.initial_close_door_and_vise,
  "Whether to close door and vise during initial machine prep.",
)
_MACHINING_TIMEOUT_SECONDS = flags.DEFINE_float(
  "machining_timeout_seconds",
  MachineConfig.machining_timeout_seconds,
  "CNC machining cycle timeout in seconds.",
)

_ARM_PART_NAME = flags.DEFINE_string(
  "arm_part_name",
  MotionConfig.arm_part_name,
  "ICON part name for the robot arm in solution.world.",
)
_TOOL_OBJECT_NAME = flags.DEFINE_string(
  "tool_object_name",
  MotionConfig.tool_object_name,
  "Object name for the robot end-effector tool.",
)
_TOOL_FRAME_NAME = flags.DEFINE_string(
  "tool_frame_name",
  MotionConfig.tool_frame_name,
  "Frame name on tool_object_name to use as moving tool reference.",
)
_APPROACH_HEIGHT_M = flags.DEFINE_float(
  "approach_height_m",
  MotionConfig.approach_height_m,
  "Approach standoff distance in meters.",
)
_MIN_SAFE_Z = flags.DEFINE_float(
  "min_safe_z",
  MotionConfig.min_safe_z,
  "Minimum safe Z coordinate in meters for detected workpieces.",
)
_GRASP_OFFSET_Z = flags.DEFINE_float(
  "grasp_offset_z",
  MotionConfig.grasp_offset_z,
  "Z offset in meters from cuboid centroid for grasp.",
)
_DISABLE_COLLISION_CHECKING = flags.DEFINE_bool(
  "disable_collision_checking",
  MotionConfig.disable_collision_checking,
  "Disable collision checking during trajectory planning.",
)

_TOUCHDOWN_FORCE_N = flags.DEFINE_float(
  "touchdown_force_n",
  Touchdown.force_n,
  "Contact force threshold in Newtons.",
)
_TOUCHDOWN_STANDOFF_M = flags.DEFINE_float(
  "touchdown_standoff_m",
  Touchdown.standoff_m,
  "Standoff distance in meters before compliant touchdown.",
)
_CONTACT_TIMEOUT_SECONDS = flags.DEFINE_float(
  "contact_timeout_seconds",
  Touchdown.timeout_s,
  "Timeout in seconds for compliant touchdown motions.",
)

_PARENT_OBJECT = flags.DEFINE_string(
  "parent_object",
  Frames.root,
  "Parent object in world for target motion frames.",
)
_VIEW_FRAME = flags.DEFINE_string(
  "view_frame",
  Frames.view,
  "Perception camera viewing target frame name.",
)
_PREGRASP_FRAME = flags.DEFINE_string(
  "pregrasp_frame",
  Frames.infeed_pre_grasp,
  "Pre-grasp approach target frame name.",
)
_GRASP_FRAME = flags.DEFINE_string(
  "grasp_frame",
  Frames.infeed_grasp,
  "Grasp target frame name.",
)
_TRANSIT_FRAME = flags.DEFINE_string(
  "transit_frame",
  Frames.transit,
  "Presentation inspection target frame name.",
)
_MACHINE_APPROACH_FRAME = flags.DEFINE_string(
  "machine_approach_frame",
  Frames.machine_approach,
  "Machine entry approach target frame name.",
)
_VISE_PRE_PLACE_FRAME = flags.DEFINE_string(
  "vise_pre_place_frame",
  Frames.vise_pre_place,
  "Frame the arm approaches the vise through.",
)
_VISE_PLACE_FRAME = flags.DEFINE_string(
  "vise_place_frame",
  Frames.vise_place,
  "Vise place target frame name.",
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
_RETURN_TO_VIEW_FRAME = flags.DEFINE_bool(
  "return_to_view_frame",
  AppConfig.return_to_view_frame,
  "Whether to return robot arm to view frame after part return.",
)
_ENABLE_OBJECT_REPARENTING = flags.DEFINE_bool(
  "enable_object_reparenting",
  AppConfig.enable_object_reparenting,
  "Enable digital twin object reparenting in ObjectWorld.",
)
_INTER_CYCLE_MAX_RETRIES = flags.DEFINE_integer(
  "inter_cycle_max_retries",
  AppConfig.inter_cycle_max_retries,
  "Maximum retries for inter-cycle health check.",
)
_RESET_WORLD_BETWEEN_CYCLES = flags.DEFINE_bool(
  "reset_world_between_cycles",
  AppConfig.reset_world_between_cycles,
  "Whether to reset stale infeed stock between cycles.",
)


def config_from_flags() -> AppConfig:
  """Builds an AppConfig instance directly from active command-line flags."""
  sensor_ids = (
    tuple(int(s.strip()) for s in _SENSOR_IDS.value if s.strip())
    if _SENSOR_IDS.value
    else PerceptionConfig.sensor_ids
  )

  frames = Frames(
    root=_PARENT_OBJECT.value,
    view=_VIEW_FRAME.value,
    transit=_TRANSIT_FRAME.value,
    machine_approach=_MACHINE_APPROACH_FRAME.value,
    infeed_pre_grasp=_PREGRASP_FRAME.value,
    infeed_grasp=_GRASP_FRAME.value,
    vise_pre_place=_VISE_PRE_PLACE_FRAME.value,
    vise_place=_VISE_PLACE_FRAME.value,
  )
  touchdown = Touchdown(
    force_n=_TOUCHDOWN_FORCE_N.value,
    standoff_m=_TOUCHDOWN_STANDOFF_M.value,
    timeout_s=_CONTACT_TIMEOUT_SECONDS.value,
  )
  gripper = GripperConfig(
    hardware_type=_GRIPPER_TYPE.value,
    joint_name=_GRIPPER_JOINT_NAME.value,
    open_position=_GRIPPER_OPEN_POSITION.value,
    close_position=_GRIPPER_CLOSE_POSITION.value,
    action_name=_GRIPPER_ACTION_NAME.value,
    dio_open_pin=_GRIPPER_DIO_OPEN_PIN.value,
    dio_close_pin=_GRIPPER_DIO_CLOSE_PIN.value,
  )
  perception = PerceptionConfig(
    estimator_id=_POSE_ESTIMATOR_ID.value,
    camera_name=_CAMERA_NAME.value,
    service_name=_PERCEPTION_SERVICE_NAME.value,
    scene_object_id=_SCENE_OBJECT_ID.value,
    sensor_ids=sensor_ids,
    min_instances=_MIN_NUM_INSTANCES.value,
    max_retries=_PERCEPTION_MAX_RETRIES.value,
    retry_delay_sec=_PERCEPTION_RETRY_DELAY_SEC.value,
    close_gripper_before_perception=_CLOSE_GRIPPER_BEFORE_PERCEPTION.value,
  )
  motion = MotionConfig(
    approach_height_m=_APPROACH_HEIGHT_M.value,
    min_safe_z=_MIN_SAFE_Z.value,
    grasp_offset_z=_GRASP_OFFSET_Z.value,
    contact_timeout_seconds=_CONTACT_TIMEOUT_SECONDS.value,
    disable_collision_checking=_DISABLE_COLLISION_CHECKING.value,
    arm_part_name=_ARM_PART_NAME.value,
    tool_object_name=_TOOL_OBJECT_NAME.value,
    tool_frame_name=_TOOL_FRAME_NAME.value,
  )
  machine = MachineConfig(
    machine_type=_MACHINE_TYPE.value,
    machining_timeout_seconds=_MACHINING_TIMEOUT_SECONDS.value,
    initial_close_door_and_vise=_INITIAL_CLOSE_DOOR_AND_VISE.value,
  )
  state = WorkcellState.create(
    total_cycles=_NUM_CYCLES.value,
    phase=_START_PHASE.value,
  )
  return AppConfig(
    frames=frames,
    touchdown=touchdown,
    gripper=gripper,
    perception=perception,
    motion=motion,
    machine=machine,
    state=state,
    infeed_mode=_INFEED_MODE.value,
    mock_hardware=_MOCK_HARDWARE.value,
    simulation_mode=_SIMULATION_MODE.value,
    solution_address=_ADDRESS.value,
    inter_cycle_max_retries=_INTER_CYCLE_MAX_RETRIES.value,
    reset_world_between_cycles=_RESET_WORLD_BETWEEN_CYCLES.value,
    enable_object_reparenting=_ENABLE_OBJECT_REPARENTING.value,
    return_to_view_frame=_RETURN_TO_VIEW_FRAME.value,
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
  if not isinstance(solution, SolutionInterface):
    solution = Solution(solution)

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

  if config.enable_object_reparenting:
    solution.world.ensure_workpiece_at_root(config.perception.scene_object_id)

  workcell_state = config.state

  if workcell_state.cycles_remaining is None:
    logging.info(
      "Configured for continuous infinite machine tending cycles"
      " (total_cycles=%d).",
      workcell_state.total_cycles,
    )

  while workcell_state.has_work_remaining:
    cycle_num = workcell_state.cycles_completed + 1
    current_stock_id = config.perception.scene_object_id

    is_ready = False
    for prep_attempt in range(1, config.inter_cycle_max_retries + 1):
      if config.mock_hardware:
        is_ready = True
        break
      try:
        solution.cancel_all_operations()
        robot.clear_faults()
        solution.clear_motion_planner_cache()
        if config.reset_world_between_cycles:
          solution.world.reset(
            robot=robot,
            solution=solution,
            workpiece_object_name=current_stock_id,
          )
        is_ready = True
        break
      except Exception as prep_err:  # pylint: disable=broad-exception-caught
        logging.warning(
          "Inter-cycle cell preparation failed on attempt %d/%d: %s",
          prep_attempt,
          config.inter_cycle_max_retries,
          prep_err,
        )
        time.sleep(1.0)

    if not is_ready:
      raise RuntimeError(
        f"Cell could not be prepared for cycle {cycle_num} "
        f"after {config.inter_cycle_max_retries} attempts."
      )

    workpiece = Workpiece(
      asset_id=current_stock_id,
      object_name=f"raw_stock_{cycle_num:02d}",
    )
    workcell_state.start_new_cycle(workpiece)

    logging.info(
      "Configuring Vision Infeed for cycle %d: workpiece %s (asset %s).",
      cycle_num,
      workpiece.object_name,
      current_stock_id,
    )
    infeed_strategy = PerceptionInfeedStrategy(
      camera_name=config.perception.camera_name,
      pose_estimator_id=config.perception.estimator_id,
      scene_object_id=current_stock_id,
      sensor_ids=config.perception.sensor_ids,
      min_num_instances=config.perception.min_instances,
      view_frame_name=config.frames.view,
    )

    logging.info(
      "Constructing SBL Behavior Tree for machine tending cycle %d...",
      cycle_num,
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
      state=workcell_state,
      machining_timeout_seconds=config.machining_timeout_seconds,
      min_safe_z=config.motion.min_safe_z,
      grasp_offset_z=config.motion.grasp_offset_z,
      close_gripper_before_perception=(
        config.perception.close_gripper_before_perception
      ),
      return_to_view_frame=config.return_to_view_frame,
      solution=solution,
      enable_object_reparenting=config.enable_object_reparenting,
      perception_max_retries=config.perception.max_retries,
      perception_retry_delay_sec=config.perception.retry_delay_sec,
      start_phase=workcell_state.phase,
    )

    executive_simulation_mode = to_executive_simulation_mode(
      config.simulation_mode
    )
    logging.info(
      "Executing OMTS Machine Tending Pipeline (cycle %d, simulation mode: %s)...",
      cycle_num,
      config.simulation_mode.value
      if config.simulation_mode
      else "executive default",
    )
    cycle_start_time = time.perf_counter()
    try:
      solution.run(tree, simulation_mode=executive_simulation_mode)
      duration = workcell_state.record_cycle_success()
      total_duration = time.perf_counter() - cycle_start_time
      logging.info(
        "Pipeline execution completed successfully in %.2fs"
        " (total cycle time: %.2fs).",
        duration,
        total_duration,
      )
      solution.clear_motion_planner_cache()
    except KeyboardInterrupt:
      logging.info(
        "Machine tending loop interrupted by operator after %d cycle(s).",
        workcell_state.cycles_completed,
      )
      break
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

  return workcell_state.cycles_completed


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  config = config_from_flags()
  run_machine_tending_cycle(config)


if __name__ == "__main__":
  app.run(main)
