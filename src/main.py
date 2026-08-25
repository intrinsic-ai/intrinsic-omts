"""Main application entrypoint for the Open Machine Tending Solution (OMTS)."""

from collections.abc import Sequence
from absl import app
from absl import flags
from intrinsic.solutions import deployments
from intrinsic.solutions.execution import ExecutionFailedError
from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.core.infeed import GridInfeedStrategy, InfeedStrategy, PerceptionInfeedStrategy
from src.core.tray import Tray
from src.core.types import InfeedMode
from src.core.workcell import WorkcellState
from src.hardware.gripper import DioGripper, MockGripper
from src.hardware.machine import DioCncMachine, MockCncMachine
from src.hardware.robot import MockRobot, UrRobot
from src.hardware.vision import MockVision, OrbbecVision
from src.utils.logging_utils import log_error, log_info, log_step

_ADDRESS = flags.DEFINE_string(
    "address",
    "localhost:17080",
    "Address of the Intrinsic solution deployment.",
)
_INFEED_MODE = flags.DEFINE_enum_class(
    "infeed_mode",
    InfeedMode.PERCEPTION,
    InfeedMode,
    "Infeed strategy: 'perception' (random placement) or 'grid' (blind tray).",
)
_MOCK_HARDWARE = flags.DEFINE_bool(
    "mock_hardware",
    False,
    "Whether to run with offline mock hardware adapters for testing.",
)
_ARM_PART_NAME = flags.DEFINE_string(
    "arm_part_name",
    "ur_module",
    "Attribute name of robot arm part in solution.world.",
)
_CAMERA_NAME = flags.DEFINE_string(
    "camera_name",
    "orbbec_camera",
    "Attribute name of 3D camera in solution.world.",
)


def run_machine_tending_cycle(
    solution_address: str,
    infeed_mode: InfeedMode,
    mock_hardware: bool = False,
    arm_part_name: str = "ur_module",
    camera_name: str = "orbbec_camera",
) -> None:
  """Executes one full machine tending cycle."""
  log_info(f"Connecting to Intrinsic solution at {solution_address}...")
  solution = deployments.connect(address=solution_address)

  # Initialize hardware adapters
  if mock_hardware:
    log_info("Using mock hardware adapters.")
    robot = MockRobot()
    gripper = MockGripper()
    machine = MockCncMachine()
    vision = MockVision()
  else:
    log_info("Using live SBL hardware adapters.")
    robot = UrRobot(solution=solution, arm_part_name=arm_part_name)
    gripper = DioGripper(solution=solution, device_name=arm_part_name)
    machine = DioCncMachine(
        solution=solution,
        device_name=arm_part_name,
        is_mock=False,
    )
    vision = OrbbecVision(solution=solution, camera_name=camera_name)

  # Configure Infeed Strategy
  if infeed_mode == InfeedMode.PERCEPTION:
    log_info("Configuring PerceptionInfeedStrategy (random part placement)...")
    infeed_strategy: InfeedStrategy = PerceptionInfeedStrategy(
        camera_name=camera_name,
        estimator_name="raw_stock_2x3x5_estimator",
        view_joint_pose_name="view_pose",
    )
  else:
    log_info("Configuring GridInfeedStrategy (blind tray grid)...")
    infeed_tray = Tray(
        name="infeed_tray",
        rows=2,
        cols=4,
        pitch_x=0.06,
        pitch_y=0.08,
        origin_frame="infeed_tray_origin",
    )
    infeed_tray.populate_all_slots()
    infeed_strategy = GridInfeedStrategy(tray=infeed_tray)

  workcell_state = WorkcellState()
  workpiece = infeed_strategy.get_target_part()
  if workpiece is None:
    raise ValueError("No available workpiece found for infeed strategy.")

  workcell_state.start_new_cycle(workpiece=workpiece)

  log_info(f"Assembling Behavior Tree for workpiece '{workpiece.id}'...")
  tree = build_machine_tending_behavior_tree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      vision=vision,
      infeed_strategy=infeed_strategy,
      workpiece=workpiece,
  )

  log_step(1, "Executing OMTS Machine Tending Master Cycle via Executive")
  try:
    solution.executive.run(tree)
    duration = workcell_state.record_cycle_success()
    log_info(f"Machine tending cycle completed successfully in {duration:.2f}s.")
  except ExecutionFailedError as e:
    workcell_state.record_cycle_failure()
    log_error("Machine tending cycle failed during execution", e)
    if hasattr(solution.executive, "get_errors"):
      log_error("Executive errors", solution.executive.get_errors())
    raise


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  run_machine_tending_cycle(
      solution_address=_ADDRESS.value,
      infeed_mode=_INFEED_MODE.value,
      mock_hardware=_MOCK_HARDWARE.value,
      arm_part_name=_ARM_PART_NAME.value,
      camera_name=_CAMERA_NAME.value,
  )


if __name__ == "__main__":
  app.run(main)
