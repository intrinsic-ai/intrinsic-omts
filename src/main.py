"""Main application entrypoint for the Open Machine Tending Solution."""

from typing import Sequence

from absl import app
from absl import flags
from absl import logging
from intrinsic.solutions import deployments
from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.core.infeed import GridInfeedStrategy, InfeedMode, PerceptionInfeedStrategy
from src.core.workcell import WorkcellState
from src.core.workpiece import Workpiece
from src.hardware.gripper import MockGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot, UrRobot
from src.hardware.vision import MockVision, OrbbecVision

_ADDRESS = flags.DEFINE_string(
    "address",
    "localhost:17080",
    "gRPC address of the running SBL solution deployment.",
)
_INFEED_MODE = flags.DEFINE_enum_class(
    "infeed_mode",
    InfeedMode.PERCEPTION,
    InfeedMode,
    "Infeed strategy mode: 'perception' (3D vision) or 'grid' (slot math).",
)
_MOCK_HARDWARE = flags.DEFINE_bool(
    "mock_hardware",
    False,
    "Use offline mock hardware adapters instead of live SBL skill stubs.",
)
_ARM_PART_NAME = flags.DEFINE_string(
    "arm_part_name",
    "ur_module",
    "ICON part name for the robot arm in solution.world.",
)
_TOOL_OBJECT_NAME = flags.DEFINE_string(
    "tool_object_name",
    "gripper",
    "Object name for the robot end-effector tool.",
)
_TOOL_FRAME_NAME = flags.DEFINE_string(
    "tool_frame_name",
    "tool_frame",
    "Frame name on tool_object_name to use as moving tool reference.",
)
_CAMERA_NAME = flags.DEFINE_string(
    "camera_name",
    "orbbec_camera",
    "Attribute name of 3D camera in solution.world.",
)
_PERCEPTION_SERVICE_NAME = flags.DEFINE_string(
    "perception_service_name",
    "pose_estimator_service",
    "Name of the IOC pose estimator service in solution resources.",
)
_POSE_ESTIMATOR_ID = flags.DEFINE_string(
    "pose_estimator_id",
    "ai.intrinsic.raw_stock_2x3x5_estimator",
    "Asset ID of the registered pose estimator model.",
)
_SCENE_OBJECT_ID = flags.DEFINE_string(
    "scene_object_id",
    "ai.intrinsic.raw_stock_2x3x5",
    "Asset ID of the target scene object to spawn in belief world.",
)
_SENSOR_IDS = flags.DEFINE_list(
    "sensor_ids",
    ["1", "4"],
    "Sensor IDs to capture from the RGB-D camera (1=RGB, 4=Depth).",
)
_MIN_NUM_INSTANCES = flags.DEFINE_integer(
    "min_num_instances",
    1,
    "Minimum number of detected workpiece instances required.",
)
_PARENT_OBJECT = flags.DEFINE_string(
    "parent_object",
    "root",
    "Parent object in world for target motion frames (default: 'root').",
)
_VIEW_FRAME = flags.DEFINE_string(
    "view_frame",
    "view",
    "Perception camera viewing target frame name.",
)
_PREGRASP_FRAME = flags.DEFINE_string(
    "pregrasp_frame",
    "pre_grasp",
    "Pre-grasp approach target frame name.",
)
_GRASP_FRAME = flags.DEFINE_string(
    "grasp_frame",
    "grasp",
    "Grasp target frame name.",
)
_MACHINE_APPROACH_FRAME = flags.DEFINE_string(
    "machine_approach_frame",
    "machine_approach",
    "Machine entry approach target frame name.",
)
_PREPLACE_VISE_FRAME = flags.DEFINE_string(
    "preplace_vise_frame",
    "pre_place_vise",
    "Pre-place CNC vise approach frame name.",
)
_PLACE_VISE_FRAME = flags.DEFINE_string(
    "place_vise_frame",
    "place_vise",
    "Place CNC vise frame name.",
)


def run_machine_tending_cycle(
    solution_address: str,
    infeed_mode: InfeedMode,
    mock_hardware: bool = False,
    arm_part_name: str = "ur_module",
    tool_object_name: str = "gripper",
    tool_frame_name: str = "tool_frame",
    camera_name: str = "orbbec_camera",
    perception_service_name: str = "pose_estimator_service",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
    sensor_ids: Sequence[int] = (1, 4),
    min_num_instances: int = 1,
    parent_object: str = "root",
    view_frame: str = "view",
    pregrasp_frame: str = "pre_grasp",
    grasp_frame: str = "grasp",
    machine_approach_frame: str = "machine_approach",
    preplace_vise_frame: str = "pre_place_vise",
    place_vise_frame: str = "place_vise",
) -> None:
  """Executes one full machine tending cycle."""
  logging.info("Connecting to Intrinsic solution at %s...", solution_address)
  solution = deployments.connect(address=solution_address)

  # Initialize hardware adapters
  if mock_hardware:
    logging.info("Using offline mock hardware adapters.")
    robot = MockRobot()
    gripper = MockGripper()
    machine = MockCncMachine()
    vision = MockVision()
  else:
    logging.info("Initializing live hardware adapters from solution deployment.")
    robot = UrRobot(
        solution=solution,
        arm_part_name=arm_part_name,
        tool_object_name=tool_object_name,
        tool_frame_name=tool_frame_name,
    )
    gripper = MockGripper()
    machine = MockCncMachine()
    vision = OrbbecVision(
        solution=solution,
        camera_name=camera_name,
        perception_service_name=perception_service_name,
        sensor_ids=sensor_ids,
    )

  # Select infeed strategy
  workpiece = Workpiece(id="raw_stock_01")
  workcell_state = WorkcellState()
  workcell_state.start_new_cycle(workpiece)

  if infeed_mode == InfeedMode.PERCEPTION:
    logging.info("Configuring Vision-Guided Perception Infeed Strategy.")
    infeed_strategy = PerceptionInfeedStrategy(
        camera_name=camera_name,
        pose_estimator_id=pose_estimator_id,
        scene_object_id=scene_object_id,
        sensor_ids=sensor_ids,
        min_num_instances=min_num_instances,
        view_frame_name=view_frame,
    )
  else:
    logging.info("Configuring Blind Grid Pallet Infeed Strategy.")
    infeed_strategy = GridInfeedStrategy(slot_count=4)

  # Build master Behavior Tree
  logging.info("Constructing SBL Behavior Tree for machine tending cycle...")
  tree = build_machine_tending_behavior_tree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      vision=vision,
      infeed_strategy=infeed_strategy,
      workpiece=workpiece,
      parent_object=parent_object,
      view_frame_name=view_frame,
      pregrasp_frame_name=pregrasp_frame,
      grasp_frame_name=grasp_frame,
      machine_approach_frame_name=machine_approach_frame,
      preplace_vise_frame_name=preplace_vise_frame,
      place_vise_frame_name=place_vise_frame,
  )

  logging.info("Executing OMTS Infeed, Acquisition & Vise Approach Pipeline...")
  try:
    solution.executive.run(tree)
    duration = workcell_state.record_cycle_success()
    logging.info("Pipeline execution completed successfully in %.2fs.", duration)
  except Exception as e:
    workcell_state.record_cycle_failure()
    logging.error("Pipeline execution failed: %s", e)
    if hasattr(solution.executive, "get_errors"):
      logging.error("Executive errors: %s", solution.executive.get_errors())
    raise


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  sensor_ids = (
      [int(s.strip()) for s in _SENSOR_IDS.value if s.strip()]
      if _SENSOR_IDS.value
      else [1, 4]
  )

  run_machine_tending_cycle(
      solution_address=_ADDRESS.value,
      infeed_mode=_INFEED_MODE.value,
      mock_hardware=_MOCK_HARDWARE.value,
      arm_part_name=_ARM_PART_NAME.value,
      tool_object_name=_TOOL_OBJECT_NAME.value,
      tool_frame_name=_TOOL_FRAME_NAME.value,
      camera_name=_CAMERA_NAME.value,
      perception_service_name=_PERCEPTION_SERVICE_NAME.value,
      pose_estimator_id=_POSE_ESTIMATOR_ID.value,
      scene_object_id=_SCENE_OBJECT_ID.value,
      sensor_ids=sensor_ids,
      min_num_instances=_MIN_NUM_INSTANCES.value,
      parent_object=_PARENT_OBJECT.value,
      view_frame=_VIEW_FRAME.value,
      pregrasp_frame=_PREGRASP_FRAME.value,
      grasp_frame=_GRASP_FRAME.value,
      machine_approach_frame=_MACHINE_APPROACH_FRAME.value,
      preplace_vise_frame=_PREPLACE_VISE_FRAME.value,
      place_vise_frame=_PLACE_VISE_FRAME.value,
  )


if __name__ == "__main__":
  app.run(main)
