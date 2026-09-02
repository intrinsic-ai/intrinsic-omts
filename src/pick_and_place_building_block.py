"""Main application entry point for Building Block Pick and Place."""

import math
import time
from typing import Any, Mapping, Sequence
from unittest import mock

from absl import app
from absl import flags
from absl import logging
from intrinsic.assets import id_utils
from intrinsic.math.python import data_types
from intrinsic.math.python import proto_conversion
from intrinsic.perception.public.proto.v1 import pose_estimator_id_pb2
from intrinsic.solutions import deployments
from intrinsic.world.public.proto import object_world_updates_pb2
from src.behaviors.building_block_bt import build_building_block_pick_place_tree
from src.behaviors.motions import create_move_to_frame_task
from src.core.types import Pose3D
from src.hardware.gripper import GripperInterface
from src.hardware.gripper import MockGripper
from src.hardware.gripper import SideloadedGripperCmd
from src.hardware.robot import MockRobot
from src.hardware.robot import RobotInterface
from src.hardware.robot import UrRobot

_ADDRESS = flags.DEFINE_string(
    "address",
    "localhost:17080",
    "gRPC address of the running SBL solution deployment.",
)
_POSE_ESTIMATOR_ID = flags.DEFINE_string(
    "pose_estimator_id",
    "ai.intrinsic.raw_stock_2x3x5_estimator",
    "Asset ID of the registered pose estimator model.",
)
_CAMERA_NAME = flags.DEFINE_string(
    "camera_name",
    "orbbec_camera",
    "Attribute name of 3D camera in solution.world/resources.",
)
_SERVICE_NAME = flags.DEFINE_string(
    "service_name",
    "pose_estimator_service",
    "Name of the IOC pose estimator service in solution resources.",
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
_PLACE_OFFSET_X = flags.DEFINE_float(
    "place_offset_x",
    0.20,
    "X offset in meters from detected pose to place target pose.",
)
_PLACE_OFFSET_Y = flags.DEFINE_float(
    "place_offset_y",
    0.00,
    "Y offset in meters from detected pose to place target pose.",
)
_APPROACH_HEIGHT_M = flags.DEFINE_float(
    "approach_height_m",
    0.10,
    "Approach height in meters above grasp and place targets.",
)
_GRIPPER_ACTION_NAME = flags.DEFINE_string(
    "gripper_action_name",
    "/gripper/gripper_action_controller/gripper_cmd",
    "Action server name for sideloaded gripper command.",
)
_GRIPPER_JOINT_NAME = flags.DEFINE_string(
    "gripper_joint_name",
    "robotiq_hande_left_finger_joint",
    "Joint name for sideloaded gripper command.",
)
_GRIPPER_OPEN_POS = flags.DEFINE_float(
    "gripper_open_pos",
    0.025,
    "Position in meters for open gripper finger state.",
)
_GRIPPER_CLOSE_POS = flags.DEFINE_float(
    "gripper_close_pos",
    0.000,
    "Position in meters for closed gripper finger state.",
)
_MOCK_HARDWARE = flags.DEFINE_bool(
    "mock_hardware",
    False,
    "Use offline mock hardware adapters instead of live SBL skill stubs.",
)
_DISABLE_COLLISION_CHECKING = flags.DEFINE_bool(
    "disable_collision_checking",
    True,
    "Disable collision checking in motion planning for unmodeled worlds.",
)
_MOVE_TO_VIEW_FIRST = flags.DEFINE_bool(
    "move_to_view_first",
    False,
    "Whether to move to view_frame before the first perception capture.",
)
_NUM_CYCLES = flags.DEFINE_integer(
    "num_cycles",
    -1,
    "Number of cycles to run (-1 for infinite loop, >0 for fixed count).",
)
_RETRY_DELAY_SEC = flags.DEFINE_float(
    "retry_delay_sec",
    2.0,
    "Delay in seconds before retrying when no workpiece is detected.",
)
_ALTERNATE_PLACE_OFFSET = flags.DEFINE_bool(
    "alternate_place_offset",
    True,
    "Invert place offset on alternating cycles to keep block within workspace.",
)
_SETTLING_TIMEOUT_SECONDS = flags.DEFINE_float(
    "settling_timeout_seconds",
    10.0,
    "Settling timeout in seconds for trajectory execution.",
)


def get_camera_resource(
    solution: Any,
    camera_name: str | None = None,
) -> Any:
  """Resolves the camera resource handle from solution resources."""
  target = camera_name or "orbbec_camera"
  if hasattr(solution, "resources"):
    if isinstance(solution.resources, dict):
      if target in solution.resources:
        return solution.resources[target]
    else:
      try:
        return solution.resources[target]
      except (KeyError, AttributeError, TypeError):
        pass
    if hasattr(solution.resources, target):
      return getattr(solution.resources, target)
  raise ValueError(f"Camera resource '{target}' not found in solution.")


def get_perception_service_resource(
    solution: Any,
    service_name: str | None = None,
) -> Any:
  """Resolves the perception service handle from solution resources."""
  target = service_name or "pose_estimator_service"
  if hasattr(solution, "resources"):
    if isinstance(solution.resources, dict):
      if target in solution.resources:
        return solution.resources[target]
    else:
      try:
        return solution.resources[target]
      except (KeyError, AttributeError, TypeError):
        pass
    if hasattr(solution.resources, target):
      return getattr(solution.resources, target)
  raise ValueError(f"Perception service '{target}' not found in solution.")


def select_best_detection(estimates: Sequence[Any]) -> Any | None:
  """Selects the detection with most negative score (highest confidence)."""
  if not estimates:
    return None
  try:
    return min(
        estimates,
        key=lambda e: (
            float(e.score)
            if hasattr(e, "score")
            else (e.get("score", 0.0) if isinstance(e, dict) else 0.0)
        ),
    )
  except (ValueError, TypeError):
    return None


select_best_estimate = select_best_detection


def get_camera_transform_in_root(
    world: Any,
    camera_name: str = "orbbec_camera",
    parent_object_name: str = "root",
) -> Any | None:
  """Retrieves the Pose3 transform of the camera sensor in the parent frame."""
  if world is None or not hasattr(world, "get_transform"):
    return None
  root_node = getattr(world, parent_object_name, getattr(world, "root", None))
  camera_obj = getattr(world, camera_name, None)
  if camera_obj is None:
    for obj_name in ["ur_module", "robot", "root"]:
      parent = getattr(world, obj_name, None)
      if parent is not None and hasattr(parent, camera_name):
        camera_obj = getattr(parent, camera_name)
        break
  if root_node is None or camera_obj is None:
    return None
  camera_sensor_node = getattr(camera_obj, "sensor", camera_obj)
  try:
    return world.get_transform(root_node, camera_sensor_node)
  except Exception as e:
    logging.warning("Failed to get camera transform in root: %s", e)
    return None


def extract_pose_from_estimate(
    estimate: Any,
    root_t_camera: Any | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
  """Extracts (position, orientation) tuples from a detection estimate."""
  if hasattr(estimate, "root_t_target"):
    pose = proto_conversion.pose_from_proto(estimate.root_t_target)
    trans = pose.translation
    quat = pose.rotation.quaternion
    return (
        (float(trans[0]), float(trans[1]), float(trans[2])),
        (float(quat.x), float(quat.y), float(quat.z), float(quat.w)),
    )
  if hasattr(estimate, "position") and hasattr(estimate, "orientation"):
    pos = tuple(float(x) for x in estimate.position)
    ori = tuple(float(x) for x in estimate.orientation)
    if root_t_camera is not None and hasattr(root_t_camera, "__mul__"):
      quat = data_types.Quaternion([ori[0], ori[1], ori[2], ori[3]])
      pose = root_t_camera * data_types.Pose3(
          data_types.Rotation3(quat), [pos[0], pos[1], pos[2]]
      )
      trans = pose.translation
      q = pose.rotation.quaternion
      return (
          (float(trans[0]), float(trans[1]), float(trans[2])),
          (float(q.x), float(q.y), float(q.z), float(q.w)),
      )
    return pos, ori
  if hasattr(estimate, "pose"):
    p = estimate.pose
    if isinstance(p, Pose3D):
      pos = p.to_translation_tuple()
      ori = p.to_quaternion_tuple()
      if root_t_camera is not None and hasattr(root_t_camera, "__mul__"):
        quat = data_types.Quaternion([ori[0], ori[1], ori[2], ori[3]])
        pose = root_t_camera * data_types.Pose3(
            data_types.Rotation3(quat), [pos[0], pos[1], pos[2]]
        )
        trans = pose.translation
        q = pose.rotation.quaternion
        return (
            (float(trans[0]), float(trans[1]), float(trans[2])),
            (float(q.x), float(q.y), float(q.z), float(q.w)),
        )
      return pos, ori
    if hasattr(p, "translation") and hasattr(p, "rotation"):
      pose = (
          root_t_camera * p
          if (root_t_camera is not None and hasattr(root_t_camera, "__mul__"))
          else p
      )
      trans = pose.translation
      quat = pose.rotation.quaternion
      return (
          (float(trans[0]), float(trans[1]), float(trans[2])),
          (float(quat.x), float(quat.y), float(quat.z), float(quat.w)),
      )
  if isinstance(estimate, dict):
    pos = tuple(float(x) for x in estimate.get("position", (0.0, 0.0, 0.0)))
    ori = tuple(
        float(x) for x in estimate.get("orientation", (0.0, 0.0, 0.0, 1.0))
    )
    if root_t_camera is not None and hasattr(root_t_camera, "__mul__"):
      quat = data_types.Quaternion([ori[0], ori[1], ori[2], ori[3]])
      pose = root_t_camera * data_types.Pose3(
          data_types.Rotation3(quat), [pos[0], pos[1], pos[2]]
      )
      trans = pose.translation
      q = pose.rotation.quaternion
      return (
          (float(trans[0]), float(trans[1]), float(trans[2])),
          (float(q.x), float(q.y), float(q.z), float(q.w)),
      )
    return pos, ori
  raise ValueError(f"Unable to extract pose from estimate: {estimate}")


def compute_dynamic_frame_poses(
    position: Sequence[float] | tuple[float, float, float] | Pose3D,
    orientation: (
        Sequence[float] | tuple[float, float, float, float] | None
    ) = None,
    approach_height_m: float = 0.10,
    place_offset_x: float = 0.20,
    place_offset_y: float = 0.00,
) -> dict[
    str, tuple[tuple[float, float, float], tuple[float, float, float, float]]
]:
  """Computes pregrasp, grasp, preplace, and place poses from 6D pose.

  Aligns the downward gripper TCP orientation along -Z with the detected
  workpiece yaw angle.
  """
  if isinstance(position, Pose3D):
    pos_x, pos_y, pos_z = position.x, position.y, position.z
    ori = (position.qx, position.qy, position.qz, position.qw)
  elif orientation is not None:
    pos_x, pos_y, pos_z = (
        float(position[0]),
        float(position[1]),
        float(position[2]),
    )
    ori = (
        float(orientation[0]),
        float(orientation[1]),
        float(orientation[2]),
        float(orientation[3]),
    )
  else:
    pos_x, pos_y, pos_z = (
        float(position[0]),
        float(position[1]),
        float(position[2]),
    )
    ori = (0.0, 0.0, 0.0, 1.0)

  qx, qy, qz, qw = ori
  norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
  if norm > 1e-6:
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
  else:
    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

  # Compute horizontal in-plane yaw angle from orientation quaternion
  vx_x = 1.0 - 2.0 * (qy * qy + qz * qz)
  vx_y = 2.0 * (qx * qy + qz * qw)
  vy_x = 2.0 * (qx * qy - qz * qw)
  vy_y = 1.0 - 2.0 * (qx * qx + qz * qz)

  if (vx_x * vx_x + vx_y * vx_y) >= 1e-4:
    yaw = math.atan2(vx_y, vx_x)
  elif (vy_x * vy_x + vy_y * vy_y) >= 1e-4:
    yaw = math.atan2(vy_y, vy_x) - math.pi / 2.0
  else:
    yaw = 0.0

  # Downward top-down orientation: R_z(yaw) * R_x(pi)
  grasp_ori = (
      math.cos(yaw / 2.0),
      math.sin(yaw / 2.0),
      0.0,
      0.0,
  )

  return {
      "dynamic_pregrasp": (
          (pos_x, pos_y, pos_z + approach_height_m),
          grasp_ori,
      ),
      "dynamic_grasp": ((pos_x, pos_y, pos_z), grasp_ori),
      "dynamic_preplace": (
          (
              pos_x + place_offset_x,
              pos_y + place_offset_y,
              pos_z + approach_height_m,
          ),
          grasp_ori,
      ),
      "dynamic_place": (
          (pos_x + place_offset_x, pos_y + place_offset_y, pos_z),
          grasp_ori,
      ),
  }


def inject_dynamic_frames(
    world: Any,
    frame_poses: Mapping[
        str,
        tuple[tuple[float, float, float], tuple[float, float, float, float]],
    ],
    parent_object_name: str = "root",
) -> None:
  """Injects or updates dynamic frames in the SBL solution world."""
  parent_obj = getattr(world, parent_object_name, None)
  if parent_obj is None:
    logging.warning("Parent object '%s' not found.", parent_object_name)
    return

  existing_frames = set()
  if hasattr(parent_obj, "list_frames") and callable(parent_obj.list_frames):
    try:
      existing_frames = set(parent_obj.list_frames())
    except Exception:  # pylint: disable=broad-exception-caught
      pass
  elif hasattr(parent_obj, "__dict__"):
    existing_frames = set(parent_obj.__dict__.keys())

  new_updates = object_world_updates_pb2.ObjectWorldUpdates()
  for frame_name, (pos, ori) in frame_poses.items():
    pose = data_types.Pose3(
        data_types.Rotation3(data_types.Quaternion(list(ori))),
        list(pos),
    )
    frame_exists = frame_name in existing_frames or (
        hasattr(parent_obj, frame_name)
        and not callable(getattr(parent_obj, frame_name))
    )
    if frame_exists:
      frame_node = getattr(parent_obj, frame_name)
      world.update_transform(node_a=parent_obj, node_b=frame_node, a_t_b=pose)
    else:
      up = new_updates.updates.add()
      cf = up.create_frame
      cf.parent_object_with_filter.reference.by_name.object_name = (
          parent_object_name
      )
      cf.new_frame_name = frame_name
      cf.parent_t_new_frame.position.x = pos[0]
      cf.parent_t_new_frame.position.y = pos[1]
      cf.parent_t_new_frame.position.z = pos[2]
      cf.parent_t_new_frame.orientation.x = ori[0]
      cf.parent_t_new_frame.orientation.y = ori[1]
      cf.parent_t_new_frame.orientation.z = ori[2]
      cf.parent_t_new_frame.orientation.w = ori[3]

  if new_updates.updates:
    world.batch_update(new_updates)


def create_pose_estimation_pipeline(
    solution: Any,
    camera_resource: Any,
    perception_resource: Any,
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    sensor_ids: Sequence[int] | None = (1, 4),
    min_num_instances: int = 1,
) -> tuple[Any, Any]:
  """Constructs capture_images and estimate_pose_multi_view skills."""
  package_name = id_utils.package_from(pose_estimator_id)
  name = id_utils.name_from(pose_estimator_id)
  skills = solution.skills

  capture_images_skill = skills.ai.intrinsic.capture_images(
      camera=camera_resource,
      sensor_ids=[int(sid) for sid in sensor_ids] if sensor_ids else [1, 4],
      log_debug_data=True,
  )

  pose_estimator_proto = pose_estimator_id_pb2.PoseEstimatorId(
      id=name,
      package=package_name,
  )

  estimate_pose_skill = skills.ai.intrinsic.estimate_pose_multi_view(
      camera_1=camera_resource,
      camera_2=camera_resource,
      camera_3=camera_resource,
      camera_4=camera_resource,
      perception=perception_resource,
      pose_estimator=pose_estimator_proto,
      capture_data=[capture_images_skill.result.capture_data],
      min_num_instances=min_num_instances,
      log_debug_data=True,
  )

  return capture_images_skill, estimate_pose_skill


def run_pick_and_place_loop(
    solution_address: str = "localhost:17080",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    camera_name: str = "orbbec_camera",
    service_name: str = "pose_estimator_service",
    sensor_ids: Sequence[int] = (1, 4),
    min_num_instances: int = 1,
    arm_part_name: str = "ur_module",
    tool_object_name: str = "gripper",
    tool_frame_name: str = "tool_frame",
    parent_object: str = "root",
    view_frame: str = "view",
    place_offset_x: float = 0.20,
    place_offset_y: float = 0.00,
    approach_height_m: float = 0.10,
    gripper_action_name: str = "/gripper/gripper_action_controller/gripper_cmd",
    gripper_joint_name: str = "robotiq_hande_left_finger_joint",
    gripper_open_pos: float = 0.025,
    gripper_close_pos: float = 0.000,
    mock_hardware: bool = False,
    disable_collision_checking: bool = True,
    alternate_place_offset: bool = True,
    move_to_view_first: bool = False,
    num_cycles: int = -1,
    retry_delay_sec: float = 2.0,
    settling_timeout_seconds: float = 10.0,
    solution: Any | None = None,
) -> int:
  """Runs the main pick and place loop for building blocks."""
  if solution is None:
    if mock_hardware:
      solution = mock.MagicMock()
    else:
      logging.info(
          "Connecting to Intrinsic solution at %s...", solution_address
      )
      solution = deployments.connect(address=solution_address)

  if mock_hardware:
    robot: RobotInterface = MockRobot()
    gripper: GripperInterface = MockGripper()
    camera_resource = getattr(solution.resources, camera_name, mock.MagicMock())
    perception_resource = getattr(
        solution.resources, service_name, mock.MagicMock()
    )
  else:
    robot = UrRobot(
        solution=solution,
        arm_part_name=arm_part_name,
        tool_object_name=tool_object_name,
        tool_frame_name=tool_frame_name,
        disable_collision_checking=disable_collision_checking,
        default_settling_timeout_seconds=settling_timeout_seconds,
    )
    gripper = SideloadedGripperCmd(
        solution=solution,
        action_name=gripper_action_name,
        joint_name=gripper_joint_name,
        open_position=gripper_open_pos,
        close_position=gripper_close_pos,
    )
    camera_resource = get_camera_resource(solution, camera_name)
    perception_resource = get_perception_service_resource(
        solution, service_name
    )

  cycle_count = 0
  while num_cycles <= 0 or cycle_count < num_cycles:
    logging.info("Starting pick and place cycle %d...", cycle_count + 1)

    # 1. Move to view frame
    if move_to_view_first or cycle_count > 0:
      logging.info("Step 1: Moving robot to view frame '%s'...", view_frame)
      move_view_task = create_move_to_frame_task(
          robot=robot,
          frame_name=view_frame,
          parent_object=parent_object,
          motion_type="ANY",
          settling_timeout_seconds=settling_timeout_seconds,
          task_name=f"Move to View Frame ({parent_object}/{view_frame})",
      )
      solution.executive.run(move_view_task)

    # 2. Perception pipeline
    logging.info("Step 2: Executing perception pipeline on executive...")
    capture_skill, estimate_skill = create_pose_estimation_pipeline(
        solution=solution,
        camera_resource=camera_resource,
        perception_resource=perception_resource,
        pose_estimator_id=pose_estimator_id,
        sensor_ids=sensor_ids,
        min_num_instances=min_num_instances,
    )
    solution.executive.run([capture_skill, estimate_skill])
    result = solution.executive.get_value(estimate_skill.result)

    estimates = getattr(result, "estimates", []) if result is not None else []
    if mock_hardware and (
        not estimates or isinstance(estimates, mock.MagicMock)
    ):
      mock_est = mock.MagicMock()
      mock_est.score = -10.0
      mock_est.position = (0.15, 0.25, 0.71)
      mock_est.orientation = (1.0, 0.0, 0.0, 0.0)
      del mock_est.root_t_target
      estimates = [mock_est]

    if not estimates:
      logging.warning(
          "Step 3: No workpiece detected in cycle %d. Retrying in %.1fs...",
          cycle_count + 1,
          retry_delay_sec,
      )
      time.sleep(retry_delay_sec)
      continue

    # 3. Select best detection and compute frame poses
    best_detection = select_best_detection(estimates)
    root_t_camera = get_camera_transform_in_root(
        world=solution.world,
        camera_name=camera_name,
        parent_object_name=parent_object,
    )
    pos, ori = extract_pose_from_estimate(
        best_detection, root_t_camera=root_t_camera
    )
    logging.info("Step 4: Selected best detection at pos=%s, ori=%s", pos, ori)

    offset_sign = (
        -1.0 if (alternate_place_offset and cycle_count % 2 == 1) else 1.0
    )
    frame_poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
        approach_height_m=approach_height_m,
        place_offset_x=place_offset_x * offset_sign,
        place_offset_y=place_offset_y * offset_sign,
    )

    # 4. Inject dynamic frames into solution.world
    inject_dynamic_frames(
        world=solution.world,
        frame_poses=frame_poses,
        parent_object_name=parent_object,
    )

    # 5. Build and execute Behavior Tree
    logging.info("Step 6: Executing Behavior Tree on executive...")
    tree = build_building_block_pick_place_tree(
        robot=robot,
        gripper=gripper,
        parent_object=parent_object,
        pregrasp_frame_name="dynamic_pregrasp",
        grasp_frame_name="dynamic_grasp",
        preplace_frame_name="dynamic_preplace",
        place_frame_name="dynamic_place",
        view_frame_name=view_frame,
        settling_timeout_seconds=settling_timeout_seconds,
    )
    solution.executive.run(tree)

    cycle_count += 1
    logging.info("Completed cycle %d successfully.", cycle_count)

  return cycle_count


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  sensor_ids = (
      [int(s.strip()) for s in _SENSOR_IDS.value if s.strip()]
      if _SENSOR_IDS.value
      else [1, 4]
  )

  run_pick_and_place_loop(
      solution_address=_ADDRESS.value,
      pose_estimator_id=_POSE_ESTIMATOR_ID.value,
      camera_name=_CAMERA_NAME.value,
      service_name=_SERVICE_NAME.value,
      sensor_ids=sensor_ids,
      min_num_instances=_MIN_NUM_INSTANCES.value,
      arm_part_name=_ARM_PART_NAME.value,
      tool_object_name=_TOOL_OBJECT_NAME.value,
      tool_frame_name=_TOOL_FRAME_NAME.value,
      parent_object=_PARENT_OBJECT.value,
      view_frame=_VIEW_FRAME.value,
      place_offset_x=_PLACE_OFFSET_X.value,
      place_offset_y=_PLACE_OFFSET_Y.value,
      approach_height_m=_APPROACH_HEIGHT_M.value,
      gripper_action_name=_GRIPPER_ACTION_NAME.value,
      gripper_joint_name=_GRIPPER_JOINT_NAME.value,
      gripper_open_pos=_GRIPPER_OPEN_POS.value,
      gripper_close_pos=_GRIPPER_CLOSE_POS.value,
      mock_hardware=_MOCK_HARDWARE.value,
      disable_collision_checking=_DISABLE_COLLISION_CHECKING.value,
      alternate_place_offset=_ALTERNATE_PLACE_OFFSET.value,
      move_to_view_first=_MOVE_TO_VIEW_FIRST.value,
      num_cycles=_NUM_CYCLES.value,
      retry_delay_sec=_RETRY_DELAY_SEC.value,
      settling_timeout_seconds=_SETTLING_TIMEOUT_SECONDS.value,
  )


if __name__ == "__main__":
  app.run(main)

