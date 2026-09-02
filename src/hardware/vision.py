"""Vision and 3D camera hardware interfaces and implementations."""

import abc
from typing import Optional, Sequence

from intrinsic.assets import id_utils
from intrinsic.assets.proto import id_pb2
from intrinsic.perception.public.proto.v1 import pose_estimator_id_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import cel
from intrinsic.solutions import deployments
from intrinsic.solutions import provided
from intrinsic.world.public.proto import object_world_refs_pb2
from src.core.types import Pose3D


def _get_camera_resource(
    solution: deployments.Solution,
    camera_name: Optional[str] = None,
) -> provided.ResourceHandle:
  """Resolves the camera resource handle from solution resources."""
  target_name = camera_name or "orbbec_camera"
  if isinstance(solution.resources, dict) and target_name in solution.resources:
    return solution.resources[target_name]
  try:
    return solution.resources[target_name]
  except (KeyError, AttributeError, TypeError):
    pass

  # Fallback to capability search
  for handle in getattr(solution.resources, "values", lambda: solution.resources)():
    if hasattr(handle, "types") and "CameraConfig" in handle.types:
      return handle
  raise ValueError(f"Camera resource '{target_name}' not found in solution.")


def _get_perception_resource(
    solution: deployments.Solution,
    service_name: Optional[str] = None,
) -> provided.ResourceHandle:
  """Resolves the perception service resource handle from solution resources."""
  target_name = service_name or "pose_estimator_service"
  if isinstance(solution.resources, dict) and target_name in solution.resources:
    return solution.resources[target_name]
  try:
    return solution.resources[target_name]
  except (KeyError, AttributeError, TypeError):
    pass

  # Fallback to capability search
  for handle in getattr(solution.resources, "values", lambda: solution.resources)():
    if (
        hasattr(handle, "types")
        and "intrinsic_proto.perception.v1.PoseEstimationService" in handle.types
    ):
      return handle
  raise ValueError(f"Perception service resource '{target_name}' not found in solution.")


class VisionInterface(abc.ABC):
  """Abstract interface for perception acquisition and pose estimation."""

  @abc.abstractmethod
  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a task to trigger camera image acquisition."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_perception_and_spawn_task(
      self,
      target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      min_num_instances: int = 1,
      approach_offset_z: float = 0.05,
      parent_object: str = "root",
      pregrasp_frame_name: str = "pre_grasp",
      grasp_frame_name: str = "grasp",
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds a composite task to capture RGB-D, estimate 6D poses, and update world frames."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_estimate_and_update_pose_task(
      self,
      target_object: str = "raw_stock",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds a task to estimate target object pose and update world frame."""
    raise NotImplementedError


class OrbbecVision(VisionInterface):
  """Orbbec 3D camera perception adapter using IOC SBL perception skills."""

  def __init__(
      self,
      solution: deployments.Solution,
      camera_name: str = "orbbec_camera",
      perception_service_name: str = "pose_estimator_service",
      sensor_ids: Sequence[int] = (1, 4),
      log_debug_data: bool = True,
  ) -> None:
    self._solution = solution
    self._camera_name = camera_name
    self._perception_service_name = perception_service_name
    self._sensor_ids = list(sensor_ids)
    self._log_debug_data = log_debug_data

    # Resolve resource handles
    self._camera_resource = _get_camera_resource(solution, camera_name)
    self._perception_resource = _get_perception_resource(
        solution, perception_service_name
    )

  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    """Builds a single image capture task."""
    task_name = name or f"Capture Image ({self._camera_name})"
    action = self._solution.skills.ai.intrinsic.capture_images(
        camera=self._camera_resource,
        sensor_ids=self._sensor_ids,
        log_debug_data=self._log_debug_data,
    )
    return bt.Task(action=action, name=task_name)

  def build_perception_and_spawn_task(
      self,
      target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      min_num_instances: int = 1,
      approach_offset_z: float = 0.05,
      parent_object: str = "root",
      pregrasp_frame_name: str = "pre_grasp",
      grasp_frame_name: str = "grasp",
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds the pipeline to capture RGB-D, estimate 6D poses, and dynamically update grasp frames."""
    task_name = name or "Perception & Dynamic Grasp Frame Update Pipeline"

    skills = self._solution.skills

    # 1. Capture RGB-D Images
    capture_action = skills.ai.intrinsic.capture_images(
        camera=self._camera_resource,
        sensor_ids=self._sensor_ids,
        log_debug_data=self._log_debug_data,
    )
    capture_task = bt.Task(
        action=capture_action, name="1. Capture RGB-D Images"
    )

    # 2. Estimate 6D Poses via Multi-View / FoundationPose
    pkg = (
        id_utils.package_from(pose_estimator_id)
        if id_utils.is_id(pose_estimator_id)
        else "ai.intrinsic"
    )
    est_name = (
        id_utils.name_from(pose_estimator_id)
        if id_utils.is_id(pose_estimator_id)
        else pose_estimator_id
    )

    pose_estimator_proto = pose_estimator_id_pb2.PoseEstimatorId(
        id=est_name,
        package=pkg,
    )

    estimate_action = skills.ai.intrinsic.estimate_pose_multi_view(
        camera_1=self._camera_resource,
        camera_2=self._camera_resource,
        camera_3=self._camera_resource,
        camera_4=self._camera_resource,
        perception=self._perception_resource,
        pose_estimator=pose_estimator_proto,
        capture_data=[capture_action.result.capture_data],
        min_num_instances=min_num_instances,
        log_debug_data=self._log_debug_data,
    )
    estimate_task = bt.Task(
        action=estimate_action, name="2. Estimate 6D Workpiece Poses"
    )

    # 3. Update Dynamic Grasp and Pre-Grasp Frames via indirect transform in update_world.
    # By specifying node_a=camera, node_b=frame_on_root, node_to_update=frame_on_root,
    # SBL's world service automatically computes the world transform using the live robot
    # kinematics chain at runtime with zero hardcoded extrinsics or manual matrix math.
    camera_ref = object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            object=object_world_refs_pb2.ObjectReferenceByName(
                object_name=self._camera_name
            )
        )
    )
    pre_grasp_ref = object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            frame=object_world_refs_pb2.FrameReferenceByName(
                object_name=parent_object, frame_name=pregrasp_frame_name
            )
        )
    )
    grasp_ref = object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            frame=object_world_refs_pb2.FrameReferenceByName(
                object_name=parent_object, frame_name=grasp_frame_name
            )
        )
    )

    update_world_skill = skills.ai.intrinsic.update_world
    uw_proto = update_world_skill.intrinsic_proto

    detected_pos = estimate_action.result.estimates[0].root_t_target.position
    detected_rot = estimate_action.result.estimates[0].root_t_target.orientation

    # Align workpiece orientation with downward gripper approach:
    # Q_tool = Q_part * [0.5, 0.5, 0.5, 0.5]
    qx, qy, qz, qw = detected_rot.x, detected_rot.y, detected_rot.z, detected_rot.w
    tool_orientation = uw_proto.Quaternion(
        x=cel.CelExpression(f"0.5 * ({qw} + {qx} + {qy} - {qz})"),
        y=cel.CelExpression(f"0.5 * ({qw} - {qx} + {qy} + {qz})"),
        z=cel.CelExpression(f"0.5 * ({qw} + {qx} - {qy} + {qz})"),
        w=cel.CelExpression(f"0.5 * ({qw} - {qx} - {qy} - {qz})"),
    )

    # Standoff in camera optical frame: reduce distance along optical Z by approach_offset_z
    pre_grasp_update = uw_proto.world.ObjectWorldUpdate(
        update_transform=uw_proto.world.UpdateTransformRequest(
            node_a=camera_ref,
            node_b=pre_grasp_ref,
            node_to_update=pre_grasp_ref,
            a_t_b=uw_proto.Pose(
                position=uw_proto.Point(
                    x=detected_pos.x,
                    y=detected_pos.y,
                    z=cel.CelExpression(f"{detected_pos.z} - {approach_offset_z}"),
                ),
                orientation=tool_orientation,
            ),
        )
    )
    grasp_update = uw_proto.world.ObjectWorldUpdate(
        update_transform=uw_proto.world.UpdateTransformRequest(
            node_a=camera_ref,
            node_b=grasp_ref,
            node_to_update=grasp_ref,
            a_t_b=uw_proto.Pose(
                position=detected_pos,
                orientation=tool_orientation,
            ),
        )
    )

    update_world_action = update_world_skill(
        updates=uw_proto.world.ObjectWorldUpdates(
            updates=[pre_grasp_update, grasp_update]
        )
    )
    update_world_task = bt.Task(
        action=update_world_action,
        name="3. Update Dynamic Grasp & Pre-Grasp Frames",
    )

    return bt.Sequence(
        name=task_name,
        children=[
            capture_task,
            estimate_task,
            update_world_task,
        ],
    )

  def build_estimate_and_update_pose_task(
      self,
      target_object: str = "raw_stock",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds a single-step estimate_and_update_pose task."""
    task_name = name or f"Estimate & Update Pose ({target_object})"
    skills = self._solution.skills

    pkg = (
        id_utils.package_from(pose_estimator_id)
        if id_utils.is_id(pose_estimator_id)
        else "ai.intrinsic"
    )
    est_name = (
        id_utils.name_from(pose_estimator_id)
        if id_utils.is_id(pose_estimator_id)
        else pose_estimator_id
    )
    pose_estimator_proto = pose_estimator_id_pb2.PoseEstimatorId(
        id=est_name,
        package=pkg,
    )

    action = skills.ai.intrinsic.estimate_and_update_pose(
        camera=self._camera_resource,
        pose_estimator=pose_estimator_proto,
        object=target_object,
        perception=self._perception_resource,
    )
    return bt.Task(action=action, name=task_name)


class MockVision(VisionInterface):
  """Mock vision sensor for offline testing."""

  def __init__(self, simulated_pose: Optional[Pose3D] = None) -> None:
    self.simulated_pose = simulated_pose or Pose3D(x=0.15, y=0.25, z=0.71)
    self.capture_count: int = 0
    self.pipeline_count: int = 0

  def build_capture_image_task(self, name: Optional[str] = None) -> bt.Node:
    self.capture_count += 1
    return bt.Sequence(name=name or "Mock Capture Image", children=[])

  def build_perception_and_spawn_task(
      self,
      target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      min_num_instances: int = 1,
      approach_offset_z: float = 0.05,
      parent_object: str = "root",
      pregrasp_frame_name: str = "pre_grasp",
      grasp_frame_name: str = "grasp",
      name: Optional[str] = None,
  ) -> bt.Node:
    self.pipeline_count += 1
    return bt.Sequence(
        name=name or "Mock Perception & Dynamic Grasp Frame Update Pipeline",
        children=[],
    )

  def build_estimate_and_update_pose_task(
      self,
      target_object: str = "raw_stock",
      pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
      name: Optional[str] = None,
  ) -> bt.Node:
    self.pipeline_count += 1
    return bt.Sequence(
        name=name or f"Mock Estimate & Update Pose ({target_object})",
        children=[],
    )
