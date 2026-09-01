"""Vision and 3D camera hardware interfaces and implementations."""

import abc
from typing import Any, Optional, Sequence

from absl import logging
from intrinsic.assets import id_utils
from intrinsic.assets.proto import id_pb2
from intrinsic.math.python import proto_conversion
from intrinsic.perception.public.proto.v1 import pose_estimator_id_pb2
from intrinsic.solutions import behavior_tree as bt
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
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds a composite task to capture RGB-D, estimate 6D poses, and spawn world objects."""
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
      name: Optional[str] = None,
  ) -> bt.Node:
    """Builds the pipeline to capture, estimate 6D pose, and spawn objects in the world."""
    task_name = name or "Perception & Object Spawning Pipeline"

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

    # 3. Spawn Detected Objects via create_object skill
    obj_pkg = (
        id_utils.package_from(target_scene_object_id)
        if id_utils.is_id(target_scene_object_id)
        else "ai.intrinsic"
    )
    obj_name = (
        id_utils.name_from(target_scene_object_id)
        if id_utils.is_id(target_scene_object_id)
        else target_scene_object_id
    )

    object_id_proto = id_pb2.Id(package=obj_pkg, name=obj_name)

    root_ref = object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            object=object_world_refs_pb2.ObjectReferenceByName(object_name="root")
        )
    )

    create_object_action = skills.ai.intrinsic.create_object(
        object_to_create=object_id_proto,
        create_at_frame=root_ref,
        object_naming_schema={
            "prefix": "workpiece",
            "suffix": 1,  # INDEX
        },
        create_in_world=1,  # BELIEF
    )
    create_object_task = bt.Task(
        action=create_object_action, name="3. Spawn Workpiece World Objects"
    )

    return bt.Sequence(
        name=task_name,
        children=[capture_task, estimate_task, create_object_task],
    )


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
      name: Optional[str] = None,
  ) -> bt.Node:
    self.pipeline_count += 1
    return bt.Sequence(
        name=name or "Mock Perception & Object Spawning Pipeline", children=[]
    )
