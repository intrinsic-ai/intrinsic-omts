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

"""Stateless vision and 3D camera hardware interfaces and implementations."""

import abc
import dataclasses
from collections.abc import Sequence
from typing import Any

from intrinsic.assets import id_utils
from intrinsic.perception.proto.v1 import pose_estimator_id_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments, provided

from src.utils.script_utils import create_dwell_task


@dataclasses.dataclass(frozen=True)
class PerceptionConfig:
  """Configuration for 3D camera capture and workpiece pose estimation."""

  camera_name: str = "orbbec_camera"
  sensor_ids: tuple[int, ...] = (1, 4)
  service_name: str = "pose_estimator_service"
  scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5"
  estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator"
  min_instances: int = 1
  max_retries: int = 3
  retry_delay_sec: float = 1.0


def _resolve_resource(
  solution: deployments.Solution,
  target_name: str,
  capability_type: str,
  label: str,
) -> provided.ResourceHandle:
  """Resolves a resource handle from solution resources by name/capability."""
  if isinstance(solution.resources, dict) and target_name in solution.resources:
    return solution.resources[target_name]
  try:
    return solution.resources[target_name]
  except (KeyError, AttributeError, TypeError):
    pass

  res_getter = getattr(solution.resources, "values", lambda: solution.resources)
  for handle in res_getter():
    if hasattr(handle, "types") and capability_type in handle.types:
      return handle
  raise ValueError(f"{label} resource '{target_name}' not found in solution.")


def _get_camera_resource(
  solution: deployments.Solution,
  camera_name: str | None = None,
) -> provided.ResourceHandle:
  """Resolves the camera resource handle."""
  return _resolve_resource(
    solution, camera_name or "orbbec_camera", "CameraConfig", "Camera"
  )


def _get_perception_resource(
  solution: deployments.Solution,
  service_name: str | None = None,
) -> provided.ResourceHandle:
  """Resolves the perception service resource handle."""
  return _resolve_resource(
    solution,
    service_name or "pose_estimator_service",
    "intrinsic_proto.perception.v1.PoseEstimationService",
    "Perception service",
  )


def _build_pose_estimator_proto(
  pose_estimator_id: str,
) -> pose_estimator_id_pb2.PoseEstimatorId:
  """Constructs a PoseEstimatorId proto from an asset or package ID."""
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
  return pose_estimator_id_pb2.PoseEstimatorId(id=est_name, package=pkg)


class VisionInterface(abc.ABC):
  """Stateless abstract interface for perception and pose estimation."""

  @classmethod
  def from_config(
    cls,
    solution: Any,
    config: PerceptionConfig | None = None,
    mock_hardware: bool = False,
  ) -> "VisionInterface":
    """Creates and initializes the vision adapter from configuration."""
    if mock_hardware:
      return MockVision()
    cfg = config or PerceptionConfig()
    return OrbbecVision(
      solution=solution,
      camera_name=cfg.camera_name,
      perception_service_name=cfg.service_name,
      sensor_ids=cfg.sensor_ids,
    )

  @abc.abstractmethod
  def build_capture_image_task(
    self,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    """Builds an image capture task and returns (capture_node, capture_data)."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_estimate_pose_task(
    self,
    capture_data: Any,
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    """Builds a pose estimation task and returns (estimate_node, estimates)."""
    raise NotImplementedError


class OrbbecVision(VisionInterface):
  """Stateless Orbbec 3D camera perception adapter using IOC SBL skills."""

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

    self._camera_resource = _get_camera_resource(solution, camera_name)
    self._perception_resource = _get_perception_resource(
      solution, perception_service_name
    )

  def build_capture_image_task(
    self,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    """Builds an image capture task and returns (capture_node, capture_data)."""
    skills = self._solution.skills
    capture_action = skills.ai.intrinsic.capture_images(
      camera=self._camera_resource,
      sensor_ids=self._sensor_ids,
      log_debug_data=self._log_debug_data,
    )
    capture_task = bt.Task(
      action=capture_action,
      name=name or f"Capture Image ({self._camera_name})",
    )
    if max_tries > 1:
      recovery_task = create_dwell_task(
        dwell_time_sec=retry_delay_sec,
        solution=self._solution,
        task_name=f"Perception Retry Dwell ({retry_delay_sec}s)",
      )
      capture_node: bt.Node = bt.Retry(
        max_tries=max_tries,
        child=capture_task,
        recovery=recovery_task,
        name=f"Retryable Image Capture (max {max_tries} tries)",
      )
    else:
      capture_node = capture_task
    return capture_node, capture_action.result.capture_data

  def build_estimate_pose_task(
    self,
    capture_data: Any,
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    """Builds a pose estimation task and returns (estimate_node, estimates)."""
    skills = self._solution.skills
    pose_estimator_proto = _build_pose_estimator_proto(pose_estimator_id)
    estimate_action = skills.ai.intrinsic.estimate_pose_multi_view(
      camera_1=self._camera_resource,
      camera_2=self._camera_resource,
      camera_3=self._camera_resource,
      camera_4=self._camera_resource,
      perception=self._perception_resource,
      pose_estimator=pose_estimator_proto,
      capture_data=[capture_data],
      min_num_instances=min_num_instances,
      log_debug_data=self._log_debug_data,
    )
    estimate_task = bt.Task(
      action=estimate_action, name=name or "Estimate 6D Workpiece Poses"
    )
    return estimate_task, estimate_action.result.estimates


class MockVision(VisionInterface):
  """Mock vision adapter for unit tests and offline tree generation."""

  def __init__(self, solution: Any = None, **kwargs: Any) -> None:
    del solution, kwargs
    self.capture_count: int = 0
    self.pipeline_count: int = 0
    self.last_max_tries: int = 0
    self.last_retry_delay_sec: float = 0.0

  def build_capture_image_task(
    self,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    self.capture_count += 1
    self.last_max_tries = max_tries
    self.last_retry_delay_sec = retry_delay_sec
    return (
      bt.Task(
        action=bt.PythonScript(function_body="pass"),
        name=name or "Mock Capture Image",
      ),
      "mock_capture_data",
    )

  def build_estimate_pose_task(
    self,
    capture_data: Any,
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    del capture_data, pose_estimator_id, min_num_instances
    self.pipeline_count += 1
    return (
      bt.Task(
        action=bt.PythonScript(function_body="pass"),
        name=name or "Mock Estimate Pose",
      ),
      "mock_estimates",
    )


__all__ = [
  "MockVision",
  "OrbbecVision",
  "PerceptionConfig",
  "VisionInterface",
]
