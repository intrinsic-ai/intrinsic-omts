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

"""Vision and 3D camera hardware interfaces and implementations."""

import abc
import dataclasses
from collections.abc import Sequence
from typing import Any

from intrinsic.assets import id_utils
from intrinsic.perception.proto.v1 import pose_estimator_id_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments, provided
from intrinsic.solutions import proto_building as pb

from src.core.types import Pose3D
from src.utils import dynamic_frame_calculator
from src.utils.script_utils import (
  ScriptArg,
  create_dwell_task,
  load_python_script,
)


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
  close_gripper_before_perception: bool = False


def _resolve_resource(
  solution: deployments.Solution,
  target_name: str,
  capability_type: str,
  label: str,
) -> provided.ResourceHandle:
  """Resolves a resource handle from solution resources by name or capability."""
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
  """Constructs a PoseEstimatorId protobuf message from an asset or package ID."""
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


def _build_dynamic_frame_signature(
  proto_builder: Any,
  approach_offset_z: float,
  parent_object: str,
  pregrasp_frame_name: str,
  grasp_frame_name: str,
  camera_name: str,
  target_scene_object_id: str,
  estimates: Any,
  min_safe_z: float,
) -> Any:
  """Builds the proto_builder signature for dynamic frame calculation."""
  if proto_builder is None:
    return None
  args = [
    ScriptArg(1, "approach_offset_z", "float", approach_offset_z),
    ScriptArg(2, "parent_object", "string", parent_object),
    ScriptArg(3, "pregrasp_frame_name", "string", pregrasp_frame_name),
    ScriptArg(4, "grasp_frame_name", "string", grasp_frame_name),
    ScriptArg(5, "camera_name", "string", camera_name),
    ScriptArg(6, "target_scene_object_id", "string", target_scene_object_id),
    ScriptArg(
      7,
      "estimates",
      "intrinsic_proto.perception.v1.PoseEstimateInRoot",
      estimates,
      repeated=True,
    ),
    ScriptArg(8, "min_safe_z", "float", min_safe_z),
  ]
  return proto_builder.create_signature_with_args(
    parameters=pb.MessageSpec(fields=[a.to_field_spec() for a in args])
  )


class VisionInterface(abc.ABC):
  """Abstract interface for perception acquisition and pose estimation."""

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
  def build_capture_task(self, name: str | None = None) -> bt.Node:
    """Builds a task to trigger camera image acquisition."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_estimate_task(
    self,
    target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    approach_offset_z: float = 0.05,
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    min_safe_z: float = 0.95,
    max_tries: int = 1,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task to estimate poses from captured data and update dynamic frames."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_perception_and_spawn_task(
    self,
    target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    approach_offset_z: float = 0.05,
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    min_safe_z: float = 0.95,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a composite task to capture RGB-D, estimate poses, and update frames."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_estimate_and_update_pose_task(
    self,
    target_object: str = "raw_stock",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    name: str | None = None,
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
    self._last_capture_action: Any | None = None

    self._camera_resource = _get_camera_resource(solution, camera_name)
    self._perception_resource = _get_perception_resource(
      solution, perception_service_name
    )

  def build_capture_task(self, name: str | None = None) -> bt.Node:
    """Builds a single image capture task."""
    task_name = name or f"Capture Image ({self._camera_name})"
    self._last_capture_action = (
      self._solution.skills.ai.intrinsic.capture_images(
        camera=self._camera_resource,
        sensor_ids=self._sensor_ids,
        log_debug_data=self._log_debug_data,
      )
    )
    return bt.Task(action=self._last_capture_action, name=task_name)

  def build_estimate_task(
    self,
    target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    approach_offset_z: float = 0.05,
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    min_safe_z: float = 0.95,
    max_tries: int = 1,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> bt.Node:
    """Builds task estimating poses from captured data and updating dynamic frames."""
    task_name = name or "Estimate 6D Workpiece Poses & Update Frames"
    skills = self._solution.skills

    pose_estimator_proto = _build_pose_estimator_proto(pose_estimator_id)

    capture_data = (
      [self._last_capture_action.result.capture_data]
      if self._last_capture_action is not None
      else []
    )
    estimate_action = skills.ai.intrinsic.estimate_pose_multi_view(
      camera_1=self._camera_resource,
      camera_2=self._camera_resource,
      camera_3=self._camera_resource,
      camera_4=self._camera_resource,
      perception=self._perception_resource,
      pose_estimator=pose_estimator_proto,
      capture_data=capture_data,
      min_num_instances=min_num_instances,
      log_debug_data=self._log_debug_data,
    )
    estimate_task = bt.Task(
      action=estimate_action, name="Estimate 6D Workpiece Poses"
    )

    signature = _build_dynamic_frame_signature(
      proto_builder=getattr(self._solution, "proto_builder", None),
      approach_offset_z=approach_offset_z,
      parent_object=parent_object,
      pregrasp_frame_name=pregrasp_frame_name,
      grasp_frame_name=grasp_frame_name,
      camera_name=self._camera_name,
      target_scene_object_id=target_scene_object_id,
      estimates=estimate_action.result.estimates,
      min_safe_z=min_safe_z,
    )

    calc_script = bt.PythonScript(
      signature_with_args=signature,
      function_body=load_python_script(
        dynamic_frame_calculator.calculate_and_update_dynamic_frames,
      ),
    )
    calc_task = bt.Task(
      action=calc_script,
      name="Calculate & Update Dynamic Grasp & Pre-Grasp Frames",
    )

    estimate_seq = bt.Sequence(
      name=task_name,
      children=[estimate_task, calc_task],
    )
    if max_tries > 1:
      recovery_task = create_dwell_task(
        dwell_time_sec=retry_delay_sec,
        solution=self._solution,
        task_name=f"Perception Retry Dwell ({retry_delay_sec}s)",
      )
      return bt.Retry(
        max_tries=max_tries,
        child=estimate_seq,
        recovery=recovery_task,
        name=f"Retryable Pose Estimation (max {max_tries} tries)",
      )
    return estimate_seq

  def build_perception_and_spawn_task(
    self,
    target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    approach_offset_z: float = 0.05,
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    min_safe_z: float = 0.95,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> bt.Node:
    """Builds composite pipeline to capture RGB-D and update dynamic frames."""
    task_name = name or "Perception & Dynamic Grasp Frame Update Pipeline"
    skills = self._solution.skills

    capture_action = skills.ai.intrinsic.capture_images(
      camera=self._camera_resource,
      sensor_ids=self._sensor_ids,
      log_debug_data=self._log_debug_data,
    )
    capture_task = bt.Task(
      action=capture_action, name="1. Capture RGB-D Images"
    )

    pose_estimator_proto = _build_pose_estimator_proto(pose_estimator_id)

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

    signature = _build_dynamic_frame_signature(
      proto_builder=getattr(self._solution, "proto_builder", None),
      approach_offset_z=approach_offset_z,
      parent_object=parent_object,
      pregrasp_frame_name=pregrasp_frame_name,
      grasp_frame_name=grasp_frame_name,
      camera_name=self._camera_name,
      target_scene_object_id=target_scene_object_id,
      estimates=estimate_action.result.estimates,
      min_safe_z=min_safe_z,
    )

    calc_script = bt.PythonScript(
      signature_with_args=signature,
      function_body=load_python_script(
        dynamic_frame_calculator.calculate_and_update_dynamic_frames,
      ),
    )
    calc_task = bt.Task(
      action=calc_script,
      name="3. Calculate & Update Dynamic Grasp & Pre-Grasp Frames",
    )

    acquisition_seq = bt.Sequence(
      name="Perception Capture & Estimation Sequence",
      children=[capture_task, estimate_task],
    )
    if max_tries > 1:
      recovery_task = create_dwell_task(
        dwell_time_sec=retry_delay_sec,
        solution=self._solution,
        task_name=f"Perception Retry Dwell ({retry_delay_sec}s)",
      )
      perception_node = bt.Retry(
        max_tries=max_tries,
        child=acquisition_seq,
        recovery=recovery_task,
        name=f"Retryable Perception Acquisition (max {max_tries} tries)",
      )
    else:
      perception_node = acquisition_seq

    return bt.Sequence(
      name=task_name,
      children=[
        perception_node,
        calc_task,
      ],
    )

  def build_estimate_and_update_pose_task(
    self,
    target_object: str = "raw_stock",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    name: str | None = None,
  ) -> bt.Node:
    """Builds a single-step estimate_and_update_pose task."""
    task_name = name or f"Estimate & Update Pose ({target_object})"
    skills = self._solution.skills

    pose_estimator_proto = _build_pose_estimator_proto(pose_estimator_id)

    action = skills.ai.intrinsic.estimate_and_update_pose(
      camera=self._camera_resource,
      pose_estimator=pose_estimator_proto,
      object=target_object,
      perception=self._perception_resource,
    )
    return bt.Task(action=action, name=task_name)


class MockVision(VisionInterface):
  """Mock vision sensor for offline testing."""

  def __init__(self, simulated_pose: Pose3D | None = None) -> None:
    self.simulated_pose = simulated_pose or Pose3D(x=0.15, y=0.25, z=0.71)
    self.capture_count: int = 0
    self.pipeline_count: int = 0
    self.last_max_tries: int = 3
    self.last_retry_delay_sec: float = 1.0

  def build_capture_task(self, name: str | None = None) -> bt.Node:
    self.capture_count += 1
    return bt.Sequence(name=name or "Mock Capture Image", children=[])

  def build_estimate_task(
    self,
    target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    approach_offset_z: float = 0.05,
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    min_safe_z: float = 0.95,
    max_tries: int = 1,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> bt.Node:
    self.pipeline_count += 1
    self.last_max_tries = int(max_tries)
    self.last_retry_delay_sec = float(retry_delay_sec)
    return bt.Sequence(
      name=name or "Mock Estimate & Dynamic Frame Calculation",
      children=[],
    )

  def build_perception_and_spawn_task(
    self,
    target_scene_object_id: str = "ai.intrinsic.raw_stock_2x3x5",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    min_num_instances: int = 1,
    approach_offset_z: float = 0.05,
    parent_object: str = "root",
    pregrasp_frame_name: str = "infeed_pre_grasp",
    grasp_frame_name: str = "infeed_grasp",
    min_safe_z: float = 0.95,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    name: str | None = None,
  ) -> bt.Node:
    self.pipeline_count += 1
    self.last_max_tries = int(max_tries)
    self.last_retry_delay_sec = float(retry_delay_sec)
    return bt.Sequence(
      name=name or "Mock Perception & Dynamic Grasp Frame Update Pipeline",
      children=[],
    )

  def build_estimate_and_update_pose_task(
    self,
    target_object: str = "raw_stock",
    pose_estimator_id: str = "ai.intrinsic.raw_stock_2x3x5_estimator",
    name: str | None = None,
  ) -> bt.Node:
    self.pipeline_count += 1
    return bt.Sequence(
      name=name or f"Mock Estimate & Update Pose ({target_object})",
      children=[],
    )
