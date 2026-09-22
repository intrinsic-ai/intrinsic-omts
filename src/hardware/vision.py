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

"""Vision and 3D camera hardware interfaces and Orbbec SBL implementation."""

import abc
from typing import Any
from unittest import mock

from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments
from intrinsic.solutions import proto_building as pb

from src.core.config import AppConfig, VisionConfig
from src.utils import dynamic_frame_calculator, math_utils
from src.utils.execution_utils import (
  build_pose_estimator_proto,
  resolve_resource,
)
from src.utils.script_utils import create_dwell_task, load_python_script


def _build_dynamic_frame_signature(
  proto_builder: Any,
  camera_name: str,
  estimates: Any,
  config: AppConfig,
) -> Any | None:
  """Builds the protobuf MessageSpec signature for dynamic_frame_calculator."""
  if proto_builder is None or isinstance(proto_builder, mock.MagicMock):
    return None
  first_est = estimates[0].root_t_target
  fields = [
    pb.FieldSpec(
      type="float",
      name="pos_x",
      number=1,
      arg=first_est.position.x,
    ),
    pb.FieldSpec(
      type="float",
      name="pos_y",
      number=2,
      arg=first_est.position.y,
    ),
    pb.FieldSpec(
      type="float",
      name="pos_z",
      number=3,
      arg=first_est.position.z,
    ),
    pb.FieldSpec(
      type="float",
      name="ori_x",
      number=4,
      arg=first_est.orientation.x,
    ),
    pb.FieldSpec(
      type="float",
      name="ori_y",
      number=5,
      arg=first_est.orientation.y,
    ),
    pb.FieldSpec(
      type="float",
      name="ori_z",
      number=6,
      arg=first_est.orientation.z,
    ),
    pb.FieldSpec(
      type="float",
      name="ori_w",
      number=7,
      arg=first_est.orientation.w,
    ),
    pb.FieldSpec(
      type="float",
      name="approach_offset_z",
      number=8,
      arg=float(config.cycle.approach_offset_z),
    ),
    pb.FieldSpec(
      type="string",
      name="parent_object",
      number=9,
      arg=config.frames.parent_object,
    ),
    pb.FieldSpec(
      type="string",
      name="pregrasp_frame_name",
      number=10,
      arg=config.frames.pregrasp_frame,
    ),
    pb.FieldSpec(
      type="string",
      name="grasp_frame_name",
      number=11,
      arg=config.frames.grasp_frame,
    ),
    pb.FieldSpec(type="string", name="camera_name", number=12, arg=camera_name),
    pb.FieldSpec(
      type="string",
      name="target_scene_object_id",
      number=13,
      arg=config.workpiece.object_name,
    ),
    pb.FieldSpec(
      type="float",
      name="min_safe_z",
      number=14,
      arg=float(config.vision.min_safe_z),
    ),
    pb.FieldSpec(
      type="string",
      name="tool_object_name",
      number=15,
      arg=config.robot.tool_object_name,
    ),
    pb.FieldSpec(
      type="string",
      name="tool_frame_name",
      number=16,
      arg=config.robot.tool_frame_name,
    ),
  ]
  return proto_builder.create_signature_with_args(
    parameters=pb.MessageSpec(fields=fields)
  )


class VisionInterface(abc.ABC):
  """Stateless abstract interface for perception and pose estimation."""

  @abc.abstractmethod
  def build_capture_image_task(
    self,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    task_name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    """Builds an image capture task and returns (capture_node, capture_data)."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_estimate_pose_task(
    self,
    capture_data: Any,
    config: VisionConfig | None = None,
    task_name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    """Builds a pose estimation task and returns (estimate_node, estimates)."""
    raise NotImplementedError

  @abc.abstractmethod
  def build_update_grasp_frames_task(
    self,
    estimates: Any,
    config: AppConfig,
    task_name: str | None = None,
  ) -> bt.Node:
    """Builds a task updating grasp frames and workpiece pose in ObjectWorld."""
    raise NotImplementedError


class OrbbecVision(VisionInterface):
  """Stateless Orbbec 3D camera perception adapter using IOC SBL skills."""

  def __init__(
    self,
    solution: deployments.Solution,
    config: VisionConfig,
  ) -> None:
    """Initializes the OrbbecVision adapter from a VisionConfig."""
    self._solution = solution
    self._config = config
    self._camera_name = config.camera_name
    self._perception_service_name = config.perception_service_name
    self._pose_estimator_id = config.pose_estimator_id
    self._scene_object_id = config.scene_object_id
    self._sensor_ids = list(config.sensor_ids)
    self._min_num_instances = config.min_num_instances
    self._min_safe_z = config.min_safe_z
    self._log_debug_data = config.log_debug_data

    self._camera_resource = resolve_resource(
      solution,
      self._camera_name,
      "CameraConfig",
      "Camera",
    )
    self._perception_resource = resolve_resource(
      solution,
      self._perception_service_name,
      "PoseEstimationService",
      "Perception service",
    )

  def build_capture_image_task(
    self,
    max_tries: int = 3,
    retry_delay_sec: float = 1.0,
    task_name: str | None = None,
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
      name=task_name or f"Capture Image ({self._camera_name})",
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
    config: VisionConfig | None = None,
    task_name: str | None = None,
  ) -> tuple[bt.Node, Any]:
    """Builds a pose estimation task and returns (estimate_node, estimates)."""
    vision_cfg = config or self._config
    skills = self._solution.skills
    pose_estimator_proto = build_pose_estimator_proto(
      vision_cfg.pose_estimator_id
    )
    estimate_action = skills.ai.intrinsic.estimate_pose_multi_view(
      camera_1=self._camera_resource,
      camera_2=self._camera_resource,
      camera_3=self._camera_resource,
      camera_4=self._camera_resource,
      perception=self._perception_resource,
      pose_estimator=pose_estimator_proto,
      capture_data=[capture_data],
      min_num_instances=vision_cfg.min_num_instances,
      log_debug_data=vision_cfg.log_debug_data,
    )
    estimate_task = bt.Task(
      action=estimate_action, name=task_name or "Estimate 6D Workpiece Poses"
    )
    return estimate_task, estimate_action.result.estimates

  def build_update_grasp_frames_task(
    self,
    estimates: Any,
    config: AppConfig,
    task_name: str | None = None,
  ) -> bt.Node:
    """Builds a task updating grasp frames and workpiece pose in ObjectWorld."""
    signature = _build_dynamic_frame_signature(
      proto_builder=getattr(self._solution, "proto_builder", None),
      camera_name=self._camera_name,
      estimates=estimates,
      config=config,
    )
    calc_script = bt.PythonScript(
      signature_with_args=signature,
      function_body=load_python_script(
        dynamic_frame_calculator.calculate_and_update_dynamic_frames,
        preludes=(math_utils,),
      ),
    )
    return bt.Task(
      action=calc_script,
      name=task_name or "Calculate & Update Dynamic Grasp & Pre-Grasp Frames",
    )
