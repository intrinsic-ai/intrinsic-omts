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

"""Unit tests for tools/pose_estimation/run_pose_estimation.py."""

from typing import Any
from unittest import mock

from absl.testing import absltest

from tools.pose_estimation.run_pose_estimation import (
  create_pose_estimation_pipeline,
  get_camera_resource,
  get_perception_service_resource,
)


class _FakeHandle:
  """Minimal resource handle stub with name and capability types."""

  def __init__(self, name: str, types: list[str]) -> None:
    self.name = name
    self.types = types


class _StrictSdkResources:
  """Mimics intrinsic.solutions.internal.resources.Resources.

  Specifically:
  - Defines __getitem__ (raising KeyError on unknown names or integer index 0)
  - Defines __getattr__ (raising KeyError on unknown names)
  - Defines __dir__ returning registered resource names
  - Does NOT define __iter__, so calling list(resources) directly raises
    KeyError(0) via Python's sequence protocol.
  """

  def __init__(self, handles: list[_FakeHandle]) -> None:
    self._map = {h.name: h for h in handles}

  def __getitem__(self, key: Any) -> _FakeHandle:
    if key not in self._map:
      raise KeyError(f"Resource {key} not registered")
    return self._map[key]

  def __getattr__(self, key: str) -> _FakeHandle:
    if key not in self._map:
      raise KeyError(f"Resource {key} not registered")
    return self._map[key]

  def __dir__(self) -> list[str]:
    return sorted(self._map.keys())


class RunPoseEstimationTest(absltest.TestCase):
  """Tests resource discovery and pipeline assembly in run_pose_estimation."""

  def test_get_camera_resource_auto_detects_on_strict_sdk_resources(
    self,
  ) -> None:
    """Verifies auto-detection works on SDK Resources without crashing on list()."""
    cam_handle = _FakeHandle("custom_orbbec", ["CameraConfig"])
    ur_handle = _FakeHandle(
      "ur_module", ["intrinsic_proto.world.RobotCalibrationDataService"]
    )
    mock_solution = mock.MagicMock()
    mock_solution.resources = _StrictSdkResources([cam_handle, ur_handle])

    # Calling list(mock_solution.resources) directly raises KeyError(0)
    with self.assertRaises(KeyError):
      list(mock_solution.resources)

    resolved = get_camera_resource(mock_solution, camera_name=None)
    self.assertIs(resolved, cam_handle)

  def test_get_camera_resource_explicit_and_missing(self) -> None:
    cam1 = _FakeHandle("cam_a", ["CameraConfig"])
    cam2 = _FakeHandle("cam_b", ["CameraConfig"])
    mock_solution = mock.MagicMock()
    mock_solution.resources = _StrictSdkResources([cam1, cam2])

    self.assertIs(get_camera_resource(mock_solution, "cam_b"), cam2)

    with self.assertRaisesRegex(
      ValueError, "Camera resource 'nonexistent' not found"
    ):
      get_camera_resource(mock_solution, "nonexistent")

    # Ambiguous auto-detection when multiple CameraConfig resources exist
    with self.assertRaisesRegex(ValueError, "Multiple camera resources found"):
      get_camera_resource(mock_solution, camera_name=None)

  def test_get_perception_service_resource_auto_detect_and_errors(self) -> None:
    pe_handle = _FakeHandle(
      "custom_pose_service",
      ["intrinsic_proto.perception.v1.PoseEstimationService"],
    )
    mock_solution = mock.MagicMock()
    mock_solution.resources = _StrictSdkResources([pe_handle])

    resolved = get_perception_service_resource(mock_solution, service_name=None)
    self.assertIs(resolved, pe_handle)

    mock_solution.resources = _StrictSdkResources([])
    with self.assertRaisesRegex(
      ValueError, "No service resource with 'PoseEstimationService'"
    ):
      get_perception_service_resource(mock_solution, service_name=None)

  def test_create_pose_estimation_pipeline_valid_and_invalid_id(self) -> None:
    mock_solution = mock.MagicMock()
    cam_handle = _FakeHandle("orbbec_camera", ["CameraConfig"])
    pe_handle = _FakeHandle(
      "pose_estimator_service",
      ["intrinsic_proto.perception.v1.PoseEstimationService"],
    )

    with self.assertRaisesRegex(ValueError, "Invalid pose estimator ID"):
      create_pose_estimation_pipeline(
        solution=mock_solution,
        camera_resource=cam_handle,
        perception_resource=pe_handle,
        pose_estimator_id="invalid_id_without_package",
      )

    capture_skill, estimate_skill = create_pose_estimation_pipeline(
      solution=mock_solution,
      camera_resource=cam_handle,
      perception_resource=pe_handle,
      pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator",
      sensor_ids=(1, 4),
      min_num_instances=2,
      log_debug_data=False,
      timeout_sec=15,
    )
    self.assertIsNotNone(capture_skill)
    self.assertIsNotNone(estimate_skill)

    mock_solution.skills.ai.intrinsic.capture_images.assert_called_once_with(
      camera=cam_handle,
      log_debug_data=False,
      sensor_ids=[1, 4],
    )
    est_kwargs = mock_solution.skills.ai.intrinsic.estimate_pose_multi_view.call_args.kwargs
    self.assertEqual(est_kwargs["pose_estimator"].package, "ai.intrinsic")
    self.assertEqual(
      est_kwargs["pose_estimator"].id, "raw_stock_2x3x5_estimator"
    )
    self.assertEqual(est_kwargs["min_num_instances"], 2)
    self.assertEqual(est_kwargs["inference_timeout_sec"], 15)


if __name__ == "__main__":
  absltest.main()
