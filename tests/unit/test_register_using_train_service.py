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

"""Unit tests for register_using_train_service helper functions."""

from unittest import mock

from absl.testing import absltest
from google.longrunning import operations_pb2
from google.rpc import status_pb2
from incode.intrinsic_perception.intrinsic.perception.service.ioc_train_service.proto import (
  ioc_pose_estimator_params_pb2,
)
from intrinsic.assets.proto import installed_assets_pb2, view_pb2
from intrinsic.perception.proto.v1 import train_service_pb2
from intrinsic.scene.proto.v1 import scene_object_pb2

from tools.pose_estimation import register_using_train_service


class RegisterUsingTrainServiceTest(absltest.TestCase):
  """Tests for pose estimator registration using IOC Train Service."""

  def setUp(self) -> None:
    super().setUp()
    self.mock_assets_stub = mock.MagicMock()
    self.mock_train_stub = mock.MagicMock()

  def test_get_create_training_job_request(self) -> None:
    """Verifies CreateTrainingJobRequest creation and packed inference params."""
    scene_obj = scene_object_pb2.SceneObject()
    req = register_using_train_service.get_create_training_job_request(
      scene_object=scene_obj,
      part_name="raw_stock_2x3x5",
      pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator",
      refinement_iters=3,
      confidence_threshold=0.6,
      visibility_threshold=0.7,
      min_distance=0.4,
      max_distance=1.2,
    )

    self.assertEqual(req.asset_metadata.asset_name, "raw_stock_2x3x5_estimator")
    self.assertEqual(req.asset_metadata.id_version.id.package, "ai.intrinsic")
    self.assertEqual(
      req.asset_metadata.id_version.id.name, "raw_stock_2x3x5_estimator"
    )
    self.assertLen(req.pose_estimation_config.targets, 1)
    target = req.pose_estimation_config.targets[0]
    self.assertEqual(target.id, "raw_stock_2x3x5")
    self.assertAlmostEqual(target.pose_range.min_distance, 0.4)
    self.assertAlmostEqual(target.pose_range.max_distance, 1.2)

    unpacked_params = ioc_pose_estimator_params_pb2.IocPoseEstimatorParams()
    self.assertTrue(
      req.pose_estimation_config.inference_params.Unpack(unpacked_params)
    )
    self.assertEqual(unpacked_params.refinement_iters, 3)
    self.assertAlmostEqual(unpacked_params.confidence_threshold, 0.6)
    self.assertAlmostEqual(unpacked_params.visibility_threshold, 0.7)

  def test_get_scene_object(self) -> None:
    """Verifies get_scene_object queries InstalledAssetsStub with full view."""
    expected_scene_obj = scene_object_pb2.SceneObject()
    installed_asset = installed_assets_pb2.InstalledAsset()
    installed_asset.deployment_data.scene_object.manifest.assets.scene_object_model.CopyFrom(
      expected_scene_obj
    )
    self.mock_assets_stub.GetInstalledAsset.return_value = installed_asset

    result = register_using_train_service.get_scene_object(
      installed_assets_stub=self.mock_assets_stub,
      package_name="ai.intrinsic",
      scene_object_name="raw_stock_2x3x5",
    )

    self.assertEqual(result, expected_scene_obj)
    self.mock_assets_stub.GetInstalledAsset.assert_called_once()
    req = self.mock_assets_stub.GetInstalledAsset.call_args[0][0]
    self.assertEqual(req.id.package, "ai.intrinsic")
    self.assertEqual(req.id.name, "raw_stock_2x3x5")
    self.assertEqual(req.view, view_pb2.AssetViewType.ASSET_VIEW_TYPE_FULL)

  def test_training_job_completed_in_progress(self) -> None:
    """Verifies training_job_completed returns False when operation is running."""
    self.mock_train_stub.GetTrainingJob.return_value = operations_pb2.Operation(
      name="operations/ioc-train-123", done=False
    )

    self.assertFalse(
      register_using_train_service.training_job_completed(
        "operations/ioc-train-123", self.mock_train_stub
      )
    )

  def test_training_job_completed_success(self) -> None:
    """Verifies training_job_completed returns True when operation succeeded."""
    self.mock_train_stub.GetTrainingJob.return_value = operations_pb2.Operation(
      name="operations/ioc-train-123", done=True
    )

    self.assertTrue(
      register_using_train_service.training_job_completed(
        "operations/ioc-train-123", self.mock_train_stub
      )
    )

  def test_training_job_completed_raises_on_error(self) -> None:
    """Verifies training_job_completed raises ValueError when operation failed."""
    op = operations_pb2.Operation(
      name="operations/ioc-train-123",
      done=True,
      error=status_pb2.Status(code=13, message="Internal error"),
    )
    self.mock_train_stub.GetTrainingJob.return_value = op

    with self.assertRaises(ValueError):
      register_using_train_service.training_job_completed(
        "operations/ioc-train-123", self.mock_train_stub
      )

  def test_save_pose_estimator(self) -> None:
    """Verifies save_pose_estimator invokes SaveTrainingJob with asset ID."""
    expected_response = train_service_pb2.SaveTrainingJobResponse()
    self.mock_train_stub.SaveTrainingJob.return_value = expected_response

    resp = register_using_train_service.save_pose_estimator(
      training_job_name="operations/ioc-train-123",
      pose_estimator_name="raw_stock_2x3x5_estimator",
      package_name="ai.intrinsic",
      train_service_stub=self.mock_train_stub,
    )

    self.assertEqual(resp, expected_response)
    self.mock_train_stub.SaveTrainingJob.assert_called_once()
    req = self.mock_train_stub.SaveTrainingJob.call_args[0][0]
    self.assertEqual(req.name, "operations/ioc-train-123")
    self.assertEqual(req.asset_id.package, "ai.intrinsic")
    self.assertEqual(req.asset_id.name, "raw_stock_2x3x5_estimator")

  def test_delete_training_job(self) -> None:
    """Verifies delete_training_job invokes DeleteTrainingJob."""
    register_using_train_service.delete_training_job(
      training_job_name="operations/ioc-train-123",
      train_service_stub=self.mock_train_stub,
    )

    self.mock_train_stub.DeleteTrainingJob.assert_called_once()
    req = self.mock_train_stub.DeleteTrainingJob.call_args[0][0]
    self.assertEqual(req.name, "operations/ioc-train-123")


if __name__ == "__main__":
  absltest.main()
