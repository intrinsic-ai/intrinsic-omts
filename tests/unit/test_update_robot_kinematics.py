"""Unit tests for update_robot_kinematics helper functions."""

from unittest import mock

from absl.testing import absltest
import grpc
from intrinsic.world.service.robot_calibration import robot_update_service_pb2
from tools.calibration import update_robot_kinematics


class UpdateRobotKinematicsTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.mock_stub = mock.MagicMock()

  def test_check_kinematics_match_returns_true(self):
    self.mock_stub.CheckWorldMatchesHardwareKinematics.return_value = (
        robot_update_service_pb2.CheckWorldMatchesHardwareKinematicsResponse(
            world_matches_hardware_kinematics=True
        )
    )

    result = update_robot_kinematics.check_kinematics_match(
        self.mock_stub, resource_id="ur_module", world_id="world"
    )

    self.assertTrue(result)
    self.mock_stub.CheckWorldMatchesHardwareKinematics.assert_called_once()
    req = self.mock_stub.CheckWorldMatchesHardwareKinematics.call_args[0][0]
    self.assertEqual(req.resource_id, "ur_module")
    self.assertEqual(req.world_id, "world")

  def test_check_kinematics_match_returns_false(self):
    self.mock_stub.CheckWorldMatchesHardwareKinematics.return_value = (
        robot_update_service_pb2.CheckWorldMatchesHardwareKinematicsResponse(
            world_matches_hardware_kinematics=False
        )
    )

    result = update_robot_kinematics.check_kinematics_match(
        self.mock_stub, resource_id="ur_module", world_id="world"
    )

    self.assertFalse(result)

  def test_update_robot_kinematics_success(self):
    self.mock_stub.UpdateRobotKinematics.return_value = (
        robot_update_service_pb2.RobotUpdateResponse()
    )

    update_robot_kinematics.update_robot_kinematics(
        self.mock_stub, resource_id="ur_module", world_id="world", timeout=30.0
    )

    self.mock_stub.UpdateRobotKinematics.assert_called_once()
    req = self.mock_stub.UpdateRobotKinematics.call_args[0][0]
    self.assertEqual(req.resource_id, "ur_module")
    self.assertEqual(req.world_id, "world")

  def test_update_robot_kinematics_raises_on_rpc_error(self):
    self.mock_stub.UpdateRobotKinematics.side_effect = grpc.RpcError(
        "Service unavailable"
    )

    with self.assertRaises(grpc.RpcError):
      update_robot_kinematics.update_robot_kinematics(
          self.mock_stub, resource_id="ur_module", world_id="world"
      )


if __name__ == "__main__":
  absltest.main()
