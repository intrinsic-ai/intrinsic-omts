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

"""Unit tests for the store_joint_config developer CLI tool."""

from unittest import mock

from absl.testing import absltest

from tools.jogging.store_joint_config import (
  parse_args,
  store_joint_configuration,
)


class StoreJointConfigTest(absltest.TestCase):
  def test_parse_args_defaults(self):
    args = parse_args(["home"])
    self.assertEqual(args.name, "home")
    self.assertEqual(args.address, "localhost:17080")
    self.assertEqual(args.robot_name, "ur_module")

  def test_parse_args_custom_values(self):
    args = parse_args(
      [
        "view_pose",
        "--address",
        "192.168.1.50:17080",
        "--robot_name",
        "custom_ur",
      ]
    )
    self.assertEqual(args.name, "view_pose")
    self.assertEqual(args.address, "192.168.1.50:17080")
    self.assertEqual(args.robot_name, "custom_ur")

  @mock.patch("tools.jogging.store_joint_config.worlds.ObjectWorld.connect")
  def test_store_joint_configuration_success(self, mock_world_connect):
    mock_solution = mock.MagicMock()
    mock_robot = mock.MagicMock()
    mock_robot.joint_positions = [
      0.71518,
      -1.10736,
      -2.30650,
      -0.98339,
      1.58574,
      3.57518,
    ]
    mock_solution.world.ur_module = mock_robot

    mock_init_world = mock.MagicMock()
    mock_init_robot = mock.MagicMock()
    mock_init_world.ur_module = mock_init_robot
    mock_world_connect.return_value = mock_init_world

    curr, norm = store_joint_configuration(
      solution=mock_solution,
      name="view_joints",
      robot_name="ur_module",
    )

    self.assertEqual(curr, mock_robot.joint_positions)
    # 3.57518 normalized is 3.57518 - 2*pi ≈ -2.70800
    self.assertAlmostEqual(norm[5], 3.57518 - 6.283185307179586, places=4)
    mock_solution.world.update_kinematic_object_joint_configurations.assert_called_once()
    mock_world_connect.assert_called_once_with(
      world_id="init_world",
      grpc_channel=mock_solution.grpc_channel,
    )
    mock_init_world.update_kinematic_object_joint_configurations.assert_called_once()

  @mock.patch("tools.jogging.store_joint_config.worlds.ObjectWorld.connect")
  def test_store_joint_configuration_init_world_failure_handled(
    self, mock_world_connect
  ):
    mock_solution = mock.MagicMock()
    mock_robot = mock.MagicMock()
    mock_robot.joint_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    mock_solution.world.ur_module = mock_robot

    mock_world_connect.side_effect = RuntimeError("init_world connection error")

    curr, norm = store_joint_configuration(
      solution=mock_solution,
      name="home",
      robot_name="ur_module",
    )

    self.assertEqual(curr, mock_robot.joint_positions)
    for n, e in zip(norm, mock_robot.joint_positions, strict=False):
      self.assertAlmostEqual(n, e, places=5)
    mock_solution.world.update_kinematic_object_joint_configurations.assert_called_once()


if __name__ == "__main__":
  absltest.main()
