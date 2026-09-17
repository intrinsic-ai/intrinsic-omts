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

"""Unit tests for tools/jogging/move_to_joint.py CLI tool."""

import inspect
from unittest import mock

from absl.testing import absltest

from src.core.types import JointPosition
from tools.jogging.move_to_joint import (
  list_available_joint_configs,
  move_robot_to_joint,
  parse_args,
  parse_joint_values,
  resolve_joint_target,
)


class MoveToJointTest(absltest.TestCase):
  def test_no_disable_collision_checking_parameter(self):
    sig = inspect.signature(move_robot_to_joint)
    self.assertNotIn("disable_collision_checking", sig.parameters)
    args = parse_args([])
    self.assertFalse(hasattr(args, "disable_collision_checking"))

  def test_parse_joint_values(self):
    self.assertEqual(
      parse_joint_values("0.0, -1.57, 1.57, -1.57, -1.57, 0.0"),
      [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    )
    self.assertIsNone(parse_joint_values("invalid values"))

  def test_resolve_joint_target(self):
    args = parse_args(["--name", "home"])
    self.assertEqual(resolve_joint_target(args), "home")
    args_joints = parse_args(["--joints", "0.1", "0.2", "0.3"])
    self.assertEqual(resolve_joint_target(args_joints), [0.1, 0.2, 0.3])

  def test_list_available_joint_configs(self):
    mock_world = mock.MagicMock()
    mock_world.ur_module.joint_configurations = {
      "home": [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
    }
    configs = list_available_joint_configs(mock_world, "ur_module")
    self.assertEqual(configs, [("home", [0.0, -1.57, 1.57, 0.0, 0.0, 0.0])])

  @mock.patch("tools.jogging.move_to_joint.UrRobot")
  def test_move_robot_to_joint_executes_task(self, mock_ur_robot_cls):
    mock_solution = mock.MagicMock()
    mock_robot = mock.MagicMock()
    mock_ur_robot_cls.return_value = mock_robot

    move_robot_to_joint(mock_solution, "home")
    mock_robot.build_move_joint_task.assert_called_once()
    mock_solution.executive.run.assert_called_once()

    mock_solution.reset_mock()
    move_robot_to_joint(
      mock_solution, JointPosition((0.0, -1.57, 1.57, 0.0, 0.0, 0.0))
    )
    mock_robot.build_move_to_joint_position_task.assert_called_once()
    mock_solution.executive.run.assert_called_once()


if __name__ == "__main__":
  absltest.main()
