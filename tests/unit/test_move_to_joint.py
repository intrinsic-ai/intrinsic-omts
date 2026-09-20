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

  def test_jog_interactive_key_decoding_and_part_resolution(self):
    from tools.jogging.jog_interactive import (
      Key,
      get_part_joint_info,
      resolve_part_name,
      send_velocity_command,
    )

    self.assertEqual(Key("\x1b[C"), Key.RIGHT)
    self.assertEqual(Key("\x1bOC"), Key.RIGHT)
    self.assertEqual(Key("\x1b[D\x1b[D"), Key.LEFT)
    self.assertEqual(Key("Q"), Key.QUIT)
    self.assertEqual(Key("\x03"), Key.QUIT)
    self.assertEqual(Key("x"), Key.UNKNOWN)

    mock_client = mock.MagicMock()
    mock_client.list_parts.return_value = []
    with self.assertRaises(LookupError):
      resolve_part_name(mock_client)

    mock_client.list_parts.return_value = ["icon"]
    with self.assertRaises(LookupError):
      resolve_part_name(mock_client)

    mock_client.list_parts.return_value = ["icon", "ur_module", "arm"]
    self.assertEqual(resolve_part_name(mock_client), "arm")
    self.assertEqual(resolve_part_name(mock_client, "ur_module"), "ur_module")
    with self.assertRaises(ValueError):
      resolve_part_name(mock_client, "nonexistent_part")

    mock_part_cfg = mock.MagicMock()
    mock_part_cfg.name = "arm"
    mock_part_cfg.HasField.return_value = True
    mock_part_cfg.generic_config.HasField.return_value = True
    mock_part_cfg.generic_config.joint_position_config.num_joints = 6
    mock_client.get_config.return_value.part_configs = [mock_part_cfg]

    ndof, limits = get_part_joint_info(mock_client, "arm")
    self.assertEqual(ndof, 6)
    self.assertEqual(
      limits,
      mock_part_cfg.generic_config.joint_limits_config.application_limits,
    )

    mock_stream = mock.MagicMock()
    send_velocity_command(mock_stream, [0.1, 0.0, -0.1, 0.0, 0.0, 0.0])
    mock_stream.write.assert_called_once()

  @mock.patch(
    "tools.jogging.store_joint_config.worlds.ObjectWorld.connect",
    autospec=True,
  )
  @mock.patch("tools.jogging.store_joint_config.deployments.connect")
  def test_store_joint_config_normalizes_and_saves_to_worlds(
    self, mock_connect, mock_init_world_connect
  ):
    import math

    from intrinsic.solutions import worlds

    from tools.jogging import store_joint_config

    mock_solution = mock.MagicMock()
    mock_solution.world.ur_module.joint_positions = [
      0.0,
      1.5 * math.pi,
      0.0,
      0.0,
      0.0,
      0.0,
    ]
    mock_connect.return_value = mock_solution
    mock_init_world = mock.MagicMock()
    mock_init_world_connect.return_value = mock_init_world

    store_joint_config.main(["home", "--robot_name", "ur_module"])

    mock_solution.world.update_kinematic_object_joint_configurations.assert_called_once()
    saved_cfg = mock_solution.world.update_kinematic_object_joint_configurations.call_args.kwargs[
      "named_joint_configurations_to_set"
    ][0]
    self.assertEqual(saved_cfg.name, "home")
    self.assertAlmostEqual(saved_cfg.joint_positions[1], -0.5 * math.pi)
    mock_init_world_connect.assert_called_once_with(
      worlds.EditWorldId.INITIAL,
      mock_solution.grpc_channel,
    )
    mock_init_world.update_kinematic_object_joint_configurations.assert_called_once()

  @mock.patch("tools.jogging.jog_interactive.tty.setcbreak")
  @mock.patch("tools.jogging.jog_interactive.termios.tcsetattr")
  @mock.patch("tools.jogging.jog_interactive.termios.tcgetattr")
  def test_raw_terminal_mode_restores_settings_and_jog_joint_loop(
    self, mock_tcgetattr, mock_tcsetattr, mock_setcbreak
  ):
    from tools.jogging.jog_interactive import (
      Key,
      jog_joint_loop,
      raw_terminal_mode,
    )

    mock_tcgetattr.return_value = ["saved_tty_attrs"]
    with self.assertRaises(RuntimeError):
      with raw_terminal_mode(fd=42):
        mock_setcbreak.assert_called_once_with(42)
        raise RuntimeError("simulated crash")

    mock_tcsetattr.assert_called_once()
    self.assertEqual(mock_tcsetattr.call_args.args[0], 42)
    self.assertEqual(mock_tcsetattr.call_args.args[2], ["saved_tty_attrs"])

    mock_stream = mock.MagicMock()
    with mock.patch(
      "tools.jogging.jog_interactive.read_key",
      side_effect=[Key.RIGHT, Key.SPACE, Key.QUIT],
    ):
      jog_joint_loop(
        stream=mock_stream,
        joint_idx=2,
        ndof=6,
        max_velocity=0.15,
        deadman_timeout_sec=0.5,
      )

    # 1st write: +0.15 on joint 2; 2nd write: 0.0 on SPACE; 3rd write: 0.0 on QUIT
    self.assertEqual(mock_stream.write.call_count, 3)
    first_cmd = mock_stream.write.call_args_list[0].args[0]
    self.assertAlmostEqual(first_cmd.goal_velocity.joints[2], 0.15)
    last_cmd = mock_stream.write.call_args_list[-1].args[0]
    self.assertEqual(list(last_cmd.goal_velocity.joints), [0.0] * 6)


if __name__ == "__main__":
  absltest.main()
