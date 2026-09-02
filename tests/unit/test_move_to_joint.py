"""Unit tests for the move_to_joint developer CLI tool."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import execution
from tools.jogging.move_to_joint import list_available_joint_configs
from tools.jogging.move_to_joint import move_robot_to_joint
from tools.jogging.move_to_joint import parse_args
from tools.jogging.move_to_joint import parse_joint_values
from tools.jogging.move_to_joint import prompt_for_joint_target
from tools.jogging.move_to_joint import resolve_joint_target


class MoveToJointTest(absltest.TestCase):

  def test_list_available_joint_configs(self):
    mock_world = mock.MagicMock()
    mock_robot = mock.MagicMock()
    mock_config_home = mock.MagicMock()
    mock_config_home.joint_position = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    mock_config_view = mock.MagicMock()
    mock_config_view.joint_position = [0.71, -1.10, -2.30, -0.98, 1.58, -2.70]

    mock_robot.joint_configurations.keys.return_value = ["home", "view_joints"]
    mock_robot.joint_configurations.__getitem__.side_effect = (
        lambda k: mock_config_home if k == "home" else mock_config_view
    )
    mock_world.ur_module = mock_robot

    configs = list_available_joint_configs(mock_world, arm_part_name="ur_module")
    expected = [
        ("home", [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]),
        ("view_joints", [0.71, -1.10, -2.30, -0.98, 1.58, -2.70]),
    ]
    self.assertEqual(configs, expected)

  def test_parse_joint_values(self):
    self.assertIsNone(parse_joint_values(""))
    self.assertIsNone(parse_joint_values("invalid input"))
    self.assertEqual(
        parse_joint_values("0.0, -1.57, 1.57, -1.57, -1.57, 0.0"),
        [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    )
    self.assertEqual(
        parse_joint_values("[0.1 0.2 0.3 0.4 0.5 0.6]"),
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    )

  def test_prompt_for_joint_target_index(self):
    available_configs = [
        ("home", [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]),
        ("view_joints", [0.71, -1.10, -2.30, -0.98, 1.58, -2.70]),
    ]
    with mock.patch("builtins.input", side_effect=["2"]):
      selected = prompt_for_joint_target(available_configs)
      self.assertEqual(selected, "view_joints")

  def test_prompt_for_joint_target_name(self):
    available_configs = [
        ("home", [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]),
        ("view_joints", [0.71, -1.10, -2.30, -0.98, 1.58, -2.70]),
    ]
    with mock.patch("builtins.input", side_effect=["home"]):
      selected = prompt_for_joint_target(available_configs)
      self.assertEqual(selected, "home")

  def test_prompt_for_joint_target_custom_angles(self):
    available_configs = [("home", [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])]
    with mock.patch(
        "builtins.input",
        side_effect=["0.1, 0.2, 0.3, 0.4, 0.5, 0.6"],
    ):
      selected = prompt_for_joint_target(available_configs)
      self.assertEqual(selected, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

  def test_prompt_for_joint_target_quit(self):
    available_configs = [("home", [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])]
    with mock.patch("builtins.input", return_value="q"):
      selected = prompt_for_joint_target(available_configs)
      self.assertIsNone(selected)

  def test_move_robot_to_joint_named_success(self):
    mock_solution = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.move_robot.return_value = (
        bt.PythonScript(function_body="pass")
    )
    mock_solution.executive.run = mock.MagicMock()

    move_robot_to_joint(
        solution=mock_solution,
        joint_target="home",
        arm_part_name="ur_module",
        disable_collision_checking=False,
    )

    mock_solution.executive.run.assert_called_once()

  def test_move_robot_to_joint_positions_success(self):
    mock_solution = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.move_robot.return_value = (
        bt.PythonScript(function_body="pass")
    )
    mock_solution.executive.run = mock.MagicMock()

    move_robot_to_joint(
        solution=mock_solution,
        joint_target=[0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        arm_part_name="ur_module",
        disable_collision_checking=True,
    )

    mock_solution.executive.run.assert_called_once()

  def test_move_robot_to_joint_execution_error_handled(self):
    mock_solution = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.move_robot.return_value = (
        bt.PythonScript(function_body="pass")
    )
    mock_solution.executive.run.side_effect = execution.ExecutionFailedError(
        "Trajectory planning failed"
    )

    # Should not raise exception
    move_robot_to_joint(
        solution=mock_solution,
        joint_target="home",
    )

  def test_resolve_joint_target(self):
    args1 = parse_args(["--joints", "0.1", "0.2", "0.3"])
    self.assertEqual(resolve_joint_target(args1), [0.1, 0.2, 0.3])

    args2 = parse_args(["--name", "view_joints"])
    self.assertEqual(resolve_joint_target(args2), "view_joints")

    args3 = parse_args(["home"])
    self.assertEqual(resolve_joint_target(args3), "home")

    args4 = parse_args(["0.1, 0.2, 0.3, 0.4, 0.5, 0.6"])
    self.assertEqual(
        resolve_joint_target(args4), [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    )

    args5 = parse_args([])
    self.assertIsNone(resolve_joint_target(args5))

  def test_parse_args_settling_timeout(self):
    args_default = parse_args([])
    self.assertEqual(args_default.settling_timeout_seconds, 10.0)

    args_custom = parse_args(["--settling_timeout_seconds", "30.0"])
    self.assertEqual(args_custom.settling_timeout_seconds, 30.0)


if __name__ == "__main__":
  absltest.main()
