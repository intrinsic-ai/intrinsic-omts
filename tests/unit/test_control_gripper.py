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

"""Unit tests for the control_gripper CLI and interactive tool."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import execution

from src.hardware.gripper import (
  DioGripper,
  GripperInterface,
  MockGripper,
  RobotiqGripper,
  SideloadedGripperCmd,
)
from tools.gripper.control_gripper import (
  create_gripper,
  execute_action,
  main,
  parse_args,
  prompt_menu,
)


class ControlGripperTest(absltest.TestCase):
  """Tests for the control_gripper CLI and interactive tool."""

  def test_parse_args_defaults(self):
    """Tests default command-line argument values."""
    args = parse_args([])
    self.assertEqual(args.address, "localhost:17080")
    self.assertIsNone(args.action)
    self.assertFalse(args.mock)
    self.assertEqual(args.gripper_type, "robotiq")
    self.assertEqual(args.joint_name, "robotiq_hande_left_finger_joint")
    self.assertEqual(args.open_position, 0.025)
    self.assertEqual(args.close_position, 0.0)
    self.assertIsNone(args.action_name)
    self.assertEqual(args.open_pin, 0)
    self.assertEqual(args.close_pin, 1)
    self.assertEqual(args.output_block_name, "standard_out")
    self.assertIsNone(args.device_name)

  def test_parse_args_custom_values(self):
    """Tests parsing custom command-line argument values."""
    args = parse_args(
      [
        "--address",
        "10.0.0.1:17080",
        "--action",
        "open",
        "--mock",
        "--gripper_type",
        "dio",
        "--joint_name",
        "custom_finger_joint",
        "--open_position",
        "0.005",
        "--close_position",
        "0.030",
        "--action_name",
        "/custom/gripper_cmd",
        "--open_pin",
        "5",
        "--close_pin",
        "6",
        "--output_block_name",
        "custom_out",
        "--device_name",
        "custom_ur",
      ]
    )
    self.assertEqual(args.address, "10.0.0.1:17080")
    self.assertEqual(args.action, "open")
    self.assertTrue(args.mock)
    self.assertEqual(args.gripper_type, "dio")
    self.assertEqual(args.joint_name, "custom_finger_joint")
    self.assertEqual(args.open_position, 0.005)
    self.assertEqual(args.close_position, 0.030)
    self.assertEqual(args.action_name, "/custom/gripper_cmd")
    self.assertEqual(args.open_pin, 5)
    self.assertEqual(args.close_pin, 6)
    self.assertEqual(args.output_block_name, "custom_out")
    self.assertEqual(args.device_name, "custom_ur")

  def test_create_gripper_mock_flag(self):
    """Tests creating mock gripper when mock flag is True."""
    args = parse_args(["--mock"])
    gripper = create_gripper(args, solution=None)
    self.assertIsInstance(gripper, MockGripper)

  def test_create_gripper_mock_type(self):
    """Tests creating mock gripper when gripper_type is mock."""
    args = parse_args(["--gripper_type", "mock"])
    gripper = create_gripper(args, solution=None)
    self.assertIsInstance(gripper, MockGripper)

  def test_create_gripper_dio(self):
    """Tests creating DioGripper with specified options."""
    args = parse_args(
      [
        "--gripper_type",
        "dio",
        "--open_pin",
        "2",
        "--close_pin",
        "3",
        "--output_block_name",
        "test_out",
        "--device_name",
        "test_dev",
      ]
    )
    mock_solution = mock.MagicMock()
    gripper = create_gripper(args, solution=mock_solution)
    self.assertIsInstance(gripper, DioGripper)
    self.assertEqual(gripper._open_pin, 2)
    self.assertEqual(gripper._close_pin, 3)
    self.assertEqual(gripper._output_block_name, "test_out")
    self.assertEqual(gripper._device_name, "test_dev")

  def test_create_gripper_robotiq(self):
    """Tests creating RobotiqGripper with specified options."""
    args = parse_args(
      [
        "--gripper_type",
        "robotiq",
        "--joint_name",
        "hande_joint",
        "--open_position",
        "0.01",
        "--close_position",
        "0.04",
        "--action_name",
        "/robotiq/cmd",
      ]
    )
    mock_solution = mock.MagicMock()
    gripper = create_gripper(args, solution=mock_solution)
    self.assertIsInstance(gripper, RobotiqGripper)
    self.assertEqual(gripper._joint_name, "hande_joint")
    self.assertEqual(gripper._open_position, 0.01)
    self.assertEqual(gripper._close_position, 0.04)
    self.assertEqual(gripper._action_name, "/robotiq/cmd")

  def test_create_gripper_sideloaded(self):
    """Tests creating SideloadedGripperCmd with specified options."""
    args = parse_args(
      [
        "--gripper_type",
        "sideloaded",
        "--joint_name",
        "sideloaded_joint",
        "--open_position",
        "0.02",
        "--close_position",
        "0.00",
        "--action_name",
        "/sideloaded/gripper_cmd",
      ]
    )
    mock_solution = mock.MagicMock()
    gripper = create_gripper(args, solution=mock_solution)
    self.assertIsInstance(gripper, SideloadedGripperCmd)
    self.assertEqual(gripper._joint_name, "sideloaded_joint")
    self.assertEqual(gripper.open_position, 0.02)
    self.assertEqual(gripper.close_position, 0.00)
    self.assertEqual(gripper._action_name, "/sideloaded/gripper_cmd")

  def test_create_gripper_unsupported_type(self):
    """Tests create_gripper raises ValueError for unsupported gripper type."""
    args = mock.MagicMock()
    args.mock = False
    args.gripper_type = "unsupported_gripper"
    with self.assertRaises(ValueError):
      create_gripper(args, solution=mock.MagicMock())

  def test_execute_action_open_mock(self):
    """Tests executing open on a mock gripper."""
    gripper = MockGripper()
    gripper.state = "closed"
    success = execute_action(gripper, "open")
    self.assertTrue(success)
    self.assertEqual(gripper.state, "open")
    self.assertEqual(gripper.command_log, ["open"])

  def test_execute_action_close_mock(self):
    """Tests executing close on a mock gripper."""
    gripper = MockGripper()
    gripper.state = "open"
    success = execute_action(gripper, "close")
    self.assertTrue(success)
    self.assertEqual(gripper.state, "closed")
    self.assertEqual(gripper.command_log, ["close"])

  def test_execute_action_unknown_action(self):
    """Tests executing an unknown action returns failure."""
    gripper = MockGripper()
    success = execute_action(gripper, "unknown_action")
    self.assertFalse(success)
    self.assertEmpty(gripper.command_log)

  def test_execute_action_live_open_success(self):
    """Tests successful live execution of open action."""
    mock_gripper = mock.MagicMock(spec=GripperInterface)
    mock_task = mock.MagicMock()
    mock_gripper.build_open_task.return_value = mock_task
    mock_solution = mock.MagicMock()

    success = execute_action(mock_gripper, "open", solution=mock_solution)
    self.assertTrue(success)
    mock_gripper.build_open_task.assert_called_once()
    mock_solution.executive.run.assert_called_once_with(mock_task)

  def test_execute_action_live_close_success(self):
    """Tests successful live execution of close action."""
    mock_gripper = mock.MagicMock(spec=GripperInterface)
    mock_task = mock.MagicMock()
    mock_gripper.build_close_task.return_value = mock_task
    mock_solution = mock.MagicMock()

    success = execute_action(mock_gripper, "close", solution=mock_solution)
    self.assertTrue(success)
    mock_gripper.build_close_task.assert_called_once()
    mock_solution.executive.run.assert_called_once_with(mock_task)

  def test_execute_action_live_execution_failed_error(self):
    """Tests handling of execution failure when running a live gripper task."""
    mock_gripper = mock.MagicMock(spec=GripperInterface)
    mock_task = mock.MagicMock()
    mock_gripper.build_close_task.return_value = mock_task
    mock_solution = mock.MagicMock()
    mock_solution.executive.run.side_effect = execution.ExecutionFailedError(
      "Gripper motion failed"
    )
    mock_solution.executive.get_errors.return_value = ["Grasp obstruction"]

    success = execute_action(mock_gripper, "close", solution=mock_solution)
    self.assertFalse(success)
    mock_gripper.build_close_task.assert_called_once()
    mock_solution.executive.run.assert_called_once_with(mock_task)
    mock_solution.executive.get_errors.assert_called_once()

  def test_execute_action_live_unexpected_exception(self):
    """Tests handling of unexpected exceptions during task execution."""
    mock_gripper = mock.MagicMock(spec=GripperInterface)
    mock_task = mock.MagicMock()
    mock_gripper.build_open_task.return_value = mock_task
    mock_solution = mock.MagicMock()
    mock_solution.executive.run.side_effect = RuntimeError("Connection lost")

    success = execute_action(mock_gripper, "open", solution=mock_solution)
    self.assertFalse(success)
    mock_gripper.build_open_task.assert_called_once()
    mock_solution.executive.run.assert_called_once_with(mock_task)

  def test_execute_action_live_without_solution(self):
    """Tests executing a non-mock gripper without solution fails."""
    mock_gripper = mock.MagicMock(spec=GripperInterface)
    success = execute_action(mock_gripper, "open", solution=None)
    self.assertFalse(success)

  def test_prompt_menu_valid_selections(self):
    """Tests prompt menu returns the correct action for valid inputs."""
    test_cases = [
      ("1", "open"),
      ("2", "close"),
    ]
    for user_input, expected_action in test_cases:
      with mock.patch("builtins.input", return_value=user_input):
        action = prompt_menu()
        self.assertEqual(action, expected_action)

  def test_prompt_menu_invalid_selection_retry(self):
    """Tests prompt menu retries on invalid input until a valid choice."""
    with mock.patch("builtins.input", side_effect=["invalid", "99", "1"]):
      action = prompt_menu()
      self.assertEqual(action, "open")

  def test_prompt_menu_quit_inputs(self):
    """Tests prompt menu returns None on quit commands."""
    for quit_cmd in ("q", "quit", "exit", "Q", "QUIT"):
      with mock.patch("builtins.input", return_value=quit_cmd):
        action = prompt_menu()
        self.assertIsNone(action)

  def test_prompt_menu_eof_error(self):
    """Tests prompt menu handles EOFError gracefully."""
    with mock.patch("builtins.input", side_effect=EOFError):
      action = prompt_menu()
      self.assertIsNone(action)

  def test_prompt_menu_keyboard_interrupt(self):
    """Tests prompt menu handles KeyboardInterrupt gracefully."""
    with mock.patch("builtins.input", side_effect=KeyboardInterrupt):
      action = prompt_menu()
      self.assertIsNone(action)

  @mock.patch("tools.gripper.control_gripper.execute_action", return_value=True)
  def test_main_mock_action_success(self, mock_execute):
    """Tests main executes an action on a mock gripper successfully."""
    main(["--mock", "--action", "open"])
    mock_execute.assert_called_once()
    self.assertIsInstance(mock_execute.call_args.kwargs["gripper"], MockGripper)
    self.assertEqual(mock_execute.call_args.kwargs["action"], "open")
    self.assertIsNone(mock_execute.call_args.kwargs["solution"])

  @mock.patch(
    "tools.gripper.control_gripper.execute_action", return_value=False
  )
  def test_main_mock_action_failure_exits_with_status_1(self, mock_execute):
    """Tests main exits with status 1 when action execution fails."""
    del mock_execute  # Unused.
    with self.assertRaises(SystemExit) as cm:
      main(["--mock", "--action", "open"])
    self.assertEqual(cm.exception.code, 1)

  @mock.patch("tools.gripper.control_gripper.execute_action", return_value=True)
  @mock.patch("tools.gripper.control_gripper.RobotiqGripper")
  @mock.patch("tools.gripper.control_gripper.deployments.connect")
  def test_main_live_connection_and_action(
    self, mock_connect, mock_robotiq_cls, mock_execute
  ):
    """Tests main connects to deployment and executes action on live gripper."""
    mock_solution = mock.MagicMock()
    mock_connect.return_value = mock_solution
    mock_gripper = mock.MagicMock()
    mock_robotiq_cls.return_value = mock_gripper

    main(["--address", "192.168.1.100:17080", "--action", "close"])

    mock_connect.assert_called_once_with(address="192.168.1.100:17080")
    mock_robotiq_cls.assert_called_once()
    mock_execute.assert_called_once_with(
      gripper=mock_gripper,
      action="close",
      solution=mock_solution,
    )

  @mock.patch("tools.gripper.control_gripper.execute_action")
  @mock.patch("tools.gripper.control_gripper.prompt_menu", return_value=None)
  def test_main_interactive_loop_exit(self, mock_prompt, mock_execute):
    """Tests main interactive loop terminates when user requests exit."""
    main(["--mock"])
    mock_prompt.assert_called_once()
    mock_execute.assert_not_called()

  @mock.patch("tools.gripper.control_gripper.execute_action", return_value=True)
  @mock.patch(
    "tools.gripper.control_gripper.prompt_menu", side_effect=["open", None]
  )
  def test_main_interactive_loop_runs_action_then_quits(
    self, mock_prompt, mock_execute
  ):
    """Tests main interactive loop executes chosen action then exits."""
    main(["--mock"])
    self.assertEqual(mock_prompt.call_count, 2)
    mock_execute.assert_called_once()
    self.assertIsInstance(mock_execute.call_args.kwargs["gripper"], MockGripper)
    self.assertEqual(mock_execute.call_args.kwargs["action"], "open")


if __name__ == "__main__":
  absltest.main()
