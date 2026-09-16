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

"""Unit tests for the control_machine CLI and interactive tool."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import execution

from src.hardware.machine import CncMachineInterface, MockCncMachine
from tools.machine.control_machine import (
  execute_action,
  main,
  prompt_menu,
)


class ControlMachineTest(absltest.TestCase):
  """Tests for the control_machine CLI and interactive tool."""

  def test_execute_action_open_door_mock(self):
    """Tests executing open_door on a mock CNC machine."""
    machine = MockCncMachine()
    success = execute_action(machine, "open_door")
    self.assertTrue(success)
    self.assertEqual(machine.command_log, ["open_door"])

  def test_execute_action_close_door_mock(self):
    """Tests executing close_door on a mock CNC machine."""
    machine = MockCncMachine()
    success = execute_action(machine, "close_door")
    self.assertTrue(success)
    self.assertEqual(machine.command_log, ["close_door"])

  def test_execute_action_open_vise_mock(self):
    """Tests executing open_vise on a mock CNC machine."""
    machine = MockCncMachine()
    success = execute_action(machine, "open_vise")
    self.assertTrue(success)
    self.assertEqual(machine.command_log, ["open_vise"])

  def test_execute_action_close_vise_mock(self):
    """Tests executing close_vise on a mock CNC machine."""
    machine = MockCncMachine()
    success = execute_action(machine, "close_vise")
    self.assertTrue(success)
    self.assertEqual(machine.command_log, ["close_vise"])

  def test_execute_action_trigger_cycle_mock(self):
    """Tests executing trigger_cycle on a mock CNC machine."""
    machine = MockCncMachine()
    success = execute_action(machine, "trigger_cycle")
    self.assertTrue(success)
    self.assertEqual(machine.command_log, ["trigger_cycle"])

  def test_execute_action_wait_cycle_mock(self):
    """Tests executing wait_cycle on a mock CNC machine."""
    machine = MockCncMachine()
    success = execute_action(machine, "wait_cycle", timeout_seconds=15.0)
    self.assertTrue(success)
    self.assertEqual(machine.command_log, ["wait_cycle_complete:15"])

  def test_execute_action_unknown_action(self):
    """Tests executing an unknown action returns failure."""
    machine = MockCncMachine()
    success = execute_action(machine, "unknown_action")
    self.assertFalse(success)
    self.assertEmpty(machine.command_log)

  def test_execute_action_live_success(self):
    """Tests successful action execution against a live solution."""
    mock_machine = mock.MagicMock(spec=CncMachineInterface)
    mock_task = mock.MagicMock()
    mock_machine.build_open_door_task.return_value = mock_task
    mock_solution = mock.MagicMock()

    success = execute_action(mock_machine, "open_door", solution=mock_solution)
    self.assertTrue(success)
    mock_machine.build_open_door_task.assert_called_once()
    mock_solution.executive.run.assert_called_once_with(mock_task)

  def test_execute_action_live_execution_failed_error(self):
    """Tests handling of execution failure when running a live task."""
    mock_machine = mock.MagicMock(spec=CncMachineInterface)
    mock_task = mock.MagicMock()
    mock_machine.build_close_door_task.return_value = mock_task
    mock_solution = mock.MagicMock()
    mock_solution.executive.run.side_effect = execution.ExecutionFailedError(
      "Door interlock failed"
    )
    mock_solution.executive.get_errors.return_value = ["Interlock open"]

    success = execute_action(mock_machine, "close_door", solution=mock_solution)
    self.assertFalse(success)
    mock_machine.build_close_door_task.assert_called_once()
    mock_solution.executive.run.assert_called_once_with(mock_task)
    mock_solution.executive.get_errors.assert_called_once()

  def test_execute_action_live_unexpected_exception(self):
    """Tests handling of unexpected exceptions during task execution."""
    mock_machine = mock.MagicMock(spec=CncMachineInterface)
    mock_task = mock.MagicMock()
    mock_machine.build_open_vise_task.return_value = mock_task
    mock_solution = mock.MagicMock()
    mock_solution.executive.run.side_effect = RuntimeError("Connection lost")

    success = execute_action(mock_machine, "open_vise", solution=mock_solution)
    self.assertFalse(success)
    mock_machine.build_open_vise_task.assert_called_once()
    mock_solution.executive.run.assert_called_once_with(mock_task)

  def test_execute_action_live_without_solution(self):
    """Tests executing a non-mock machine without solution fails."""
    mock_machine = mock.MagicMock(spec=CncMachineInterface)
    success = execute_action(mock_machine, "open_door", solution=None)
    self.assertFalse(success)

  def test_prompt_menu_valid_selections(self):
    """Tests prompt menu returns the correct action for valid inputs."""
    test_cases = [
      ("1", "open_door"),
      ("2", "close_door"),
      ("3", "open_vise"),
      ("4", "close_vise"),
      ("5", "trigger_cycle"),
      ("6", "wait_cycle"),
    ]
    for user_input, expected_action in test_cases:
      with mock.patch("builtins.input", return_value=user_input):
        action = prompt_menu()
        self.assertEqual(action, expected_action)

  def test_prompt_menu_invalid_selection_retry(self):
    """Tests prompt menu retries on invalid input until a valid choice."""
    with mock.patch("builtins.input", side_effect=["invalid", "99", "1"]):
      action = prompt_menu()
      self.assertEqual(action, "open_door")

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

  @mock.patch("tools.machine.control_machine.execute_action", return_value=True)
  def test_main_mock_action_success(self, mock_execute):
    """Tests main executes an action on a mock machine successfully."""
    main(["--mock", "--action", "open_door"])
    mock_execute.assert_called_once()
    self.assertIsInstance(
      mock_execute.call_args.kwargs["machine"], MockCncMachine
    )
    self.assertEqual(mock_execute.call_args.kwargs["action"], "open_door")
    self.assertIsNone(mock_execute.call_args.kwargs["solution"])

  @mock.patch(
    "tools.machine.control_machine.execute_action", return_value=False
  )
  def test_main_mock_action_failure_exits_with_status_1(self, mock_execute):
    """Tests main exits with status 1 when action execution fails."""
    del mock_execute  # Unused.
    with self.assertRaises(SystemExit) as cm:
      main(["--mock", "--action", "open_door"])
    self.assertEqual(cm.exception.code, 1)

  @mock.patch("tools.machine.control_machine.execute_action", return_value=True)
  @mock.patch("tools.machine.control_machine.DioCncMachine")
  @mock.patch("tools.machine.control_machine.deployments.connect")
  def test_main_live_connection_and_action(
    self, mock_connect, mock_dio_cls, mock_execute
  ):
    """Tests main connects to deployment and executes action on live machine."""
    mock_solution = mock.MagicMock()
    mock_connect.return_value = mock_solution
    mock_machine = mock.MagicMock()
    mock_dio_cls.return_value = mock_machine

    main(["--address", "192.168.1.100:17080", "--action", "close_door"])

    mock_connect.assert_called_once_with(address="192.168.1.100:17080")
    mock_dio_cls.assert_called_once()
    mock_execute.assert_called_once_with(
      machine=mock_machine,
      action="close_door",
      solution=mock_solution,
      timeout_seconds=30.0,
      args=mock.ANY,
    )

  def test_execute_action_prep_mock(self):
    """Tests executing prep on a mock CNC machine."""
    machine = MockCncMachine()
    success = execute_action(machine, "prep")
    self.assertTrue(success)
    self.assertEqual(machine.command_log, ["prep"])

  @mock.patch("tools.machine.control_machine.execute_action")
  @mock.patch("tools.machine.control_machine.prompt_menu", return_value=None)
  def test_main_interactive_loop_exit(self, mock_prompt, mock_execute):
    """Tests main interactive loop terminates when user requests exit."""
    main(["--mock"])
    mock_prompt.assert_called_once()
    mock_execute.assert_not_called()


if __name__ == "__main__":
  absltest.main()
