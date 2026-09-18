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

"""Unit tests for tools/machine/control_machine.py CLI tool."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import execution

from src.hardware.machine import CncMachineInterface
from tools.machine.control_machine import (
  execute_action,
  main,
  parse_args,
  prompt_menu,
)


class ControlMachineTest(absltest.TestCase):
  def test_parse_args_defaults(self):
    args = parse_args([])
    self.assertEqual(args.address, "localhost:17080")
    self.assertIsNone(args.action)
    self.assertEqual(args.timeout_seconds, 30.0)
    self.assertEqual(args.door_open_pin, 2)
    self.assertEqual(args.door_close_pin, 3)
    self.assertEqual(args.vise_open_pin, 4)
    self.assertEqual(args.vise_close_pin, 5)
    self.assertEqual(args.cycle_start_pin, 6)
    self.assertEqual(args.cycle_done_input_pin, 0)
    self.assertEqual(args.device_name, "ur_module")

  def test_execute_action_calls_machine_methods_and_runs_tree(self):
    mock_machine = mock.MagicMock(spec=CncMachineInterface)
    mock_solution = mock.MagicMock()

    for action, method_name in [
      ("open_door", "build_open_door_task"),
      ("close_door", "build_close_door_task"),
      ("open_vise", "build_open_vise_task"),
      ("close_vise", "build_close_vise_task"),
      ("trigger_cycle", "build_trigger_cycle_task"),
      ("wait_cycle", "build_wait_cycle_complete_task"),
    ]:
      mock_solution.reset_mock()
      result = execute_action(
        machine=mock_machine,
        action=action,
        solution=mock_solution,
        timeout_seconds=15.0,
      )
      self.assertTrue(result)
      getattr(mock_machine, method_name).assert_called()
      mock_solution.executive.run.assert_called_once()

  def test_execute_action_handles_execution_failed_error(self):
    mock_machine = mock.MagicMock(spec=CncMachineInterface)
    mock_solution = mock.MagicMock()
    mock_solution.executive.run.side_effect = execution.ExecutionFailedError(
      "DIO error"
    )
    self.assertFalse(
      execute_action(
        machine=mock_machine, action="open_door", solution=mock_solution
      )
    )

  def test_prompt_menu_selection_and_quit(self):
    with mock.patch("builtins.input", return_value="1"):
      self.assertEqual(prompt_menu(), "open_door")
    with mock.patch("builtins.input", return_value="q"):
      self.assertIsNone(prompt_menu())

  @mock.patch("tools.machine.control_machine.deployments.connect")
  @mock.patch("tools.machine.control_machine.execute_action", return_value=True)
  def test_main_runs_single_action(self, mock_execute, mock_connect):
    mock_connect.return_value = mock.MagicMock()
    main(["--action", "open_door"])
    mock_execute.assert_called_once()


if __name__ == "__main__":
  absltest.main()
