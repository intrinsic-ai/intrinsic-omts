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

"""Unit tests for CNC machining handshake subtree generation and execution."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.behaviors.machining import build_machining_handshake_subtree
from src.core.types import MachineDoorState
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot


class MachiningSubtreeTest(absltest.TestCase):
  """Tests for build_machining_handshake_subtree sequence and execution."""

  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.machine = MockCncMachine()

  def test_build_machining_handshake_subtree_default_sequence(self):
    """Verifies default 5-step subtree sequence structure and task names."""
    subtree = build_machining_handshake_subtree(
      robot=self.robot,
      machine=self.machine,
    )

    self.assertIsNotNone(subtree)
    self.assertIsInstance(subtree, bt.Sequence)
    self.assertEqual(subtree.name, "4. Machining Handshake Subtree")
    self.assertLen(subtree.children, 5)

    # Step 4a: Retract robot arm to machine_approach (LINEAR)
    step_4a = subtree.children[0]
    self.assertIsInstance(step_4a, bt.Node)
    self.assertEqual(
      step_4a.name,
      "Move to root/machine_approach (LINEAR)",
    )

    # Step 4b: Close CNC door
    step_4b = subtree.children[1]
    self.assertIsInstance(step_4b, bt.Task)
    self.assertEqual(step_4b.name, "Step 4b: Close CNC Door")

    # Step 4c: Trigger CNC cycle start
    step_4c = subtree.children[2]
    self.assertIsInstance(step_4c, bt.Task)
    self.assertEqual(step_4c.name, "Step 4c: Trigger CNC Cycle Start")

    # Step 4d: Wait for CNC cycle complete
    step_4d = subtree.children[3]
    self.assertIsInstance(step_4d, bt.Task)
    self.assertEqual(step_4d.name, "Step 4d: Wait for CNC Cycle Complete")

    # Step 4e: Open CNC door
    step_4e = subtree.children[4]
    self.assertIsInstance(step_4e, bt.Task)
    self.assertEqual(step_4e.name, "Step 4e: Open CNC Door")

  def test_build_machining_handshake_subtree_mock_hardware_execution(self):
    """Verifies mock hardware commands recorded during subtree build."""
    build_machining_handshake_subtree(
      robot=self.robot,
      machine=self.machine,
    )

    # CNC door should end up open after step 4e
    self.assertEqual(self.machine.door_state, MachineDoorState.OPEN)
    self.assertEqual(
      self.machine.command_log,
      ["close_door", "trigger_cycle", "wait_cycle_complete", "open_door"],
    )

    # Robot linear motion to safe standby position
    self.assertEqual(
      self.robot.executed_commands,
      ["move_cartesian:root/machine_approach:LINEAR"],
    )

  def test_build_machining_handshake_subtree_custom_arguments(self):
    """Verifies custom frame name, timeouts, parent object, and tree name."""
    subtree = build_machining_handshake_subtree(
      robot=self.robot,
      machine=self.machine,
      parent_object="cnc_fixture",
      standby_frame_name="safe_retract",
      machining_timeout_seconds=45.0,
      name="Custom Machining Tree",
    )

    self.assertEqual(subtree.name, "Custom Machining Tree")
    self.assertLen(subtree.children, 5)
    self.assertEqual(
      subtree.children[0].name,
      "Move to cnc_fixture/safe_retract (LINEAR)",
    )
    self.assertEqual(
      self.robot.executed_commands,
      ["move_cartesian:cnc_fixture/safe_retract:LINEAR"],
    )
    self.assertEqual(
      self.machine.command_log,
      ["close_door", "trigger_cycle", "wait_cycle_complete", "open_door"],
    )

  def test_build_machining_handshake_subtree_timeout_propagation(self):
    """Verifies custom machining timeout is forwarded."""
    with (
      mock.patch.object(
        self.machine,
        "build_wait_cycle_complete_task",
        wraps=self.machine.build_wait_cycle_complete_task,
      ) as mock_wait_task,
      mock.patch.object(
        self.robot,
        "build_move_cartesian_task",
        wraps=self.robot.build_move_cartesian_task,
      ) as mock_move_task,
    ):
      build_machining_handshake_subtree(
        robot=self.robot,
        machine=self.machine,
        machining_timeout_seconds=60.0,
      )

      mock_wait_task.assert_called_once_with(
        timeout_seconds=60.0,
        name="Step 4d: Wait for CNC Cycle Complete",
      )
      mock_move_task.assert_called_once_with(
        target_frame_name="machine_approach",
        target_object_name="root",
        motion_type="LINEAR",
        target_frame_offset=None,
        name="Move to root/machine_approach (LINEAR)",
      )


if __name__ == "__main__":
  absltest.main()
