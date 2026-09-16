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

"""Unit tests for CNC machine loading subtree generation and execution."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.behaviors.load_machine import build_load_machine_subtree
from src.core.types import FixtureState, GripperState
from src.core.workpiece import Workpiece
from src.hardware.gripper import MockGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot


class LoadMachineSubtreeTest(absltest.TestCase):
  """Tests for build_load_machine_subtree sequence and hardware execution."""

  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.gripper = MockGripper()
    self.machine = MockCncMachine()
    self.workpiece = Workpiece(
      asset_id="ai.intrinsic.raw_stock_2x3x5", object_name="raw_stock_2x3x5"
    )

  def test_build_load_machine_subtree_default_sequence(self):
    """Verifies default 7-step subtree sequence structure and task names."""
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
    )

    self.assertIsNotNone(subtree)
    self.assertIsInstance(subtree, bt.Sequence)
    self.assertEqual(subtree.name, "3. Load Machine Subtree")
    # 7 steps by default when object reparenting is disabled
    self.assertLen(subtree.children, 7)

    # Step 3a: Blended move arm to machine approach via transit
    self.assertEqual(
      subtree.children[0].name,
      "Step 3a: Blended Move to Machine Approach via transit (root/transit -> root/machine_approach)",
    )

    # Step 3b: Move arm to vise approach
    self.assertEqual(
      subtree.children[1].name,
      "Step 3b: Move to Vise Approach (root/vise_pre_place)",
    )

    # Step 3c: Standoff approach and compliant touchdown
    self.assertEqual(
      subtree.children[2].name,
      "Step 3c: Linear Approach to Standoff (root/vise_place)",
    )
    self.assertEqual(
      subtree.children[3].name,
      "Step 3c: Compliant Touchdown (+Z Tool)",
    )

    # Step 3d: Clamp CNC vise
    self.assertEqual(subtree.children[4].name, "Step 3d: Clamp CNC Vise")

    # Step 3e: Open gripper to release part
    self.assertEqual(
      subtree.children[5].name,
      "Step 3e: Open Gripper (Release Part in Vise)",
    )

    # Step 3h: Linear retract arm to vise approach
    self.assertEqual(
      subtree.children[6].name,
      "Step 3h: Linear Retract to Vise Approach (root/vise_pre_place)",
    )

  def test_build_load_machine_subtree_with_reparenting_enabled(self):
    """Verifies 8-step structure when enable_object_reparenting=True."""
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      enable_object_reparenting=True,
    )
    self.assertEqual(subtree.name, "3. Load Machine Subtree")
    self.assertLen(subtree.children, 8)
    self.assertEqual(
      subtree.children[6].name,
      "Step 3f: Detach Part to root in Digital Twin",
    )

  def test_build_load_machine_subtree_mock_hardware_execution(self):
    """Verifies mock hardware commands recorded during subtree build."""
    build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
    )

    # CNC vise should be clamped
    self.assertEqual(self.machine.vise_state, FixtureState.CLAMPED)
    self.assertEqual(self.machine.command_log, ["close_vise"])

    # Gripper should be opened
    self.assertEqual(self.gripper.commanded_state, GripperState.OPEN)
    self.assertEqual(self.gripper.command_log, ["open"])

    # Robot motions: transit/approach -> vise_pre_place -> vise_place -> touchdown -> retract
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/transit->root/machine_approach:ANY",
        "move_cartesian:root/vise_pre_place:ANY",
        "move_cartesian:root/vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_cartesian:root/vise_pre_place:LINEAR",
      ],
    )

  def test_build_load_machine_subtree_custom_arguments(self):
    """Verifies custom frame names, contact force, and subtree name."""
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      parent_object="fixture",
      machine_approach_frame_name="custom_entry",
      vise_approach_frame_name="custom_vise",
      vise_place_frame_name="custom_place",
      contact_force_newtons=14.0,
      settling_timeout_seconds=3.0,
      name="Custom Load Machine",
    )

    self.assertEqual(subtree.name, "Custom Load Machine")
    self.assertLen(subtree.children, 7)
    self.assertEqual(
      subtree.children[0].name,
      "Step 3a: Blended Move to Machine Approach via transit (fixture/transit -> fixture/custom_entry)",
    )
    self.assertEqual(
      subtree.children[1].name,
      "Step 3b: Move to Vise Approach (fixture/custom_vise)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 3c: Linear Approach to Standoff (fixture/custom_place)",
    )
    self.assertEqual(
      subtree.children[3].name,
      "Step 3c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[6].name,
      "Step 3h: Linear Retract to Vise Approach (fixture/custom_vise)",
    )

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:fixture/transit->fixture/custom_entry:ANY",
        "move_cartesian:fixture/custom_vise:ANY",
        "move_cartesian:fixture/custom_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=14.0",
        "move_cartesian:fixture/custom_vise:LINEAR",
      ],
    )

  def test_build_load_machine_subtree_custom_vise_frames_reach_every_step(self):
    """Verifies custom vise frame names reach approach, place and retract."""
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      vise_approach_frame_name="custom_vise_approach",
      vise_place_frame_name="custom_vise_place",
    )

    self.assertEqual(
      subtree.children[1].name,
      "Step 3b: Move to Vise Approach (root/custom_vise_approach)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 3c: Linear Approach to Standoff (root/custom_vise_place)",
    )
    self.assertEqual(
      subtree.children[3].name,
      "Step 3c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[6].name,
      "Step 3h: Linear Retract to Vise Approach (root/custom_vise_approach)",
    )
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/transit->root/machine_approach:ANY",
        "move_cartesian:root/custom_vise_approach:ANY",
        "move_cartesian:root/custom_vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_cartesian:root/custom_vise_approach:LINEAR",
      ],
    )

  def test_build_load_machine_subtree_workpiece_identifier_resolution(self):
    """Verifies identifier resolution with world.build_reparent_task."""
    mock_world = mock.MagicMock()
    mock_world.build_reparent_task.return_value = bt.Task(
      action=bt.PythonScript(function_body="pass"),
      name="Step 3f: Detach Part to root in Digital Twin",
    )
    wp_asset = Workpiece(asset_id="custom_asset_123", object_name="custom_part")
    tree_asset = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=wp_asset,
      enable_object_reparenting=True,
      world=mock_world,
    )
    self.assertIsNotNone(tree_asset.children[6])
    mock_world.build_reparent_task.assert_called_with(
      target="custom_asset_123",
      new_parent="root",
      name="Step 3f: Detach Part to root in Digital Twin",
    )

  def test_build_load_machine_subtree_with_solution(self):
    """Verifies passing solution handle to load_machine subtree."""
    mock_solution = mock.MagicMock()
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      solution=mock_solution,
    )
    self.assertIsNotNone(subtree)
    self.assertLen(subtree.children, 7)

  def test_build_load_machine_subtree_contact_timeout_seconds(self):
    """Verifies contact_force_newtons is forwarded to compliant touchdown."""
    build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      contact_force_newtons=9.5,
      contact_timeout_seconds=45.0,
    )
    self.assertIn(
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=9.5",
      self.robot.executed_commands,
    )

  def test_build_load_machine_subtree_with_entry_via_frame(self):
    """Verifies that entry_via_frame_name creates a blended approach move."""
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      entry_via_frame_name="transit",
    )
    self.assertIsNotNone(subtree)
    self.assertEqual(
      subtree.children[0].name,
      "Step 3a: Blended Move to Machine Approach via transit"
      " (root/transit -> root/machine_approach)",
    )
    self.assertIn(
      "move_blended_cartesian:root/transit->root/machine_approach:ANY",
      self.robot.executed_commands,
    )


if __name__ == "__main__":
  absltest.main()
