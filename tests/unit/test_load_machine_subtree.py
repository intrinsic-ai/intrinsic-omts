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
from src.behaviors.motions import Touchdown
from src.core.workpiece import Workpiece
from src.core.world import World
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
    """Verifies default 6-step subtree sequence structure and task names."""
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
    )

    self.assertIsNotNone(subtree)
    self.assertIsInstance(subtree, bt.Sequence)
    self.assertEqual(subtree.name, "3. Load Machine Subtree")
    # 6 steps by default when object reparenting is disabled
    self.assertLen(subtree.children, 6)

    # Step 3a: One blended move from transit through the machine approach and
    # on to the vise approach, landing on the last leg linearly.
    self.assertEqual(
      subtree.children[0].name,
      "Step 3a: Blended Move to Vise Approach (root/transit ->"
      " root/machine_approach -> root/vise_pre_place)",
    )

    # Step 3c: Standoff approach and compliant touchdown
    self.assertEqual(
      subtree.children[1].name,
      "Step 3c: Linear Approach to Standoff (root/vise_place)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 3c: Compliant Touchdown (+Z Tool)",
    )

    # Step 3d: Clamp CNC vise
    self.assertEqual(subtree.children[3].name, "Step 3d: Clamp CNC Vise")

    # Step 3e: Open gripper to release part
    self.assertEqual(
      subtree.children[4].name,
      "Step 3e: Open Gripper (Release Part in Vise)",
    )

    # Step 3h: Linear retract arm to vise approach
    self.assertEqual(
      subtree.children[5].name,
      "Step 3h: Linear Retract to Vise Approach (root/vise_pre_place)",
    )

  def test_build_load_machine_subtree_release_touchdown_has_no_retract(self):
    """Verifies a release touchdown seats the part without lifting off it."""
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      touchdown=Touchdown(retract_after_m=0.0),
    )

    self.assertLen(subtree.children, 6)
    self.assertNotIn(
      "move_relative_cartesian:(0.0, 0.0, -0.0):LINEAR",
      self.robot.executed_commands,
    )

  def test_build_load_machine_subtree_with_reparenting_enabled(self):
    """Verifies 7-step structure when enable_object_reparenting=True."""
    subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      enable_object_reparenting=True,
    )
    self.assertEqual(subtree.name, "3. Load Machine Subtree")
    self.assertLen(subtree.children, 7)
    self.assertEqual(
      subtree.children[5].name,
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

    self.assertEqual(self.machine.command_log, ["close_vise"])
    self.assertEqual(self.gripper.command_log, ["open"])

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/transit->root/machine_approach->"
        "root/vise_pre_place:ANY/ANY/LINEAR",
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
      entry_via_frame_name="custom_transit",
      machine_approach_frame_name="custom_entry",
      vise_approach_frame_name="custom_vise",
      vise_place_frame_name="custom_place",
      touchdown=Touchdown(force_n=14.0, timeout_s=3.0),
      name="Custom Load Machine",
    )

    self.assertEqual(subtree.name, "Custom Load Machine")
    self.assertLen(subtree.children, 6)
    self.assertEqual(
      subtree.children[0].name,
      "Step 3a: Blended Move to Vise Approach (fixture/custom_transit ->"
      " fixture/custom_entry -> fixture/custom_vise)",
    )
    self.assertEqual(
      subtree.children[1].name,
      "Step 3c: Linear Approach to Standoff (fixture/custom_place)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 3c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[5].name,
      "Step 3h: Linear Retract to Vise Approach (fixture/custom_vise)",
    )

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:fixture/custom_transit->fixture/custom_entry->"
        "fixture/custom_vise:ANY/ANY/LINEAR",
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
      subtree.children[0].name,
      "Step 3a: Blended Move to Vise Approach (root/transit ->"
      " root/machine_approach -> root/custom_vise_approach)",
    )
    self.assertEqual(
      subtree.children[1].name,
      "Step 3c: Linear Approach to Standoff (root/custom_vise_place)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 3c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[5].name,
      "Step 3h: Linear Retract to Vise Approach (root/custom_vise_approach)",
    )
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/transit->root/machine_approach->"
        "root/custom_vise_approach:ANY/ANY/LINEAR",
        "move_cartesian:root/custom_vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_cartesian:root/custom_vise_approach:LINEAR",
      ],
    )

  def test_build_load_machine_subtree_workpiece_identifier_resolution(self):
    """Verifies identifier resolution with world.build_detach_from_gripper_task."""
    mock_world = mock.MagicMock(spec=World)
    mock_world.build_detach_from_gripper_task.return_value = bt.Task(
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
    self.assertEqual(
      tree_asset.children[5].name,
      "Step 3f: Detach Part to root in Digital Twin",
    )
    mock_world.build_detach_from_gripper_task.assert_called_with(
      object_name="custom_part",
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
    self.assertLen(subtree.children, 6)

  def test_build_load_machine_subtree_touchdown_force_forwarded(self):
    """Verifies the touchdown contact force reaches the compliant move."""
    build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      touchdown=Touchdown(force_n=9.5, timeout_s=45.0),
    )
    self.assertIn(
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=9.5",
      self.robot.executed_commands,
    )


if __name__ == "__main__":
  absltest.main()
