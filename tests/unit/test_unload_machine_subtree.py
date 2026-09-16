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

"""Unit tests for CNC machine unloading subtree generation and execution."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import Touchdown
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.workpiece import Workpiece
from src.core.world import World
from src.hardware.gripper import MockGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot


class UnloadMachineSubtreeTest(absltest.TestCase):
  """Tests for build_unload_machine_subtree sequence and hardware execution."""

  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.gripper = MockGripper()
    self.machine = MockCncMachine()
    self.workpiece = Workpiece(
      asset_id="ai.intrinsic.raw_stock_2x3x5", object_name="raw_stock_2x3x5"
    )

  def test_build_unload_machine_subtree_default_sequence(self):
    """Verifies default 6-step subtree sequence structure and task names."""
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
    )

    self.assertIsNotNone(subtree)
    self.assertIsInstance(subtree, bt.Sequence)
    self.assertEqual(subtree.name, "5. Unload Machine Subtree")
    # 6 steps by default when object reparenting is disabled
    self.assertLen(subtree.children, 6)

    # Step 5a: One blended move from the machine approach into the vise
    # approach, landing on the last leg linearly.
    self.assertEqual(
      subtree.children[0].name,
      "Step 5a: Blended Move to Vise Approach (root/machine_approach ->"
      " root/vise_pre_place)",
    )

    # Step 5c: Standoff approach and compliant touchdown
    self.assertEqual(
      subtree.children[1].name,
      "Step 5c: Linear Approach to Standoff (root/vise_place)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 5c: Compliant Touchdown (+Z Tool)",
    )

    # Step 5e: Close gripper to grasp part
    self.assertEqual(
      subtree.children[3].name,
      "Step 5e: Grasp Machined Part",
    )

    # Step 5g: Command CNC vise open (unclamp)
    self.assertEqual(
      subtree.children[4].name,
      "Step 5g: Open CNC Vise (Unclamp Part)",
    )

    # Step 5i: One blended linear retract back out to the machine approach.
    self.assertEqual(
      subtree.children[5].name,
      "Step 5i: Blended Linear Retract to Machine Approach"
      " (root/vise_pre_place -> root/machine_approach)",
    )

  def test_build_unload_machine_subtree_with_reparenting_enabled(self):
    """Verifies 7-step structure when enable_object_reparenting=True."""
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      enable_object_reparenting=True,
    )
    self.assertEqual(subtree.name, "5. Unload Machine Subtree")
    self.assertLen(subtree.children, 7)
    self.assertEqual(
      subtree.children[4].name,
      "Step 5f: Attach Part to Gripper in Digital Twin",
    )

  def test_build_unload_machine_subtree_mock_hardware_execution(self):
    """Verifies mock hardware commands recorded during subtree build."""
    build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
    )

    self.assertEqual(self.machine.command_log, ["open_vise"])
    self.assertEqual(self.gripper.command_log, ["close"])

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/machine_approach->"
        "root/vise_pre_place:ANY/LINEAR",
        "move_cartesian:root/vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_blended_cartesian:root/vise_pre_place->"
        "root/machine_approach:LINEAR",
      ],
    )

  def test_build_unload_machine_subtree_custom_arguments(self):
    """Verifies custom frame names, contact force, and subtree name."""
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      parent_object="fixture",
      machine_approach_frame_name="custom_entry",
      vise_approach_frame_name="custom_vise",
      vise_place_frame_name="custom_place",
      touchdown=Touchdown(force_n=18.0, retract_after_m=0.025),
      name="Custom Unload Machine",
    )

    self.assertEqual(subtree.name, "Custom Unload Machine")
    self.assertLen(subtree.children, 7)
    self.assertEqual(
      subtree.children[0].name,
      "Step 5a: Blended Move to Vise Approach (fixture/custom_entry ->"
      " fixture/custom_vise)",
    )
    self.assertEqual(
      subtree.children[1].name,
      "Step 5c: Linear Approach to Standoff (fixture/custom_place)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 5c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[3].name,
      "Step 5c: Linear Retract (2.5 cm, -Z Tool)",
    )
    self.assertEqual(
      subtree.children[6].name,
      "Step 5i: Blended Linear Retract to Machine Approach"
      " (fixture/custom_vise -> fixture/custom_entry)",
    )

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:fixture/custom_entry->"
        "fixture/custom_vise:ANY/LINEAR",
        "move_cartesian:fixture/custom_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=18.0",
        "move_relative_cartesian:(0.0, 0.0, -0.025):LINEAR",
        "move_blended_cartesian:fixture/custom_vise->"
        "fixture/custom_entry:LINEAR",
      ],
    )

  def test_build_unload_machine_subtree_custom_vise_frames_reach_every_step(
    self,
  ):
    """Verifies custom vise frame names reach approach, place and retract."""
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      vise_approach_frame_name="custom_vise_approach",
      vise_place_frame_name="custom_vise_place",
    )

    self.assertEqual(
      subtree.children[0].name,
      "Step 5a: Blended Move to Vise Approach (root/machine_approach ->"
      " root/custom_vise_approach)",
    )
    self.assertEqual(
      subtree.children[1].name,
      "Step 5c: Linear Approach to Standoff (root/custom_vise_place)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 5c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[5].name,
      "Step 5i: Blended Linear Retract to Machine Approach"
      " (root/custom_vise_approach -> root/machine_approach)",
    )
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/machine_approach->"
        "root/custom_vise_approach:ANY/LINEAR",
        "move_cartesian:root/custom_vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_blended_cartesian:root/custom_vise_approach->"
        "root/machine_approach:LINEAR",
      ],
    )

  def test_build_unload_machine_subtree_workpiece_identifier_resolution(self):
    """Verifies identifier resolution via world.build_attach_to_gripper_task."""
    mock_world = mock.MagicMock(spec=World)
    mock_world.build_attach_to_gripper_task.return_value = bt.Task(
      action=bt.PythonScript(function_body="pass"),
      name="Step 5f: Attach Part to Gripper in Digital Twin",
    )
    wp_asset = Workpiece(
      asset_id="ai.intrinsic.custom_stock", object_name="custom_stock"
    )
    build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=wp_asset,
      enable_object_reparenting=True,
      world=mock_world,
    )
    mock_world.build_attach_to_gripper_task.assert_called_once_with(
      object_name="custom_stock",
      name="Step 5f: Attach Part to Gripper in Digital Twin",
    )

  def test_build_unload_machine_subtree_with_solution(self):
    """Verifies passing solution handle to unload_machine subtree."""
    from src.core.world import MockSolution

    mock_solution = MockSolution()
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      solution=mock_solution,
      enable_object_reparenting=True,
    )
    self.assertIsNotNone(subtree)
    self.assertLen(subtree.children, 7)

  def test_build_unload_machine_subtree_touchdown_force_forwarded(self):
    """Verifies the touchdown contact force reaches the compliant move."""
    build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      touchdown=Touchdown(force_n=12.0, timeout_s=50.0),
    )
    self.assertIn(
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=12.0",
      self.robot.executed_commands,
    )

  def test_build_unload_machine_subtree_retract_uses_touchdown_retract(self):
    """Verifies the post-grasp lift distance comes from the touchdown."""
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      touchdown=Touchdown(retract_after_m=0.005),
    )
    self.assertLen(subtree.children, 7)
    self.assertEqual(
      subtree.children[3].name,
      "Step 5c: Linear Retract (0.5 cm, -Z Tool)",
    )
    self.assertIn(
      "move_relative_cartesian:(0.0, 0.0, -0.005):LINEAR",
      self.robot.executed_commands,
    )


if __name__ == "__main__":
  absltest.main()
