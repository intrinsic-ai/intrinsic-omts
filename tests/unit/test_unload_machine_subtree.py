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

from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.types import FixtureState, GripperState
from src.core.workpiece import Workpiece
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
    """Verifies default 8-step subtree sequence structure and task names."""
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
    )

    self.assertIsNotNone(subtree)
    self.assertIsInstance(subtree, bt.Sequence)
    self.assertEqual(subtree.name, "5. Unload Machine Subtree")
    # 8 steps by default when object reparenting is disabled
    self.assertLen(subtree.children, 8)

    # Step 5a: Move arm to machine entry approach
    self.assertEqual(
      subtree.children[0].name,
      "Step 5a: Move to Machine Approach (root/machine_approach)",
    )

    # Step 5b: Move arm to vise approach
    self.assertEqual(
      subtree.children[1].name,
      "Step 5b: Move to Vise Approach (root/vise_pre_place)",
    )

    # Step 5c: Standoff approach and compliant touchdown
    self.assertEqual(
      subtree.children[2].name,
      "Step 5c: Linear Approach to Standoff (root/vise_place)",
    )
    self.assertEqual(
      subtree.children[3].name,
      "Step 5c: Compliant Touchdown (+Z Tool)",
    )

    # Step 5e: Close gripper to grasp part
    self.assertEqual(
      subtree.children[4].name,
      "Step 5e: Grasp Machined Part",
    )

    # Step 5g: Command CNC vise open (unclamp)
    self.assertEqual(
      subtree.children[5].name,
      "Step 5g: Open CNC Vise (Unclamp Part)",
    )

    # Step 5i: Linear retract arm to vise approach
    self.assertEqual(
      subtree.children[6].name,
      "Step 5i: Linear Retract to Vise Approach (root/vise_pre_place)",
    )

    # Step 5j: Linear retract arm out of enclosure to machine approach
    self.assertEqual(
      subtree.children[7].name,
      "Step 5j: Linear Retract to Machine Approach (root/machine_approach)",
    )

  def test_build_unload_machine_subtree_with_reparenting_enabled(self):
    """Verifies 9-step structure when enable_object_reparenting=True."""
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      enable_object_reparenting=True,
    )
    self.assertEqual(subtree.name, "5. Unload Machine Subtree")
    self.assertLen(subtree.children, 9)
    self.assertEqual(
      subtree.children[5].name,
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

    # CNC vise should be opened (unclamped)
    self.assertEqual(self.machine.vise_state, FixtureState.OPEN)
    self.assertEqual(self.machine.command_log, ["open_vise"])

    # Gripper should be closed (grasped)
    self.assertEqual(self.gripper.commanded_state, GripperState.CLOSED)
    self.assertEqual(self.gripper.command_log, ["close"])

    # Robot motions: machine_approach (ANY) -> vise_pre_place (LINEAR)
    # -> vise_place -> touchdown -> retracts
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_cartesian:root/machine_approach:ANY",
        "move_cartesian:root/vise_pre_place:LINEAR",
        "move_cartesian:root/vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_cartesian:root/vise_pre_place:LINEAR",
        "move_cartesian:root/machine_approach:LINEAR",
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
      contact_force_newtons=18.0,
      grasp_offset_z=0.025,
      name="Custom Unload Machine",
    )

    self.assertEqual(subtree.name, "Custom Unload Machine")
    self.assertLen(subtree.children, 9)
    self.assertEqual(
      subtree.children[0].name,
      "Step 5a: Move to Machine Approach (fixture/custom_entry)",
    )
    self.assertEqual(
      subtree.children[1].name,
      "Step 5b: Move to Vise Approach (fixture/custom_vise)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 5c: Linear Approach to Standoff (fixture/custom_place)",
    )
    self.assertEqual(
      subtree.children[3].name,
      "Step 5c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[4].name,
      "Step 5c: Linear Retract (2.5 cm, -Z Tool)",
    )
    self.assertEqual(
      subtree.children[7].name,
      "Step 5i: Linear Retract to Vise Approach (fixture/custom_vise)",
    )
    self.assertEqual(
      subtree.children[8].name,
      "Step 5j: Linear Retract to Machine Approach (fixture/custom_entry)",
    )

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_cartesian:fixture/custom_entry:ANY",
        "move_cartesian:fixture/custom_vise:LINEAR",
        "move_cartesian:fixture/custom_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=18.0",
        "move_relative_cartesian:(0.0, 0.0, -0.025):LINEAR",
        "move_cartesian:fixture/custom_vise:LINEAR",
        "move_cartesian:fixture/custom_entry:LINEAR",
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
      "Step 5a: Move to Machine Approach (root/machine_approach)",
    )
    self.assertEqual(
      subtree.children[1].name,
      "Step 5b: Move to Vise Approach (root/custom_vise_approach)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 5c: Linear Approach to Standoff (root/custom_vise_place)",
    )
    self.assertEqual(
      subtree.children[3].name,
      "Step 5c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[6].name,
      "Step 5i: Linear Retract to Vise Approach (root/custom_vise_approach)",
    )
    self.assertEqual(
      subtree.children[7].name,
      "Step 5j: Linear Retract to Machine Approach (root/machine_approach)",
    )
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_cartesian:root/machine_approach:ANY",
        "move_cartesian:root/custom_vise_approach:LINEAR",
        "move_cartesian:root/custom_vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_cartesian:root/custom_vise_approach:LINEAR",
        "move_cartesian:root/machine_approach:LINEAR",
      ],
    )

  def test_build_unload_machine_subtree_workpiece_identifier_resolution(self):
    """Verifies identifier resolution via world.build_reparent_task."""
    mock_world = mock.MagicMock()
    mock_world.build_reparent_task.return_value = bt.Task(
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
    mock_world.build_reparent_task.assert_called_once_with(
      target="ai.intrinsic.custom_stock",
      new_parent="gripper",
      name="Step 5f: Attach Part to Gripper in Digital Twin",
    )

  def test_build_unload_machine_subtree_with_solution(self):
    """Verifies passing solution handle to unload_machine subtree."""
    mock_solution = mock.MagicMock()
    mock_solution.world.build_reparent_task.return_value = bt.Task(
      action=bt.PythonScript(function_body="pass"),
      name="Step 5f: Attach Part to Gripper in Digital Twin",
    )
    subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      solution=mock_solution,
      enable_object_reparenting=True,
    )
    self.assertIsNotNone(subtree)
    self.assertLen(subtree.children, 9)

  def test_build_unload_machine_subtree_contact_timeout_seconds(self):
    """Verifies contact_timeout_seconds is forwarded to compliant touchdown."""
    build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      contact_force_newtons=12.0,
      contact_timeout_seconds=50.0,
    )
    self.assertIn(
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=12.0",
      self.robot.executed_commands,
    )

  def test_build_unload_machine_subtree_retract_uses_grasp_offset_z(self):
    """Verifies retract uses grasp_offset_z."""
    build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      workpiece=self.workpiece,
      grasp_offset_z=0.005,
    )
    self.assertIn(
      "move_relative_cartesian:(0.0, 0.0, -0.005):LINEAR",
      self.robot.executed_commands,
    )


if __name__ == "__main__":
  absltest.main()
