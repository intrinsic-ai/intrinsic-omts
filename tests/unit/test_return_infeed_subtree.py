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

"""Unit tests for the infeed return subtree."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import Touchdown
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.core.workpiece import Workpiece
from src.core.world import World
from src.hardware.gripper import MockGripper
from src.hardware.robot import MockRobot


class ReturnInfeedSubtreeTest(absltest.TestCase):
  """Tests for build_return_to_infeed_subtree sequence and execution."""

  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.gripper = MockGripper()
    self.workpiece = Workpiece(
      asset_id="ai.intrinsic.raw_stock_2x3x5", object_name="raw_stock_2x3x5"
    )

  def test_build_return_to_infeed_subtree_default_sequence(self):
    """Verifies default 5-step subtree sequence structure and task names."""
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
    )

    self.assertIsNotNone(subtree)
    self.assertIsInstance(subtree, bt.Sequence)
    self.assertEqual(subtree.name, "6. Obstacle-Aware Scatter Return Subtree")
    # 5 steps by default when object reparenting is disabled
    self.assertLen(subtree.children, 5)

    # Step 6a: Blended move to pre-place via transit (ANY)
    self.assertEqual(
      subtree.children[0].name,
      "Step 6a: Blended Move to Pre-Place via transit"
      " (root/transit -> root/infeed_pre_grasp)",
    )

    # Step 6c: Standoff approach and compliant touchdown
    self.assertEqual(
      subtree.children[1].name,
      "Step 6c: Linear Approach to Standoff (root/infeed_grasp)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 6c: Compliant Touchdown (+Z Tool)",
    )

    # Step 6d: Open gripper to release part
    self.assertEqual(
      subtree.children[3].name,
      "Step 6d: Release Part at Scatter Placement",
    )

    # Step 6f: Linear retract off the part, blended onward to the view frame.
    self.assertEqual(
      subtree.children[4].name,
      "Step 6f: Linear Retract to infeed_pre_grasp Blended to View Frame"
      " (root/infeed_pre_grasp -> root/view)",
    )

  def test_build_return_to_infeed_subtree_release_touchdown_has_no_retract(
    self,
  ):
    """Verifies a release touchdown seats the part without lifting off it."""
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      touchdown=Touchdown(retract_after_m=0.0),
    )

    self.assertLen(subtree.children, 5)
    self.assertNotIn(
      "Step 6c: Linear Retract (0.0 cm, -Z Tool)",
      [child.name for child in subtree.children],
    )
    self.assertNotIn(
      "move_relative_cartesian:(0.0, 0.0, -0.0):LINEAR",
      self.robot.executed_commands,
    )

  def test_build_return_to_infeed_subtree_with_reparenting_enabled(self):
    """Verifies 6-step structure when enable_object_reparenting=True."""
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      enable_object_reparenting=True,
    )
    self.assertEqual(subtree.name, "6. Obstacle-Aware Scatter Return Subtree")
    self.assertLen(subtree.children, 6)
    self.assertEqual(
      subtree.children[4].name,
      "Step 6e: Detach Part to root in Digital Twin",
    )

  def test_build_return_to_infeed_subtree_mock_hardware_execution(self):
    """Verifies mock hardware commands recorded during subtree build."""
    build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
    )

    self.assertEqual(self.gripper.command_log, ["open"])

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/transit->root/infeed_pre_grasp:ANY",
        "move_cartesian:root/infeed_grasp:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_blended_cartesian:root/infeed_pre_grasp->root/view:LINEAR/ANY",
      ],
    )

  def test_build_return_to_infeed_subtree_custom_arguments(self):
    """Verifies custom frame names, parent object, and subtree name."""
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      parent_object="tray",
      transit_frame_name="custom_present",
      preplace_frame_name="custom_preplace",
      place_frame_name="custom_place",
      view_frame_name="custom_view",
      name="Custom Scatter Return",
    )

    self.assertEqual(subtree.name, "Custom Scatter Return")
    self.assertLen(subtree.children, 5)
    self.assertEqual(
      subtree.children[0].name,
      "Step 6a: Blended Move to Pre-Place via custom_present"
      " (tray/custom_present -> tray/custom_preplace)",
    )
    self.assertEqual(
      subtree.children[1].name,
      "Step 6c: Linear Approach to Standoff (tray/custom_place)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 6c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[4].name,
      "Step 6f: Linear Retract to custom_preplace Blended to View Frame"
      " (tray/custom_preplace -> tray/custom_view)",
    )

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:tray/custom_present->tray/custom_preplace:ANY",
        "move_cartesian:tray/custom_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_blended_cartesian:tray/custom_preplace->tray/custom_view"
        ":LINEAR/ANY",
      ],
    )

  def test_build_return_to_infeed_subtree_workpiece_identifier_resolution(
    self,
  ):
    """Verifies resolution with world.build_detach_from_gripper_task."""
    mock_world = mock.MagicMock(spec=World)
    mock_world.build_detach_from_gripper_task.return_value = bt.Task(
      action=bt.PythonScript(function_body="pass"),
      name="Step 6e: Detach Part to root in Digital Twin",
    )
    wp_asset = Workpiece(asset_id="custom_asset_123", object_name="custom_part")
    tree_asset = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=wp_asset,
      enable_object_reparenting=True,
      world=mock_world,
    )
    self.assertEqual(
      tree_asset.children[4].name,
      "Step 6e: Detach Part to root in Digital Twin",
    )
    mock_world.build_detach_from_gripper_task.assert_called_with(
      object_name="custom_part",
      name="Step 6e: Detach Part to root in Digital Twin",
    )

  def test_build_return_to_infeed_subtree_with_solution(self):
    """Verifies passing solution handle to return_infeed subtree."""
    mock_solution = mock.MagicMock()
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      solution=mock_solution,
    )
    self.assertIsNotNone(subtree)
    self.assertLen(subtree.children, 5)

  def test_build_return_to_infeed_subtree_compliant_touchdown(self):
    """Verifies two-stage linear standoff and compliant touchdown mode."""
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      touchdown=Touchdown(force_n=10.0, standoff_m=0.010),
    )
    self.assertIsNotNone(subtree)
    self.assertLen(subtree.children, 5)
    self.assertEqual(
      subtree.children[1].name,
      "Step 6c: Linear Approach to Standoff (root/infeed_grasp)",
    )
    self.assertEqual(
      subtree.children[2].name,
      "Step 6c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/transit->root/infeed_pre_grasp:ANY",
        "move_cartesian:root/infeed_grasp:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=10.0",
        "move_blended_cartesian:root/infeed_pre_grasp->root/view:LINEAR/ANY",
      ],
    )

  def test_build_return_to_infeed_subtree_compliant_touchdown_default_force(
    self,
  ):
    """Verifies compliant touchdown uses default 8.0N contact force."""
    self.robot.executed_commands.clear()
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
    )
    self.assertIsNotNone(subtree)
    self.assertLen(subtree.children, 5)
    self.assertIn(
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
      self.robot.executed_commands,
    )

  def test_build_return_to_infeed_subtree_cartesian_motions_wrapped_in_retry(
    self,
  ):
    """Verifies that Cartesian motions are wrapped in bt.Retry."""
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      enable_object_reparenting=True,
    )
    self.assertIsInstance(subtree.children[0], bt.Retry)
    self.assertIsInstance(subtree.children[1], bt.Retry)
    self.assertIsInstance(subtree.children[5], bt.Retry)


if __name__ == "__main__":
  absltest.main()
