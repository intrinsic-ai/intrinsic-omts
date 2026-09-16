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

from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.core.types import GripperState
from src.core.workpiece import Workpiece
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
    """Verifies default 4-step subtree sequence structure and task names."""
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

    # Step 6a: Blended move to pre-place via presentation view (ANY)
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

    # Step 6f: Linear retract arm to pre-place
    self.assertEqual(
      subtree.children[4].name,
      "Step 6f: Linear Retract to Pre-Place (root/infeed_pre_grasp)",
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

    # Gripper should be opened
    self.assertEqual(self.gripper.commanded_state, GripperState.OPEN)
    self.assertEqual(self.gripper.command_log, ["open"])

    # Robot motions: blended [transit, preplace] (ANY) -> standoff -> touchdown -> retract (LINEAR)
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:root/transit->root/infeed_pre_grasp:ANY",
        "move_cartesian:root/infeed_grasp:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_cartesian:root/infeed_pre_grasp:LINEAR",
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
      "Step 6f: Linear Retract to Pre-Place (tray/custom_preplace)",
    )

    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_blended_cartesian:tray/custom_present->tray/custom_preplace:ANY",
        "move_cartesian:tray/custom_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_cartesian:tray/custom_preplace:LINEAR",
      ],
    )

  def test_build_return_to_infeed_subtree_workpiece_identifier_resolution(
    self,
  ):
    """Verifies identifier resolution with world.build_reparent_task."""
    mock_world = mock.MagicMock()
    mock_world.build_reparent_task.return_value = bt.Task(
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
    self.assertIsNotNone(tree_asset.children[4])
    mock_world.build_reparent_task.assert_called_with(
      target="custom_asset_123",
      new_parent="root",
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
      contact_force_newtons=10.0,
      standoff_distance_m=0.010,
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
        "move_cartesian:root/infeed_pre_grasp:LINEAR",
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

  def test_build_return_to_infeed_subtree_clear_motion_planner_cache(self):
    """Verifies clear_motion_planner_cache steps when enabled."""
    mock_machine = mock.MagicMock()
    mock_machine.build_close_door_task.return_value = bt.Task(
      name="Step 6a-2: Close CNC Door",
      action=bt.PythonScript(function_body="pass\n"),
    )
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      machine=mock_machine,
      enable_object_reparenting=True,
      clear_motion_planner_cache=True,
    )
    task_names = [child.name for child in subtree.children]
    self.assertIn("Step 6a-4: Clear Motion Planner Cache", task_names)
    self.assertIn("Step 6e-2: Clear Motion Planner Cache", task_names)

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

  def test_build_return_to_infeed_subtree_return_to_view_frame(self):
    """Verifies step 6f blends to the view frame with a LINEAR departure."""
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      return_to_view_frame=True,
      view_frame_name="view",
    )

    # Step 6f replaces the linear retract, so the step count is unchanged.
    self.assertLen(subtree.children, 5)
    self.assertEqual(
      subtree.children[4].name,
      "Step 6f: Linear Retract to infeed_pre_grasp Blended to View Frame"
      " (root/infeed_pre_grasp -> root/view)",
    )
    # The lift off the just-released part must not be planned freely.
    self.assertEqual(
      self.robot.executed_commands[-1],
      "move_blended_cartesian:root/infeed_pre_grasp->root/view:LINEAR/ANY",
    )

  def test_build_return_to_infeed_subtree_view_frame_defaults_to_view(
    self,
  ):
    """Verifies the view retreat falls back to view."""
    subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      workpiece=self.workpiece,
      return_to_view_frame=True,
      view_frame_name=None,
    )

    self.assertEqual(
      subtree.children[4].name,
      "Step 6f: Linear Retract to infeed_pre_grasp Blended to View Frame"
      " (root/infeed_pre_grasp -> root/view)",
    )


if __name__ == "__main__":
  absltest.main()
