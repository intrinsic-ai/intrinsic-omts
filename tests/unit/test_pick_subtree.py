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

"""Unit tests for infeed pick subtree generation and parallel execution."""

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.behaviors.pick import build_pick_from_infeed_subtree
from src.core.infeed import GridInfeedStrategy, PerceptionInfeedStrategy
from src.core.tray import Tray
from src.core.workpiece import Workpiece
from src.hardware.gripper import MockGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot
from src.hardware.vision import MockVision


class PickSubtreeTest(absltest.TestCase):
  """Tests for build_pick_from_infeed_subtree structure and execution."""

  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.gripper = MockGripper()
    self.machine = MockCncMachine()
    self.vision = MockVision()
    self.infeed_strategy = PerceptionInfeedStrategy()
    self.workpiece = Workpiece(
      asset_id="ai.intrinsic.raw_stock_2x3x5", object_name="raw_stock_2x3x5"
    )

  def test_build_pick_subtree_with_machine_structure(self):
    """Verifies bt.Parallel perception and machine prep with machine adapter."""
    subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      machine=self.machine,
      parent_object="root",
      view_frame_name="view",
      pregrasp_frame_name="infeed_pre_grasp",
      grasp_frame_name="infeed_grasp",
    )

    self.assertIsNotNone(subtree)
    self.assertIsInstance(subtree, bt.Sequence)
    self.assertEqual(len(subtree.children), 7)

    # Step 01: Parallel move to view frame, close gripper, open door & vise
    step_01 = subtree.children[0]
    self.assertIsInstance(step_01, bt.Parallel)
    self.assertEqual(step_01.name, "Step 01: Move to View & Prep Machine")
    self.assertEqual(len(step_01.children), 4)
    self.assertEqual(
      step_01.children[0].name, "Step 01a: Move to View Frame (root/view)"
    )
    self.assertEqual(
      step_01.children[1].name, "Step 01b: Close Gripper (Clear Camera FOV)"
    )
    self.assertEqual(step_01.children[2].name, "Open CNC Door")
    self.assertEqual(step_01.children[3].name, "Open CNC Vise")

    # Step 02a: Capture RGB-D images
    step_02a = subtree.children[1]
    self.assertEqual(step_02a.name, "Step 02a: Capture RGB-D Images")

    # Step 02b: Parallel perception and gripper open
    step_02b = subtree.children[2]
    self.assertIsInstance(step_02b, bt.Parallel)
    self.assertEqual(
      step_02b.name, "Step 02b: Parallel Estimation & Open Gripper"
    )
    self.assertEqual(len(step_02b.children), 2)

    branch_a = step_02b.children[0]
    self.assertEqual(
      branch_a.name, "Perception & Dynamic Grasp Frame Update Pipeline"
    )

    branch_b = step_02b.children[1]
    self.assertEqual(branch_b.name, "Open Gripper")

    # Step 03: Blended approach to standoff and compliant touchdown
    step_03a = subtree.children[3]
    self.assertEqual(
      step_03a.name,
      "Step 03: Blended Approach to Standoff (root/infeed_grasp)",
    )
    step_03b = subtree.children[4]
    self.assertEqual(step_03b.name, "Step 03: Compliant Touchdown (+Z Tool)")

    # Step 04: Close gripper
    step_04 = subtree.children[5]
    self.assertEqual(step_04.name, "Step 04: Close Gripper (Grasp Part)")

    # Step 06: Linear retract
    step_06 = subtree.children[6]
    self.assertEqual(
      step_06.name,
      "Step 06: Linear Retract to Pre-Grasp (root/infeed_pre_grasp)",
    )

    # Verify mock executions
    self.assertEqual(self.machine.command_log, ["open_door", "open_vise"])
    self.assertEqual(self.gripper.command_log, ["close", "open", "close"])
    self.assertEqual(
      self.robot.executed_commands,
      [
        "move_cartesian:root/view:ANY",
        "move_blended_cartesian:root/infeed_pre_grasp->root/infeed_grasp:ANY/LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_cartesian:root/infeed_pre_grasp:LINEAR",
      ],
    )

  def test_build_pick_subtree_with_reparenting_enabled(self):
    """Verifies 8-step structure when enable_object_reparenting=True."""
    subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      machine=self.machine,
      parent_object="root",
      view_frame_name="view",
      pregrasp_frame_name="infeed_pre_grasp",
      grasp_frame_name="infeed_grasp",
      enable_object_reparenting=True,
    )
    self.assertEqual(len(subtree.children), 8)
    self.assertEqual(
      subtree.children[6].name,
      "Step 05: Attach Part to Gripper in Digital Twin",
    )

  def test_build_pick_subtree_without_machine_structure(self):
    """Verifies parallel estimation and gripper open when machine is None."""
    subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      machine=None,
      parent_object="root",
      view_frame_name="view",
      pregrasp_frame_name="infeed_pre_grasp",
      grasp_frame_name="infeed_grasp",
    )

    self.assertIsNotNone(subtree)
    self.assertIsInstance(subtree, bt.Sequence)
    self.assertEqual(len(subtree.children), 7)

    step_01 = subtree.children[0]
    self.assertIsInstance(step_01, bt.Parallel)
    self.assertEqual(len(step_01.children), 2)

    step_02a = subtree.children[1]
    self.assertEqual(step_02a.name, "Step 02a: Capture RGB-D Images")

    step_02b = subtree.children[2]
    self.assertIsInstance(step_02b, bt.Parallel)
    self.assertEqual(
      step_02b.name, "Step 02b: Parallel Estimation & Open Gripper"
    )
    self.assertEqual(len(step_02b.children), 2)
    self.assertEqual(
      step_02b.children[0].name,
      "Perception & Dynamic Grasp Frame Update Pipeline",
    )
    self.assertEqual(step_02b.children[1].name, "Open Gripper")

    self.assertEqual(self.machine.command_log, [])
    self.assertEqual(self.gripper.command_log, ["close", "open", "close"])

  def test_build_pick_subtree_custom_frame_names(self):
    """Verifies custom frame and parent object arguments."""
    subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      machine=self.machine,
      parent_object="tray_base",
      view_frame_name="custom_cam_view",
      pregrasp_frame_name="custom_pregrasp",
      grasp_frame_name="custom_grasp",
    )

    self.assertEqual(
      subtree.children[0].children[0].name,
      "Step 01a: Move to View Frame (tray_base/custom_cam_view)",
    )
    self.assertEqual(
      subtree.children[3].name,
      "Step 03: Blended Approach to Standoff (tray_base/custom_grasp)",
    )
    self.assertEqual(
      subtree.children[4].name,
      "Step 03: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      subtree.children[6].name,
      "Step 06: Linear Retract to Pre-Grasp (tray_base/custom_pregrasp)",
    )

  def test_build_pick_subtree_workpiece_identifier_fallbacks(self):
    """Verifies workpiece attribute hierarchy for ObjectWorld attachment."""
    wp_asset = Workpiece(
      asset_id="ai.intrinsic.special_asset", object_name="special_asset"
    )
    tree_asset = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=wp_asset,
      enable_object_reparenting=True,
    )
    self.assertEqual(
      tree_asset.children[6].name,
      "Step 05: Attach Part to Gripper in Digital Twin",
    )

  def test_build_pick_subtree_grid_infeed_fallback(self):
    """Verifies grid infeed strategy skips perception steps."""
    tray = Tray(
      name="test_tray",
      rows=1,
      cols=1,
      pitch_x=0.1,
      pitch_y=0.1,
      origin_frame="tray_origin",
    )
    grid_strategy = GridInfeedStrategy(tray=tray)
    subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=grid_strategy,
      workpiece=self.workpiece,
      machine=self.machine,
    )
    self.assertIsNotNone(subtree)
    self.assertEqual(len(subtree.children), 5)
    self.assertEqual(subtree.children[0].name, "CNC Machine & Gripper Prep")

  def test_build_pick_subtree_forwards_perception_max_retries(self):
    subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      machine=self.machine,
      perception_max_retries=4,
    )
    self.assertIsNotNone(subtree)
    self.assertEqual(self.vision.last_max_tries, 4)

  def test_build_pick_subtree_cartesian_motions_wrapped_in_retry(self):
    """Verifies that Cartesian motions are wrapped in bt.Retry."""
    subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      machine=self.machine,
      enable_object_reparenting=True,
    )
    self.assertIsInstance(subtree.children[0].children[0], bt.Retry)
    self.assertIsInstance(subtree.children[3], bt.Retry)
    self.assertIsInstance(subtree.children[7], bt.Retry)

  def test_build_pick_subtree_propagates_perception_retry_parameters(self):
    """Verifies that perception retry parameters are passed to vision."""
    build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      machine=self.machine,
      perception_max_retries=5,
      perception_retry_delay_sec=2.0,
    )
    self.assertEqual(self.vision.last_max_tries, 5)
    self.assertEqual(self.vision.last_retry_delay_sec, 2.0)


if __name__ == "__main__":
  absltest.main()
