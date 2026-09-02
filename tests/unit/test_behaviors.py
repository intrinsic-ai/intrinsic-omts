"""Unit tests for Behavior Tree construction and structure."""

from absl.testing import absltest
from src.behaviors.building_block_bt import build_building_block_pick_place_tree
from src.behaviors.load_machine import build_load_machine_subtree
from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.behaviors.pick import build_pick_from_infeed_subtree
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.infeed import PerceptionInfeedStrategy
from src.core.workpiece import Workpiece
from src.hardware.gripper import MockGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot
from src.hardware.vision import MockVision


class BehaviorsTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.gripper = MockGripper()
    self.machine = MockCncMachine()
    self.vision = MockVision()
    self.infeed_strategy = PerceptionInfeedStrategy()
    self.workpiece = Workpiece(id="test_part_01")

  def test_build_master_behavior_tree_hierarchy(self):
    tree = build_machine_tending_behavior_tree(
        robot=self.robot,
        gripper=self.gripper,
        machine=self.machine,
        vision=self.vision,
        infeed_strategy=self.infeed_strategy,
        workpiece=self.workpiece,
        tree_name="Test Machine Tending Master Tree",
    )

    self.assertIsNotNone(tree)
    self.assertEqual(tree.name, "Test Machine Tending Master Tree")
    self.assertIsNotNone(tree.root)
    # Root should be a Sequence with 5 major subtrees
    self.assertEqual(len(tree.root.children), 5)

  def test_build_infeed_pick_subtree_steps(self):
    pick_subtree = build_pick_from_infeed_subtree(
        robot=self.robot,
        gripper=self.gripper,
        vision=self.vision,
        infeed_strategy=self.infeed_strategy,
        workpiece=self.workpiece,
        parent_object="root",
        view_frame_name="view",
        pregrasp_frame_name="pre_grasp",
        grasp_frame_name="grasp",
    )

    self.assertIsNotNone(pick_subtree)
    # Sequence of 6 steps:
    # 1: Move to view (ANY)
    # 2: Mock Perception
    # 3: Move to pre_grasp (ANY)
    # 4: Compliant Touchdown (-Z)
    # 5: Mock Close Gripper
    # 6: Linear Retract to pre_grasp (LINEAR)
    self.assertEqual(len(pick_subtree.children), 6)

  def test_build_load_machine_subtree_steps(self):
    load_subtree = build_load_machine_subtree(
        robot=self.robot,
        gripper=self.gripper,
        machine=self.machine,
        workpiece=self.workpiece,
        parent_object="root",
        machine_approach_frame_name="machine_approach",
        preplace_vise_frame_name="pre_place_vise",
        place_vise_frame_name="place_vise",
    )

    self.assertIsNotNone(load_subtree)
    # Sequence of 9 steps:
    # 1: Open CNC Door (Mock)
    # 2: Open CNC Vise (Mock)
    # 3: Move to machine_approach (ANY)
    # 4: Move to pre_place_vise (ANY)
    # 5: Compliant Touchdown into Vise (+Z Tool)
    # 6: Clamp CNC Vise (Mock)
    # 7: Release Part in Vise (Mock)
    # 8: Retract Arm to pre_place_vise (LINEAR)
    # 9: Retract Arm to machine_approach (LINEAR)
    self.assertEqual(len(load_subtree.children), 9)

  def test_build_unload_machine_subtree_steps(self):
    unload_subtree = build_unload_machine_subtree(
        robot=self.robot,
        gripper=self.gripper,
        machine=self.machine,
        workpiece=self.workpiece,
        parent_object="root",
        machine_approach_frame_name="machine_approach",
        preplace_vise_frame_name="pre_place_vise",
        place_vise_frame_name="place_vise",
    )

    self.assertIsNotNone(unload_subtree)
    # Sequence of 8 steps:
    # 1: Open CNC Door (Mock)
    # 2: Open CNC Vise (Mock)
    # 3: Move to machine_approach (ANY)
    # 4: Move to pre_place_vise (ANY)
    # 5: Compliant Touchdown to Machined Part (+Z Tool)
    # 6: Grasp Machined Part (Mock)
    # 7: Retract Arm to pre_place_vise (LINEAR)
    # 8: Retract Arm to machine_approach (LINEAR)
    self.assertEqual(len(unload_subtree.children), 8)

  def test_build_return_to_infeed_subtree_steps(self):
    return_subtree = build_return_to_infeed_subtree(
        robot=self.robot,
        gripper=self.gripper,
        workpiece=self.workpiece,
        parent_object="root",
        pregrasp_frame_name="pre_grasp",
        grasp_frame_name="grasp",
        view_frame_name="view",
    )

    self.assertIsNotNone(return_subtree)
    # Sequence of 5 steps:
    # 1: Move to pre_grasp (ANY)
    # 2: Compliant Touchdown to Table (-Z)
    # 3: Release Finished Part (Mock)
    # 4: Retract Arm to pre_grasp (LINEAR)
    # 5: Return to view (ANY)
    self.assertEqual(len(return_subtree.children), 5)

  def test_build_building_block_pick_place_tree(self):
    tree = build_building_block_pick_place_tree(
        robot=self.robot,
        gripper=self.gripper,
        parent_object="root",
        pregrasp_frame_name="dynamic_pregrasp",
        grasp_frame_name="dynamic_grasp",
        preplace_frame_name="dynamic_preplace",
        place_frame_name="dynamic_place",
        view_frame_name="view",
    )

    self.assertIsNotNone(tree)
    self.assertEqual(tree.name, "Building Block Pick & Place Cycle")
    self.assertIsNotNone(tree.root)
    # Sequence of 10 steps:
    # 1: Move to Pre-Grasp (ANY)
    # 2: Open Gripper (Prepare Grasp)
    # 3: Move to Grasp (LINEAR)
    # 4: Close Gripper (Grasp Block)
    # 5: Linear Retract Up (LINEAR)
    # 6: Transit to Pre-Place (ANY)
    # 7: Move to Place (LINEAR)
    # 8: Open Gripper (Release Block)
    # 9: Linear Retract from Place (LINEAR)
    # 10: Return to View (ANY)
    self.assertEqual(len(tree.root.children), 10)
    self.assertEqual(
        self.robot.executed_commands,
        [
            "move_cartesian:root/dynamic_pregrasp:ANY:z_rot=False",
            "move_cartesian:root/dynamic_grasp:LINEAR:z_rot=False",
            "move_cartesian:root/dynamic_pregrasp:LINEAR:z_rot=False",
            "move_cartesian:root/dynamic_preplace:ANY:z_rot=False",
            "move_cartesian:root/dynamic_place:LINEAR:z_rot=False",
            "move_cartesian:root/dynamic_preplace:LINEAR:z_rot=False",
            "move_cartesian:root/view:ANY:z_rot=False",
        ],
    )
    self.assertEqual(self.gripper.command_log, ["open", "close", "open"])

  def test_build_building_block_pick_place_tree_with_vision(self):
    tree = build_building_block_pick_place_tree(
        robot=self.robot,
        gripper=self.gripper,
        vision=self.vision,
        target_object="ai.intrinsic.raw_stock_2x3x5",
        pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator",
        parent_object="root",
        pregrasp_frame_name="dynamic_pregrasp",
        grasp_frame_name="dynamic_grasp",
        preplace_frame_name="dynamic_preplace",
        place_frame_name="dynamic_place",
        view_frame_name="view",
    )

    self.assertIsNotNone(tree)
    self.assertEqual(tree.name, "Building Block Pick & Place Cycle")
    self.assertIsNotNone(tree.root)
    # Sequence of 11 steps:
    # 1: Move to Pre-Grasp (ANY)
    # 2: Estimate & Update Pose (ai.intrinsic.raw_stock_2x3x5)
    # 3: Open Gripper (Prepare Grasp)
    # 4: Move to Grasp (LINEAR)
    # 5: Close Gripper (Grasp Block)
    # 6: Linear Retract Up (LINEAR)
    # 7: Transit to Pre-Place (ANY)
    # 8: Move to Place (LINEAR)
    # 9: Open Gripper (Release Block)
    # 10: Linear Retract from Place (LINEAR)
    # 11: Return to View (ANY)
    self.assertEqual(len(tree.root.children), 11)
    self.assertEqual(
        tree.root.children[1].name,
        "2. Estimate & Update Pose (ai.intrinsic.raw_stock_2x3x5)",
    )
    self.assertEqual(
        tree.root.children[2].name,
        "3. Open Gripper (Prepare Grasp)",
    )
    self.assertEqual(
        tree.root.children[3].name,
        "4. Move to Grasp (root/dynamic_grasp)",
    )
    self.assertEqual(self.vision.pipeline_count, 1)
    self.assertEqual(
        self.robot.executed_commands,
        [
            "move_cartesian:root/dynamic_pregrasp:ANY:z_rot=False",
            "move_cartesian:root/dynamic_grasp:LINEAR:z_rot=False",
            "move_cartesian:root/dynamic_pregrasp:LINEAR:z_rot=False",
            "move_cartesian:root/dynamic_preplace:ANY:z_rot=False",
            "move_cartesian:root/dynamic_place:LINEAR:z_rot=False",
            "move_cartesian:root/dynamic_preplace:LINEAR:z_rot=False",
            "move_cartesian:root/view:ANY:z_rot=False",
        ],
    )
    self.assertEqual(self.gripper.command_log, ["open", "close", "open"])


if __name__ == "__main__":
  absltest.main()
