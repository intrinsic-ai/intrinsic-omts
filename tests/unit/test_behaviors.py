"""Unit tests for Behavior Tree construction and structure."""

from absl.testing import absltest
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
    # Sequence of 8 steps:
    # 1: Move to view (ANY)
    # 2: Mock Perception
    # 3: Mock Open Gripper
    # 4: Move to pre_grasp (ANY)
    # 5: Compliant Touchdown (+Z Tool)
    # 6: Linear Retract 3cm (-Z Tool)
    # 7: Mock Close Gripper
    # 8: Linear Retract to pre_grasp (LINEAR)
    self.assertEqual(len(pick_subtree.children), 8)

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
    # Sequence of 9 steps:
    # 1: Open CNC Door (Mock)
    # 2: Open CNC Vise (Mock)
    # 3: Move to machine_approach (ANY)
    # 4: Move to pre_place_vise (ANY)
    # 5: Compliant Touchdown to Machined Part (+Z Tool)
    # 6: Linear Retract 3cm (-Z Tool)
    # 7: Grasp Machined Part (Mock)
    # 8: Retract Arm to pre_place_vise (LINEAR)
    # 9: Retract Arm to machine_approach (LINEAR)
    self.assertEqual(len(unload_subtree.children), 9)

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


if __name__ == "__main__":
  absltest.main()
