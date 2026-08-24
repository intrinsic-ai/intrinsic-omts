"""Unit tests for Behavior Tree construction and structure."""

from absl.testing import absltest
from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.core.infeed import PerceptionInfeedStrategy
from src.core.workpiece import Workpiece
from src.hardware.gripper import MockGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot
from src.hardware.vision import MockVision


class BehaviorsTest(absltest.TestCase):

  def test_build_master_behavior_tree_hierarchy(self):
    robot = MockRobot()
    gripper = MockGripper()
    machine = MockCncMachine()
    vision = MockVision()
    infeed_strategy = PerceptionInfeedStrategy()
    workpiece = Workpiece(id="test_part_01")

    tree = build_machine_tending_behavior_tree(
        robot=robot,
        gripper=gripper,
        machine=machine,
        vision=vision,
        infeed_strategy=infeed_strategy,
        workpiece=workpiece,
        tree_name="Test Machine Tending Master Tree",
    )

    self.assertIsNotNone(tree)
    self.assertEqual(tree.name, "Test Machine Tending Master Tree")
    self.assertIsNotNone(tree.root)
    # Root should be a Sequence with 5 major subtrees
    self.assertEqual(len(tree.root.children), 5)


if __name__ == "__main__":
  absltest.main()
