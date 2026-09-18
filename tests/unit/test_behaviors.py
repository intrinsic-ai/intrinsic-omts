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

"""Unit tests for Behavior Tree construction and structure."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.behaviors.load_machine import build_load_machine_subtree
from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.behaviors.pick import build_pick_from_infeed_subtree
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.config import VisionConfig
from src.core.infeed import PerceptionInfeedStrategy
from src.core.workpiece import Workpiece
from src.hardware.gripper import GripperInterface
from src.hardware.machine import CncMachineInterface
from src.hardware.robot import RobotInterface
from src.hardware.vision import VisionInterface


def _make_mock_node_builder(name_prefix: str):
  def _builder(*args, **kwargs):
    del args
    node_name = kwargs.get("name", name_prefix)
    return bt.Sequence(name=node_name, children=[])

  return _builder


class BehaviorsTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.robot = mock.MagicMock(spec=RobotInterface)
    self.robot.build_move_joint_task.side_effect = _make_mock_node_builder(
      "Move Joint"
    )
    self.robot.build_move_cartesian_task.side_effect = _make_mock_node_builder(
      "Move Cartesian"
    )
    self.robot.build_move_blended_cartesian_task.side_effect = (
      _make_mock_node_builder("Move Blended")
    )
    self.robot.build_move_relative_cartesian_task.side_effect = (
      _make_mock_node_builder("Move Relative")
    )
    self.robot.build_move_to_contact_task.side_effect = _make_mock_node_builder(
      "Move Contact"
    )
    self.robot.build_attach_object_task.side_effect = _make_mock_node_builder(
      "Attach Object"
    )
    self.robot.build_detach_object_task.side_effect = _make_mock_node_builder(
      "Detach Object"
    )

    self.gripper = mock.MagicMock(spec=GripperInterface)
    self.gripper.build_open_task.side_effect = _make_mock_node_builder(
      "Open Gripper"
    )
    self.gripper.build_close_task.side_effect = _make_mock_node_builder(
      "Close Gripper"
    )

    self.machine = mock.MagicMock(spec=CncMachineInterface)
    self.machine.build_open_door_task.side_effect = _make_mock_node_builder(
      "Open Door"
    )
    self.machine.build_close_door_task.side_effect = _make_mock_node_builder(
      "Close Door"
    )
    self.machine.build_open_vise_task.side_effect = _make_mock_node_builder(
      "Open Vise"
    )
    self.machine.build_close_vise_task.side_effect = _make_mock_node_builder(
      "Close Vise"
    )
    self.machine.build_trigger_cycle_task.side_effect = _make_mock_node_builder(
      "Trigger Cycle"
    )
    self.machine.build_wait_cycle_complete_task.side_effect = (
      _make_mock_node_builder("Wait Cycle")
    )

    self.vision = mock.MagicMock(spec=VisionInterface)
    self.vision.build_capture_image_task.side_effect = _make_mock_node_builder(
      "Capture Image"
    )
    self.vision.build_perception_and_spawn_task.side_effect = (
      _make_mock_node_builder("Perception Pipeline")
    )

    vision_config = VisionConfig(
      camera_name="orbbec_camera",
      perception_service_name="pose_estimator_service",
      pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator",
      scene_object_id="ai.intrinsic.raw_stock_2x3x5",
      sensor_ids=(1, 4),
      min_num_instances=1,
      infeed_mode="perception",
      min_safe_z=0.95,
    )
    self.infeed_strategy = PerceptionInfeedStrategy(
      config=vision_config,
      view_frame_name="view",
    )
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
    self.assertEqual(len(return_subtree.children), 5)


if __name__ == "__main__":
  absltest.main()
