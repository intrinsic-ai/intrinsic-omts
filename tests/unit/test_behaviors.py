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
from src.behaviors.machining import build_machining_handshake_subtree
from src.behaviors.pick import build_pick_from_infeed_subtree
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.config import load_app_config
from src.core.infeed import PerceptionInfeedStrategy
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
    self.config = load_app_config("configs/omts/app_config.yaml")

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

    self.infeed_strategy = PerceptionInfeedStrategy(
      config=self.config.vision,
      view_frame_name=self.config.frames.view_frame,
    )

  def _build_master_tree(self, num_cycles: int, tree_name: str = "Test Tree"):
    return build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      config=self.config,
      num_cycles_override=num_cycles,
      tree_name=tree_name,
    )

  def test_build_master_behavior_tree_single_cycle(self):
    tree = self._build_master_tree(
      num_cycles=1, tree_name="Test Single Cycle Master Tree"
    )
    self.assertIsNotNone(tree)
    self.assertEqual(tree.name, "Test Single Cycle Master Tree")
    self.assertIsInstance(tree.root, bt.Sequence)
    self.assertEqual(len(tree.root.children), 5)

  def test_build_master_behavior_tree_repeat_cycles(self):
    tree = self._build_master_tree(num_cycles=4)
    self.assertIsInstance(tree.root, bt.Loop)
    self.assertEqual(tree.root.max_times, 4)

  def test_build_master_behavior_tree_continuous_loop(self):
    tree = self._build_master_tree(num_cycles=0)
    self.assertIsInstance(tree.root, bt.Loop)
    self.assertEqual(tree.root.max_times, 0)

  def test_build_infeed_pick_subtree_with_machine_prep(self):
    pick_subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      config=self.config,
      machine=self.machine,
    )

    self.assertIsNotNone(pick_subtree)
    self.assertEqual(len(pick_subtree.children), 11)
    self.machine.build_open_door_task.assert_called_once_with(
      name="Open CNC Door"
    )
    self.machine.build_open_vise_task.assert_called_once_with(
      name="Open CNC Vise"
    )
    self.vision.build_perception_and_spawn_task.assert_called_once_with(
      approach_offset_z=0.08,
      parent_object="root",
      pregrasp_frame_name="pre_grasp",
      grasp_frame_name="grasp",
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
      name="Perception & Dynamic Grasp Frame Update Pipeline",
    )
    self.robot.build_attach_object_task.assert_called_once_with(
      object_name="raw_stock_2x3x5",
      name="Attach raw_stock_2x3x5 to Gripper",
    )
    self.robot.build_move_relative_cartesian_task.assert_called_once_with(
      translation=(0.0, 0.0, -0.03),
      motion_type="LINEAR",
      exclude_collision=True,
      excluded_collision_objects=("raw_stock_2x3x5",),
      name="Linear Retract (3.0 cm, -Z Tool)",
    )

  def test_build_load_machine_subtree_steps_and_detachment(self):
    load_subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      config=self.config,
    )

    self.assertIsNotNone(load_subtree)
    self.assertEqual(len(load_subtree.children), 10)
    self.robot.build_move_blended_cartesian_task.assert_called_once()
    self.robot.build_detach_object_task.assert_called_once_with(
      object_name="raw_stock_2x3x5",
      name="Detach raw_stock_2x3x5 from Gripper",
    )
    expected_vise_pairs = [
      ("raw_stock_2x3x5", "schunk_egp_64nnb"),
      ("gripper", "schunk_egp_64nnb"),
      ("gripper", "raw_stock_2x3x5"),
    ]
    self.robot.build_move_cartesian_task.assert_has_calls(
      [
        mock.call(
          target_frame_name="vise_pre_place",
          target_object_name="root",
          motion_type="ANY",
          allow_tool_z_rotation=False,
          cone_opening_half_angle=0.0,
          moving_frame_offset=None,
          target_frame_offset=None,
          excluded_collision_pairs=expected_vise_pairs,
          name="Approach CNC Vise (root/vise_pre_place)",
        ),
        mock.call(
          target_frame_name="vise_pre_place",
          target_object_name="root",
          motion_type="LINEAR",
          allow_tool_z_rotation=False,
          cone_opening_half_angle=0.0,
          moving_frame_offset=None,
          target_frame_offset=None,
          excluded_collision_pairs=expected_vise_pairs,
          name="Retract Arm to Vise Approach (root/vise_pre_place)",
        ),
      ],
      any_order=False,
    )

  def test_build_machining_handshake_subtree_steps(self):
    machining_subtree = build_machining_handshake_subtree(
      robot=self.robot,
      machine=self.machine,
      config=self.config,
    )

    self.assertIsNotNone(machining_subtree)
    self.assertEqual(len(machining_subtree.children), 4)
    self.robot.build_move_cartesian_task.assert_called_once_with(
      target_frame_name="machine_approach",
      target_object_name="root",
      motion_type="ANY",
      allow_tool_z_rotation=False,
      cone_opening_half_angle=0.0,
      moving_frame_offset=None,
      target_frame_offset=None,
      excluded_collision_pairs=None,
      name="Move to Safe Standby (root/machine_approach)",
    )
    self.machine.build_close_door_task.assert_called_once_with(
      name="Close CNC Door"
    )
    self.machine.build_trigger_cycle_task.assert_called_once_with(
      name="Trigger CNC Cycle Start"
    )
    self.machine.build_wait_cycle_complete_task.assert_called_once_with(
      timeout_seconds=self.config.cycle.machining_timeout_seconds,
      name="Wait for CNC Cycle Complete",
    )

  def test_build_unload_machine_subtree_steps_and_attachment(self):
    unload_subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      config=self.config,
    )

    self.assertIsNotNone(unload_subtree)
    self.assertEqual(len(unload_subtree.children), 10)
    self.robot.build_attach_object_task.assert_called_once_with(
      object_name="raw_stock_2x3x5",
      name="Attach raw_stock_2x3x5 to Gripper",
    )
    self.robot.build_move_relative_cartesian_task.assert_has_calls(
      [
        mock.call(
          translation=(0.0, 0.0, -0.03),
          motion_type="LINEAR",
          exclude_collision=True,
          excluded_collision_objects=("raw_stock_2x3x5",),
          name="Linear Retract (3.0 cm, -Z Tool)",
        ),
        mock.call(
          translation=(0.0, 0.0, -0.03),
          motion_type="LINEAR",
          exclude_collision=True,
          excluded_collision_objects=("raw_stock_2x3x5",),
          name="Linear Retract Clear of Vise (3.0 cm, -Z Tool)",
        ),
      ],
      any_order=False,
    )
    expected_vise_pairs = [
      ("raw_stock_2x3x5", "schunk_egp_64nnb"),
      ("gripper", "schunk_egp_64nnb"),
      ("gripper", "raw_stock_2x3x5"),
    ]
    self.robot.build_move_cartesian_task.assert_has_calls(
      [
        mock.call(
          target_frame_name="vise_pre_place",
          target_object_name="root",
          motion_type="ANY",
          allow_tool_z_rotation=False,
          cone_opening_half_angle=0.0,
          moving_frame_offset=None,
          target_frame_offset=None,
          excluded_collision_pairs=expected_vise_pairs,
          name="Approach Machined Part (root/vise_pre_place)",
        ),
      ],
      any_order=False,
    )

  def test_build_return_to_infeed_subtree_steps_and_detachment(self):
    return_subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      config=self.config,
    )

    self.assertIsNotNone(return_subtree)
    self.assertEqual(len(return_subtree.children), 6)
    self.robot.build_move_blended_cartesian_task.assert_called_once()
    self.robot.build_detach_object_task.assert_called_once_with(
      object_name="raw_stock_2x3x5",
      name="Detach raw_stock_2x3x5 from Gripper",
    )
    self.robot.build_move_cartesian_task.assert_has_calls(
      [
        mock.call(
          target_frame_name="pre_grasp",
          target_object_name="root",
          motion_type="LINEAR",
          allow_tool_z_rotation=False,
          cone_opening_half_angle=0.0,
          moving_frame_offset=None,
          target_frame_offset=None,
          excluded_collision_pairs=[("gripper", "raw_stock_2x3x5")],
          name="Retract Arm from Table (root/pre_grasp)",
        ),
      ],
      any_order=False,
    )

  def test_build_master_behavior_tree_without_cnc_machine(self):
    lab_config = load_app_config("configs/lab_bb_01/app_config.yaml")
    tree = build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=None,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      config=lab_config,
      num_cycles_override=1,
    )
    self.assertIsNotNone(tree)
    self.assertIsInstance(tree.root, bt.Sequence)
    self.assertEqual(len(tree.root.children), 5)
    self.vision.build_perception_and_spawn_task.assert_called_once_with(
      approach_offset_z=0.08,
      parent_object="root",
      pregrasp_frame_name="pre_grasp",
      grasp_frame_name="grasp",
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
      name="Perception & Dynamic Grasp Frame Update Pipeline",
    )
    self.machine.build_open_door_task.assert_not_called()
    self.machine.build_close_door_task.assert_not_called()
    self.machine.build_open_vise_task.assert_not_called()
    self.machine.build_close_vise_task.assert_not_called()
    self.machine.build_trigger_cycle_task.assert_not_called()
    self.machine.build_wait_cycle_complete_task.assert_not_called()

  def test_all_subtrees_have_unique_child_names_and_no_legacy_prefixes(self):
    tree = self._build_master_tree(num_cycles=1)
    for subtree in tree.root.children:
      child_names = [child.name for child in subtree.children]
      self.assertEqual(
        len(child_names),
        len(set(child_names)),
        f"Duplicate child task names in {subtree.name}: {child_names}",
      )
      for name in child_names:
        self.assertNotIn("Step ", name)
        self.assertNotIn("Prep:", name)


if __name__ == "__main__":
  absltest.main()
