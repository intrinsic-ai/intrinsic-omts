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

from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.behaviors.motions import (
  Touchdown,
  create_move_through_frames_task,
  create_seated_approach_tasks,
)
from src.core.infeed import PerceptionInfeedStrategy
from src.core.types import Frames
from src.core.workpiece import Workpiece
from src.core.world import MockWorld
from src.hardware.gripper import MockGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot
from src.hardware.vision import MockVision, OrbbecVision


class BehaviorsTest(absltest.TestCase):
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
    self.assertEqual(tree.root.name, "OMTS Master Machine Tending Pipeline")
    self.assertLen(tree.root.children, 5)

    expected_subtree_names = [
      "1. Infeed Pick Subtree",
      "3. Load Machine Subtree",
      "4. Machining Handshake Subtree",
      "5. Unload Machine Subtree",
      "6. Obstacle-Aware Scatter Return Subtree",
    ]
    for idx, expected_name in enumerate(expected_subtree_names):
      self.assertEqual(tree.root.children[idx].name, expected_name)

    # Pick subtree: 9 steps (includes view, capture, parallel estimate/prep, pregrasp, approach, touchdown, retract, close, retract; no attach)
    self.assertLen(tree.root.children[0].children, 9)
    # Load machine subtree: 8 steps (blended approach + standoff + compliant touchdown into vise; cache clear; no detach/attach)
    self.assertLen(tree.root.children[1].children, 8)
    # Machining subtree: 5 steps
    self.assertLen(tree.root.children[2].children, 5)
    # Unload machine subtree: 10 steps (standoff approach + compliant touchdown from vise and retract; no detach/attach)
    self.assertLen(tree.root.children[3].children, 10)
    # Return infeed subtree: 5 steps (compliant placement to table; no detach)
    self.assertLen(tree.root.children[4].children, 5)
    self.assertIsNotNone(tree.proto)

    # Verify mock hardware execution across complete cycle
    self.assertEqual(self.vision.pipeline_count, 1)
    self.assertEqual(
      self.machine.command_log,
      [
        "open_door",
        "open_vise",
        "close_vise",
        "close_door",
        "trigger_cycle",
        "wait_cycle_complete",
        "open_door",
        "open_vise",
      ],
    )
    self.assertEqual(
      self.gripper.command_log,
      ["open", "close", "open", "close", "open"],
    )
    expected_robot_commands = [
      "move_cartesian:root/view:ANY",
      "move_cartesian:root/infeed_pre_grasp:ANY",
      "move_cartesian:root/infeed_grasp:LINEAR",
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
      "move_relative_cartesian:(0.0, 0.0, -0.005):LINEAR",
      "move_cartesian:root/infeed_pre_grasp:LINEAR",
      "move_blended_cartesian:root/transit->root/machine_approach:ANY",
      "move_cartesian:root/vise_pre_place:ANY",
      "move_cartesian:root/vise_place:LINEAR",
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
      "move_cartesian:root/vise_pre_place:LINEAR",
      "move_cartesian:root/machine_approach:LINEAR",
      "move_cartesian:root/machine_approach:ANY",
      "move_cartesian:root/vise_pre_place:LINEAR",
      "move_cartesian:root/vise_place:LINEAR",
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
      "move_relative_cartesian:(0.0, 0.0, -0.005):LINEAR",
      "move_cartesian:root/vise_pre_place:LINEAR",
      "move_cartesian:root/machine_approach:LINEAR",
      "move_blended_cartesian:root/transit->root/infeed_pre_grasp:ANY",
      "move_cartesian:root/infeed_grasp:LINEAR",
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
      "move_cartesian:root/infeed_pre_grasp:LINEAR",
    ]
    self.assertEqual(self.robot.executed_commands, expected_robot_commands)

  def test_build_master_behavior_tree_with_compliant_touchdown_enabled(self):
    tree = build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      return_compliant_touchdown=True,
    )
    # Return infeed subtree has 5 steps when compliant touchdown is enabled without reparenting
    self.assertLen(tree.root.children[4].children, 5)
    self.assertEqual(
      tree.root.children[4].children[1].name,
      "Step 6c: Linear Approach to Standoff (root/infeed_grasp)",
    )
    self.assertEqual(
      tree.root.children[4].children[2].name,
      "Step 6c: Compliant Touchdown (+Z Tool)",
    )
    self.assertIn(
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
      self.robot.executed_commands,
    )

  def test_build_master_behavior_tree_with_return_to_view_frame(self):
    tree = build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      return_to_view_frame=True,
      view_frame_name="view",
    )
    return_subtree = tree.root.children[4]
    self.assertEqual(
      return_subtree.children[-1].name,
      "Step 6f: Linear Retract to infeed_pre_grasp Blended to View Frame"
      " (root/infeed_pre_grasp -> root/view)",
    )
    self.assertIn(
      "move_blended_cartesian:root/infeed_pre_grasp->root/view:LINEAR/ANY",
      self.robot.executed_commands,
    )

  def test_build_master_bt_with_load_compliant_touchdown(self):
    tree = build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      load_compliant_touchdown=True,
      load_contact_force_n=12.0,
    )
    self.assertEqual(
      tree.root.children[1].children[2].name,
      "Step 3c: Linear Approach to Standoff (root/vise_place)",
    )
    self.assertEqual(
      tree.root.children[1].children[3].name,
      "Step 3c: Compliant Touchdown (+Z Tool)",
    )
    self.assertIn(
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=12.0",
      self.robot.executed_commands,
    )

  def test_build_master_bt_with_unload_compliant_touchdown(self):
    tree = build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      unload_compliant_touchdown=True,
      unload_contact_force_n=14.0,
    )
    self.assertLen(tree.root.children[3].children, 10)
    self.assertEqual(
      tree.root.children[3].children[2].name,
      "Step 5c: Linear Approach to Standoff (root/vise_place)",
    )
    self.assertEqual(
      tree.root.children[3].children[3].name,
      "Step 5c: Compliant Touchdown (+Z Tool)",
    )
    self.assertIn(
      "move_to_contact:dir=(0.0, 0.0, 1.0),force=14.0",
      self.robot.executed_commands,
    )

  def test_build_master_behavior_tree_custom_vise_approach_and_timeout(self):
    tree = build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      vise_approach_frame_name="custom_vise_approach",
      machining_timeout_seconds=45.0,
    )
    self.assertLen(tree.root.children, 5)
    load_step_3b = tree.root.children[1].children[1]
    self.assertEqual(
      load_step_3b.name,
      "Step 3b: Move to Vise Approach (root/custom_vise_approach)",
    )

  def test_build_machine_tending_bt_start_phase_unload(self):
    """Verifies start_phase='unload' produces only unload and return subtrees."""
    tree = build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      workpiece=self.workpiece,
      start_phase="unload",
    )
    children = tree.root.children
    self.assertEqual(len(children), 2)
    self.assertIn("Unload", children[0].name)
    self.assertIn("Return", children[1].name)

  def test_build_machine_tending_bt_start_phase_invalid(self):
    """Verifies that an invalid start_phase raises ValueError."""
    with self.assertRaises(ValueError):
      build_machine_tending_behavior_tree(
        robot=self.robot,
        gripper=self.gripper,
        machine=self.machine,
        vision=self.vision,
        infeed_strategy=self.infeed_strategy,
        workpiece=self.workpiece,
        start_phase="nonexistent_phase",
      )

  def test_world_build_reparent_tasks(self):
    world = MockWorld()
    attach_task = world.build_reparent_task(
      target="ai.intrinsic.raw_stock_2x3x5",
      new_parent="gripper",
      name="Step 07: Attach Block to Gripper",
    )
    self.assertIsNotNone(attach_task)
    self.assertEqual(attach_task.name, "Step 07: Attach Block to Gripper")

    detach_task = world.build_reparent_task(
      target="ai.intrinsic.raw_stock_2x3x5",
      new_parent="root",
      name="Step 12: Detach Block from Gripper",
    )
    self.assertIsNotNone(detach_task)
    self.assertEqual(detach_task.name, "Step 12: Detach Block from Gripper")

  def test_orbbec_vision_proto_signature_matches_calculator_parameters(self):
    mock_solution = mock.MagicMock()
    mock_action = mock.MagicMock(spec=bt.ActionBase)
    mock_solution.skills.ai.intrinsic.capture_images.return_value = mock_action
    mock_solution.skills.ai.intrinsic.estimate_pose_multi_view.return_value = (
      mock_action
    )
    vision = OrbbecVision(
      solution=mock_solution,
      camera_name="camera",
      perception_service_name="service",
    )
    vision.build_perception_and_spawn_task()
    mock_builder = mock_solution.proto_builder.create_signature_with_args
    mock_builder.assert_called_once()
    fields = mock_builder.call_args.kwargs["parameters"].fields
    field_dict = {f.name: f for f in fields}

    # The field names become the parameter names the injected calculator
    # script reads, so the set and the numbering are both load-bearing.
    self.assertEqual(
      {name: f.number for name, f in field_dict.items()},
      {
        "approach_offset_z": 1,
        "parent_object": 2,
        "pregrasp_frame_name": 3,
        "grasp_frame_name": 4,
        "camera_name": 5,
        "target_scene_object_id": 6,
        "estimates": 7,
        "min_safe_z": 8,
      },
    )
    self.assertEqual(field_dict["pregrasp_frame_name"].arg, "infeed_pre_grasp")
    self.assertEqual(field_dict["grasp_frame_name"].arg, "infeed_grasp")

  def test_create_move_through_frames_task_single_frame(self):
    task = create_move_through_frames_task(
      robot=self.robot,
      frame_names=["target_frame"],
      parent_object="root",
      motion_type="ANY",
    )
    self.assertIsInstance(task, bt.Node)
    self.assertIn(
      "move_cartesian:root/target_frame:ANY",
      self.robot.executed_commands,
    )

  def test_create_move_through_frames_task_multiple_frames(self):
    task = create_move_through_frames_task(
      robot=self.robot,
      frame_names=["transit", "machine_approach"],
      parent_object="root",
      motion_type="ANY",
      max_tries=2,
      retry_delay_sec=1.0,
    )
    self.assertIsInstance(task, bt.Retry)
    self.assertIn(
      "move_blended_cartesian:root/transit->root/machine_approach:ANY",
      self.robot.executed_commands,
    )

  def test_create_move_through_frames_task_empty_raises(self):
    with self.assertRaises(ValueError):
      create_move_through_frames_task(
        robot=self.robot,
        frame_names=[],
      )

  def test_main_defines_start_phase_flag(self):
    import inspect

    from absl import flags as absl_flags

    import src.main as main_mod

    self.assertIn("start_phase", absl_flags.FLAGS)
    self.assertIn(
      "start_phase",
      inspect.signature(main_mod.run_machine_tending_cycle).parameters,
    )

  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_run_machine_tending_cycle_clamps_num_cycles_for_non_pick(
    self, mock_build_bt
  ):
    import src.main as main_mod

    mock_solution = mock.MagicMock()
    mock_tree = mock.MagicMock()
    mock_build_bt.return_value = mock_tree

    mock_robot = mock.MagicMock()
    mock_gripper = mock.MagicMock()
    mock_machine = mock.MagicMock()
    mock_vision = mock.MagicMock()

    completed = main_mod.run_machine_tending_cycle(
      solution=mock_solution,
      robot=mock_robot,
      gripper=mock_gripper,
      machine=mock_machine,
      vision=mock_vision,
      start_phase="load",
      num_cycles=5,
      initial_close_door_and_vise=False,
    )

    self.assertEqual(completed, 1)
    self.assertEqual(mock_build_bt.call_count, 1)
    self.assertEqual(mock_build_bt.call_args.kwargs.get("start_phase"), "load")

  def test_run_machine_tending_cycle_invalid_start_phase_raises_value_error(
    self,
  ):
    import src.main as main_mod

    with self.assertRaises(ValueError):
      main_mod.run_machine_tending_cycle(
        start_phase="invalid_phase",
        solution=mock.MagicMock(),
      )

  def test_frame_names_defaults(self):
    fn = Frames()
    self.assertEqual(fn.root, "root")
    self.assertEqual(fn.view, "view")
    self.assertEqual(fn.transit, "transit")
    self.assertEqual(fn.machine_approach, "machine_approach")
    self.assertEqual(fn.infeed_pre_grasp, "infeed_pre_grasp")
    self.assertEqual(fn.infeed_grasp, "infeed_grasp")
    self.assertEqual(fn.vise_pre_place, "vise_pre_place")
    self.assertEqual(fn.vise_place, "vise_place")

  def test_touchdown_defaults(self):
    td = Touchdown()
    self.assertEqual(td.force_n, 8.0)
    self.assertEqual(td.standoff_m, 0.010)
    self.assertEqual(td.timeout_s, 40.0)
    self.assertEqual(td.retract_after_m, 0.0)

  def test_create_seated_approach_tasks_golden_comparisons(self):
    # 1. Pick interaction (gripping: standoff + touchdown + retract)
    pick_robot = MockRobot()
    pick_td = Touchdown(force_n=8.0, standoff_m=0.010, retract_after_m=0.005)
    pick_tasks = create_seated_approach_tasks(
      robot=pick_robot,
      frame_name="infeed_grasp",
      parent_object="root",
      touchdown=pick_td,
      label="Step 04",
    )
    self.assertLen(pick_tasks, 3)
    self.assertEqual(
      pick_tasks[0].name,
      "Step 04: Linear Approach to Standoff (root/infeed_grasp)",
    )
    self.assertEqual(
      pick_tasks[1].name,
      "Step 04: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      pick_tasks[2].name,
      "Step 04: Linear Retract (0.5 cm, -Z Tool)",
    )
    self.assertEqual(
      pick_robot.executed_commands,
      [
        "move_cartesian:root/infeed_grasp:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_relative_cartesian:(0.0, 0.0, -0.005):LINEAR",
      ],
    )

    # 2. Load interaction (releasing: standoff + touchdown, no retract)
    load_robot = MockRobot()
    load_td = Touchdown(force_n=8.0, standoff_m=0.010, retract_after_m=0.0)
    load_tasks = create_seated_approach_tasks(
      robot=load_robot,
      frame_name="vise_place",
      parent_object="root",
      touchdown=load_td,
      label="Step 3c",
    )
    self.assertLen(load_tasks, 2)
    self.assertEqual(
      load_tasks[0].name,
      "Step 3c: Linear Approach to Standoff (root/vise_place)",
    )
    self.assertEqual(
      load_tasks[1].name,
      "Step 3c: Compliant Touchdown (+Z Tool)",
    )
    self.assertEqual(
      load_robot.executed_commands,
      [
        "move_cartesian:root/vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
      ],
    )

    # 3. Unload interaction (gripping: standoff + touchdown + retract)
    unload_robot = MockRobot()
    unload_td = Touchdown(force_n=8.0, standoff_m=0.010, retract_after_m=0.005)
    unload_tasks = create_seated_approach_tasks(
      robot=unload_robot,
      frame_name="vise_place",
      parent_object="root",
      touchdown=unload_td,
      label="Step 5c",
    )
    self.assertLen(unload_tasks, 3)
    self.assertEqual(
      unload_robot.executed_commands,
      [
        "move_cartesian:root/vise_place:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
        "move_relative_cartesian:(0.0, 0.0, -0.005):LINEAR",
      ],
    )

    # 4. Return interaction (releasing: standoff + touchdown, no retract)
    return_robot = MockRobot()
    return_td = Touchdown(force_n=8.0, standoff_m=0.010, retract_after_m=0.0)
    return_tasks = create_seated_approach_tasks(
      robot=return_robot,
      frame_name="infeed_grasp",
      parent_object="root",
      touchdown=return_td,
      label="Step 6c",
    )
    self.assertLen(return_tasks, 2)
    self.assertEqual(
      return_robot.executed_commands,
      [
        "move_cartesian:root/infeed_grasp:LINEAR",
        "move_to_contact:dir=(0.0, 0.0, 1.0),force=8.0",
      ],
    )


MachineTendingBehaviorTreeTest = BehaviorsTest


if __name__ == "__main__":
  absltest.main()
