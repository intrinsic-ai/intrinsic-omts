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

"""Unit tests for the grasp planning subtree and grasp planner adapters."""

from absl.testing import absltest

from src.behaviors.grasp_planning import build_grasp_planning_subtree
from src.hardware.grasp_planner import (
  SURFACE_Z_NEG,
  SURFACE_Z_POS,
  MockGraspPlanner,
)
from src.hardware.robot import MockRobot
from tools.grasping.plan_and_move import (
  DEFAULT_TARGET_OBJECT,
  resolve_target_objects,
)


class GraspPlanningTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.grasp_planner = MockGraspPlanner()

  def test_subtree_contains_plan_and_approach_steps(self):
    subtree = build_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      robot=self.robot,
      candidate_objects=["raw_stock_50x50x75_1"],
    )

    self.assertIsNotNone(subtree)
    self.assertEqual(subtree.name, "Plan Grasp and Approach")
    self.assertLen(subtree.children, 2)
    self.assertEqual(
      self.grasp_planner.planned_objects, [("raw_stock_50x50x75_1",)]
    )

  def test_default_candidate_object_matches_the_default_scene(self):
    build_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      move_to_pregrasp=False,
    )

    self.assertEqual(
      self.grasp_planner.planned_objects, [(DEFAULT_TARGET_OBJECT,)]
    )

  def test_plan_only_subtree_omits_approach(self):
    subtree = build_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      candidate_objects=["raw_stock_50x50x75_1"],
      move_to_pregrasp=False,
    )

    self.assertLen(subtree.children, 1)

  def test_approach_without_robot_raises(self):
    with self.assertRaises(ValueError):
      build_grasp_planning_subtree(
        grasp_planner=self.grasp_planner,
        candidate_objects=["raw_stock_50x50x75_1"],
        move_to_pregrasp=True,
      )

  def test_multiple_candidate_objects_are_forwarded(self):
    build_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      robot=self.robot,
      candidate_objects=[
        "raw_stock_50x50x75_1",
        "raw_stock_50x50x75_2",
        "raw_stock_50x50x75_3",
      ],
      surfaces=[SURFACE_Z_POS, SURFACE_Z_NEG],
      num_rotations=8,
    )

    self.assertEqual(
      self.grasp_planner.planned_objects,
      [
        (
          "raw_stock_50x50x75_1",
          "raw_stock_50x50x75_2",
          "raw_stock_50x50x75_3",
        )
      ],
    )

  def test_approach_targets_the_configured_pregrasp_frame(self):
    build_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      robot=self.robot,
      candidate_objects=["raw_stock_50x50x75_1"],
      parent_object="root",
      pregrasp_frame="pre_grasp",
      motion_type="LINEAR",
    )

    self.assertTrue(
      any("pre_grasp" in command for command in self.robot.executed_commands),
      f"Expected a pre_grasp motion, got {self.robot.executed_commands}",
    )

  def test_mock_planner_task_is_named_after_candidates(self):
    task = self.grasp_planner.build_plan_grasps_task(
      candidate_objects=["raw_stock_50x50x75_1"],
    )

    self.assertIn("raw_stock_50x50x75_1", task.name)

  def test_empty_candidate_list_raises(self):
    with self.assertRaises(ValueError):
      build_grasp_planning_subtree(
        grasp_planner=MockGraspPlanner(),
        robot=self.robot,
        candidate_objects=[],
      )


class ResolveTargetObjectsTest(absltest.TestCase):
  def test_missing_flag_falls_back_to_default(self):
    self.assertEqual(resolve_target_objects(None), [DEFAULT_TARGET_OBJECT])
    self.assertEqual(resolve_target_objects([]), [DEFAULT_TARGET_OBJECT])

  def test_repeated_flag_occurrences_are_collected(self):
    self.assertEqual(
      resolve_target_objects(["raw_stock_50x50x75_1", "raw_stock_50x50x75_2"]),
      ["raw_stock_50x50x75_1", "raw_stock_50x50x75_2"],
    )

  def test_comma_separated_values_are_split(self):
    self.assertEqual(
      resolve_target_objects(["raw_stock_50x50x75_1, raw_stock_50x50x75_3"]),
      ["raw_stock_50x50x75_1", "raw_stock_50x50x75_3"],
    )

  def test_duplicates_are_removed_preserving_order(self):
    self.assertEqual(
      resolve_target_objects(
        ["raw_stock_50x50x75_2,raw_stock_50x50x75_1", "raw_stock_50x50x75_2"]
      ),
      ["raw_stock_50x50x75_2", "raw_stock_50x50x75_1"],
    )


if __name__ == "__main__":
  absltest.main()
