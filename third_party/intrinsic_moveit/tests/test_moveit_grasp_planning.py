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

from collections.abc import Sequence
from typing import Any

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.hardware.robot import RobotInterface
from third_party.intrinsic_moveit.moveit_grasp_planner import (
  SURFACE_Z_NEG,
  SURFACE_Z_POS,
  MockMoveItGraspPlanner,
)
from third_party.intrinsic_moveit.moveit_grasp_planning import (
  build_moveit_grasp_planning_subtree,
)
from third_party.intrinsic_moveit.tools.moveit_plan_grasp_and_move import (
  resolve_target_objects,
)


class MockRobot(RobotInterface):
  """Mock robot adapter for offline testing."""

  def __init__(self) -> None:
    self.executed_commands: list[str] = []

  def build_move_joint_task(
    self,
    joint_target: str | Any,
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or f"Mock Move to {joint_target}"
    self.executed_commands.append(f"move_joint:{joint_target}")
    return bt.Sequence(name=task_name, children=[])

  def build_move_cartesian_task(
    self,
    target_frame_name: str | None,
    target_object_name: str,
    motion_type: str,
    allow_tool_z_rotation: bool = False,
    cone_opening_half_angle: float = 0.0,
    moving_frame_offset: tuple[float, float, float] | None = None,
    target_frame_offset: (
      tuple[tuple[float, float, float], tuple[float, float, float, float]]
      | None
    ) = None,
    excluded_collision_pairs: Sequence[tuple[str, str]] | None = None,
    name: str | None = None,
  ) -> bt.Node:
    target_desc = (
      f"{target_object_name}/{target_frame_name}"
      if target_frame_name
      else target_object_name
    )
    task_name = name or f"Mock Move to {target_desc} ({motion_type})"
    self.executed_commands.append(
      f"move_cartesian:{target_desc}:{motion_type}:z_rot={allow_tool_z_rotation}"
    )
    return bt.Sequence(name=task_name, children=[])

  def build_move_blended_cartesian_task(
    self,
    target_frames: Sequence[tuple[str, str]],
    motion_type: str | Sequence[str] = "ANY",
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or f"Mock Blended Move ({len(target_frames)} frames)"
    self.executed_commands.append(f"move_blended:{len(target_frames)}")
    return bt.Sequence(name=task_name, children=[])

  def build_move_relative_cartesian_task(
    self,
    translation: tuple[float, float, float],
    motion_type: str = "LINEAR",
    exclude_collision: bool = False,
    excluded_collision_objects: Sequence[str] | None = None,
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or f"Mock Move Relative ({translation}) [{motion_type}]"
    self.executed_commands.append(
      f"move_relative_cartesian:{translation}:{motion_type}"
    )
    return bt.Sequence(name=task_name, children=[])

  def build_move_to_contact_task(
    self,
    direction: tuple[float, float, float],
    contact_force_newtons: float,
    timeout_seconds: float,
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or "Mock Move to Contact"
    self.executed_commands.append(
      f"move_to_contact:dir={direction},force={contact_force_newtons}"
    )
    return bt.Sequence(name=task_name, children=[])

  def build_attach_object_task(
    self,
    object_name: str,
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or f"Mock Attach {object_name}"
    self.executed_commands.append(f"attach:{object_name}")
    return bt.Sequence(name=task_name, children=[])

  def build_detach_object_task(
    self,
    object_name: str,
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or f"Mock Detach {object_name}"
    self.executed_commands.append(f"detach:{object_name}")
    return bt.Sequence(name=task_name, children=[])


class GraspPlanningTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.grasp_planner = MockMoveItGraspPlanner()

  def test_subtree_contains_plan_and_approach_steps(self):
    subtree = build_moveit_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      robot=self.robot,
      candidate_objects=["raw_stock_2x3x5"],
    )

    self.assertIsNotNone(subtree)
    self.assertEqual(subtree.name, "Plan Grasp and Approach")
    self.assertLen(subtree.children, 2)
    self.assertEqual(self.grasp_planner.planned_objects, [("raw_stock_2x3x5",)])

  def test_missing_candidate_objects_raises(self):
    with self.assertRaises(ValueError):
      build_moveit_grasp_planning_subtree(
        grasp_planner=self.grasp_planner,
        candidate_objects=[],
        move_to_pregrasp=False,
      )

  def test_plan_only_subtree_omits_approach(self):
    subtree = build_moveit_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      candidate_objects=["raw_stock_2x3x5"],
      move_to_pregrasp=False,
    )

    self.assertLen(subtree.children, 1)

  def test_approach_without_robot_raises(self):
    with self.assertRaises(ValueError):
      build_moveit_grasp_planning_subtree(
        grasp_planner=self.grasp_planner,
        candidate_objects=["raw_stock_2x3x5"],
        move_to_pregrasp=True,
      )

  def test_multiple_candidate_objects_are_forwarded(self):
    build_moveit_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      robot=self.robot,
      candidate_objects=[
        "part_a",
        "part_b",
        "part_c",
      ],
      surfaces=[SURFACE_Z_POS, SURFACE_Z_NEG],
      num_rotations=8,
    )

    self.assertEqual(
      self.grasp_planner.planned_objects,
      [
        (
          "part_a",
          "part_b",
          "part_c",
        )
      ],
    )

  def test_approach_targets_the_configured_pregrasp_frame(self):
    build_moveit_grasp_planning_subtree(
      grasp_planner=self.grasp_planner,
      robot=self.robot,
      candidate_objects=["raw_stock_2x3x5"],
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
      candidate_objects=["raw_stock_2x3x5"],
    )

    self.assertIn("raw_stock_2x3x5", task.name)

  def test_empty_candidate_list_raises(self):
    with self.assertRaises(ValueError):
      build_moveit_grasp_planning_subtree(
        grasp_planner=MockMoveItGraspPlanner(),
        robot=self.robot,
        candidate_objects=[],
      )


class ResolveTargetObjectsTest(absltest.TestCase):
  def test_missing_flag_raises(self):
    with self.assertRaises(ValueError):
      resolve_target_objects(None)
    with self.assertRaises(ValueError):
      resolve_target_objects([])
    with self.assertRaises(ValueError):
      resolve_target_objects([""])
    with self.assertRaises(ValueError):
      resolve_target_objects(["  ", ", "])

  def test_repeated_flag_occurrences_are_collected(self):
    self.assertEqual(
      resolve_target_objects(["part_a", "part_b"]),
      ["part_a", "part_b"],
    )

  def test_comma_separated_values_are_split(self):
    self.assertEqual(
      resolve_target_objects(["part_a, part_c"]),
      ["part_a", "part_c"],
    )

  def test_duplicates_are_removed_preserving_order(self):
    self.assertEqual(
      resolve_target_objects(["part_b,part_a", "part_b"]),
      ["part_b", "part_a"],
    )


if __name__ == "__main__":
  absltest.main()
