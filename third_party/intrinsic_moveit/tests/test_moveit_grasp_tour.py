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

"""Unit tests for the sequential grasp tour subtree and its CLI parsing."""

import argparse

from absl.testing import absltest

from src.hardware.robot import MockRobot
from third_party.intrinsic_moveit.moveit_grasp_planner import (
  SURFACE_ALL,
  MockMoveItGraspPlanner,
)
from third_party.intrinsic_moveit.moveit_grasp_tour import (
  DEFAULT_TOUR_OBJECTS,
  ObjectSpec,
  build_moveit_grasp_tour_subtree,
)
from third_party.intrinsic_moveit.tools.moveit_grasp_tour import (
  parse_object_spec,
  resolve_object_specs,
)

_SPEC_DEFAULTS = {
  "default_surfaces": SURFACE_ALL,
  "default_rotations": 4,
  "default_retract": None,
}


class GraspTourTest(absltest.TestCase):
  def test_each_object_only_plans_and_approaches(self):
    robot = MockRobot()
    planner = MockMoveItGraspPlanner()

    build_moveit_grasp_tour_subtree(
      grasp_planner=planner,
      robot=robot,
      object_specs=[ObjectSpec(name="raw_stock_50x50x75_1")],
    )

    # The tour stops at the pre-grasp: no descent to the grasp pose, no
    # contact, and no gripper command.
    self.assertEqual(
      robot.executed_commands,
      ["move_cartesian:root/pre_grasp:ANY:z_rot=False"],
    )
    self.assertEqual(planner.planned_objects, [("raw_stock_50x50x75_1",)])

  def test_objects_are_planned_one_at_a_time_in_order(self):
    robot = MockRobot()
    planner = MockMoveItGraspPlanner()
    build_moveit_grasp_tour_subtree(
      grasp_planner=planner,
      robot=robot,
      object_specs=[ObjectSpec(name=name) for name in DEFAULT_TOUR_OBJECTS],
    )
    # One plan per object, never pooled: pooling would rank them jointly and
    # only approach the winner.
    self.assertEqual(
      planner.planned_objects, [(name,) for name in DEFAULT_TOUR_OBJECTS]
    )
    self.assertLen(robot.executed_commands, len(DEFAULT_TOUR_OBJECTS))

  def test_per_object_overrides_reach_the_planner(self):
    # The default scene deliberately runs on stock surfaces and retract, so
    # this is the only place the override path is exercised.
    planner = MockMoveItGraspPlanner()
    build_moveit_grasp_tour_subtree(
      grasp_planner=planner,
      robot=MockRobot(),
      object_specs=[
        ObjectSpec(name="raw_stock_50x50x75_1"),
        ObjectSpec(
          name="raw_stock_50x50x75_3",
          surfaces=(0, 1, 2, 3),
          num_rotations=8,
          retract_dist_m=0.06,
        ),
      ],
    )
    self.assertEqual(planner.plan_calls[0]["surfaces"], SURFACE_ALL)
    self.assertIsNone(planner.plan_calls[0]["retract_dist_m"])
    self.assertEqual(planner.plan_calls[1]["surfaces"], (0, 1, 2, 3))
    self.assertEqual(planner.plan_calls[1]["num_rotations"], 8)
    self.assertAlmostEqual(planner.plan_calls[1]["retract_dist_m"], 0.06)

  def test_plan_only_mode_never_moves_the_arm(self):
    robot = MockRobot()
    planner = MockMoveItGraspPlanner()
    build_moveit_grasp_tour_subtree(
      grasp_planner=planner,
      robot=robot,
      object_specs=[ObjectSpec(name=name) for name in DEFAULT_TOUR_OBJECTS],
      move_to_pregrasp=False,
    )
    self.assertEmpty(robot.executed_commands)
    self.assertLen(planner.planned_objects, len(DEFAULT_TOUR_OBJECTS))

  def test_plan_only_mode_works_without_a_robot(self):
    planner = MockMoveItGraspPlanner()
    build_moveit_grasp_tour_subtree(
      grasp_planner=planner,
      robot=None,
      object_specs=[ObjectSpec(name="raw_stock_50x50x75_1")],
      move_to_pregrasp=False,
    )
    self.assertLen(planner.planned_objects, 1)

  def test_missing_robot_raises_when_approaching(self):
    with self.assertRaises(ValueError):
      build_moveit_grasp_tour_subtree(
        grasp_planner=MockMoveItGraspPlanner(),
        robot=None,
        object_specs=[ObjectSpec(name="raw_stock_50x50x75_1")],
      )

  def test_transit_frame_is_inserted_between_but_not_before_objects(self):
    robot = MockRobot()
    build_moveit_grasp_tour_subtree(
      grasp_planner=MockMoveItGraspPlanner(),
      robot=robot,
      object_specs=[
        ObjectSpec(name="raw_stock_50x50x75_1"),
        ObjectSpec(name="raw_stock_50x50x75_2"),
      ],
      transit_frame="view",
    )
    transits = [
      command
      for command in robot.executed_commands
      if "root/view" in command
    ]
    self.assertLen(transits, 1)
    # The tour must not start with a transit move.
    self.assertNotIn("root/view", robot.executed_commands[0])

  def test_transit_frame_is_ignored_in_plan_only_mode(self):
    robot = MockRobot()
    build_moveit_grasp_tour_subtree(
      grasp_planner=MockMoveItGraspPlanner(),
      robot=robot,
      object_specs=[
        ObjectSpec(name="raw_stock_50x50x75_1"),
        ObjectSpec(name="raw_stock_50x50x75_2"),
      ],
      move_to_pregrasp=False,
      transit_frame="view",
    )
    self.assertEmpty(robot.executed_commands)

  def test_empty_specs_raise(self):
    with self.assertRaises(ValueError):
      build_moveit_grasp_tour_subtree(
        grasp_planner=MockMoveItGraspPlanner(),
        robot=MockRobot(),
        object_specs=[],
      )

  def test_duplicate_objects_raise(self):
    with self.assertRaises(ValueError):
      build_moveit_grasp_tour_subtree(
        grasp_planner=MockMoveItGraspPlanner(),
        robot=MockRobot(),
        object_specs=[
          ObjectSpec(name="raw_stock_50x50x75_1"),
          ObjectSpec(name="raw_stock_50x50x75_1"),
        ],
      )


class ObjectSpecParsingTest(absltest.TestCase):
  def test_bare_name_inherits_the_defaults(self):
    spec = parse_object_spec("raw_stock_50x50x75_1", **_SPEC_DEFAULTS)
    self.assertEqual(spec.name, "raw_stock_50x50x75_1")
    self.assertEqual(spec.surfaces, SURFACE_ALL)
    self.assertEqual(spec.num_rotations, 4)
    self.assertIsNone(spec.retract_dist_m)

  def test_all_override_keys_parse(self):
    spec = parse_object_spec(
      "raw_stock_50x50x75_3:surfaces=0,1,2,3:rotations=8:retract=0.06",
      **_SPEC_DEFAULTS,
    )
    self.assertEqual(spec.name, "raw_stock_50x50x75_3")
    self.assertEqual(spec.surfaces, (0, 1, 2, 3))
    self.assertEqual(spec.num_rotations, 8)
    self.assertAlmostEqual(spec.retract_dist_m, 0.06)

  def test_surfaces_all_parses_to_the_empty_tuple(self):
    # Empty is the planning service's own encoding for every surface.
    spec = parse_object_spec(
      "raw_stock_50x50x75_1:surfaces=all", **_SPEC_DEFAULTS
    )
    self.assertEqual(spec.surfaces, SURFACE_ALL)

  def test_surfaces_all_is_case_insensitive(self):
    spec = parse_object_spec(
      "raw_stock_50x50x75_1:surfaces=ALL", **_SPEC_DEFAULTS
    )
    self.assertEqual(spec.surfaces, SURFACE_ALL)

  def test_empty_surfaces_value_is_rejected(self):
    # 'all' is the way to ask for everything; a blank value is a typo.
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec("raw_stock_50x50x75_1:surfaces=", **_SPEC_DEFAULTS)

  def test_unknown_key_is_rejected(self):
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec("raw_stock_50x50x75_1:wiggle=3", **_SPEC_DEFAULTS)

  def test_retired_force_key_is_rejected(self):
    # 'force' belonged to the contact step, which the tour no longer performs.
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec("raw_stock_50x50x75_1:force=6", **_SPEC_DEFAULTS)

  def test_missing_equals_is_rejected(self):
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec("raw_stock_50x50x75_1:retract", **_SPEC_DEFAULTS)

  def test_out_of_range_surface_is_rejected(self):
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec("raw_stock_50x50x75_1:surfaces=9", **_SPEC_DEFAULTS)

  def test_non_numeric_surface_is_rejected(self):
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec("raw_stock_50x50x75_1:surfaces=top", **_SPEC_DEFAULTS)

  def test_non_positive_rotations_is_rejected(self):
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec("raw_stock_50x50x75_1:rotations=0", **_SPEC_DEFAULTS)

  def test_negative_retract_is_rejected(self):
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec("raw_stock_50x50x75_1:retract=-1", **_SPEC_DEFAULTS)

  def test_empty_name_is_rejected(self):
    with self.assertRaises(argparse.ArgumentTypeError):
      parse_object_spec(":retract=0.06", **_SPEC_DEFAULTS)

  def test_resolve_falls_back_to_the_default_tour(self):
    specs = resolve_object_specs(None, **_SPEC_DEFAULTS)
    self.assertEqual(
      [spec.name for spec in specs], list(DEFAULT_TOUR_OBJECTS)
    )

  def test_resolve_preserves_order_and_does_not_split_on_commas(self):
    # Commas belong to the surfaces override, so a value is never split on
    # them the way plan_and_move's --target_object is.
    specs = resolve_object_specs(
      ["raw_stock_50x50x75_2", "raw_stock_50x50x75_3:surfaces=0,1"],
      **_SPEC_DEFAULTS,
    )
    self.assertEqual(
      [spec.name for spec in specs],
      ["raw_stock_50x50x75_2", "raw_stock_50x50x75_3"],
    )
    self.assertEqual(specs[1].surfaces, (0, 1))


if __name__ == "__main__":
  absltest.main()
