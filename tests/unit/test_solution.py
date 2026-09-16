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

"""Unit tests for Solution facade and MockSolution in src/core/solution.py."""

from unittest import mock

from absl.testing import absltest

from src.core.solution import MockSolution, OfflineExecutive, Solution
from src.core.world import MockWorld, World


class MockSolutionTest(absltest.TestCase):
  def test_initialization_defaults(self):
    sol = MockSolution()
    self.assertIsInstance(sol.world, MockWorld)
    self.assertIsInstance(sol.executive, OfflineExecutive)
    self.assertIsNone(sol.skills)
    self.assertIsNone(sol.proto_builder)
    self.assertEqual(sol.resources, {})

  def test_run_and_clear_cache(self):
    sol = MockSolution()
    tree = mock.MagicMock()
    sol.run(tree)
    self.assertEqual(sol.executive.executed, [tree])
    self.assertTrue(sol.clear_motion_planner_cache())


class SolutionFacadeTest(absltest.TestCase):
  def test_accessors(self):
    mock_raw = mock.MagicMock()
    mock_raw.skills = "mock_skills"
    mock_raw.resources = {"res1": 1}
    mock_raw.proto_builder = None
    mock_raw.custom_attr = "custom"

    sol = Solution(mock_raw)
    self.assertEqual(sol.skills, "mock_skills")
    self.assertEqual(sol.resources, {"res1": 1})
    self.assertIsNone(sol.proto_builder)
    self.assertIsInstance(sol.world, World)
    self.assertEqual(sol.custom_attr, "custom")

  def test_run_via_executive(self):
    mock_raw = mock.MagicMock()
    sol = Solution(mock_raw)
    tree = mock.MagicMock()
    sol.run(tree)
    mock_raw.executive.run.assert_called_once_with(tree)

  def test_run_via_raw_run_fallback(self):
    mock_raw = mock.MagicMock(spec=["run"])
    sol = Solution(mock_raw)
    tree = mock.MagicMock()
    sol.run(tree)
    mock_raw.run.assert_called_once_with(tree)

  def test_run_missing_executor_raises(self):
    mock_raw = object()
    sol = Solution(mock_raw)
    tree = mock.MagicMock()
    with self.assertRaises(RuntimeError):
      sol.run(tree)


if __name__ == "__main__":
  absltest.main()
