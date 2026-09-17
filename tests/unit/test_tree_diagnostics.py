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

"""Unit tests for tree diagnostics telemetry."""

from unittest import mock

from absl.testing import absltest

from src.utils.tree_diagnostics import (
  extract_failed_nodes,
  log_tree_failure_diagnostics,
)


class DummyNode:
  def __init__(
    self,
    name: str,
    state: int = 6,
    failure_reason: int = 0,
    children: list["DummyNode"] | None = None,
  ):
    self.name = name
    self.state = state
    self.failure_reason = failure_reason
    self.children = children or []

  def HasField(self, field_name: str) -> bool:
    if field_name == "sequence":
      return bool(self.children)
    return False

  @property
  def sequence(self):
    m = mock.MagicMock()
    m.children = self.children
    return m


class DummyTree:
  def __init__(self, root: DummyNode):
    self.root = root

  def HasField(self, field_name: str) -> bool:
    return field_name == "root"


class TreeDiagnosticsTest(absltest.TestCase):
  def test_extract_failed_nodes_empty_or_none(self):
    self.assertEqual(extract_failed_nodes(None), [])
    self.assertEqual(extract_failed_nodes(mock.MagicMock(spec=[])), [])

  def test_extract_failed_nodes_finds_failed_leaf(self):
    leaf1 = DummyNode("Step 3a: Approach", state=6)
    leaf2 = DummyNode("Step 3b: Touchdown", state=7, failure_reason=2)
    subtree = DummyNode(
      "3. Load Machine Subtree", state=7, children=[leaf1, leaf2]
    )
    root = DummyNode("Master Tree", state=7, children=[subtree])
    tree = DummyTree(root)

    failed = extract_failed_nodes(tree)
    self.assertEqual(len(failed), 3)

    # Master Tree
    self.assertEqual(failed[0][0], ["Master Tree"])
    self.assertEqual(failed[0][1], "FAILED")

    # Subtree
    self.assertEqual(failed[1][0], ["Master Tree", "3. Load Machine Subtree"])
    self.assertEqual(failed[1][1], "FAILED")

    # Leaf
    self.assertEqual(
      failed[2][0],
      ["Master Tree", "3. Load Machine Subtree", "Step 3b: Touchdown"],
    )
    self.assertEqual(failed[2][1], "FAILED")
    self.assertEqual(failed[2][2], "FAILED_EXECUTION")

  def test_log_tree_failure_diagnostics_calls_logger(self):
    leaf = DummyNode("Step 5c: Touchdown", state=7, failure_reason=2)
    tree = DummyTree(leaf)
    op = mock.MagicMock()
    op.metadata.behavior_tree = tree

    solution = mock.MagicMock()
    solution.executive.operation = op

    with mock.patch("src.utils.tree_diagnostics.logging.error") as mock_log:
      log_tree_failure_diagnostics(solution)
      self.assertTrue(mock_log.called)
      log_calls = [c[0][0] for c in mock_log.call_args_list]
      self.assertTrue(
        any("Behavior Tree Failure Telemetry" in c for c in log_calls)
      )
      self.assertTrue(
        any("Step 5c: Touchdown" in str(c) for c in mock_log.call_args_list)
      )


if __name__ == "__main__":
  absltest.main()
