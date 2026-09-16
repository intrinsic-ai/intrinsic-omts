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

"""Diagnostics helper for SBL behavior trees and executive operations."""

import dataclasses
import logging
from collections.abc import Iterator
from typing import Any

STATE_NAMES: dict[int, str] = {
  0: "UNSPECIFIED",
  1: "ACCEPTED",
  2: "SELECTED",
  3: "EVALUATING_CONDITION",
  4: "READY",
  5: "RUNNING",
  6: "SUCCEEDED",
  7: "FAILED",
  8: "SUSPENDED",
  9: "CANCELING",
  10: "CANCELED",
}

FAILURE_REASON_NAMES: dict[int, str] = {
  0: "UNKNOWN_FAILURE",
  1: "FAILED_CONDITION",
  2: "FAILED_EXECUTION",
}


@dataclasses.dataclass(frozen=True)
class FailedNodeDiagnostic:
  """Diagnostic record for a failed or active behavior tree node."""

  path: list[str]
  state: str
  reason: str

  def __getitem__(self, idx: int) -> Any:
    return (self.path, self.state, self.reason)[idx]

  def __iter__(self) -> Iterator[Any]:
    yield self.path
    yield self.state
    yield self.reason

  def format(self) -> str:
    path_str = " -> ".join(self.path)
    return (
      f"Failed/Active Node: {path_str} "
      f"[State: {self.state}, Reason: {self.reason}]"
    )


def _children_of(node: Any) -> list[Any]:
  """Extracts child nodes from a behavior tree protobuf node."""
  if not hasattr(node, "HasField"):
    return []
  for composite in ("sequence", "parallel"):
    if node.HasField(composite):
      return list(getattr(node, composite).children)
  if node.HasField("selector"):
    return list(node.selector.children) + [
      b.node for b in node.selector.branches
    ]
  if node.HasField("fallback"):
    return list(node.fallback.children) + [t.node for t in node.fallback.tries]
  if node.HasField("branch"):
    branch_children = []
    if node.branch.HasField("then"):
      branch_children.append(node.branch.then)
    if node.branch.HasField("else"):
      branch_children.append(getattr(node.branch, "else"))
    return branch_children
  if node.HasField("retry") and node.retry.HasField("child"):
    return [node.retry.child]
  return []


def extract_failed_nodes(
  operation_or_bt_proto: Any,
) -> list[FailedNodeDiagnostic]:
  """Extracts failing or active nodes from an Executive operation or BT proto."""
  if operation_or_bt_proto is None:
    return []

  bt_proto = getattr(
    getattr(operation_or_bt_proto, "metadata", None), "behavior_tree", None
  )
  if bt_proto is None:
    bt_proto = getattr(
      operation_or_bt_proto, "behavior_tree", operation_or_bt_proto
    )

  if bt_proto is None or not hasattr(bt_proto, "root"):
    return []

  diagnostics: list[FailedNodeDiagnostic] = []

  def _traverse(node: Any, path: list[str]) -> None:
    if node is None:
      return
    name = (
      getattr(node, "name", "")
      or getattr(node, "description", "")
      or f"Node_{getattr(node, 'id', 'unknown')}"
    )
    current_path = path + [name]
    state = getattr(node, "state", 0)
    reason = getattr(node, "failure_reason", 0)

    if state in (5, 7, 9, 10):
      diagnostics.append(
        FailedNodeDiagnostic(
          path=current_path,
          state=STATE_NAMES.get(state, str(state)),
          reason=FAILURE_REASON_NAMES.get(reason, str(reason)),
        )
      )

    for child in _children_of(node):
      _traverse(child, current_path)

  _traverse(bt_proto.root, [])
  return diagnostics


def log_tree_failure_diagnostics(solution: Any) -> None:
  """Logs diagnostic information about failed nodes in the executive operation."""
  if not hasattr(solution, "executive"):
    return
  op = getattr(solution.executive, "operation", None) or getattr(
    solution.executive, "_operation", None
  )
  if op is None:
    return

  failed_nodes = extract_failed_nodes(op)
  if not failed_nodes:
    return

  logging.error("=== Behavior Tree Failure Telemetry ===")
  for diag in failed_nodes:
    logging.error("%s", diag.format())
  logging.error("=======================================")
