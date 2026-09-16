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

"""Facade and stand-ins for Flowstate solution handle and execution."""

import abc
import logging
from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.core.world import MockWorld, World

__all__ = [
  "MockSolution",
  "OfflineExecutive",
  "Solution",
  "SolutionInterface",
]


class SolutionInterface(abc.ABC):
  """Abstract interface for Flowstate solution execution and world access."""

  @abc.abstractmethod
  def run(self, tree: bt.Node, simulation_mode: Any = None) -> None:
    """Executes a behavior tree on the solution executive."""
    raise NotImplementedError

  @abc.abstractmethod
  def clear_motion_planner_cache(self) -> bool:
    """Clears volatile motion planner cache to prevent stale entity ID lookups."""
    raise NotImplementedError


class OfflineExecutive:
  """Executive that accepts behavior trees and runs nothing."""

  def __init__(self) -> None:
    self.executed: list[Any] = []

  def run(self, tree: Any, simulation_mode: Any = None) -> None:
    """Records the tree without executing it."""
    del simulation_mode
    self.executed.append(tree)
    logging.info("Offline executive accepted a behavior tree; not executing.")


class MockSolution(SolutionInterface):
  """Stand-in solution handle for `--mock_hardware` and unit tests."""

  def __init__(self, world: MockWorld | None = None) -> None:
    """Initializes the mock solution."""
    self.world = world if world is not None else MockWorld()
    self.executive = OfflineExecutive()
    self.skills = None
    self.proto_builder = None
    self.resources: dict[str, Any] = {}

  def run(self, tree: bt.Node, simulation_mode: Any = None) -> None:
    self.executive.run(tree, simulation_mode=simulation_mode)

  def clear_motion_planner_cache(self) -> bool:
    return True


class Solution(SolutionInterface):
  """Facade over the Flowstate solution handle."""

  def __init__(self, raw_solution: Any) -> None:
    self._raw_solution = raw_solution
    raw_world = getattr(raw_solution, "world", None)
    self._world = (
      raw_world
      if isinstance(raw_world, World)
      else World(raw_world, solution=self)
    )

  @property
  def raw(self) -> Any:
    return self._raw_solution

  @property
  def world(self) -> World:
    return self._world

  @property
  def skills(self) -> Any:
    return getattr(self._raw_solution, "skills", None)

  @property
  def resources(self) -> Any:
    return getattr(self._raw_solution, "resources", {})

  @property
  def proto_builder(self) -> Any | None:
    builder = getattr(self._raw_solution, "proto_builder", None)
    return builder if builder is not None else None

  @property
  def executive(self) -> Any:
    return getattr(self._raw_solution, "executive", None)

  def __getattr__(self, name: str) -> Any:
    return getattr(self._raw_solution, name)

  def run(self, tree: bt.Node, simulation_mode: Any = None) -> None:
    """Executes a behavior tree on the underlying solution executive."""
    executive = getattr(self._raw_solution, "executive", None)
    if executive is not None and hasattr(executive, "run"):
      if simulation_mode is not None:
        executive.run(tree, simulation_mode=simulation_mode)
      else:
        executive.run(tree)
    elif hasattr(self._raw_solution, "run"):
      if simulation_mode is not None:
        self._raw_solution.run(tree, simulation_mode=simulation_mode)
      else:
        self._raw_solution.run(tree)
    else:
      raise RuntimeError(
        "Solution object provides no executor (missing executive.run and run)"
      )

  def clear_motion_planner_cache(self) -> bool:
    """Clears volatile motion planner cache to prevent stale entity ID lookups."""
    if self._raw_solution is None:
      return False
    try:
      skills = self.skills
      if skills is None:
        return False
      ai_skills = getattr(skills, "ai", None)
      intrinsic_skills = (
        getattr(ai_skills, "intrinsic", None) if ai_skills else None
      )
      if intrinsic_skills and hasattr(
        intrinsic_skills, "clear_motion_planner_service_cache"
      ):
        clear_action = intrinsic_skills.clear_motion_planner_service_cache()
        tree = bt.Sequence(
          name="Clear Motion Planner Cache Sequence",
          children=[
            bt.Task(action=clear_action, name="Clear Motion Planner Cache")
          ],
        )
        self.run(tree)
        logging.info("Successfully cleared motion planner service cache.")
        return True
    except Exception as e:  # pylint: disable=broad-exception-caught
      logging.warning("Failed to clear motion planner cache: %s", e)
    return False
