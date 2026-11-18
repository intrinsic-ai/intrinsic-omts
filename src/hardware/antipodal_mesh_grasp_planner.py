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

"""Antipodal mesh grasp planner backend, simulation only.

Runs the `antipodal_mesh_plan_grasp` skill, which plans a top-down Robotiq Hand-E grasp
on the workpiece's mesh and checks it with the motion planner.
"""

from typing import Any

from intrinsic.solutions import behavior_tree as bt

from src.core.config import GRIPPER_CATALOG, GraspConfig
from src.hardware.grasping import GraspPlannerInterface


class AntipodalMeshGraspPlanner(GraspPlannerInterface):
  """Plans grasps on the workpiece's mesh with the `antipodal_mesh_plan_grasp` skill."""

  # The planned contacts are exact, so the pick opens the jaws before planning
  # and approaches the contacts directly instead of touching down first.
  uses_contact_touchdown = False

  def __init__(
    self,
    solution: Any,
    config: GraspConfig,
    robot_object_name: str,
    tool_object_name: str,
    tool_frame_name: str,
  ):
    # Not yet tested on hardware: remove this check for a supervised hardware
    # test.
    if not solution.is_simulated:
      raise ValueError(
        "The antipodal mesh grasp planner is for simulation only; it has not "
        "been tested on hardware."
      )
    if not robot_object_name:
      raise ValueError(
        "The antipodal mesh grasp planner needs the robot object name."
      )
    if config.antipodal_mesh_jaws is None:
      raise ValueError(
        "The antipodal mesh grasp planner has no default gripper; set "
        f"grasp.antipodal_mesh_gripper to a gripper in {GRIPPER_CATALOG}."
      )
    self._jaws = config.antipodal_mesh_jaws
    self._skill = solution.skills.ai.intrinsic.antipodal_mesh_plan_grasp
    self._config = config
    self._robot_object_name = robot_object_name
    self._tool_object_name = tool_object_name
    self._tool_frame_name = tool_frame_name

  def build_plan_grasp_task(
    self,
    workpiece_object_name: str,
    parent_object: str = "root",
    grasp_frame_name: str = "grasp",
    pregrasp_frame_name: str = "pre_grasp",
    name: str | None = None,
  ) -> bt.Node:
    if parent_object != "root":
      raise ValueError("Mesh grasp frames must be parented to root.")
    return bt.Task(
      name=name or "Plan Mesh Grasp",
      action=self._skill(
        workpiece_object=workpiece_object_name,
        robot_object=self._robot_object_name,
        tool_object=self._tool_object_name,
        tool_frame=self._tool_frame_name,
        grasp_frame=grasp_frame_name,
        pregrasp_frame=pregrasp_frame_name,
        pregrasp_distance_m=self._config.antipodal_mesh_pregrasp_distance_m,
        max_approach_tilt_deg=self._config.antipodal_mesh_max_approach_tilt_deg,
        jaw_min_width_m=self._jaws.min_width_m,
        jaw_max_width_m=self._jaws.max_width_m,
        jaw_contact_depth_m=self._jaws.contact_depth_m,
      ),
    )
