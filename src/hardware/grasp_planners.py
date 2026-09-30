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

"""Construction of the configured grasp planner.

This is the only first-party module that knows the `intrinsic-moveit`
integration exists. Everything downstream depends on `GraspPlannerInterface`
alone, so adding a backend is a change to this file plus a config value.
"""

from intrinsic.solutions import deployments

from src.core.config import GraspConfig
from src.core.types import GraspPlannerType
from src.hardware.grasping import GraspPlannerInterface
from third_party.intrinsic_moveit.moveit_grasp_planner import MoveItGraspPlanner


def create_grasp_planner(
  planner_type: GraspPlannerType,
  solution: deployments.Solution,
  config: GraspConfig,
  tool_object_name: str,
) -> GraspPlannerInterface | None:
  """Builds the grasp planner selected for this run.

  Args:
      planner_type: Grasp planner backend to use.
      solution: Connected SBL deployment instance, for backends that invoke
        skills. Unused by `CUBOID_CENTER`.
      config: Grasp planner configuration for the cell.
      tool_object_name: Object World object owning the gripper tool frame.

  Returns:
      Grasp planner to run after perception, or None for
      `GraspPlannerType.CUBOID_CENTER`, whose frames the perception pipeline
      publishes itself.

  Raises:
      ValueError: If the backend is not handled here, or if MoveIt is selected
        but its grasp planning skill is missing from the solution.
  """
  if planner_type is GraspPlannerType.CUBOID_CENTER:
    return None
  if planner_type is GraspPlannerType.MOVEIT:
    return MoveItGraspPlanner(
      solution=solution,
      tool_frame_name=config.moveit_tool_frame_name,
      tool_object_name=tool_object_name,
      group_name=config.moveit_group_name,
      end_effector_group=config.moveit_end_effector_group,
      retract_dist_m=config.moveit_retract_dist_m,
      timeout_ms=config.moveit_timeout_ms,
      surfaces=config.moveit_surfaces,
      num_rotations=config.moveit_num_rotations,
      settle_seconds=config.moveit_settle_seconds,
    )
  raise ValueError(f"Unhandled grasp planner backend '{planner_type}'.")
