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

Supports CUBOID_CENTER (frames supplied by perception), MOVEIT
(intrinsic-moveit), and MESH (OMTS mesh-based planning).
Constructs backend adapters here so downstream behavior code uses
GraspPlannerInterface rather than concrete planner classes.
"""

from intrinsic.solutions import deployments

from src.core.config import GraspConfig
from src.core.types import GraspPlannerType
from src.hardware.grasping import GraspPlannerInterface
from src.hardware.mesh_grasp_planner import MeshGraspPlanner
from third_party.intrinsic_moveit.moveit_grasp_planner import MoveItGraspPlanner


def create_grasp_planner(
  planner_type: GraspPlannerType,
  solution: deployments.Solution,
  config: GraspConfig,
  tool_object_name: str,
  tool_frame_name: str = "tool_frame",
  robot_object_name: str = "",
) -> GraspPlannerInterface | None:
  """Builds the grasp planner selected for this run.

  Args:
      planner_type: Grasp planner backend to use.
      solution: Connected SBL deployment instance, for backends that invoke
        skills. Unused by `CUBOID_CENTER`.
      config: Grasp planner configuration for the cell.
      tool_object_name: Object World object owning the gripper tool frame.
      tool_frame_name: Gripper tool frame that `MESH` plans the grasp for.
      robot_object_name: Robot arm that `MESH` checks the grasp with.

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
  if planner_type is GraspPlannerType.MESH:
    return MeshGraspPlanner(
      solution, config, robot_object_name, tool_object_name, tool_frame_name
    )
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
