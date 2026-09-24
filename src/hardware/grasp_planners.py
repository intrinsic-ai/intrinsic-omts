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

This is the single place that maps a `GraspPlannerType` onto a concrete
planner. Entry points resolve which backend was asked for and call
`create_grasp_planner`; everything downstream depends only on
`GraspPlannerInterface`, so adding a backend is a change to this file plus a
config value.
"""

from intrinsic.solutions import deployments

from src.core.config import GraspConfig
from src.core.types import GraspPlannerType
from src.hardware.grasping import GraspPlannerInterface


def create_grasp_planner(
  planner_type: GraspPlannerType,
  solution: deployments.Solution,
  config: GraspConfig,
) -> GraspPlannerInterface | None:
  """Builds the grasp planner selected for this run.

  Args:
      planner_type: Grasp planner backend to use.
      solution: Connected SBL deployment instance, for backends that invoke
        skills. Unused by `CUBOID_CENTER`.
      config: Grasp planner configuration for the cell.

  Returns:
      Grasp planner to run after perception, or None for
      `GraspPlannerType.CUBOID_CENTER`, whose frames the perception pipeline
      publishes itself.

  Raises:
      ValueError: If the backend is not handled here. `GraspConfig` already
        rejects unknown names, so this fires when a new `GraspPlannerType`
        member is added without a corresponding branch.
  """
  del solution, config  # Only needed by model-based backends.
  if planner_type is GraspPlannerType.CUBOID_CENTER:
    return None
  raise ValueError(f"Unhandled grasp planner backend '{planner_type}'.")
