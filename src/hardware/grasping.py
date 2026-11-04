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

import abc

from intrinsic.solutions import behavior_tree as bt


class GraspPlannerInterface(abc.ABC):
  """Update grasp and pre-grasp frames for an already-localized workpiece."""

  # Whether the pick seats the gripper with a force-controlled touchdown at the
  # grasp frame. Planners that place the jaws on exact contacts set False; the
  # pick then moves linearly to the grasp frame, and the unload re-plans the
  # grasp on the placed part instead of touching down. Such planners must also
  # write a `<grasp_frame>_workpiece` frame, the workpiece pose as a child of
  # the grasp frame, from which the return solves its tool target after the
  # re-grasp.
  uses_contact_touchdown: bool = True

  @abc.abstractmethod
  def build_plan_grasp_task(
    self,
    workpiece_object_name: str,
    parent_object: str = "root",
    grasp_frame_name: str = "grasp",
    pregrasp_frame_name: str = "pre_grasp",
    name: str | None = None,
  ) -> bt.Node:
    """Builds the task that plans a grasp and writes it into the world.

    Implementations update pre-existing frames rather than creating them, so
    both frames must already be declared in the world (they are, via
    `configs/<cell>/scene.updates.pbtxt`).

    Args:
        workpiece_object_name: Object World name of the part to grasp.
        parent_object: Object owning the output frames.
        grasp_frame_name: Pre-existing frame updated to the planned grasp pose.
        pregrasp_frame_name: Pre-existing frame updated to the planned
          pre-grasp pose, which the pick subtree approaches first.
        name: Optional custom behavior tree task name.

    Returns:
        Behavior tree node that publishes the planned frames when executed.
    """
    raise NotImplementedError


def plans_exact_contacts(grasp_planner: GraspPlannerInterface | None) -> bool:
  """Returns whether `grasp_planner` grasps on exact contacts, not touchdown."""
  return grasp_planner is not None and not grasp_planner.uses_contact_touchdown
