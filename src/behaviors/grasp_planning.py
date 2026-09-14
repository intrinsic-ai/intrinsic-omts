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

"""Grasp planning subtree combining MoveIt grasp planning with an approach move.

The grasp skill publishes the top-ranked grasp and pre-grasp poses into the
Object World Service by updating pre-existing frames. The approach motion
therefore only has to target those frames *by name*: no blackboard plumbing or
pose hand-off between the two steps is required.
"""

from collections.abc import Sequence

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import create_move_to_frame_task
from src.hardware.grasp_planner import SURFACE_Z_POS, GraspPlannerInterface
from src.hardware.robot import RobotInterface


def build_grasp_planning_subtree(
  grasp_planner: GraspPlannerInterface,
  robot: RobotInterface | None = None,
  candidate_objects: Sequence[str] = ("raw_stock_50x50x75_1",),
  parent_object: str = "root",
  grasp_frame: str = "grasp",
  pregrasp_frame: str = "pre_grasp",
  surfaces: Sequence[int] = (SURFACE_Z_POS,),
  num_rotations: int = 4,
  obj_dims_in_meters: tuple[float, float, float] | None = None,
  move_to_pregrasp: bool = True,
  motion_type: str = "ANY",
  allow_tool_z_rotation: bool = False,
  cone_opening_half_angle: float = 0.0,
  plan_id: str | None = None,
  subtree_name: str = "Plan Grasp and Approach",
) -> bt.Node:
  """Builds a subtree planning a grasp and approaching the resulting pre-grasp.

  Args:
    grasp_planner: Grasp planner adapter invoking the MoveIt grasp skill.
    robot: Robot adapter used for the approach motion. Required unless
      `move_to_pregrasp` is False.
    candidate_objects: Object World object names to plan grasps for. All
      candidates are ranked jointly and the best one wins.
    parent_object: Object owning the output frames (default: 'root').
    grasp_frame: Pre-existing frame updated to the planned grasp pose.
    pregrasp_frame: Pre-existing frame updated to the planned pre-grasp pose and
      used as the approach target.
    surfaces: Object surfaces to generate grasp candidates on. Defaults to the
      top face (+Z) only.
    num_rotations: Grasp candidates per surface, spread evenly about the surface
      normal.
    obj_dims_in_meters: Optional explicit box dimensions. When omitted, the
      planning service derives them from the MoveIt planning scene.
    move_to_pregrasp: Whether to append the approach motion. Set to False to
      validate grasp planning without moving the arm.
    motion_type: Motion segment type for the approach ('ANY', 'LINEAR',
      'JOINT').
    allow_tool_z_rotation: Whether to free wrist rotation about the tool
      approach axis during the approach.
    cone_opening_half_angle: Rotation cone half angle in radians, used only when
      `allow_tool_z_rotation` is True.
    plan_id: Optional identifier for the plan event.
    subtree_name: Descriptive name for the returned sequence.

  Returns:
    Behavior tree Sequence containing the grasp planning task and, optionally,
    the approach motion to the resulting pre-grasp frame.

  Raises:
    ValueError: If no candidate objects are supplied, or if the approach motion
      is requested without a robot adapter.
  """
  if not candidate_objects:
    raise ValueError(
      "build_grasp_planning_subtree requires at least one candidate object"
      " name. Object names must match the world exactly, including any"
      " instance suffix (e.g. 'raw_stock_50x50x75_1')."
    )
  if move_to_pregrasp and robot is None:
    raise ValueError(
      "build_grasp_planning_subtree requires a robot adapter when"
      " move_to_pregrasp is True. Pass robot=... or set"
      " move_to_pregrasp=False to plan without moving the arm."
    )

  children: list[bt.Node] = [
    grasp_planner.build_plan_grasps_task(
      candidate_objects=candidate_objects,
      grasp_frame_name=grasp_frame,
      pregrasp_frame_name=pregrasp_frame,
      output_parent_object=parent_object,
      surfaces=surfaces,
      num_rotations=num_rotations,
      obj_dims_in_meters=obj_dims_in_meters,
      plan_id=plan_id,
      name=f"Plan Grasps ({', '.join(candidate_objects)})",
    )
  ]

  if move_to_pregrasp:
    children.append(
      create_move_to_frame_task(
        robot=robot,
        frame_name=pregrasp_frame,
        parent_object=parent_object,
        motion_type=motion_type,
        allow_tool_z_rotation=allow_tool_z_rotation,
        cone_opening_half_angle=cone_opening_half_angle,
        task_name=(
          f"Approach Planned Pre-Grasp ({parent_object}/{pregrasp_frame})"
        ),
      )
    )

  return bt.Sequence(name=subtree_name, children=children)
