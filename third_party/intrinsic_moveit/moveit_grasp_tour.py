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

"""Sequential grasp tour across several objects.

For each object in turn the tour plans a grasp and approaches the resulting
pre-grasp frame, then moves on to the next object. It stops at the pre-grasp
and never descends to the grasp pose, so nothing is touched or grasped.

The grasp skill writes its result into the *singleton* frames `root/grasp` and
`root/pre_grasp`, overwriting them on every call. A sequential tour is therefore
safe with no extra frames and no blackboard plumbing, provided each object's
motion completes before the next plan runs -- which a `bt.Sequence` guarantees.
"""

import dataclasses
from collections.abc import Sequence

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import create_move_to_frame_task
from src.hardware.robot import RobotInterface
from third_party.intrinsic_moveit.moveit_grasp_planner import (
  SURFACE_ALL,
  MoveItGraspPlannerInterface,
)
from third_party.intrinsic_moveit.moveit_grasp_planning import (
  build_moveit_grasp_planning_subtree,
)

# The default scene spawns raw_stock_50x50x75_1 .. _3. Names must match the
# world exactly, including the instance suffix.
DEFAULT_TOUR_OBJECTS: tuple[str, ...] = (
  "raw_stock_50x50x75_1",
  "raw_stock_50x50x75_2",
  "raw_stock_50x50x75_3",
)


@dataclasses.dataclass(frozen=True)
class ObjectSpec:
  """Per-object grasp tour parameters.

  Attributes:
    name: Object World object name, matched exactly.
    surfaces: Object surfaces to generate grasp candidates on. Empty means
      every surface, which is the default.
    num_rotations: Grasp candidates per surface.
    retract_dist_m: Distance between the grasp and pre-grasp frames, which is
      how far above the part the tour stops. None uses the grasp planner's own
      default.
  """

  name: str
  surfaces: tuple[int, ...] = SURFACE_ALL
  num_rotations: int = 4
  retract_dist_m: float | None = None


def build_moveit_grasp_tour_subtree(
  grasp_planner: MoveItGraspPlannerInterface,
  robot: RobotInterface | None,
  object_specs: Sequence[ObjectSpec],
  parent_object: str = "root",
  grasp_frame: str = "grasp",
  pregrasp_frame: str = "pre_grasp",
  move_to_pregrasp: bool = True,
  motion_type: str = "ANY",
  allow_tool_z_rotation: bool = False,
  transit_frame: str | None = None,
  transit_parent_object: str = "root",
  continue_on_failure: bool = False,
  subtree_name: str = "Grasp Tour",
) -> bt.Node:
  """Builds a sequential tour planning and approaching each object in turn.

  Args:
    grasp_planner: Grasp planner adapter invoking the MoveIt grasp skill.
    robot: Robot adapter used for the approach motions. May be None only when
      `move_to_pregrasp` is False.
    object_specs: Objects to visit, in order.
    parent_object: Object owning the output frames (default: 'root').
    grasp_frame: Pre-existing frame updated to each planned grasp pose.
    pregrasp_frame: Pre-existing frame updated to each planned pre-grasp pose.
    move_to_pregrasp: Whether to approach each pre-grasp frame after planning
      it. When False the tour plans every object without moving the arm.
    motion_type: Motion segment type for each approach ('ANY', 'LINEAR',
      'JOINT').
    allow_tool_z_rotation: Whether to free wrist rotation during approaches.
    transit_frame: Optional frame the arm moves to *between* objects, keeping
      the transit path predictable instead of sweeping from one pre-grasp
      straight to the next. `root/view` is a reasonable choice in the default
      scene. No transit move is inserted when None, nor when
      `move_to_pregrasp` is False.
    transit_parent_object: Parent object of `transit_frame`.
    continue_on_failure: Whether a failed object is skipped so the tour carries
      on. When False the whole tour aborts on the first failure, which is
      usually what you want during bring-up.
    subtree_name: Descriptive name for the returned sequence.

  Returns:
    Behavior tree Sequence visiting every object in order.

  Raises:
    ValueError: If `object_specs` is empty, contains duplicate names, or if
      `move_to_pregrasp` is requested without a robot adapter.
  """
  if not object_specs:
    raise ValueError(
      "build_moveit_grasp_tour_subtree requires at least one object spec."
      " Object names must match the world exactly, including the instance"
      " suffix (e.g. 'raw_stock_50x50x75_1')."
    )

  if move_to_pregrasp and robot is None:
    raise ValueError(
      "build_moveit_grasp_tour_subtree requires a robot adapter to approach the"
      " pre-grasp frames. Pass move_to_pregrasp=False to plan every object"
      " without moving the arm."
    )

  seen: set[str] = set()
  for spec in object_specs:
    if spec.name in seen:
      raise ValueError(
        f"Duplicate object '{spec.name}' in the tour. Each object may only be"
        " visited once; the grasp output frames are overwritten per plan."
      )
    seen.add(spec.name)

  total = len(object_specs)
  children: list[bt.Node] = []
  for index, spec in enumerate(object_specs):
    if transit_frame and move_to_pregrasp and index > 0:
      children.append(
        create_move_to_frame_task(
          robot=robot,
          frame_name=transit_frame,
          parent_object=transit_parent_object,
          motion_type="ANY",
          task_name=(
            f"Transit to {transit_parent_object}/{transit_frame}"
            f" before {spec.name}"
          ),
        )
      )

    verb = "Plan and Approach" if move_to_pregrasp else "Plan Only"
    block = build_moveit_grasp_planning_subtree(
      grasp_planner=grasp_planner,
      robot=robot,
      candidate_objects=(spec.name,),
      parent_object=parent_object,
      grasp_frame=grasp_frame,
      pregrasp_frame=pregrasp_frame,
      surfaces=spec.surfaces,
      num_rotations=spec.num_rotations,
      retract_dist_m=spec.retract_dist_m,
      move_to_pregrasp=move_to_pregrasp,
      motion_type=motion_type,
      allow_tool_z_rotation=allow_tool_z_rotation,
      subtree_name=f"Object {index + 1}/{total}: {verb} ({spec.name})",
    )

    if continue_on_failure:
      block = bt.Fallback(
        name=f"Try {spec.name}",
        children=[
          block,
          bt.Task(
            action=bt.PythonScript(
              function_body=(
                f'print("[GraspTour] Skipping {spec.name}: plan or approach'
                ' failed.")'
              )
            ),
            name=f"Skip {spec.name}",
          ),
        ],
      )

    children.append(block)

  return bt.Sequence(name=subtree_name, children=children)
