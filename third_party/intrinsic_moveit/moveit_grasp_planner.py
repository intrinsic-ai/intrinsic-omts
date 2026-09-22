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

"""Grasp planner interfaces and implementations.

`MoveItGraspPlanner` wraps the sideloaded `ai.intrinsic.moveit_plan_grasp_skill`
asset. The skill forwards a `PlanGrasps` request to the
`moveit_planning_service` ROS 2 node, ranks the returned candidates by grasp
quality, and writes the top-ranked grasp and pre-grasp poses back into the
Object World Service by updating pre-existing frames.

Because the poses are published to the world rather than returned on the
blackboard, downstream Cartesian motions can simply target the output frames by
name (see `third_party.intrinsic_moveit.moveit_grasp_planning`).
"""

import abc
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import behavior_tree as bt
from intrinsic.world.proto import object_world_refs_pb2

# Object surface indices understood by the grasp planning service. Indices refer
# to the object's local axes: 0:+X, 1:-X, 2:+Y, 3:-Y, 4:+Z (top), 5:-Z (bottom).
SURFACE_X_POS = 0
SURFACE_X_NEG = 1
SURFACE_Y_POS = 2
SURFACE_Y_NEG = 3
SURFACE_Z_POS = 4
SURFACE_Z_NEG = 5

# An empty `surfaces` list is the planning service's own encoding for "every
# surface", so it is preferred over enumerating all six indices explicitly.
# Sampling every surface is the default because parts arrive in arbitrary
# orientations; collision checking prunes the candidates that are unreachable.
SURFACE_ALL: tuple[int, ...] = ()

# The skill flattens `tool_frame` to a bare frame *name* before handing it to
# MoveIt, so it must name a link in the MoveIt robot model rather than an Object
# World path. The gripper xacro publishes "hande_tcp", "hande_tool_frame" and an
# unqualified "tool_frame" as zero-offset alias links, so all three resolve to
# the same pose; "hande_tcp" is chosen here because it is the gripper-native
# name and matches the planning service's own fallback default.
DEFAULT_TOOL_FRAME = "hande_tcp"

# Attribute name of the sideloaded skill under `solution.skills.ai.intrinsic`.
DEFAULT_GRASP_SKILL_NAME = "moveit_plan_grasp_skill"


class MoveItGraspPlannerInterface(abc.ABC):
  """Abstract interface for model-based grasp planning."""

  @abc.abstractmethod
  def build_plan_grasps_task(
    self,
    candidate_objects: Sequence[str],
    grasp_frame_name: str | None = "grasp",
    pregrasp_frame_name: str | None = "pre_grasp",
    output_parent_object: str = "root",
    surfaces: Sequence[int] = SURFACE_ALL,
    num_rotations: int = 4,
    obj_dims_in_meters: tuple[float, float, float] | None = None,
    retract_dist_m: float | None = None,
    plan_id: str | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a task planning grasps and publishing the result to the world."""
    raise NotImplementedError


class MoveItGraspPlanner(MoveItGraspPlannerInterface):
  """Grasp planner adapter backed by the sideloaded MoveIt grasp skill."""

  def __init__(
    self,
    solution: Any,
    tool_frame_name: str = DEFAULT_TOOL_FRAME,
    tool_object_name: str = "gripper",
    group_name: str = "ur_manipulator",
    end_effector_group: str = "hand",
    retract_dist_m: float = 0.05,
    timeout_ms: float = 15000.0,
    max_num_grasps: int = 1,
    gripper_motion_duration_sec: float = 0.5,
    skill_name: str = DEFAULT_GRASP_SKILL_NAME,
  ) -> None:
    """Initializes the MoveIt grasp planner adapter.

    Args:
      solution: Connected SBL deployment instance (from deployments.connect).
      tool_frame_name: MoveIt robot model link used as the grasp TCP (default:
        'hande_tcp'). This is a MoveIt link name, not an Object World frame.
      tool_object_name: Object World object owning the tool frame. Recorded for
        provenance only; the skill forwards the frame name alone.
      group_name: MoveIt planning group for the arm (default: 'ur_manipulator').
      end_effector_group: MoveIt end-effector group (default: 'hand').
      retract_dist_m: Distance in meters between the grasp and pre-grasp frames.
      timeout_ms: Planning timeout in milliseconds.
      max_num_grasps: Maximum number of ranked grasps returned in the result.
      gripper_motion_duration_sec: Time budget for the gripper open/close
        posture used while planning.
      skill_name: Attribute name of the grasp skill under
        `solution.skills.ai.intrinsic`.

    Raises:
      ValueError: If the grasp skill asset is not installed in the solution.
    """
    self._solution = solution
    self._tool_frame_name = tool_frame_name
    self._tool_object_name = tool_object_name
    self._group_name = group_name
    self._end_effector_group = end_effector_group
    self._retract_dist_m = retract_dist_m
    self._timeout_ms = timeout_ms
    self._max_num_grasps = max_num_grasps
    self._gripper_motion_duration_sec = gripper_motion_duration_sec
    self._skill_name = skill_name

    try:
      self._grasp_skill = getattr(solution.skills.ai.intrinsic, skill_name)
    except AttributeError as exc:
      raise ValueError(
        f"Grasp skill 'ai.intrinsic.{skill_name}' is not available in this"
        " solution. Install the bundle with 'inctl asset install"
        " --address <address> <path-to-bundle>.tar' and then call"
        " 'solution.skills.update()' or reconnect."
      ) from exc

  @property
  def tool_frame_reference(
    self,
  ) -> object_world_refs_pb2.TransformNodeReference:
    """Constructs the tool frame reference forwarded to the planning service."""
    return object_world_refs_pb2.TransformNodeReference(
      by_name=object_world_refs_pb2.TransformNodeReferenceByName(
        frame=object_world_refs_pb2.FrameReferenceByName(
          object_name=self._tool_object_name,
          frame_name=self._tool_frame_name,
        )
      )
    )

  def _frame_reference(
    self, parent_object: str, frame_name: str
  ) -> object_world_refs_pb2.TransformNodeReference:
    """Constructs a reference to an existing `object/frame` pair."""
    return object_world_refs_pb2.TransformNodeReference(
      by_name=object_world_refs_pb2.TransformNodeReferenceByName(
        frame=object_world_refs_pb2.FrameReferenceByName(
          object_name=parent_object,
          frame_name=frame_name,
        )
      )
    )

  def _box_annotations(
    self,
    surfaces: Sequence[int],
    num_rotations: int,
    obj_dims_in_meters: tuple[float, float, float] | None,
  ) -> Any:
    """Builds the box-shaped grasp annotation parameters for the request."""
    annotations = (
      self._grasp_skill.intrinsic_proto.grasping.BoxShapedGraspAnnotationParams(
        surfaces=list(surfaces),
        num_rotations=num_rotations,
      )
    )
    if obj_dims_in_meters is not None:
      annotations.obj_dims_in_meters.CopyFrom(
        self._grasp_skill.intrinsic_proto.Vector3(
          x=obj_dims_in_meters[0],
          y=obj_dims_in_meters[1],
          z=obj_dims_in_meters[2],
        )
      )
    return annotations

  def build_plan_grasps_task(
    self,
    candidate_objects: Sequence[str],
    grasp_frame_name: str | None = "grasp",
    pregrasp_frame_name: str | None = "pre_grasp",
    output_parent_object: str = "root",
    surfaces: Sequence[int] = SURFACE_ALL,
    num_rotations: int = 4,
    obj_dims_in_meters: tuple[float, float, float] | None = None,
    retract_dist_m: float | None = None,
    plan_id: str | None = None,
    name: str | None = None,
  ) -> bt.Node:
    """Builds a grasp planning task writing the top grasp into the world.

    Args:
      candidate_objects: Object World object names to plan grasps for. All
        candidates are evaluated and ranked jointly by grasp quality; the single
        best candidate across the pool wins.
      grasp_frame_name: Pre-existing frame updated to the top grasp pose, or
        None to leave the grasp pose unpublished.
      pregrasp_frame_name: Pre-existing frame updated to the top pre-grasp pose,
        or None to leave the pre-grasp pose unpublished.
      output_parent_object: Object owning the output frames (default: 'root').
      surfaces: Object surfaces to generate grasp candidates on. Defaults to
        every surface, encoded as an empty list, which lets the planner handle
        parts lying in arbitrary orientations. Narrow it to bias the approach.
      num_rotations: Grasp candidates per surface, spread evenly about the
        surface normal.
      obj_dims_in_meters: Optional explicit box dimensions. When omitted, the
        planning service derives them from the collision object in the MoveIt
        planning scene.
      retract_dist_m: Distance in meters between the grasp and pre-grasp frames
        for this plan. Falls back to the value given at construction.
      plan_id: Optional identifier for the plan event. A timestamped id is
        generated by the skill when omitted.
      name: Optional custom behavior tree task name.

    Returns:
      Executable behavior tree Node invoking the grasp planning skill.

    Raises:
      ValueError: If no candidate objects are supplied, if num_rotations is not
        positive, or if retract_dist_m is not positive.
    """
    if not candidate_objects:
      raise ValueError(
        "build_plan_grasps_task requires at least one candidate object name."
      )
    if num_rotations <= 0:
      raise ValueError(f"num_rotations must be positive, got {num_rotations}.")
    if retract_dist_m is not None and retract_dist_m <= 0.0:
      raise ValueError(
        f"retract_dist_m must be positive, got {retract_dist_m}."
      )

    task_name = name or f"Plan Grasps ({', '.join(candidate_objects)})"

    kwargs: dict[str, Any] = {
      "candidate_objects": [
        object_world_refs_pb2.ObjectReference(
          by_name=object_world_refs_pb2.ObjectReferenceByName(
            object_name=object_name
          )
        )
        for object_name in candidate_objects
      ],
      "tool_frame": self.tool_frame_reference,
      "box_grasp_annotations": self._box_annotations(
        surfaces=surfaces,
        num_rotations=num_rotations,
        obj_dims_in_meters=obj_dims_in_meters,
      ),
      "retract_dist_m": (
        self._retract_dist_m if retract_dist_m is None else retract_dist_m
      ),
      "group_name": self._group_name,
      "end_effector_group": self._end_effector_group,
      "timeout_ms": self._timeout_ms,
      "max_num_grasps": self._max_num_grasps,
      "gripper_motion_duration_sec": self._gripper_motion_duration_sec,
    }

    if grasp_frame_name:
      kwargs["output_grasp_frame"] = self._frame_reference(
        output_parent_object, grasp_frame_name
      )
    if pregrasp_frame_name:
      kwargs["output_pregrasp_frame"] = self._frame_reference(
        output_parent_object, pregrasp_frame_name
      )
    if plan_id:
      kwargs["plan_id"] = plan_id

    return bt.Task(action=self._grasp_skill(**kwargs), name=task_name)


class MockMoveItGraspPlanner(MoveItGraspPlannerInterface):
  """Mock grasp planner for offline tests without a deployed grasp skill."""

  def __init__(self) -> None:
    self.planned_objects: list[tuple[str, ...]] = []
    self.plan_calls: list[dict[str, Any]] = []

  def build_plan_grasps_task(
    self,
    candidate_objects: Sequence[str],
    grasp_frame_name: str | None = "grasp",
    pregrasp_frame_name: str | None = "pre_grasp",
    output_parent_object: str = "root",
    surfaces: Sequence[int] = SURFACE_ALL,
    num_rotations: int = 4,
    obj_dims_in_meters: tuple[float, float, float] | None = None,
    retract_dist_m: float | None = None,
    plan_id: str | None = None,
    name: str | None = None,
  ) -> bt.Node:
    task_name = name or f"Mock Plan Grasps ({', '.join(candidate_objects)})"
    self.planned_objects.append(tuple(candidate_objects))
    self.plan_calls.append(
      {
        "candidate_objects": tuple(candidate_objects),
        "surfaces": tuple(surfaces),
        "num_rotations": num_rotations,
        "retract_dist_m": retract_dist_m,
        "obj_dims_in_meters": obj_dims_in_meters,
      }
    )
    return bt.Task(
      action=bt.PythonScript(
        function_body=(
          f'print("[MockMoveItGraspPlanner] Planned grasp for'
          f' {", ".join(candidate_objects)}")'
        )
      ),
      name=task_name,
    )
