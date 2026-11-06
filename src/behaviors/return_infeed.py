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

"""Infeed return / outfeed placement subtree."""

import dataclasses
from unittest import mock

from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import proto_building as pb

from src.behaviors.motions import (
  create_move_through_frames_task,
  create_seated_approach_tasks,
)
from src.core.config import AppConfig
from src.hardware.grasping import GraspPlannerInterface, plans_exact_contacts
from src.hardware.gripper import GripperInterface
from src.hardware.robot import Robot
from src.utils.script_utils import load_python_script


def _python_script_task(
  robot: Robot,
  function_name: str,
  fields: list[tuple[str, str, object]],
) -> bt.PythonScript:
  """Builds a `random_placement` script task with `(type, name, arg)` params.

  The signature is left unset for mocked solutions (unit tests).
  """
  signature = None
  solution = getattr(robot, "_solution", None)
  if (
    solution
    and hasattr(solution, "proto_builder")
    and not isinstance(solution.proto_builder, mock.MagicMock)
  ):
    signature = solution.proto_builder.create_signature_with_args(
      parameters=pb.MessageSpec(
        fields=[
          pb.FieldSpec(type=type_name, name=name, number=number, arg=arg)
          for number, (type_name, name, arg) in enumerate(fields, 1)
        ]
      )
    )
  return bt.PythonScript(
    signature_with_args=signature,
    function_body=load_python_script(
      "src.utils.random_placement", function_name=function_name
    ),
  )


def build_return_to_infeed_subtree(
  robot: Robot,
  gripper: GripperInterface,
  config: AppConfig,
  grasp_planner: GraspPlannerInterface | None = None,
) -> bt.Node:
  """Builds the Behavior Tree subtree for returning the finished part to infeed.

  Sequence:
  1. Calculate random XY shift for placement position based on config.
     For mesh planners (no contact touchdown at pick), then solve
     `<grasp>_return` / `<pregrasp>_return` tool targets that place the part
     at its saved infeed pose given its current pose in the gripper, and
     use the plain cycle standoff (no 15 mm pick-retract compensation).
  2. Approach infeed `pregrasp_frame` (`ANY`), blending through `transit_frame`
     if configured.
  3. Approach `grasp_frame` standoff (`LINEAR`) and perform compliant touchdown
     along tool +Z (`create_seated_approach_tasks` with
     `config.return_touchdown`) to place finished part on table surface.
  4. Open gripper to release part and detach workpiece entity from gripper in
     the belief world.
  5. Blended retract (`[pregrasp_frame, view_frame]`, `["LINEAR", "ANY"]`) with
     segment-scoped collision exclusions to return arm to `view_frame`.

  Args:
      robot: Robot controller adapter.
      gripper: End-effector gripper adapter.
      config: Validated application configuration dataclass.
      grasp_planner: Optional planner selecting grip-corrected mesh placement.

  Returns:
      Behavior tree sequence node executing infeed return.
  """
  parent_object = config.frames.parent_object
  pregrasp_frame_name = config.frames.pregrasp_frame
  grasp_frame_name = config.frames.grasp_frame
  view_frame_name = config.frames.view_frame
  transit_frame_name = config.frames.transit_frame
  workpiece_object_name = config.cycle.workpiece_id

  tool_to_workpiece_collision_pair = (
    config.robot.tool_object_name,
    workpiece_object_name,
  )

  detach_collision_pairs = [tool_to_workpiece_collision_pair]
  if config.robot.enclosure_object_name:
    detach_collision_pairs.extend(
      [
        (config.robot.enclosure_object_name, workpiece_object_name),
        (config.robot.tool_object_name, config.robot.enclosure_object_name),
      ]
    )

  tasks: list[bt.Node] = []

  if config.cycle.return_shift is not None:
    shift = config.cycle.return_shift
    tasks.append(
      bt.Task(
        name="0. Shift Return Placement Frame",
        action=_python_script_task(
          robot,
          "randomize_placement_frame",
          [
            ("string", "parent_object", parent_object),
            ("string", "frame_name", pregrasp_frame_name),
            ("float", "return_center_x", shift.center_x),
            ("float", "return_center_y", shift.center_y),
            ("float", "return_bounds_x", shift.bounds_x),
            ("float", "return_bounds_y", shift.bounds_y),
            ("float", "return_bounds_rz_degrees", shift.bounds_rz_degrees),
            ("string", "grasp_frame_name", grasp_frame_name),
          ],
        ),
      )
    )

  touchdown = config.return_touchdown
  if plans_exact_contacts(grasp_planner):
    # Mesh re-picks change how the part sits in the gripper, so the tool target
    # is solved from the part pose saved at the infeed pick (see
    # `update_mesh_return_frames`).
    tasks.append(
      bt.Task(
        name="Update Mesh Return Frames",
        action=_python_script_task(
          robot,
          "update_mesh_return_frames",
          [
            ("string", "workpiece_object_name", workpiece_object_name),
            ("string", "tool_object_name", config.robot.tool_object_name),
            ("string", "tool_frame_name", config.robot.tool_frame_name),
            ("string", "grasp_frame_name", grasp_frame_name),
            ("string", "pregrasp_frame_name", pregrasp_frame_name),
            (
              "double",
              "pregrasp_distance_m",
              config.grasp.mesh_pregrasp_distance_m,
            ),
          ],
        ),
      )
    )
    grasp_frame_name += "_return"
    pregrasp_frame_name += "_return"
    # Mesh picks close at the planned pose, without the legacy pre-close
    # retract, so the return needs no extra standoff or fallback offset.
    touchdown = dataclasses.replace(
      touchdown, standoff_m=config.cycle.standoff_distance_meters
    )

  entry_frames = [pregrasp_frame_name]
  if transit_frame_name:
    entry_frames.insert(0, transit_frame_name)
  tasks.append(
    create_move_through_frames_task(
      robot=robot,
      frame_names=entry_frames,
      parent_object=parent_object,
      config=config,
      motion_type="ANY",
      excluded_collision_pairs=detach_collision_pairs,
      task_name=(
        f"Blended Transit to Infeed Placement ({' -> '.join(entry_frames)})"
        if len(entry_frames) > 1
        else (
          f"Approach Infeed Placement ({parent_object}/{pregrasp_frame_name})"
        )
      ),
    )
  )

  tasks.extend(
    create_seated_approach_tasks(
      robot=robot,
      frame_name=grasp_frame_name,
      parent_object=parent_object,
      touchdown=touchdown,
      config=config,
      label="Return Infeed",
      excluded_collision_pairs=detach_collision_pairs,
    )
  )

  exit_frames = [
    pregrasp_frame_name,
    view_frame_name,
  ]
  tasks.extend(
    [
      gripper.build_open_task(name="Release Finished Part"),
      robot.build_detach_object_task(
        object_name=workpiece_object_name,
        name=f"Detach {workpiece_object_name} from Gripper",
      ),
      create_move_through_frames_task(
        robot=robot,
        frame_names=exit_frames,
        parent_object=parent_object,
        config=config,
        motion_type=["LINEAR", "ANY"],
        excluded_collision_pairs=detach_collision_pairs,
        task_name=f"Blended Retract to View ({' -> '.join(exit_frames)})",
      ),
    ]
  )

  return bt.Sequence(name="5. Return to Infeed Subtree", children=tasks)
