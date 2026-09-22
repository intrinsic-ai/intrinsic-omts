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

from intrinsic.solutions import behavior_tree as bt

from src.behaviors.motions import (
  create_move_through_frames_task,
  create_seated_approach_tasks,
)
from src.core.config import AppConfig
from src.hardware.gripper import GripperInterface
from src.hardware.robot import RobotInterface
from src.utils.script_utils import load_python_script


def build_return_to_infeed_subtree(
  robot: RobotInterface,
  gripper: GripperInterface,
  config: AppConfig,
) -> bt.Node:
  """Builds the Behavior Tree subtree for returning the finished part to infeed.

  Sequence:
  1. Calculate random XY shift for placement position based on config.
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

  tasks: list[bt.Node] = []

  if config.cycle.return_shift is not None:
    args_str = (
      f"context, {parent_object!r}, {pregrasp_frame_name!r}, "
      f"{config.cycle.return_shift.center_x}, "
      f"{config.cycle.return_shift.center_y}, "
      f"{config.cycle.return_shift.bounds_x}, "
      f"{config.cycle.return_shift.bounds_y}, "
      f"{config.cycle.return_shift.bounds_rz_degrees}"
    )
    script_body = load_python_script(
      "src.utils.random_placement",
      function_name="randomize_placement_frame",
      call_args=args_str,
    )

    shift_task = bt.PythonScript(function_body=script_body)
    tasks.append(
      bt.Task(action=shift_task, name="0. Shift Return Placement Frame")
    )

  entry_frames = [f for f in (transit_frame_name, pregrasp_frame_name) if f]
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
      touchdown=config.return_touchdown,
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
