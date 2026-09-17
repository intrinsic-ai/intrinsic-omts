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

"""CLI tool to plan a grasp with MoveIt and approach the resulting pre-grasp.

Runs the sideloaded `ai.intrinsic.moveit_plan_grasp_skill` against one or more
candidate objects, then moves the arm to the pre-grasp frame the skill wrote
back into the Object World Service.
"""

import argparse
import sys
import time
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments, execution
from intrinsic.world.proto import object_world_refs_pb2

from src.hardware.robot import UrRobot
from third_party.intrinsic_moveit.moveit_grasp_planner import (
  DEFAULT_TOOL_FRAME,
  SURFACE_ALL,
  MoveItGraspPlanner,
)
from third_party.intrinsic_moveit.moveit_grasp_planning import (
  build_moveit_grasp_planning_subtree,
)

_EPILOG = """\
examples (run through Bazel as `bazel run
  //third_party/intrinsic_moveit/tools:moveit_plan_grasp_and_move -- <flags>`):

  # Dry run: plan a grasp on raw_stock_50x50x75 without moving the arm.
  moveit_plan_grasp_and_move --target_object=raw_stock_50x50x75 --plan_only --surfaces=0,1,4,5

  # Plan and approach the pre-grasp of the workpiece on the surface.
  moveit_plan_grasp_and_move --target_object=raw_stock_50x50x75 --surfaces=0,1,4,5

  # Move raw_stock_50x50x75 into the vice via scene updates and plan again:
  # bazel run //tools/world:apply_scene_updates -- --files configs/raw_stock_in_vice.updates.pbtxt
  moveit_plan_grasp_and_move --target_object=raw_stock_50x50x75 --surfaces=0,1,4,5

  # Narrow further to force a strictly vertical approach.
  moveit_plan_grasp_and_move --target_object=raw_stock_50x50x75 --surfaces=4

prerequisites:
  This is a third-party integration. It does nothing until you have completed
  the intrinsic-moveit integration; OMTS does not deploy any of it for you.

  * moveit_planning_service is running and reachable over Zenoh.
  * ai.intrinsic.moveit_plan_grasp_skill is installed in the target solution.
  * The output frames already exist in the world
    (see configs/scene.updates.pbtxt).

See third_party/intrinsic_moveit/README.md for the setup walkthrough.
"""


def _parse_surfaces(raw: str) -> list[int]:
  """Parses a surface index list or the literal 'all'.

  An empty list is the planning service's own encoding for every surface, so
  'all' is returned as `[]` rather than as an explicit 0..5 enumeration.
  """
  if raw.strip().lower() == "all":
    return list(SURFACE_ALL)
  surfaces: list[int] = []
  for token in raw.split(","):
    token = token.strip()
    if not token:
      continue
    try:
      index = int(token)
    except ValueError as exc:
      raise argparse.ArgumentTypeError(
        f"Invalid surface index '{token}'. Expected 'all' or a"
        " comma-separated list of integers in 0..5 (0:+X, 1:-X, 2:+Y, 3:-Y,"
        " 4:+Z top, 5:-Z bottom)."
      ) from exc
    if not 0 <= index <= 5:
      raise argparse.ArgumentTypeError(
        f"Surface index {index} is out of range. Valid indices are 0..5"
        " (0:+X, 1:-X, 2:+Y, 3:-Y, 4:+Z top, 5:-Z bottom)."
      )
    surfaces.append(index)
  if not surfaces:
    raise argparse.ArgumentTypeError(
      "--surfaces requires 'all' or at least one index, e.g. --surfaces=4."
    )
  return surfaces


def _parse_dimensions(raw: str) -> tuple[float, float, float]:
  """Parses a comma-separated x,y,z box dimension triple in meters."""
  tokens = [token.strip() for token in raw.split(",") if token.strip()]
  if len(tokens) != 3:
    raise argparse.ArgumentTypeError(
      f"--obj_dims_m expects exactly three values 'x,y,z', got '{raw}'."
    )
  try:
    dimensions = tuple(float(token) for token in tokens)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(
      f"--obj_dims_m values must be numbers in meters, got '{raw}'."
    ) from exc
  if any(value <= 0.0 for value in dimensions):
    raise argparse.ArgumentTypeError(
      f"--obj_dims_m values must all be positive, got '{raw}'."
    )
  return dimensions  # type: ignore[return-value]


def resolve_target_objects(raw_values: Sequence[str] | None) -> list[str]:
  """Flattens repeated and comma-separated `--target_object` values.

  Args:
    raw_values: Raw flag occurrences, or None when the flag was not supplied.

  Returns:
    Ordered, de-duplicated object names.

  Raises:
    ValueError: If no target objects were provided or all values are empty.
  """
  if not raw_values:
    raise ValueError(
      "--target_object is required. Specify at least one target object name"
      " (e.g. --target_object=raw_stock_50x50x75)."
    )

  names: list[str] = []
  for value in raw_values:
    for name in value.split(","):
      name = name.strip()
      if name and name not in names:
        names.append(name)
  if not names:
    raise ValueError(
      "--target_object is required. Specify at least one target object name"
      " (e.g. --target_object=raw_stock_50x50x75)."
    )
  return names


def _transform_node(
  solution: Any,
  object_name: str,
  frame_name: str | None = None,
) -> Any:
  """Resolves a world transform node for an object, or a frame on an object."""
  by_name = object_world_refs_pb2.TransformNodeReferenceByName()
  if frame_name:
    by_name.frame.CopyFrom(
      object_world_refs_pb2.FrameReferenceByName(
        object_name=object_name, frame_name=frame_name
      )
    )
  else:
    by_name.object.CopyFrom(
      object_world_refs_pb2.ObjectReferenceByName(object_name=object_name)
    )
  return solution.world.get_transform_node(
    object_world_refs_pb2.TransformNodeReference(by_name=by_name)
  )


def get_root_transform(
  solution: Any,
  object_name: str,
  frame_name: str | None = None,
) -> Any | None:
  """Returns the transform of a node relative to the world root, or None."""
  try:
    node = _transform_node(solution, object_name, frame_name)
    return solution.world.get_transform(solution.world.root, node)
  except Exception:  # pylint: disable=broad-except
    return None


def print_frame_transform(
  solution: Any,
  parent_object: str,
  frame_name: str,
  label: str,
) -> None:
  """Prints the transform of `parent_object/frame_name` relative to the root.

  Args:
    solution: Connected SBL deployment instance.
    parent_object: Object owning the frame.
    frame_name: Frame name under `parent_object`.
    label: Short prefix describing when the reading was taken.
  """
  transform = get_root_transform(solution, parent_object, frame_name)
  if transform is None:
    print(
      f"{label} could not read '{parent_object}/{frame_name}'.\n"
      "  Ensure the frame exists in the world. Frames are declared in"
      " configs/scene.updates.pbtxt and applied by"
      " tools/world:apply_scene_updates.",
      file=sys.stderr,
    )
    return
  print(f"{label} {parent_object}/{frame_name} in root: {transform}")


def report_selected_object(
  solution: Any,
  candidate_objects: Sequence[str],
  parent_object: str,
  grasp_frame: str,
) -> None:
  """Reports which candidate object the winning grasp was planned on.

  The skill ranks every candidate jointly and writes the top grasp relative to
  whichever object won, so the candidate nearest the resulting grasp frame is
  the one that was selected.

  Args:
    solution: Connected SBL deployment instance.
    candidate_objects: Object names that were offered to the planner.
    parent_object: Object owning the grasp frame.
    grasp_frame: Frame updated to the planned grasp pose.
  """
  if len(candidate_objects) < 2:
    return

  grasp_transform = get_root_transform(solution, parent_object, grasp_frame)
  if grasp_transform is None:
    return

  print("\nCandidate objects (distance from the planned grasp):")
  ranked: list[tuple[float, str]] = []
  for object_name in candidate_objects:
    object_transform = get_root_transform(solution, object_name)
    if object_transform is None:
      print(f"  {object_name}: not resolvable in the world")
      continue
    offset = object_transform.translation - grasp_transform.translation
    distance = float(sum(component * component for component in offset) ** 0.5)
    ranked.append((distance, object_name))
    print(f"  {object_name}: {distance:.4f} m")

  if ranked:
    ranked.sort()
    print(f"Grasp was planned on '{ranked[0][1]}' (nearest candidate).")


def moveit_plan_grasp_and_move(
  solution: Any,
  candidate_objects: Sequence[str],
  parent_object: str = "root",
  grasp_frame: str = "grasp",
  pregrasp_frame: str = "pre_grasp",
  surfaces: Sequence[int] = SURFACE_ALL,
  num_rotations: int = 4,
  obj_dims_in_meters: tuple[float, float, float] | None = None,
  retract_dist_m: float = 0.05,
  timeout_ms: float = 15000.0,
  group_name: str = "ur_manipulator",
  end_effector_group: str = "hand",
  tool_frame_name: str = DEFAULT_TOOL_FRAME,
  arm_part_name: str = "ur_module",
  tool_object_name: str = "gripper",
  motion_type: str = "ANY",
  allow_tool_z_rotation: bool = False,
  plan_only: bool = False,
  settle_sec: float = 0.3,
) -> bool:
  """Plans a grasp and optionally approaches the resulting pre-grasp frame.

  Args:
    solution: Connected SBL deployment instance.
    candidate_objects: Object World object names to plan grasps for.
    parent_object: Object owning the output frames.
    grasp_frame: Pre-existing frame updated to the planned grasp pose.
    pregrasp_frame: Pre-existing frame updated to the planned pre-grasp pose.
    surfaces: Object surfaces to generate grasp candidates on. Empty means
      every surface.
    num_rotations: Grasp candidates per surface.
    obj_dims_in_meters: Optional explicit box dimensions in meters.
    retract_dist_m: Distance between the grasp and pre-grasp frames in meters.
    timeout_ms: Grasp planning timeout in milliseconds.
    group_name: MoveIt planning group for the arm.
    end_effector_group: MoveIt end-effector group.
    tool_frame_name: MoveIt robot model link used as the grasp TCP.
    arm_part_name: Robot arm part name in solution.world.
    tool_object_name: End-effector tool object name in solution.world.
    motion_type: Motion segment type for the approach.
    allow_tool_z_rotation: Whether to free wrist rotation during the approach.
    plan_only: Whether to skip the approach motion.
    settle_sec: Delay before planning, giving the planning scene time to catch
      up with recent world changes.

  Returns:
    True when the behavior tree ran to completion, False otherwise.
  """
  print_frame_transform(solution, parent_object, pregrasp_frame, "[before]")

  if settle_sec > 0.0:
    print(f"Waiting {settle_sec:.2f}s for the planning scene to settle...")
    time.sleep(settle_sec)

  grasp_planner = MoveItGraspPlanner(
    solution=solution,
    tool_frame_name=tool_frame_name,
    tool_object_name=tool_object_name,
    group_name=group_name,
    end_effector_group=end_effector_group,
    retract_dist_m=retract_dist_m,
    timeout_ms=timeout_ms,
  )

  robot = None
  if not plan_only:
    robot = UrRobot(
      solution=solution,
      arm_part_name=arm_part_name,
      tool_object_name=tool_object_name,
    )

  subtree = build_moveit_grasp_planning_subtree(
    grasp_planner=grasp_planner,
    robot=robot,
    candidate_objects=candidate_objects,
    parent_object=parent_object,
    grasp_frame=grasp_frame,
    pregrasp_frame=pregrasp_frame,
    surfaces=surfaces,
    num_rotations=num_rotations,
    obj_dims_in_meters=obj_dims_in_meters,
    move_to_pregrasp=not plan_only,
    motion_type=motion_type,
    allow_tool_z_rotation=allow_tool_z_rotation,
  )

  candidates = ", ".join(candidate_objects)
  # An empty list means every surface, so the candidate estimate has to count
  # all six faces rather than the zero entries actually present.
  surface_count = len(surfaces) or 6
  surface_label = str(list(surfaces)) if surfaces else "all"
  grasps_per_object = surface_count * num_rotations
  print(
    f"\nPlanning grasps for [{candidates}] on surfaces {surface_label} with"
    f" {num_rotations} rotations each ({grasps_per_object} candidates per"
    f" object, {grasps_per_object * len(candidate_objects)} total), tool frame"
    f" '{tool_frame_name}', group '{group_name}'..."
  )
  if len(candidate_objects) > 1:
    print(
      "All objects are ranked jointly; the single best grasp across the pool"
      " is written to the output frames."
    )
  if plan_only:
    print("Plan-only mode: the arm will not move.")

  try:
    solution.executive.run(subtree)
  except execution.ExecutionFailedError as exc:
    print(f"\n[!] Behavior tree execution failed: {exc}", file=sys.stderr)
    if hasattr(solution.executive, "get_errors"):
      print(
        f"Executive errors: {solution.executive.get_errors()}",
        file=sys.stderr,
      )
    print(
      "\nCommon causes:\n"
      "  * moveit_planning_service is down or not bridged over Zenoh.\n"
      f"  * '{candidates}' is not present in the MoveIt planning scene.\n"
      "  * No collision-free grasp reachable. Every surface is sampled by\n"
      "    default, so raise --num_rotations or relax --surfaces if you\n"
      "    narrowed it.\n"
      f"  * Output frames '{parent_object}/{grasp_frame}' and"
      f" '{parent_object}/{pregrasp_frame}' do not exist in the world.",
      file=sys.stderr,
    )
    return False
  except Exception as exc:  # pylint: disable=broad-except
    print(f"\n[!] Error during execution: {exc}", file=sys.stderr)
    return False

  print_frame_transform(solution, parent_object, grasp_frame, "[after] ")
  print_frame_transform(solution, parent_object, pregrasp_frame, "[after] ")
  report_selected_object(
    solution, candidate_objects, parent_object, grasp_frame
  )

  if plan_only:
    print(
      f"\n[✓] Grasp planned. '{parent_object}/{pregrasp_frame}' updated."
      " Re-run without --plan_only to approach it."
    )
  else:
    print(
      f"\n[✓] Grasp planned and arm moved to"
      f" '{parent_object}/{pregrasp_frame}'."
      " Approach the grasp with: bazel run //tools/jogging:move_to_frame --"
      f" --frame={grasp_frame} --motion_type=LINEAR"
    )
  return True


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    prog="moveit_plan_grasp_and_move",
    description=(
      "Plan a grasp with the MoveIt grasp planning service and move the arm"
      " to the resulting pre-grasp frame."
    ),
    epilog=_EPILOG,
    formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution gRPC address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
    "--target_object",
    type=str,
    action="append",
    required=True,
    dest="target_objects",
    metavar="NAME[,NAME...]",
    help=(
      "Object World object name to plan grasps for (e.g. raw_stock_50x50x75)."
      " Repeatable, and each value may be a comma-separated list. All"
      " candidates are ranked jointly and the single best grasp wins. Required."
    ),
  )
  parser.add_argument(
    "--plan_only",
    action="store_true",
    default=False,
    help="Plan and publish the frames without moving the arm.",
  )
  parser.add_argument(
    "--parent_object",
    type=str,
    default="root",
    help="Object owning the output grasp frames (default: 'root').",
  )
  parser.add_argument(
    "--grasp_frame",
    type=str,
    default="grasp",
    help="Pre-existing frame updated to the planned grasp pose.",
  )
  parser.add_argument(
    "--pregrasp_frame",
    type=str,
    default="pre_grasp",
    help="Pre-existing frame updated to the planned pre-grasp pose.",
  )
  parser.add_argument(
    "--surfaces",
    type=_parse_surfaces,
    default=list(SURFACE_ALL),
    metavar="LIST",
    help=(
      "Comma-separated object surfaces to sample: 0:+X, 1:-X, 2:+Y, 3:-Y,"
      " 4:+Z top, 5:-Z bottom, or 'all' (default: all). Use 0,1,4,5 for"
      " raw_stock_50x50x75 to bar end-cap grasps on its 75 mm Y axis."
    ),
  )
  parser.add_argument(
    "--num_rotations",
    type=int,
    default=4,
    help="Grasp candidates per surface, spread about its normal (default: 4).",
  )
  parser.add_argument(
    "--obj_dims_m",
    type=_parse_dimensions,
    default=None,
    metavar="X,Y,Z",
    help=(
      "Explicit box dimensions in meters. Omit to derive them from the MoveIt"
      " planning scene."
    ),
  )
  parser.add_argument(
    "--retract_dist_m",
    type=float,
    default=0.05,
    help="Grasp to pre-grasp standoff distance in meters (default: 0.05).",
  )
  parser.add_argument(
    "--timeout_ms",
    type=float,
    default=15000.0,
    help="Grasp planning timeout in milliseconds (default: 15000).",
  )
  parser.add_argument(
    "--motion_type",
    type=str,
    default="ANY",
    choices=["ANY", "LINEAR", "JOINT"],
    help="Approach motion trajectory type (default: ANY).",
  )
  parser.add_argument(
    "--allow_tool_z_rotation",
    action="store_true",
    default=False,
    help="Free wrist rotation about the tool approach axis during approach.",
  )
  parser.add_argument(
    "--group_name",
    type=str,
    default="ur_manipulator",
    help="MoveIt planning group for the arm (default: 'ur_manipulator').",
  )
  parser.add_argument(
    "--end_effector_group",
    type=str,
    default="hand",
    help="MoveIt end-effector group (default: 'hand').",
  )
  parser.add_argument(
    "--tool_frame_name",
    type=str,
    default=DEFAULT_TOOL_FRAME,
    help=(
      "MoveIt robot model link used as the grasp TCP (default:"
      f" '{DEFAULT_TOOL_FRAME}'). Must be a link in the SRDF; 'hande_tcp',"
      " 'hande_tool_frame' and 'tool_frame' are coincident aliases."
    ),
  )
  parser.add_argument(
    "--arm_part_name",
    type=str,
    default="ur_module",
    help="Robot arm part name in solution.world (default: 'ur_module').",
  )
  parser.add_argument(
    "--tool_object_name",
    type=str,
    default="gripper",
    help="End-effector tool object name (default: 'gripper').",
  )
  parser.add_argument(
    "--settle_sec",
    type=float,
    default=0.3,
    help=(
      "Delay before planning, letting the planning scene catch up with recent"
      " world changes (default: 0.3)."
    ),
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
  """Entry point. Returns a process exit code."""
  args = parse_args(argv)

  if args.num_rotations <= 0:
    print(
      f"--num_rotations must be positive, got {args.num_rotations}.",
      file=sys.stderr,
    )
    return 2
  if args.retract_dist_m <= 0.0:
    print(
      f"--retract_dist_m must be positive, got {args.retract_dist_m}.",
      file=sys.stderr,
    )
    return 2
  if args.timeout_ms <= 0.0:
    print(
      f"--timeout_ms must be positive, got {args.timeout_ms}.", file=sys.stderr
    )
    return 2

  try:
    target_objects = resolve_target_objects(args.target_objects)
  except ValueError as exc:
    print(f"Error: {exc}", file=sys.stderr)
    return 2

  print(f"Connecting to solution at {args.address}...")
  try:
    solution = deployments.connect(address=args.address)
  except Exception as exc:  # pylint: disable=broad-except
    print(
      f"Could not connect to '{args.address}': {exc}\n"
      "  Check that the solution is running and that --address points at its"
      " gRPC endpoint (IOC default: localhost:17080).",
      file=sys.stderr,
    )
    return 1

  try:
    succeeded = moveit_plan_grasp_and_move(
      solution=solution,
      candidate_objects=target_objects,
      parent_object=args.parent_object,
      grasp_frame=args.grasp_frame,
      pregrasp_frame=args.pregrasp_frame,
      surfaces=args.surfaces,
      num_rotations=args.num_rotations,
      obj_dims_in_meters=args.obj_dims_m,
      retract_dist_m=args.retract_dist_m,
      timeout_ms=args.timeout_ms,
      group_name=args.group_name,
      end_effector_group=args.end_effector_group,
      tool_frame_name=args.tool_frame_name,
      arm_part_name=args.arm_part_name,
      tool_object_name=args.tool_object_name,
      motion_type=args.motion_type,
      allow_tool_z_rotation=args.allow_tool_z_rotation,
      plan_only=args.plan_only,
      settle_sec=args.settle_sec,
    )
  except ValueError as exc:
    print(f"{exc}", file=sys.stderr)
    return 2
  except KeyboardInterrupt:
    print("\nInterrupted.", file=sys.stderr)
    return 130

  return 0 if succeeded else 1


if __name__ == "__main__":
  sys.exit(main())
