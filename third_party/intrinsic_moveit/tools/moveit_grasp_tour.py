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

"""CLI tool running a sequential grasp tour over several objects.

For each object in turn: plan a grasp with MoveIt, then approach the resulting
pre-grasp frame. The tour stops at the pre-grasp and never descends to the
grasp pose, so no part is touched and the gripper is never commanded.
"""

import argparse
import sys
import time
from collections.abc import Sequence
from typing import Any

from intrinsic.solutions import deployments, execution

from src.hardware.robot import UrRobot
from third_party.intrinsic_moveit.moveit_grasp_planner import (
  DEFAULT_TOOL_FRAME,
  SURFACE_ALL,
  MoveItGraspPlanner,
)
from third_party.intrinsic_moveit.moveit_grasp_tour import (
  DEFAULT_TOUR_OBJECTS,
  ObjectSpec,
  build_moveit_grasp_tour_subtree,
)
from third_party.intrinsic_moveit.tools.moveit_plan_and_move import (
  print_frame_transform,
)

_SPEC_KEYS = ("surfaces", "rotations", "retract")

_EPILOG = """\
examples (run through Bazel as `bazel run
  //third_party/intrinsic_moveit/tools:moveit_grasp_tour -- <flags>`):

  # Dry run over every part: plan each grasp, never move the arm.
  moveit_grasp_tour --plan_only --surfaces=0,1,4,5

  # Full tour over the default objects (_1 .. _3).
  moveit_grasp_tour --surfaces=0,1,4,5

  # A single object, with motion.
  moveit_grasp_tour --surfaces=0,1,4,5 --object=raw_stock_50x50x75_1

  # Per-object overrides. The flag is repeatable; one object per occurrence.
  # Overrides are ':'-separated key=value pairs appended to the name and win
  # over the matching global flag.
  moveit_grasp_tour --surfaces=0,1,4,5 --object=raw_stock_50x50x75_1 \\
      --object=raw_stock_50x50x75_3:rotations=8

  # Keep going if one object cannot be planned or reached.
  moveit_grasp_tour --surfaces=0,1,4,5 --continue_on_failure

  # Route via root/view between objects instead of sweeping pre-grasp to
  # pre-grasp.
  moveit_grasp_tour --surfaces=0,1,4,5 --transit_frame=view

per-object override keys:
  surfaces=<i,j,...>  Surface indices 0..5 (0:+X 1:-X 2:+Y 3:-Y 4:+Z 5:-Z),
                      or 'all' for every surface (the default). Use
                      0,1,4,5 for raw_stock_50x50x75: those are its four
                      long faces, and it bars grasps on the two end caps
                      of the 75 mm Y axis, which hold the part by its tip.
  rotations=<n>       Grasp candidates per surface.
  retract=<meters>    Gap between the grasp and pre-grasp frames, i.e. how
                      far above the part the tour stops.

prerequisites:
  This is a third-party integration. It does nothing until you have completed
  the intrinsic-moveit integration; OMTS does not deploy any of it for you.

  * moveit_planning_service is running and reachable over Zenoh.
  * ai.intrinsic.moveit_plan_grasp_skill is installed in the target solution.
  * The output frames already exist in the world
    (see configs/scene.updates.pbtxt).

See third_party/intrinsic_moveit/README.md for the setup walkthrough.
"""


def _parse_surfaces(raw: str, context: str) -> tuple[int, ...]:
  """Parses a surface index list or the literal 'all'.

  An empty tuple is the planning service's own encoding for every surface, so
  'all' is returned as `()` rather than as an explicit 0..5 enumeration.
  """
  if raw.strip().lower() == "all":
    return SURFACE_ALL
  surfaces: list[int] = []
  for token in raw.split(","):
    token = token.strip()
    if not token:
      continue
    try:
      index = int(token)
    except ValueError as exc:
      raise argparse.ArgumentTypeError(
        f"{context}: invalid surface index '{token}'. Expected 'all' or"
        " integers in 0..5 (0:+X, 1:-X, 2:+Y, 3:-Y, 4:+Z top, 5:-Z bottom)."
      ) from exc
    if not 0 <= index <= 5:
      raise argparse.ArgumentTypeError(
        f"{context}: surface index {index} is out of range. Valid indices are"
        " 0..5 (0:+X, 1:-X, 2:+Y, 3:-Y, 4:+Z top, 5:-Z bottom)."
      )
    surfaces.append(index)
  if not surfaces:
    raise argparse.ArgumentTypeError(
      f"{context}: 'surfaces' requires 'all' or at least one index, e.g."
      " surfaces=4."
    )
  return tuple(surfaces)


def parse_object_spec(
  raw: str,
  default_surfaces: Sequence[int],
  default_rotations: int,
  default_retract: float | None,
) -> ObjectSpec:
  """Parses one `--object` value into an `ObjectSpec`.

  Syntax is `name[:key=value]...` where key is one of surfaces, rotations or
  retract. Unspecified keys inherit the corresponding global flag.

  Args:
    raw: Raw flag value.
    default_surfaces: Fallback surfaces from --surfaces.
    default_rotations: Fallback rotation count from --num_rotations.
    default_retract: Fallback pre-grasp gap from --retract_dist_m.

  Returns:
    The parsed per-object spec.

  Raises:
    argparse.ArgumentTypeError: On an empty name, an unknown key, a malformed
      key=value pair, or an out-of-range value.
  """
  parts = raw.split(":")
  name = parts[0].strip()
  if not name:
    raise argparse.ArgumentTypeError(
      f"--object='{raw}' has no object name. Expected"
      " 'name[:key=value]...', e.g."
      " 'raw_stock_50x50x75_1:rotations=8'."
    )

  context = f"--object='{raw}'"
  surfaces = tuple(default_surfaces)
  rotations = default_rotations
  retract = default_retract

  for part in parts[1:]:
    part = part.strip()
    if not part:
      continue
    if "=" not in part:
      raise argparse.ArgumentTypeError(
        f"{context}: override '{part}' is not a key=value pair. Known keys:"
        f" {', '.join(_SPEC_KEYS)}."
      )
    key, _, value = part.partition("=")
    key = key.strip().lower()
    value = value.strip()

    if key == "surfaces":
      surfaces = _parse_surfaces(value, context)
    elif key == "rotations":
      rotations = _parse_positive_int(value, key, context)
    elif key == "retract":
      retract = _parse_positive_float(value, key, context)
    else:
      raise argparse.ArgumentTypeError(
        f"{context}: unknown override key '{key}'. Known keys:"
        f" {', '.join(_SPEC_KEYS)}."
      )

  return ObjectSpec(
    name=name,
    surfaces=surfaces,
    num_rotations=rotations,
    retract_dist_m=retract,
  )


def _parse_positive_int(value: str, key: str, context: str) -> int:
  """Parses a strictly positive integer override value."""
  try:
    parsed = int(value)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(
      f"{context}: '{key}' must be an integer, got '{value}'."
    ) from exc
  if parsed <= 0:
    raise argparse.ArgumentTypeError(
      f"{context}: '{key}' must be positive, got {parsed}."
    )
  return parsed


def _parse_positive_float(value: str, key: str, context: str) -> float:
  """Parses a strictly positive float override value."""
  try:
    parsed = float(value)
  except ValueError as exc:
    raise argparse.ArgumentTypeError(
      f"{context}: '{key}' must be a number, got '{value}'."
    ) from exc
  if parsed <= 0.0:
    raise argparse.ArgumentTypeError(
      f"{context}: '{key}' must be positive, got {parsed}."
    )
  return parsed


def resolve_object_specs(
  raw_values: Sequence[str] | None,
  default_surfaces: Sequence[int],
  default_rotations: int,
  default_retract: float | None,
) -> list[ObjectSpec]:
  """Resolves repeated `--object` flags, falling back to the default tour.

  Unlike moveit_plan_and_move's --target_object, values are NOT split on commas:
  commas already separate surface indices inside an override.

  Args:
    raw_values: Raw flag occurrences, or None when the flag was not supplied.
    default_surfaces: Fallback surfaces from --surfaces.
    default_rotations: Fallback rotation count from --num_rotations.
    default_retract: Fallback pre-grasp gap from --retract_dist_m.

  Returns:
    Ordered per-object specs.
  """
  values = list(raw_values) if raw_values else list(DEFAULT_TOUR_OBJECTS)
  return [
    parse_object_spec(
      value,
      default_surfaces=default_surfaces,
      default_rotations=default_rotations,
      default_retract=default_retract,
    )
    for value in values
  ]


def describe_specs(specs: Sequence[ObjectSpec]) -> str:
  """Renders the resolved tour as an aligned, human-readable table."""
  width = max(len(spec.name) for spec in specs)
  lines = [f"  {'object'.ljust(width)}  surfaces      rot  pre-grasp gap"]
  for spec in specs:
    surfaces = ",".join(str(s) for s in spec.surfaces) or "all"
    retract = (
      f"{spec.retract_dist_m:.3f} m"
      if spec.retract_dist_m is not None
      else "planner default"
    )
    lines.append(
      f"  {spec.name.ljust(width)}  {surfaces.ljust(12)}"
      f"  {spec.num_rotations:>3}  {retract}"
    )
  return "\n".join(lines)


def run_tour(
  solution: Any,
  specs: Sequence[ObjectSpec],
  parent_object: str = "root",
  grasp_frame: str = "grasp",
  pregrasp_frame: str = "pre_grasp",
  transit_frame: str | None = None,
  continue_on_failure: bool = False,
  motion_type: str = "ANY",
  allow_tool_z_rotation: bool = False,
  timeout_ms: float = 15000.0,
  group_name: str = "ur_manipulator",
  end_effector_group: str = "hand",
  tool_frame_name: str = DEFAULT_TOOL_FRAME,
  arm_part_name: str = "ur_module",
  tool_object_name: str = "gripper",
  disable_collision_checking: bool = False,
  plan_only: bool = False,
  settle_sec: float = 0.3,
) -> bool:
  """Builds and runs the whole tour as a single behavior tree.

  Args:
    solution: Connected SBL deployment instance.
    specs: Objects to visit, in order.
    parent_object: Object owning the output frames.
    grasp_frame: Pre-existing frame updated to each planned grasp pose.
    pregrasp_frame: Pre-existing frame updated to each planned pre-grasp pose.
    transit_frame: Optional frame visited between objects.
    continue_on_failure: Whether to skip a failed object and carry on.
    motion_type: Motion segment type for each approach.
    allow_tool_z_rotation: Whether to free wrist rotation during approaches.
    timeout_ms: Grasp planning timeout in milliseconds.
    group_name: MoveIt planning group for the arm.
    end_effector_group: MoveIt end-effector group.
    tool_frame_name: MoveIt robot model link used as the grasp TCP.
    arm_part_name: Robot arm part name in solution.world.
    tool_object_name: End-effector tool object name in solution.world.
    disable_collision_checking: Whether to plan motions without collision
      checking. Off by default: the tour drives into the enclosure.
    plan_only: Whether to plan every object without moving the arm.
    settle_sec: Delay before the run, giving the MoveIt planning scene time to
      catch up with recent world changes.

  Returns:
    True when the behavior tree ran to completion, False otherwise.
  """
  print_frame_transform(solution, parent_object, pregrasp_frame, "[before]")

  # One settle is enough for the whole tour: only the arm moves during it, and
  # the per-request scene sync race only bites when a *part* was just moved.
  if settle_sec > 0.0:
    print(f"Waiting {settle_sec:.2f}s for the planning scene to settle...")
    time.sleep(settle_sec)

  # retract_dist_m is supplied per object, so the constructor default only
  # applies to specs that did not override it.
  grasp_planner = MoveItGraspPlanner(
    solution=solution,
    tool_frame_name=tool_frame_name,
    tool_object_name=tool_object_name,
    group_name=group_name,
    end_effector_group=end_effector_group,
    timeout_ms=timeout_ms,
  )

  robot = (
    None
    if plan_only
    else UrRobot(
      solution=solution,
      arm_part_name=arm_part_name,
      tool_object_name=tool_object_name,
      disable_collision_checking=disable_collision_checking,
    )
  )
  tree = build_moveit_grasp_tour_subtree(
    grasp_planner=grasp_planner,
    robot=robot,
    object_specs=specs,
    parent_object=parent_object,
    grasp_frame=grasp_frame,
    pregrasp_frame=pregrasp_frame,
    move_to_pregrasp=not plan_only,
    motion_type=motion_type,
    allow_tool_z_rotation=allow_tool_z_rotation,
    transit_frame=transit_frame,
    transit_parent_object=parent_object,
    continue_on_failure=continue_on_failure,
    subtree_name="Grasp Tour (plan only)" if plan_only else "Grasp Tour",
  )

  print(f"\nTour over {len(specs)} object(s):")
  print(describe_specs(specs))
  if plan_only:
    print("\nPlan-only mode: the arm will not move.")
  else:
    print(
      "\nPer object: plan -> approach pre-grasp. The arm stops at the"
      " pre-grasp frame; it never descends to the grasp pose."
    )
    if not disable_collision_checking:
      print("Motions are collision checked.")
    else:
      print("[!] Collision checking is DISABLED for all motions.")
    if transit_frame:
      print(f"Routing via '{parent_object}/{transit_frame}' between objects.")
    if continue_on_failure:
      print("A failed object is skipped; the tour continues.")

  try:
    solution.executive.run(tree)
  except execution.ExecutionFailedError as exc:
    print(f"\n[!] Tour execution failed: {exc}", file=sys.stderr)
    if hasattr(solution.executive, "get_errors"):
      print(
        f"Executive errors: {solution.executive.get_errors()}",
        file=sys.stderr,
      )
    print(
      "\nCommon causes:\n"
      "  * moveit_planning_service is down or not bridged over Zenoh.\n"
      "  * An object name does not resolve in the MoveIt planning scene.\n"
      "    Confirm with: bazel run //tools/world:inspect_world\n"
      "  * No collision-free grasp reachable on the requested surfaces. Every\n"
      "    surface is sampled by default; if the winning candidate approaches\n"
      "    from an awkward side, pin it with --object=<name>:surfaces=4.\n"
      "  * The pre-grasp pose is out of reach or collides. Shorten the gap\n"
      "    with --object=<name>:retract=0.06.\n"
      f"  * Output frames '{parent_object}/{grasp_frame}' and"
      f" '{parent_object}/{pregrasp_frame}' do not exist in the world.\n"
      "  * Use --continue_on_failure to visit the remaining objects anyway.",
      file=sys.stderr,
    )
    return False
  except Exception as exc:  # pylint: disable=broad-except
    print(f"\n[!] Error during execution: {exc}", file=sys.stderr)
    return False

  print_frame_transform(solution, parent_object, grasp_frame, "[after] ")
  print_frame_transform(solution, parent_object, pregrasp_frame, "[after] ")

  last = specs[-1].name
  if plan_only:
    print(
      f"\n[✓] Planned {len(specs)} object(s). The output frames now hold the"
      f" last plan ('{last}'). Re-run without --plan_only to drive the tour."
    )
  else:
    print(
      f"\n[✓] Tour complete over {len(specs)} object(s); the arm is at"
      f" '{parent_object}/{pregrasp_frame}' for '{last}'."
    )
  return True


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description=(
      "Run a sequential grasp tour: for each object, plan a grasp and approach"
      " the resulting pre-grasp frame. Nothing is touched or grasped."
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
    "--object",
    dest="objects",
    type=str,
    action="append",
    default=None,
    help=(
      "Object to visit, as 'name[:key=value]...'. Repeatable; one object per"
      " occurrence, in visit order. Defaults to"
      f" {', '.join(DEFAULT_TOUR_OBJECTS)}."
    ),
  )
  parser.add_argument(
    "--surfaces",
    type=str,
    default="all",
    help=(
      "Default surface indices for objects without a 'surfaces' override, or"
      " 'all' for every surface (default: all). Use 0,1,4,5 for"
      " raw_stock_50x50x75 to bar end-cap grasps on its 75 mm Y axis."
    ),
  )
  parser.add_argument(
    "--num_rotations",
    type=int,
    default=4,
    help="Default grasp candidates per surface (default: 4).",
  )
  parser.add_argument(
    "--retract_dist_m",
    type=float,
    default=None,
    help=(
      "Default gap in meters between the grasp and pre-grasp frames, i.e. how"
      " far above the part the tour stops. Omitted means the grasp planner's"
      " own default (0.05 m)."
    ),
  )
  parser.add_argument(
    "--transit_frame",
    type=str,
    default=None,
    help=(
      "Frame visited between objects, keeping transit paths predictable."
      " 'view' is a reasonable choice in the default scene. Off by default."
    ),
  )
  parser.add_argument(
    "--continue_on_failure",
    action="store_true",
    default=False,
    help=(
      "Skip an object that cannot be planned or reached and carry on, instead"
      " of aborting the tour."
    ),
  )
  parser.add_argument(
    "--plan_only",
    action="store_true",
    default=False,
    help="Plan a grasp for every object without moving the arm.",
  )
  parser.add_argument(
    "--dry_run",
    action="store_true",
    default=False,
    help="Print the resolved tour and exit without connecting or moving.",
  )
  parser.add_argument(
    "--parent_object",
    type=str,
    default="root",
    help="Object owning the output frames (default: 'root').",
  )
  parser.add_argument(
    "--grasp_frame",
    type=str,
    default="grasp",
    help="Frame updated to each planned grasp pose (default: 'grasp').",
  )
  parser.add_argument(
    "--pregrasp_frame",
    type=str,
    default="pre_grasp",
    help="Frame updated to each planned pre-grasp pose (default: 'pre_grasp').",
  )
  parser.add_argument(
    "--motion_type",
    type=str,
    default="ANY",
    choices=["ANY", "LINEAR", "JOINT"],
    help="Motion type for each approach (default: ANY).",
  )
  parser.add_argument(
    "--allow_tool_z_rotation",
    action="store_true",
    default=False,
    help="Allow free rotation about the tool Z approach axis.",
  )
  parser.add_argument(
    "--timeout_ms",
    type=float,
    default=15000.0,
    help="Grasp planning timeout in milliseconds (default: 15000).",
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
      f" '{DEFAULT_TOOL_FRAME}'). 'hande_tcp', 'hande_tool_frame' and"
      " 'tool_frame' are coincident alias links."
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
    "--disable_collision_checking",
    action="store_true",
    default=False,
    help=(
      "Plan motions without collision checking. Off by default because the"
      " tour drives into the enclosure."
    ),
  )
  parser.add_argument(
    "--settle_sec",
    type=float,
    default=0.3,
    help=(
      "Delay before planning, letting the MoveIt planning scene catch up with"
      " recent world changes (default: 0.3)."
    ),
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
  args = parse_args(argv)

  if args.retract_dist_m is not None and args.retract_dist_m <= 0.0:
    print(
      f"[!] --retract_dist_m must be positive, got {args.retract_dist_m}.",
      file=sys.stderr,
    )
    return 2
  if args.num_rotations <= 0:
    print(
      f"[!] --num_rotations must be positive, got {args.num_rotations}.",
      file=sys.stderr,
    )
    return 2

  try:
    default_surfaces = _parse_surfaces(args.surfaces, "--surfaces")
    specs = resolve_object_specs(
      args.objects,
      default_surfaces=default_surfaces,
      default_rotations=args.num_rotations,
      default_retract=args.retract_dist_m,
    )
  except argparse.ArgumentTypeError as exc:
    print(f"[!] {exc}", file=sys.stderr)
    return 2

  seen: set[str] = set()
  for spec in specs:
    if spec.name in seen:
      print(
        f"[!] Object '{spec.name}' appears twice in the tour. Each object may"
        " only be visited once; the output frames are overwritten per plan.",
        file=sys.stderr,
      )
      return 2
    seen.add(spec.name)

  if args.dry_run:
    print(f"Resolved tour over {len(specs)} object(s):")
    print(describe_specs(specs))
    print("\n[dry run] Not connecting; nothing was planned or moved.")
    return 0

  print(f"Connecting to solution at {args.address}...")
  try:
    solution = deployments.connect(address=args.address)
  except Exception as exc:  # pylint: disable=broad-except
    print(f"[!] Could not connect to {args.address}: {exc}", file=sys.stderr)
    return 1

  try:
    ok = run_tour(
      solution=solution,
      specs=specs,
      parent_object=args.parent_object,
      grasp_frame=args.grasp_frame,
      pregrasp_frame=args.pregrasp_frame,
      transit_frame=args.transit_frame,
      continue_on_failure=args.continue_on_failure,
      motion_type=args.motion_type,
      allow_tool_z_rotation=args.allow_tool_z_rotation,
      timeout_ms=args.timeout_ms,
      group_name=args.group_name,
      end_effector_group=args.end_effector_group,
      tool_frame_name=args.tool_frame_name,
      arm_part_name=args.arm_part_name,
      tool_object_name=args.tool_object_name,
      disable_collision_checking=args.disable_collision_checking,
      plan_only=args.plan_only,
      settle_sec=args.settle_sec,
    )
  except ValueError as exc:
    print(f"\n[!] {exc}", file=sys.stderr)
    return 2
  except KeyboardInterrupt:
    print("\nInterrupted.", file=sys.stderr)
    return 130

  return 0 if ok else 1


if __name__ == "__main__":
  sys.exit(main())
