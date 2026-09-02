"""Interactive and CLI tool to move the robot to a target joint configuration or positions."""

import argparse
from typing import Any, Sequence

from intrinsic.solutions import deployments
from intrinsic.solutions import execution
from src.hardware.robot import UrRobot


def list_available_joint_configs(
    world: Any,
    arm_part_name: str = "ur_module",
) -> list[tuple[str, list[float]]]:
  """Discovers and returns all available named joint configurations for the robot.

  Args:
    world: The connected SBL ObjectWorld instance.
    arm_part_name: Name of the robot kinematic object in the world.

  Returns:
    A list of (config_name, joint_positions) tuples.
  """
  configs: list[tuple[str, list[float]]] = []
  try:
    robot = getattr(world, arm_part_name)
  except AttributeError:
    try:
      robot = world.get_kinematic_object(arm_part_name)
    except Exception:
      return configs

  if hasattr(robot, "joint_configurations"):
    jc = robot.joint_configurations
    if hasattr(jc, "keys"):
      for name in jc.keys():
        cfg = jc[name]
        pos = (
            list(cfg.joint_position)
            if hasattr(cfg, "joint_position")
            else list(cfg)
        )
        configs.append((name, pos))

  return configs


def parse_joint_values(joint_str: str) -> list[float] | None:
  """Parses a string of comma- or space-separated float joint angles."""
  cleaned = (
      joint_str.replace(",", " ")
      .replace("[", "")
      .replace("]", "")
      .strip()
  )
  if not cleaned:
    return None
  try:
    values = [float(x) for x in cleaned.split()]
    return values if values else None
  except ValueError:
    return None


def prompt_for_joint_target(
    available_configs: list[tuple[str, list[float]]],
) -> str | list[float] | None:
  """Prompts the user to select a named joint config or enter joint positions.

  Args:
    available_configs: List of (name, joint_positions) tuples.

  Returns:
    Config name (str), joint positions (list[float]), or None if user quit.
  """
  print("\nAvailable Joint Configurations:")
  if available_configs:
    for i, (name, positions) in enumerate(available_configs, start=1):
      formatted_pos = ", ".join(f"{p:.4f}" for p in positions)
      print(f"  [{i}] {name} -> [{formatted_pos}]")
  else:
    print("  (No named joint configurations stored in active world model)")

  while True:
    try:
      choice = input(
          f"\nEnter config index (1-{len(available_configs)}), config name,"
          " joint angles (e.g. 0.0 -1.57 1.57 -1.57 -1.57 0.0), or 'q' to"
          " quit: "
      ).strip()
      if choice.lower() in ("q", "quit", "exit"):
        return None

      if choice.isdigit() and available_configs:
        idx = int(choice)
        if 1 <= idx <= len(available_configs):
          return available_configs[idx - 1][0]
        print(
            "Invalid index. Please enter a number between 1 and"
            f" {len(available_configs)}."
        )
        continue

      # Check if choice matches a configuration name directly
      for name, _ in available_configs:
        if choice == name:
          return name

      # Check if choice is a list of numbers
      parsed_joints = parse_joint_values(choice)
      if parsed_joints is not None and len(parsed_joints) >= 1:
        return parsed_joints

      print(
          "Invalid input. Please enter a valid index, name, numbers, or 'q'."
      )
    except (ValueError, EOFError):
      print("Invalid input.")


def move_robot_to_joint(
    solution: Any,
    joint_target: str | Sequence[float],
    arm_part_name: str = "ur_module",
    disable_collision_checking: bool = False,
    settling_timeout_seconds: float = 10.0,
) -> None:
  """Plans and executes a joint motion moving the robot to the target pose.

  Args:
    solution: Connected SBL deployment instance.
    joint_target: Named joint configuration name or sequence of joint angles.
    arm_part_name: Robot arm part name in solution.world (default:
      'ur_module').
    disable_collision_checking: If True, disables collision checking.
    settling_timeout_seconds: Settling timeout in seconds for trajectory motion.
  """
  target_desc = (
      joint_target
      if isinstance(joint_target, str)
      else [round(x, 4) for x in joint_target]
  )
  print(
      f"\nPlanning joint motion for '{arm_part_name}' -> {target_desc} "
      f"(collision checking:"
      f" {'disabled' if disable_collision_checking else 'enabled'})..."
  )

  robot = UrRobot(
      solution=solution,
      arm_part_name=arm_part_name,
      disable_collision_checking=disable_collision_checking,
      default_settling_timeout_seconds=settling_timeout_seconds,
  )

  task = robot.build_move_joint_task(
      joint_target=joint_target,
      settling_timeout_seconds=settling_timeout_seconds,
      name=f"Move {arm_part_name} to {target_desc}",
  )

  try:
    solution.executive.run(task)
    print(f"[✓] Successfully moved robot to '{target_desc}'.")
  except execution.ExecutionFailedError as e:
    print(f"\n[!] Motion execution failed: {e}")
    if hasattr(solution.executive, "get_errors"):
      print(f"Executive errors: {solution.executive.get_errors()}")
  except Exception as e:
    print(f"\n[!] Error during motion execution: {e}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
      description=(
          "Developer CLI tool to move robot to a joint configuration or"
          " explicit joint positions."
      )
  )
  parser.add_argument(
      "target",
      nargs="?",
      default=None,
      type=str,
      help=(
          "Named joint configuration (e.g. 'home', 'view_joints') or joint"
          " angles."
      ),
  )
  parser.add_argument(
      "--joints",
      "-j",
      nargs="+",
      type=float,
      default=None,
      help=(
          "Explicit joint positions in radians (e.g. --joints 0.0 -1.57 1.57"
          " -1.57 -1.57 0.0)."
      ),
  )
  parser.add_argument(
      "--name",
      "-n",
      type=str,
      default=None,
      help="Name of stored joint configuration to move to.",
  )
  parser.add_argument(
      "--address",
      type=str,
      default="localhost:17080",
      help="Solution gRPC address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
      "--robot_name",
      "--arm_part_name",
      dest="robot_name",
      type=str,
      default="ur_module",
      help="Robot object name in the world (default: 'ur_module').",
  )
  parser.add_argument(
      "--disable_collision_checking",
      action="store_true",
      help="Disable collision checking for the motion segment.",
  )
  parser.add_argument(
      "--settling_timeout_seconds",
      type=float,
      default=10.0,
      help="Settling timeout in seconds for trajectory motion (default: 10.0).",
  )
  return parser.parse_args(argv)


def resolve_joint_target(
    args: argparse.Namespace,
) -> str | list[float] | None:
  """Resolves the target joint positions or configuration name from parsed arguments."""
  if args.joints:
    return list(args.joints)
  if args.name:
    return args.name
  if args.target:
    parsed = parse_joint_values(args.target)
    if parsed is not None and len(parsed) > 1:
      return parsed
    return args.target
  return None


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)

  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  target = resolve_joint_target(args)
  if target is not None:
    move_robot_to_joint(
        solution=solution,
        joint_target=target,
        arm_part_name=args.robot_name,
        disable_collision_checking=args.disable_collision_checking,
        settling_timeout_seconds=args.settling_timeout_seconds,
    )
    return

  available_configs = list_available_joint_configs(
      world=world, arm_part_name=args.robot_name
  )
  while True:
    selected = prompt_for_joint_target(available_configs)
    if selected is None:
      print("Exiting move_to_joint tool.")
      break

    move_robot_to_joint(
        solution=solution,
        joint_target=selected,
        arm_part_name=args.robot_name,
        disable_collision_checking=args.disable_collision_checking,
        settling_timeout_seconds=args.settling_timeout_seconds,
    )


if __name__ == "__main__":
  main()
