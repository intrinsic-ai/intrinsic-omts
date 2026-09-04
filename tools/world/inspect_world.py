"""Utility to inspect and list scene objects, frames, and joint configs in SBL world."""

import argparse
from collections.abc import Sequence

from intrinsic.solutions import deployments


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description="Inspect objects and joint configurations in the solution world."
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution address to connect to (default: localhost:17080).",
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  print("\n=== Objects in Active Belief World ===")
  objects = world.list_objects()
  for obj_name in objects:
    print(f" - {obj_name}")
    try:
      obj = getattr(world, obj_name)
      if hasattr(obj, "joint_configurations") and list(
        obj.joint_configurations.keys()
      ):
        print("   Joint Configurations:")
        for cfg_name in obj.joint_configurations.keys():
          cfg = obj.joint_configurations[cfg_name]
          print(f"     * {cfg_name}: {list(cfg.joint_position)}")
    except Exception:
      pass

  print("\n=== Solution Resources ===")
  try:
    solution.resources.update()
    for res_name in dir(solution.resources):
      if not res_name.startswith("_"):
        res = getattr(solution.resources, res_name)
        if hasattr(res, "name"):
          print(f" - {res.name}")
  except Exception as e:
    print(f"   Error listing resources: {e}")

  print("\n=== Transforms Inspection ===")
  try:
    if hasattr(world, "ur_module") and hasattr(world.ur_module, "flange"):
      print(
        f"Flange in root: {world.get_transform(world.root, world.ur_module.flange)}"
      )
    if hasattr(world, "gripper") and hasattr(world.gripper, "tool_frame"):
      print(
        "Tool Frame in root:"
        f" {world.get_transform(world.root, world.gripper.tool_frame)}"
      )
    if hasattr(world.root, "view"):
      print(
        f"View frame in root: {world.get_transform(world.root, world.root.view)}"
      )
    if hasattr(world.root, "pre_grasp"):
      print(
        f"Pre-grasp frame in root: {world.get_transform(world.root, world.root.pre_grasp)}"
      )
    if hasattr(world.root, "grasp"):
      print(
        f"Grasp frame in root: {world.get_transform(world.root, world.root.grasp)}"
      )
  except Exception as e:
    print(f"   Error checking transforms: {e}")


if __name__ == "__main__":
  main()
