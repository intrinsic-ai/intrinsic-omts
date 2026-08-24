"""Utility to inspect and list scene objects, frames, and joint configs in SBL world."""

import argparse
from typing import Sequence
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

  print("\n=== Objects in Active Belief World ===")
  objects = solution.world.list_objects()
  for obj_name in objects:
    print(f" - {obj_name}")
    try:
      obj = getattr(solution.world, obj_name)
      if hasattr(obj, "joint_configurations") and list(obj.joint_configurations.keys()):
        print("   Joint Configurations:")
        for cfg_name in obj.joint_configurations.keys():
          cfg = obj.joint_configurations[cfg_name]
          print(f"     * {cfg_name}: {list(cfg.joint_position)}")
    except Exception:
      pass

  print("\n=== Solution Resources ===")
  solution.resources.update()
  for res_name in dir(solution.resources):
    if not res_name.startswith("_"):
      res = getattr(solution.resources, res_name)
      if hasattr(res, "name"):
        print(f" - {res.name}")


if __name__ == "__main__":
  main()
