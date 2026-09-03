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
  world = solution.world

  print("\n=== Objects in Active Belief World ===")
  if hasattr(world, "cnc_enclosure"):
    cnc = world.cnc_enclosure
    print(f"\n--- cnc_enclosure inspect ---")
    print(f"  Type: {type(cnc)}")
    print(f"  Dir: {[m for m in dir(cnc) if not m.startswith('_')]}")
    if hasattr(cnc, "joint_names"):
      print(f"  Joint names: {cnc.joint_names}")
    if hasattr(cnc, "joint_configurations"):
      print(f"  Joint configs: {list(cnc.joint_configurations.keys())}")
      for k in cnc.joint_configurations.keys():
        print(f"    {k}: {cnc.joint_configurations[k].joint_position}")
    if hasattr(cnc, "list_frames"):
      print(f"  Frames: {cnc.list_frames()}")
    if hasattr(cnc, "frames"):
      print(f"  Frames attr: {cnc.frames}")

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

  print("\n=== Solution Pose Estimators ===")
  try:
    if hasattr(solution, "pose_estimators"):
      for est_name in dir(solution.pose_estimators):
        if not est_name.startswith("_"):
          print(f" - {est_name}")
  except Exception as e:
    print(f"   Error listing pose estimators: {e}")

  print("\n=== Solution Skills ===")
  try:
    if hasattr(solution, "skills"):
      if hasattr(solution.skills, "ai") and hasattr(solution.skills.ai, "intrinsic"):
        for skill_name in dir(solution.skills.ai.intrinsic):
          if not skill_name.startswith("_"):
            print(f" - ai.intrinsic.{skill_name}")
  except Exception as e:
    print(f"   Error listing skills: {e}")

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
    for frame_name in ["dynamic_pregrasp", "dynamic_grasp", "dynamic_preplace", "dynamic_place", "pre_grasp", "grasp", "view", "side_view", "place_vise", "pre_place_vise"]:
      if hasattr(world.root, frame_name):
        tf = world.get_transform(world.root, getattr(world.root, frame_name))
        rpy = tf.rotation.euler_angles(radians=False)
        print(f"Frame root.{frame_name}: pos={tf.translation}, quat={tf.rotation.quaternion}, rpy_deg={rpy}")
    if hasattr(world, "raw_stock_2x3x5"):
      tf = world.get_transform(world.root, world.raw_stock_2x3x5)
      rpy = tf.rotation.euler_angles(radians=False)
      print(f"Object raw_stock_2x3x5 in root: pos={tf.translation}, quat={tf.rotation.quaternion}, rpy_deg={rpy}")
    if hasattr(world, "building_block"):
      tf = world.get_transform(world.root, world.building_block)
      rpy = tf.rotation.euler_angles(radians=False)
      print(f"Object building_block in root: pos={tf.translation}, quat={tf.rotation.quaternion}, rpy_deg={rpy}")
  except Exception as e:
    print(f"   Error checking transforms: {e}")


if __name__ == "__main__":
  main()
