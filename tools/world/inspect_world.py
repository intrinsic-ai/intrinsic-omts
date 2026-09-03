"""Utility to inspect scene objects, frames, and joint configs."""

import argparse
from typing import Any, Sequence
from intrinsic.solutions import deployments


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
      description=(
          "Inspect objects, frames, and joint configurations in the solution"
          " world."
      )
  )
  parser.add_argument(
      "--address",
      type=str,
      default="localhost:17080",
      help="Solution address to connect to (default: localhost:17080).",
  )
  return parser.parse_args(argv)


def print_transform(label: str, tf: Any) -> None:
  """Helper to print a Pose3 transform in a clean, readable format."""
  pos = tf.translation
  quat = tf.rotation.quaternion
  rpy_deg = tf.rotation.euler_angles(radians=False)
  print(f"{label}:")
  print(f"  pos:     [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
  print(f"  quat:    {quat}")
  print(f"  rpy_deg: [{rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}]")


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  print("\n=== Objects in Active Belief World ===")
  objects = []
  try:
    if hasattr(world, "list_objects"):
      objects = world.list_objects()
      print(f"Total objects: {len(objects)}")
      for obj in objects:
        print(f"\nObject: {obj.name} (id={getattr(obj, 'id', None)})")
        if hasattr(obj, "joint_positions") and obj.joint_positions:
          print(f"  Joint positions: {obj.joint_positions}")
        if hasattr(obj, "joint_entity_names") and obj.joint_entity_names:
          print(f"  Joint entity names: {obj.joint_entity_names}")
        if (
            hasattr(obj, "joint_application_limits")
            and obj.joint_application_limits
        ):
          print(f"  Joint application limits:\n{obj.joint_application_limits}")
        if hasattr(obj, "joint_system_limits") and obj.joint_system_limits:
          print(f"  Joint system limits:\n{obj.joint_system_limits}")
        frames = []
        if hasattr(obj, "list_frames"):
          frames = obj.list_frames()
        elif hasattr(obj, "frame_names"):
          frames = obj.frame_names
        elif hasattr(obj, "frames"):
          frames = [
              f.name if hasattr(f, "name") else str(f) for f in obj.frames
          ]
        print(f"  Frames ({len(frames)}): {frames}")
    elif hasattr(world, "list_object_names"):
      names = world.list_object_names()
      print(f"Object names: {names}")
  except Exception as e:
    print(f"Error listing objects: {e}")

  print("\n=== All Frames in World ===")
  total_frames = 0
  try:
    for obj in objects:
      frame_list = []
      if hasattr(obj, "frames") and callable(obj.frames):
        frame_list = obj.frames()
      elif hasattr(obj, "frames") and isinstance(obj.frames, list):
        frame_list = obj.frames
      elif hasattr(obj, "list_frames"):
        frame_list = [getattr(obj, f_name) for f_name in obj.list_frames()]
      elif hasattr(obj, "frame_names"):
        frame_list = [getattr(obj, f_name) for f_name in obj.frame_names]

      for frame in frame_list:
        frame_name = getattr(frame, "name", str(frame))
        total_frames += 1
        try:
          tf_in_root = world.get_transform(world.root, frame)
          print_transform(
              f"Frame '{obj.name}.{frame_name}' in root", tf_in_root
          )
          if obj.name != "root":
            try:
              tf_in_obj = world.get_transform(obj, frame)
              pos = tf_in_obj.translation
              rpy_deg = tf_in_obj.rotation.euler_angles(radians=False)
              print(
                  f"  (rel to {obj.name}: pos=[{pos[0]:.4f}, {pos[1]:.4f},"
                  f" {pos[2]:.4f}], rpy_deg=[{rpy_deg[0]:.2f},"
                  f" {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}])"
              )
            except Exception:
              pass
        except Exception as e:
          print(
              f"Frame '{obj.name}.{frame_name}': error retrieving transform:"
              f" {e}"
          )
    print(f"\nTotal frames found: {total_frames}")
  except Exception as e:
    print(f"Error enumerating frames: {e}")

  print("\n=== Object Transforms in Root ===")
  try:
    for obj in objects:
      if obj.name == "root":
        continue
      try:
        tf = world.get_transform(world.root, obj)
        print_transform(f"Object '{obj.name}' in root", tf)
      except Exception as e:
        print(f"Object '{obj.name}': error retrieving transform in root: {e}")
  except Exception as e:
    print(f"Error inspecting object transforms: {e}")

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
      if hasattr(solution.skills, "ai") and hasattr(
          solution.skills.ai, "intrinsic"
      ):
        for skill_name in dir(solution.skills.ai.intrinsic):
          if not skill_name.startswith("_"):
            print(f" - ai.intrinsic.{skill_name}")
  except Exception as e:
    print(f"   Error listing skills: {e}")


if __name__ == "__main__":
  main()
