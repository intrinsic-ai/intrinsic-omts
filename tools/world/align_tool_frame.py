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

"""Interactive tool to align gripper.tool_frame orientation with root.view orientation."""

import argparse
import math
from typing import Sequence
from intrinsic.math.python import data_types
from intrinsic.solutions import deployments


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
      description="Align gripper.tool_frame orientation with root.view."
  )
  parser.add_argument(
      "--address",
      type=str,
      default="localhost:17080",
      help="Solution address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
      "--angle_deg",
      type=float,
      default=None,
      help="Explicit yaw angle in degrees to apply around Z for tool_frame.",
  )
  return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  root = world.root
  gripper = world.gripper
  tool_frame = world.gripper.tool_frame
  view = world.root.view

  print("\n--- Current State ---")
  t_tool_in_root = world.get_transform(root, tool_frame)
  t_view_in_root = world.get_transform(root, view)
  t_tool_in_gripper = world.get_transform(gripper, tool_frame)

  print(f"gripper.tool_frame in root:    {t_tool_in_root}")
  print(f"root.view in root:             {t_view_in_root}")
  print(f"gripper.tool_frame in gripper: {t_tool_in_gripper}")

  test_angles = (
      [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
      if args.angle_deg is None
      else [args.angle_deg]
  )

  best_angle = None
  min_rot_diff = float("inf")

  print("\n--- Evaluating candidate Z rotations for gripper.tool_frame ---")
  for deg in test_angles:
    rad = math.radians(deg)
    qz = math.sin(rad / 2.0)
    qw = math.cos(rad / 2.0)
    quat = data_types.Quaternion([0.0, 0.0, qz, qw])
    new_rot = data_types.Rotation3(quat)
    new_pose = data_types.Pose3(new_rot, [0.0, 0.0, 0.128])

    world.update_transform(node_a=gripper, node_b=tool_frame, a_t_b=new_pose)

    curr_tool_in_root = world.get_transform(root, tool_frame)
    rel = world.get_transform(tool_frame, view)

    q_rel = rel.rotation.quaternion.xyzw
    w_val = q_rel[3]
    w_clamped = max(-1.0, min(1.0, w_val))
    angle_diff_deg = math.degrees(2.0 * math.acos(abs(w_clamped)))

    print(
        f"Angle: {deg:6.1f}° | Quat (xyzw): (0, 0, {qz:+.6f}, {qw:+.6f}) |"
        f" Offset to view: {angle_diff_deg:6.2f}° | Tool in Root:"
        f" {curr_tool_in_root.rotation}"
    )

    if angle_diff_deg < min_rot_diff:
      min_rot_diff = angle_diff_deg
      best_angle = deg

  print(f"\n=> Best angle: {best_angle}° (residual difference: {min_rot_diff:.2f}°)")

  # Apply best angle
  rad = math.radians(best_angle)
  qz = math.sin(rad / 2.0)
  qw = math.cos(rad / 2.0)
  best_quat = data_types.Quaternion([0.0, 0.0, qz, qw])
  best_pose = data_types.Pose3(
      data_types.Rotation3(best_quat), [0.0, 0.0, 0.128]
  )
  world.update_transform(node_a=gripper, node_b=tool_frame, a_t_b=best_pose)

  final_tool_in_root = world.get_transform(root, tool_frame)
  final_view_in_root = world.get_transform(root, view)
  print("\n--- Final Applied State in Live Solution ---")
  print(f"gripper.tool_frame in root: {final_tool_in_root}")
  print(f"root.view in root:         {final_view_in_root}")
  print(
      f"Proto for ur_module.attachments.updates.pbtxt:\n"
      f"  position {{ x: 0 y: 0 z: 0.128 }}\n"
      f"  orientation {{ x: 0 y: 0 z: {qz:.8f} w: {qw:.8f} }}\n"
  )


if __name__ == "__main__":
  main()
