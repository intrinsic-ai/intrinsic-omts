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

"""Interactive tool to align gripper body and tool_frame with root.view."""

import argparse
import math
from collections.abc import Sequence

from intrinsic.math.python import data_types
from intrinsic.solutions import deployments


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
  """Parses command line arguments."""
  parser = argparse.ArgumentParser(
    description=(
      "Align gripper body and/or tool_frame orientation with camera long"
      " axis or root.view."
    )
  )
  parser.add_argument(
    "--address",
    type=str,
    default="localhost:17080",
    help="Solution address to connect to (default: localhost:17080).",
  )
  parser.add_argument(
    "--reference",
    type=str,
    choices=["camera", "view"],
    default="camera",
    help=(
      "Reference to align with: 'camera' (camera long axis) or 'view'"
      " (root.view frame)."
    ),
  )
  parser.add_argument(
    "--camera_name",
    type=str,
    default="orbbec_camera",
    help="Name of camera in ObjectWorld (default: orbbec_camera).",
  )
  parser.add_argument(
    "--target",
    type=str,
    choices=["tool_frame", "gripper", "both"],
    default="both",
    help=(
      "Which node to align: 'gripper' (flange->gripper), 'tool_frame'"
      " (gripper->tool_frame), or 'both'."
    ),
  )
  parser.add_argument(
    "--angle_deg",
    type=float,
    default=None,
    help="Explicit yaw angle in degrees to apply around Z.",
  )
  return parser.parse_args(argv)


def _compute_angle_diff_deg(
  rot_a: data_types.Rotation3, rot_b: data_types.Rotation3
) -> float:
  """Calculates angular difference in degrees between two 3D rotations."""
  diff = rot_a.inverse() * rot_b
  w_val = diff.quaternion.xyzw[3]
  w_clamped = max(-1.0, min(1.0, w_val))
  return math.degrees(2.0 * math.acos(abs(w_clamped)))


def _compute_camera_x_yaw_deg(
  t_cam_in_flange: data_types.Pose3,
) -> float:
  """Computes the yaw angle in degrees of camera X axis in flange frame."""
  q = t_cam_in_flange.rotation.quaternion
  x, y, z, w = q.x, q.y, q.z, q.w
  cam_x_vx = 1.0 - 2.0 * (y * y + z * z)
  cam_x_vy = 2.0 * (x * y + z * w)
  return math.degrees(math.atan2(cam_x_vy, cam_x_vx))


def main(argv: Sequence[str] | None = None) -> None:
  args = parse_args(argv)
  print(f"Connecting to solution at {args.address}...")
  solution = deployments.connect(address=args.address)
  world = solution.world

  root = world.root
  flange = world.ur_module.flange
  gripper = world.gripper
  tool_frame = world.gripper.tool_frame
  view = getattr(world.root, "view", None)

  t_gripper_in_flange = world.get_transform(flange, gripper)
  t_tool_in_gripper = world.get_transform(gripper, tool_frame)

  # Ensure gripper is parented to flange
  try:
    world.reparent_object(child_object=gripper, new_parent=flange)
  except Exception as e:  # pylint: disable=broad-exception-caught
    print(f"Notice: reparent_object returned: {e}")

  print("\n--- Current State ---")
  t_flange_in_root = world.get_transform(root, flange)
  t_tool_in_root = world.get_transform(root, tool_frame)
  print(f"ur_module.flange in root:      {t_flange_in_root}")
  print(f"gripper in ur_module.flange:   {t_gripper_in_flange}")
  print(f"gripper.tool_frame in gripper: {t_tool_in_gripper}")
  print(f"gripper.tool_frame in root:    {t_tool_in_root}")
  if view is not None:
    t_view_in_root = world.get_transform(root, view)
    print(f"root.view in root:             {t_view_in_root}")

  camera_obj = getattr(world, args.camera_name, None)
  if camera_obj is not None:
    t_cam_in_flange = world.get_transform(flange, camera_obj)
    cam_yaw_deg = _compute_camera_x_yaw_deg(t_cam_in_flange)
    print(
      f"{args.camera_name} in flange:     {t_cam_in_flange}\n"
      f"  => Camera long axis yaw in flange: {cam_yaw_deg:+.2f}°"
    )
  else:
    cam_yaw_deg = 0.0

  gripper_trans = list(t_gripper_in_flange.translation)
  tool_trans = list(t_tool_in_gripper.translation)

  test_angles = (
    [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
    if args.angle_deg is None
    else [args.angle_deg]
  )

  # 1. Align flange -> gripper if requested
  if args.target in ["gripper", "both"]:
    identity_tool_pose = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([0.0, 0.0, 0.0, 1.0])),
      tool_trans,
    )
    world.update_transform(
      node_a=gripper,
      node_b=tool_frame,
      a_t_b=identity_tool_pose,
      node_to_update=tool_frame,
    )

    print(
      "\n--- Evaluating candidate Z rotations for ur_module.flange ->"
      f" gripper (ref: {args.reference}) ---"
    )
    best_gripper_angle = None
    min_gripper_diff = float("inf")

    for deg in test_angles:
      rad = math.radians(deg)
      qz = math.sin(rad / 2.0)
      qw = math.cos(rad / 2.0)
      quat = data_types.Quaternion([0.0, 0.0, qz, qw])
      pose = data_types.Pose3(data_types.Rotation3(quat), gripper_trans)
      world.update_transform(
        node_a=flange, node_b=gripper, a_t_b=pose, node_to_update=gripper
      )

      if args.reference == "camera":
        # Line-of-opening alignment (symmetric mod 180 deg)
        raw_diff = (deg - cam_yaw_deg) % 180.0
        diff = min(raw_diff, 180.0 - raw_diff)
        ref_label = f"Offset to cam long axis: {diff:6.2f}°"
      else:
        curr_tool_in_root = world.get_transform(root, tool_frame)
        diff = (
          _compute_angle_diff_deg(
            curr_tool_in_root.rotation, t_view_in_root.rotation
          )
          if view is not None
          else 0.0
        )
        ref_label = f"Offset to view: {diff:6.2f}°"

      print(
        f"Angle: {deg:6.1f}° | Quat (xyzw): (0, 0, {qz:+.6f}, {qw:+.6f}) |"
        f" {ref_label}"
      )
      if diff < min_gripper_diff:
        min_gripper_diff = diff
        best_gripper_angle = deg

    print(
      f"\n=> Best gripper angle: {best_gripper_angle}°"
      f" (residual diff: {min_gripper_diff:.2f}°)"
    )
    rad = math.radians(best_gripper_angle)
    qz_g = math.sin(rad / 2.0)
    qw_g = math.cos(rad / 2.0)
    best_gripper_pose = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([0.0, 0.0, qz_g, qw_g])),
      gripper_trans,
    )
    world.update_transform(
      node_a=flange,
      node_b=gripper,
      a_t_b=best_gripper_pose,
      node_to_update=gripper,
    )

  # 2. Align gripper -> tool_frame only if explicitly targeting tool_frame
  if args.target == "tool_frame":
    print(
      "\n--- Evaluating candidate Z rotations for gripper -> tool_frame ---"
    )
    best_tool_angle = None
    min_tool_diff = float("inf")

    for deg in test_angles:
      rad = math.radians(deg)
      qz = math.sin(rad / 2.0)
      qw = math.cos(rad / 2.0)
      quat = data_types.Quaternion([0.0, 0.0, qz, qw])
      pose = data_types.Pose3(data_types.Rotation3(quat), tool_trans)
      world.update_transform(
        node_a=gripper,
        node_b=tool_frame,
        a_t_b=pose,
        node_to_update=tool_frame,
      )

      if args.reference == "camera":
        raw_diff = (deg - cam_yaw_deg) % 180.0
        diff = min(raw_diff, 180.0 - raw_diff)
        ref_label = f"Offset to cam long axis: {diff:6.2f}°"
      else:
        curr_tool_in_root = world.get_transform(root, tool_frame)
        diff = (
          _compute_angle_diff_deg(
            curr_tool_in_root.rotation, t_view_in_root.rotation
          )
          if view is not None
          else 0.0
        )
        ref_label = f"Offset to view: {diff:6.2f}°"

      print(
        f"Angle: {deg:6.1f}° | Quat (xyzw): (0, 0, {qz:+.6f}, {qw:+.6f}) |"
        f" {ref_label}"
      )
      if diff < min_tool_diff:
        min_tool_diff = diff
        best_tool_angle = deg

    print(
      f"\n=> Best tool_frame angle: {best_tool_angle}°"
      f" (residual diff: {min_tool_diff:.2f}°)"
    )
    rad = math.radians(best_tool_angle)
    qz_t = math.sin(rad / 2.0)
    qw_t = math.cos(rad / 2.0)
    best_tool_pose = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([0.0, 0.0, qz_t, qw_t])),
      tool_trans,
    )
    world.update_transform(
      node_a=gripper,
      node_b=tool_frame,
      a_t_b=best_tool_pose,
      node_to_update=tool_frame,
    )

  final_tool_in_root = world.get_transform(root, tool_frame)
  final_view_in_root = world.get_transform(root, view)
  final_gripper_in_flange = world.get_transform(flange, gripper)
  final_tool_in_gripper = world.get_transform(gripper, tool_frame)

  print("\n--- Final Applied State in Live Solution ---")
  print(f"gripper in ur_module.flange:   {final_gripper_in_flange}")
  print(f"gripper.tool_frame in gripper: {final_tool_in_gripper}")
  print(f"gripper.tool_frame in root:    {final_tool_in_root}")
  print(f"root.view in root:             {final_view_in_root}")

  q_g = final_gripper_in_flange.rotation.quaternion
  q_t = final_tool_in_gripper.rotation.quaternion
  t_g = final_gripper_in_flange.translation
  t_t = final_tool_in_gripper.translation

  print(
    "\n=== Config Proto Snippet (for"
    " configs/common/ur_module.attachments.updates.pbtxt) ==="
  )
  print("# 1. ur_module.flange -> gripper:")
  print(
    f"  position {{ x: {t_g[0]:.3f} y: {t_g[1]:.3f} z: {t_g[2]:.3f} }}\n"
    f"  orientation {{ x: {q_g.x:.8f} y: {q_g.y:.8f} z: {q_g.z:.8f} w:"
    f" {q_g.w:.8f} }}"
  )
  print("\n# 2. gripper -> gripper.tool_frame:")
  print(
    f"  position {{ x: {t_t[0]:.3f} y: {t_t[1]:.3f} z: {t_t[2]:.3f} }}\n"
    f"  orientation {{ x: {q_t.x:.8f} y: {q_t.y:.8f} z: {q_t.z:.8f} w:"
    f" {q_t.w:.8f} }}"
  )


if __name__ == "__main__":
  main()
