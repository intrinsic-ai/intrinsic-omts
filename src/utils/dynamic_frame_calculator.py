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

"""Dynamic grasp and pre-grasp frame calculation for vision perception pipeline."""

import math
from typing import Any
from intrinsic.math.python import data_types


def calculate_and_update_dynamic_frames(context: Any, params: Any) -> None:
  """Calculates workpiece grasp frames and injects them into the SBL ObjectWorld.

  Aligns gripper Z rotation with the short side of the workpiece based on
  detected 6D pose and the current robot tool frame, and updates pre_grasp
  and grasp frames in the active belief world.

  Args:
      context: SBL BT PythonScript execution context providing `object_world`.
      params: Dynamic parameters protobuf containing detected pose and frame
        names.
  """
  world = context.object_world
  parent_obj = getattr(
      world, params.parent_object, getattr(world, "root", None)
  )

  # Resolve camera sensor transform in parent object (root)
  camera_obj = getattr(world, params.camera_name, None)
  if camera_obj is None:
    for obj_name in ["ur_module", "robot", "root"]:
      p = getattr(world, obj_name, None)
      if p is not None and hasattr(p, params.camera_name):
        camera_obj = getattr(p, params.camera_name)
        break

  camera_sensor_node = (
      getattr(camera_obj, "sensor", camera_obj) if camera_obj else None
  )
  root_t_camera = (
      world.get_transform(parent_obj, camera_sensor_node)
      if camera_sensor_node
      else None
  )

  # Construct detected pose in camera frame
  cam_q = data_types.Quaternion(
      [params.ori_x, params.ori_y, params.ori_z, params.ori_w]
  )
  cam_pose = data_types.Pose3(
      data_types.Rotation3(cam_q), [params.pos_x, params.pos_y, params.pos_z]
  )

  if root_t_camera is not None:
    root_t_target = root_t_camera * cam_pose
  else:
    root_t_target = cam_pose

  target_pos = root_t_target.translation
  target_rot = root_t_target.rotation

  # Determine longest axis alignment in root XY plane:
  # Local X is the longest axis (5"), local Z is the medium axis (3"), local Y is the thickness (2").
  ax = target_rot.rotate_point([1.0, 0.0, 0.0])
  az = target_rot.rotate_point([0.0, 0.0, 1.0])

  if abs(float(ax[2])) < 0.7:
    vx, vy = float(ax[0]), float(ax[1])
  else:
    vx, vy = float(az[0]), float(az[1])

  theta_longest = math.atan2(vy, vx)
  # Grasp on the short side of the workpiece (rotated 90° around Z relative to long-side grasp):
  psi = theta_longest

  half_psi = psi / 2.0
  c1 = (math.cos(half_psi), math.sin(half_psi), 0.0, 0.0)
  c2 = (-math.sin(half_psi), math.cos(half_psi), 0.0, 0.0)
  candidates = [
      c1,
      (-c1[0], -c1[1], 0.0, 0.0),
      c2,
      (-c2[0], -c2[1], 0.0, 0.0),
  ]

  tool_obj = getattr(world, "gripper", None)
  tool_node = getattr(tool_obj, "tool_frame", None) if tool_obj else None
  cur_tool_tf = world.get_transform(parent_obj, tool_node) if tool_node else None

  if cur_tool_tf is not None:
    cur_q = cur_tool_tf.rotation.quaternion
    grasp_ori = max(
        candidates,
        key=lambda c: (
            c[0] * float(cur_q.x)
            + c[1] * float(cur_q.y)
            + c[2] * float(cur_q.z)
            + c[3] * float(cur_q.w)
        ),
    )
  else:
    grasp_ori = c1

  grasp_pos = (float(target_pos[0]), float(target_pos[1]), float(target_pos[2]))
  pregrasp_pos = (
      float(target_pos[0]),
      float(target_pos[1]),
      float(target_pos[2]) + params.approach_offset_z,
  )

  frame_poses = {
      params.pregrasp_frame_name: (pregrasp_pos, grasp_ori),
      params.grasp_frame_name: (grasp_pos, grasp_ori),
  }

  existing_frames = set()
  if hasattr(parent_obj, "list_frames"):
    existing_frames = set(parent_obj.list_frames())
  elif hasattr(parent_obj, "__dict__"):
    existing_frames = set(parent_obj.__dict__.keys())

  for fname, (pos, ori) in frame_poses.items():
    pose = data_types.Pose3(
        data_types.Rotation3(
            data_types.Quaternion([ori[0], ori[1], ori[2], ori[3]])
        ),
        [pos[0], pos[1], pos[2]],
    )
    if fname in existing_frames or hasattr(parent_obj, fname):
      frame_node = getattr(parent_obj, fname)
      world.update_transform(node_a=parent_obj, node_b=frame_node, a_t_b=pose)
    else:
      world.create_frame(
          frame_name=fname, parent=parent_obj, parent_t_frame=pose
      )
