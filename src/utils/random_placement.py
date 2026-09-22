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

"""Dynamic frame shifting for return infeed positioning."""

from typing import Any


def randomize_placement_frame(
  context: Any,
  parent_object: str,
  frame_name: str,
  return_center_x: float,
  return_center_y: float,
  return_bounds_x: float,
  return_bounds_y: float,
  return_bounds_rz_degrees: float = 0.0,
  grasp_frame_name: str | None = None,
) -> None:
  """Shifts return placement frame(s) to a random pose within a bounding box.

  Args:
      context: SBL BT PythonScript execution context providing `object_world`.
      parent_object: Name of the parent object owning the frame.
      frame_name: Name of the primary return frame (e.g. `pre_grasp`) to shift.
      return_center_x: Fixed X position center.
      return_center_y: Fixed Y position center.
      return_bounds_x: Random shift bounding box width in X.
      return_bounds_y: Random shift bounding box width in Y.
      return_bounds_rz_degrees: Random shift angular bounds around Z in degrees.
      grasp_frame_name: Optional secondary contact frame (e.g. `grasp`) shifted
        to the same XY and Z-rotation while preserving its own Z height.
  """
  import math
  import random

  from intrinsic.math.python import data_types

  world = context.object_world
  parent_obj = getattr(world, parent_object, getattr(world, "root", None))
  frame_node = getattr(parent_obj, frame_name)

  # Get current transform to preserve Z and orientation
  parent_t_frame = world.get_transform(parent_obj, frame_node)
  pos = parent_t_frame.translation
  rot = parent_t_frame.rotation

  # Calculate random offsets within bounds
  half_bound_x = return_bounds_x / 2.0
  half_bound_y = return_bounds_y / 2.0

  shift_x = random.uniform(-half_bound_x, half_bound_x)
  shift_y = random.uniform(-half_bound_y, half_bound_y)

  # Apply random shift to the fixed center position
  new_x = return_center_x + shift_x
  new_y = return_center_y + shift_y
  new_z = float(pos[2])

  # Calculate random rotation around Z-axis
  half_bound_rz = return_bounds_rz_degrees / 2.0
  shift_rz_degrees = random.uniform(-half_bound_rz, half_bound_rz)
  shift_rz_radians = math.radians(shift_rz_degrees)

  sz = math.sin(shift_rz_radians / 2.0)
  cz = math.cos(shift_rz_radians / 2.0)

  q = rot.quaternion
  # q_z = (0, 0, sz, cz) -> w=cz, x=0, y=0, z=sz
  # q_new = q_z * q
  w_new = cz * float(q.w) - sz * float(q.z)
  x_new = cz * float(q.x) - sz * float(q.y)
  y_new = cz * float(q.y) + sz * float(q.x)
  z_new = cz * float(q.z) + sz * float(q.w)

  new_rot = data_types.Rotation3(
    data_types.Quaternion([x_new, y_new, z_new, w_new])
  )

  new_pose = data_types.Pose3(new_rot, [new_x, new_y, new_z])
  world.update_transform(node_a=parent_obj, node_b=frame_node, a_t_b=new_pose)

  if grasp_frame_name and grasp_frame_name != frame_name:
    grasp_node = getattr(parent_obj, grasp_frame_name, None)
    if grasp_node is not None:
      parent_t_grasp = world.get_transform(parent_obj, grasp_node)
      grasp_z = float(parent_t_grasp.translation[2])
      grasp_pose = data_types.Pose3(new_rot, [new_x, new_y, grasp_z])
      world.update_transform(
        node_a=parent_obj, node_b=grasp_node, a_t_b=grasp_pose
      )
