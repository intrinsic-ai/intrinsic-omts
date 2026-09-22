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
) -> None:
  """Shifts the return frame to a random position within a bounding box centered at a fixed position.

  Args:
      context: SBL BT PythonScript execution context providing `object_world`.
      parent_object: Name of the parent object owning the frame.
      frame_name: Name of the return frame to shift.
      return_center_x: Fixed X position center.
      return_center_y: Fixed Y position center.
      return_bounds_x: Random shift bounding box width in X.
      return_bounds_y: Random shift bounding box width in Y.
  """
  import random
  from intrinsic.math.python import data_types

  world = context.object_world
  parent_obj = getattr(world, parent_object, getattr(world, 'root', None))
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
  
  new_pose = data_types.Pose3(rot, [new_x, new_y, new_z])
  world.update_transform(node_a=parent_obj, node_b=frame_node, a_t_b=new_pose)
