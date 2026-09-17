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

"""Dynamic grasp and pre-grasp frame calculation for perception pipeline."""

import logging
from collections.abc import Sequence
from typing import Any

from intrinsic.math.python import data_types

from src.utils.math_utils import compute_top_down_grasp_quaternion


def _resolve_camera_transform(
  world: Any, parent_obj: Any, camera_name: str
) -> data_types.Pose3 | None:
  """Resolves the camera sensor transform relative to parent_obj."""
  cam_obj = getattr(world, camera_name, None)
  target = (
    getattr(cam_obj, "sensor", cam_obj) if cam_obj is not None else camera_name
  )
  tf = world.get_transform(parent_obj, target)
  return estimate_to_pose(tf) if tf is not None else None


def estimate_to_pose(est: Any) -> data_types.Pose3:
  """Converts a perception estimate or pose into data_types.Pose3."""
  if isinstance(est, data_types.Pose3):
    return est
  target = getattr(est, "root_t_target", None) or getattr(
    est, "pose_t_target", est
  )
  pos = target.position
  ori = getattr(target, "orientation", None) or target.rotation.quaternion
  return data_types.Pose3(
    data_types.Rotation3(
      data_types.Quaternion(
        [float(ori.x), float(ori.y), float(ori.z), float(ori.w)]
      )
    ),
    [float(pos.x), float(pos.y), float(pos.z)],
  )


def _extract_target_estimate(params: Any) -> data_types.Pose3 | None:
  """Extracts the highest-confidence FoundationPose estimate."""
  raw = getattr(params, "estimates", None)
  if raw is None:
    est_res = getattr(params, "estimate_result", None)
    raw = getattr(est_res, "estimates", None) if est_res is not None else None

  if not isinstance(raw, (list, tuple, Sequence)) or isinstance(
    raw, (str, bytes)
  ):
    return None

  valid = [e for e in raw if float(getattr(e, "score", 0.0)) < 0.0]
  if not valid:
    logging.warning("No confident negative FoundationPose estimates found.")
    return None

  valid.sort(key=lambda e: float(getattr(e, "score", 0.0)))
  return estimate_to_pose(valid[0])


def _sync_frame(
  world: Any, parent_obj: Any, frame_name: str, pose: data_types.Pose3
) -> None:
  """Creates or updates a dynamic frame on parent_obj in ObjectWorld."""
  existing = (
    set(str(f) for f in parent_obj.list_frames())
    if hasattr(parent_obj, "list_frames")
    else set()
  )
  if frame_name in existing or hasattr(parent_obj, frame_name):
    frame_node = getattr(parent_obj, frame_name)
    world.update_transform(node_a=parent_obj, node_b=frame_node, a_t_b=pose)
  else:
    world.create_frame(
      frame_name=frame_name, parent=parent_obj, parent_t_frame=pose
    )


def _resolve_current_tool_quaternion(
  world: Any, parent_obj: Any
) -> tuple[float, float, float, float] | None:
  """Resolves the active tool frame quaternion relative to parent_obj."""
  tool_obj = getattr(world, "gripper", None)
  tool_node = getattr(tool_obj, "tool_frame", None) if tool_obj else None
  if tool_node is not None:
    tf = world.get_transform(parent_obj, tool_node)
    if tf is not None:
      q = tf.rotation.quaternion
      return (float(q.x), float(q.y), float(q.z), float(q.w))
  return None


def calculate_and_update_dynamic_frames(context: Any, params: Any) -> None:
  """Calculates workpiece grasp frames and updates SBL ObjectWorld."""
  world = context.object_world
  parent_name = getattr(params, "parent_object", "root") or "root"
  parent_obj = getattr(world, parent_name, getattr(world, "root", None))

  cam_pose = _extract_target_estimate(params)
  if cam_pose is None:
    return

  camera_name = getattr(params, "camera_name", "") or "orbbec_camera"
  root_t_camera = _resolve_camera_transform(world, parent_obj, camera_name)
  if root_t_camera is None:
    raise ValueError(f"Camera transform for '{camera_name}' not found in world")

  root_t_target = root_t_camera * cam_pose
  target_pos = root_t_target.translation
  target_rot = root_t_target.rotation

  min_safe_z = getattr(params, "min_safe_z", None)
  if min_safe_z is not None and float(target_pos[2]) < float(min_safe_z):
    raise ValueError(
      f"Calculated workpiece target Z ({float(target_pos[2]):.4f}m) is below "
      f"minimum safe height min_safe_z ({float(min_safe_z):.4f}m)."
    )

  target_id = getattr(params, "target_scene_object_id", "")
  if target_id:
    target_obj = getattr(world, target_id, None) or getattr(
      world, target_id.split(".")[-1], None
    )
    if target_obj is not None:
      actual_parent = (
        getattr(target_obj, "parent", None)
        or getattr(target_obj, "parent_object", None)
        or parent_obj
      )
      world.update_transform(
        node_a=actual_parent, node_b=target_obj, a_t_b=root_t_target
      )

  current_tool_q = _resolve_current_tool_quaternion(world, parent_obj)
  grasp_ori = compute_top_down_grasp_quaternion(target_rot, current_tool_q)
  grasp_rot = data_types.Rotation3(data_types.Quaternion(list(grasp_ori)))

  approach_offset_z = float(getattr(params, "approach_offset_z", 0.08))
  gx, gy, gz = float(target_pos[0]), float(target_pos[1]), float(target_pos[2])

  grasp_pose = data_types.Pose3(grasp_rot, [gx, gy, gz])
  pregrasp_pose = data_types.Pose3(grasp_rot, [gx, gy, gz + approach_offset_z])

  pregrasp_name = (
    getattr(params, "pregrasp_frame_name", None) or "infeed_pre_grasp"
  )
  grasp_name = getattr(params, "grasp_frame_name", None) or "infeed_grasp"

  _sync_frame(world, parent_obj, pregrasp_name, pregrasp_pose)
  _sync_frame(world, parent_obj, grasp_name, grasp_pose)
