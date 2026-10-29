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

"""Skill that plans a mesh grasp and checks it with the motion planner."""

import grpc
import numpy as np
from absl import logging
from intrinsic.geometry.proto import geometry_service_pb2
from intrinsic.icon.proto import joint_space_pb2
from intrinsic.math.python import data_types, proto_conversion
from intrinsic.motion_planning import motion_planner_client
from intrinsic.motion_planning.proto import motion_target_pb2
from intrinsic.skills.proto import footprint_pb2
from intrinsic.skills.python import skill_interface
from intrinsic.world.proto import collision_settings_pb2

from src.grasping import candidates
from src.skills.mesh_plan_grasp import mesh_plan_grasp_pb2
from src.utils.math_utils import create_transform_node_ref

# Status code for "no candidate passed the motion planner checks".
NO_FEASIBLE_GRASP = 10400

# Object World name for the geometry an entity is rendered with.
_VISUAL_GEOMETRY = "Intrinsic_Visual"


def _value(params, field, default):
  return getattr(params, field) if params.HasField(field) else default


def _matrix(transform):
  """Returns a GeometricTransform as a 4x4 matrix."""
  if transform.HasField("matrix4d"):
    return proto_conversion.ndarray_from_matrix_proto(transform.matrix4d)
  matrix = np.eye(4)
  if transform.HasField("trs"):
    trs = transform.trs
    rpy = trs.rotation_rpy
    matrix[:3, :3] = data_types.Rotation3.from_euler_angles(
      rpy_radians=[rpy.r, rpy.p, rpy.y]
    ).matrix3x3()
    if trs.HasField("scale"):
      matrix[:3, :3] *= [trs.scale.x, trs.scale.y, trs.scale.z]
    matrix[:3, 3] = [trs.translation.x, trs.translation.y, trs.translation.z]
  return matrix


def workpiece_mesh(workpiece, geometry_service):
  """Returns the visual mesh of an Object World object in its own frame."""
  proto = workpiece.proto
  vertices, triangles = [], []
  for entity in proto.entities.values():
    named = entity.geometry_component.named_geometries
    if _VISUAL_GEOMETRY not in named:
      continue
    object_t_entity = np.eye(4)
    link = entity
    while link.id != proto.root_entity_id:
      parent_t_link = proto_conversion.pose_from_proto(link.parent_t_this)
      object_t_entity = parent_t_link.matrix4x4() @ object_t_entity
      link = proto.entities[link.parent_id]
    for shape in named[_VISUAL_GEOMETRY].named_geometries.values():
      inline = shape.geometry.inline_geometry_data
      if shape.geometry.HasField("geo_ref"):
        inline = geometry_service.GetGeometry(
          geometry_service_pb2.GetGeometryRequest(
            geometry_storage_refs=shape.geometry.geo_ref
          )
        ).inline_geometry
      if not inline.exact_geometry.HasField("triangle_mesh"):
        raise ValueError(
          f"{proto.name} has a visual geometry that is not a mesh."
        )
      mesh = inline.exact_geometry.triangle_mesh
      object_t_shape = object_t_entity @ _matrix(shape.ref_t_shape)
      points = np.reshape(mesh.vertices, (-1, 3))
      triangles.append(
        np.reshape(mesh.faces, (-1, 3)) + sum(map(len, vertices))
      )
      vertices.append(points @ object_t_shape[:3, :3].T + object_t_shape[:3, 3])
  if not vertices:
    raise ValueError(f"{proto.name} has no visual mesh.")
  return candidates.validate_mesh(
    np.concatenate(vertices), np.concatenate(triangles)
  )


class MeshPlanGrasp(skill_interface.Skill):
  """Plans a top-down mesh grasp and writes it to the grasp frames."""

  def get_footprint(self, request, context):
    del request, context
    return footprint_pb2.Footprint(lock_the_universe=True)

  def execute(self, request, context):
    params = request.params
    world = context.object_world
    root = world.root
    workpiece = world.get_object(params.workpiece_object)
    world_t_object = world.get_transform(root, workpiece).matrix4x4()
    world_t_tool = world.get_transform(
      root, world.get_frame(params.tool_frame, params.tool_object)
    ).matrix4x4()

    vertices, triangles = workpiece_mesh(workpiece, context.geometry_service)
    logging.info(
      "Workpiece mesh: %d triangles, extent %s m.",
      len(triangles),
      np.ptp(vertices, axis=0).round(4),
    )
    found = candidates.top_down_grasps(
      vertices,
      triangles,
      world_t_object,
      pregrasp_distance_m=_value(params, "pregrasp_distance_m", 0.1),
      max_tilt_deg=_value(params, "max_approach_tilt_deg", 2.0),
      preferred_jaw_axis=world_t_tool[:3, 0],
    )
    settings = collision_settings_pb2.CollisionSettings(
      minimum_margin=_value(params, "collision_margin_m", 0.002)
    )
    # As in Core's motion planning skills: prefer the resolved service.
    planner = None
    if params.HasField("motion_planner_service"):
      planner = motion_planner_client.get_motion_planner_service_asset_client(
        world.world_id, params.motion_planner_service
      )
    planner = planner or context.motion_planner
    checked = found[: _value(params, "max_checked_candidates", 10)]
    for index, candidate in enumerate(checked, 1):
      if not self._feasible(planner, params, candidate, settings):
        continue
      for frame, pose in (
        (params.grasp_frame, candidate.grasp),
        (params.pregrasp_frame, candidate.pregrasp),
      ):
        world.update_transform(
          node_a=root,
          node_b=world.get_frame(frame, "root"),
          a_t_b=data_types.Pose3.from_matrix4x4(pose),
        )
      logging.info(
        "Selected mesh grasp %d of %d (width %.4f m).",
        index,
        len(found),
        candidate.width_m,
      )
      return mesh_plan_grasp_pb2.MeshPlanGraspResult(
        width_m=candidate.width_m,
        num_candidates=len(found),
        num_checked=index,
      )
    raise skill_interface.SkillError(
      NO_FEASIBLE_GRASP,
      f"None of the {len(checked)} checked mesh grasps ({len(found)} found) "
      "is reachable without collision.",
    )

  def _feasible(self, planner, params, candidate, settings):
    """IK with collision checks at both poses, then the approach between."""
    tool = create_transform_node_ref(params.tool_object, params.tool_frame)
    root = create_transform_node_ref("root")
    joints = []
    for pose in (candidate.pregrasp, candidate.grasp):
      target = motion_target_pb2.CartesianMotionTarget(
        tool=tool,
        frame=root,
        offset=proto_conversion.pose_to_proto(
          data_types.Pose3.from_matrix4x4(pose)
        ),
      )
      options = motion_planner_client.IKOptions(
        max_num_solutions=1,
        collision_settings=settings,
        ensure_same_branch=bool(joints),
      )
      try:
        result = planner.compute_ik(
          params.robot_object,
          target,
          starting_joints=joints[-1] if joints else None,
          options=options,
        )
      except grpc.RpcError as error:
        logging.info(
          "Mesh grasp IK failed: %s %s", error.code(), error.details()
        )
        return False
      if not result.solutions:
        return False
      joints.append(result.solutions[0])
    response = planner.check_collisions(
      params.robot_object,
      [joint_space_pb2.JointVec(joints=q) for q in joints],
      motion_planner_client.CheckCollisionsOptions(collision_settings=settings),
    )
    return not response.has_collision
