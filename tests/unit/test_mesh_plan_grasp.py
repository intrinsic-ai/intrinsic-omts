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

"""Tests for the mesh_plan_grasp skill and the mesh grasp planner backend."""

import types
from unittest import mock

import grpc
import numpy as np
from absl.testing import absltest
from google.protobuf import json_format
from intrinsic.geometry.proto.v1 import inline_geometry_pb2
from intrinsic.math.python import data_types
from intrinsic.skills.python import skill_interface
from intrinsic.world.proto import object_world_service_pb2

from src.core.config import GraspConfig
from src.hardware.mesh_grasp_planner import MeshGraspPlanner
from src.skills.mesh_plan_grasp import mesh_plan_grasp, mesh_plan_grasp_pb2

# A 40 x 100 x 30 mm box: graspable across x only.
_BOX_MESH = {
  "vertices": [
    c * s
    for corner in (
      (-1, -1, -1),
      (1, -1, -1),
      (1, 1, -1),
      (-1, 1, -1),
      (-1, -1, 1),
      (1, -1, 1),
      (1, 1, 1),
      (-1, 1, 1),
    )
    for c, s in zip(corner, (0.02, 0.05, 0.015), strict=True)
  ],
  "faces": [
    *(0, 2, 1, 0, 3, 2),
    *(4, 5, 6, 4, 6, 7),
    *(0, 1, 5, 0, 5, 4),
    *(1, 2, 6, 1, 6, 5),
    *(2, 3, 7, 2, 7, 6),
    *(3, 0, 4, 3, 4, 7),
  ],
}


def _box_object(geometry, link_offset=(0.0, 0.0, 0.0), shape_transform=None):
  """An Object World object proto whose link entity carries the box mesh."""
  shape = {"geometry": geometry}
  if shape_transform:
    shape["refTShape"] = shape_transform
  x, y, z = link_offset
  return json_format.ParseDict(
    {
      "name": "box",
      "rootEntityId": "root_entity",
      "entities": {
        "root_entity": {"id": "root_entity"},
        "link": {
          "id": "link",
          "parentId": "root_entity",
          "parentTThis": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"w": 1.0},
          },
          "geometryComponent": {
            "namedGeometries": {
              "Intrinsic_Visual": {"namedGeometries": {"box": shape}}
            }
          },
        },
      },
    },
    object_world_service_pb2.Object(),
  )


_INLINE_BOX = {
  "inlineGeometryData": {"exactGeometry": {"triangleMesh": _BOX_MESH}}
}


class _World:
  """Object World stand-in: the box at (0.3, 0.2, 0.7), tool at the origin."""

  def __init__(self):
    self.root = types.SimpleNamespace(frame_names=["grasp", "pre_grasp"])
    self.box = types.SimpleNamespace(proto=_box_object(_INLINE_BOX))
    self.object_pose = data_types.Pose3(translation=[0.3, 0.2, 0.7])
    self.updates = {}
    self.created = []

  def get_object(self, name):
    assert name == "box"
    return self.box

  def get_frame(self, frame, owner):
    if owner == "root" and frame not in self.root.frame_names:
      raise ValueError(f"Missing frame {frame}")
    return ("frame", owner, frame)

  def get_transform(self, node_a, node_b):
    del node_a
    if node_b is self.box:
      return self.object_pose
    return data_types.Pose3()

  def update_transform(self, node_a, node_b, a_t_b):
    del node_a
    self.updates[node_b[2]] = a_t_b.matrix4x4()

  def create_frame(self, frame_name, parent, parent_t_frame):
    assert parent is self.root
    assert frame_name not in self.root.frame_names
    self.root.frame_names.append(frame_name)
    self.created.append(frame_name)
    self.updates[frame_name] = parent_t_frame.matrix4x4()


class _PlanningError(grpc.RpcError):
  def code(self):
    return grpc.StatusCode.FAILED_PRECONDITION

  def details(self):
    return "No collision-free Cartesian approach"


class _Planner:
  """Motion planner stand-in that fails IK for the first `ik_failures` calls."""

  def __init__(
    self, ik_failures=0, collides=False, plan_failures=0, empty_path=False
  ):
    self.ik_failures = ik_failures
    self.collides = collides
    self.ik_calls = 0
    self.plan_failures = plan_failures
    self.empty_path = empty_path
    self.paths = []

  def compute_ik(self, robot, target, starting_joints=None, options=None):
    del robot, target, starting_joints
    assert options.collision_settings.minimum_margin > 0
    self.ik_calls += 1
    solved = self.ik_calls > self.ik_failures
    return types.SimpleNamespace(
      solutions=[[float(self.ik_calls)] * 6] if solved else []
    )

  def check_collisions(self, robot, waypoints, options):
    del robot, options
    assert len(waypoints) == 2
    return types.SimpleNamespace(has_collision=self.collides)

  def plan_path(self, robot, specification):
    self.paths.append((robot, specification))
    if self.collides or len(self.paths) <= self.plan_failures:
      raise _PlanningError()
    return types.SimpleNamespace(
      path=types.SimpleNamespace(
        waypoints=[] if self.empty_path else [robot.start_configuration]
      )
    )


def _execute(
  planner, *, world=None, grasp_frame="grasp", pregrasp_frame="pre_grasp"
):
  world = world or _World()
  request = types.SimpleNamespace(
    params=mesh_plan_grasp_pb2.MeshPlanGraspParams(
      workpiece_object="box",
      robot_object="ur_module",
      tool_object="gripper",
      tool_frame="tool_frame",
      grasp_frame=grasp_frame,
      pregrasp_frame=pregrasp_frame,
    )
  )
  context = types.SimpleNamespace(
    object_world=world, motion_planner=planner, geometry_service=None
  )
  return mesh_plan_grasp.MeshPlanGrasp().execute(request, context), world


class MeshPlanGraspSkillTest(absltest.TestCase):
  def test_checks_a_cartesian_linear_approach_from_the_pregrasp(self):
    planner = _Planner()
    planner.compute_ik = mock.Mock(wraps=planner.compute_ik)
    planner.check_collisions = mock.Mock(wraps=planner.check_collisions)

    _, world = _execute(planner)

    planner.compute_ik.assert_called_once()
    pregrasp = planner.compute_ik.call_args.args[1].offset.position
    np.testing.assert_allclose(
      [pregrasp.x, pregrasp.y, pregrasp.z], world.updates["pre_grasp"][:3, 3]
    )
    self.assertLen(planner.paths, 1)
    robot, specification = planner.paths[0]
    self.assertEqual(
      robot.robot_reference.object_id.by_name.object_name, "ur_module"
    )
    self.assertEqual(list(robot.start_configuration.joints), [1.0] * 6)
    self.assertLen(specification.motion_segments, 1)
    segment = specification.motion_segments[0]
    self.assertEqual(segment.motion_type, segment.LINEAR)
    self.assertAlmostEqual(segment.collision_settings.minimum_margin, 0.002)
    target = segment.target.cartesian_pose
    self.assertEqual(target.moving_frame.by_name.frame.object_name, "gripper")
    self.assertEqual(target.moving_frame.by_name.frame.frame_name, "tool_frame")
    self.assertEqual(target.target_frame.by_name.object.object_name, "root")
    position = target.target_frame_offset.position
    np.testing.assert_allclose(
      [position.x, position.y, position.z], world.updates["grasp"][:3, 3]
    )
    planner.check_collisions.assert_not_called()

  def test_skips_an_approach_that_cannot_be_planned(self):
    result, _ = _execute(_Planner(plan_failures=1))
    self.assertEqual(result.num_checked, 2)

  def test_rejects_an_empty_approach_path(self):
    with self.assertRaises(skill_interface.SkillError):
      _execute(_Planner(empty_path=True))

  def test_unload_frames_preserve_infeed_targets_and_are_reused(self):
    _, world = _execute(_Planner())
    infeed = {name: pose.copy() for name, pose in world.updates.items()}
    for position in ([0.5, -0.2, 0.8], [0.4, -0.1, 0.75]):
      world.object_pose = data_types.Pose3(translation=position)
      _execute(
        _Planner(),
        world=world,
        grasp_frame="grasp_unload",
        pregrasp_frame="pre_grasp_unload",
      )
      for name, pose in infeed.items():
        np.testing.assert_array_equal(world.updates[name], pose)
      np.testing.assert_allclose(
        world.updates["grasp_unload"][:2, 3], position[:2], atol=0.01
      )
    self.assertCountEqual(world.created, ["grasp_unload", "pre_grasp_unload"])

  def test_writes_the_first_feasible_candidate(self):
    result, world = _execute(_Planner())

    self.assertEqual(result.num_checked, 1)
    self.assertAlmostEqual(result.width_m, 0.04, places=6)
    grasp, pregrasp = world.updates["grasp"], world.updates["pre_grasp"]
    np.testing.assert_allclose(grasp[:3, 2], [0, 0, -1], atol=1e-9)
    np.testing.assert_allclose(grasp[:2, 3], [0.3, 0.2], atol=0.01)
    np.testing.assert_allclose(pregrasp[2, 3] - grasp[2, 3], 0.1, atol=1e-9)

  def test_skips_candidates_without_an_ik_solution(self):
    # The first candidate's pregrasp IK fails; the second passes.
    result, world = _execute(_Planner(ik_failures=1))

    self.assertEqual(result.num_checked, 2)
    self.assertIn("grasp", world.updates)

  def test_fails_when_every_approach_collides(self):
    world = _World()
    world.root.frame_names.clear()
    with self.assertRaises(skill_interface.SkillError):
      _execute(_Planner(collides=True), world=world)
    self.assertEmpty(world.updates)
    self.assertEmpty(world.created)


class WorkpieceMeshTest(absltest.TestCase):
  def test_reads_a_stored_mesh_in_the_object_frame(self):
    # The mesh is stored in the geometry service, on a link 0.1 m along x,
    # shifted another 0.02 m up within the link.
    box = _box_object(
      {"geoRef": {"exactGeometryRef": "box_mesh"}},
      link_offset=(0.1, 0.0, 0.0),
      shape_transform={"trs": {"translation": {"z": 0.02}}},
    )
    service = mock.Mock()
    service.GetGeometry.return_value = types.SimpleNamespace(
      inline_geometry=json_format.ParseDict(
        _INLINE_BOX["inlineGeometryData"], inline_geometry_pb2.InlineGeometry()
      )
    )

    vertices, triangles = mesh_plan_grasp.workpiece_mesh(
      types.SimpleNamespace(proto=box), service
    )

    request = service.GetGeometry.call_args.args[0]
    self.assertEqual(
      request.geometry_storage_refs.exact_geometry_ref, "box_mesh"
    )
    self.assertEqual(triangles.shape, (12, 3))
    np.testing.assert_allclose(vertices.min(axis=0), [0.08, -0.05, 0.005])
    np.testing.assert_allclose(vertices.max(axis=0), [0.12, 0.05, 0.035])

  def test_object_without_a_visual_mesh_is_rejected(self):
    box = _box_object(_INLINE_BOX)
    del box.entities["link"].geometry_component.named_geometries[
      "Intrinsic_Visual"
    ]
    with self.assertRaisesRegex(ValueError, "no visual mesh"):
      mesh_plan_grasp.workpiece_mesh(types.SimpleNamespace(proto=box), None)


class MeshGraspPlannerTest(absltest.TestCase):
  def test_requires_the_simulation_only_acknowledgement(self):
    # As when --grasp_planner=mesh overrides a cuboid_center config.
    with self.assertRaisesRegex(ValueError, "simulation only"):
      MeshGraspPlanner(
        mock.MagicMock(), GraspConfig(), "ur_module", "gripper", "tool_frame"
      )

  @mock.patch("src.hardware.mesh_grasp_planner.bt.Task")
  def test_builds_the_skill_task(self, unused_task):
    solution = mock.MagicMock()
    config = GraspConfig(planner="mesh", mesh_simulation_only=True)
    planner = MeshGraspPlanner(
      solution, config, "ur_module", "gripper", "tool_frame"
    )
    self.assertFalse(planner.uses_contact_touchdown)

    planner.build_plan_grasp_task("box")

    kwargs = solution.skills.ai.intrinsic.mesh_plan_grasp.call_args.kwargs
    self.assertEqual(kwargs["workpiece_object"], "box")
    self.assertEqual(kwargs["robot_object"], "ur_module")


if __name__ == "__main__":
  absltest.main()
