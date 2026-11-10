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

"""Tests for top-down mesh grasp candidates."""

import math

import numpy as np
from absl.testing import absltest

from src.grasping import candidates

# Outward-facing triangles of the unit cube [0, 1]^3.
_CUBE_TRIANGLES = (
  (0, 2, 1),
  (0, 3, 2),
  (4, 5, 6),
  (4, 6, 7),
  (0, 1, 5),
  (0, 5, 4),
  (1, 2, 6),
  (1, 6, 5),
  (2, 3, 7),
  (2, 7, 6),
  (3, 0, 4),
  (3, 4, 7),
)


def _boxes(*boxes):
  """Returns (vertices, triangles) for boxes given as (min corner, size)."""
  vertices, triangles = [], []
  for low, size in boxes:
    base = len(vertices)
    for x, y, z in (
      (0, 0, 0),
      (1, 0, 0),
      (1, 1, 0),
      (0, 1, 0),
      (0, 0, 1),
      (1, 0, 1),
      (1, 1, 1),
      (0, 1, 1),
    ):
      vertices.append([low[i] + (x, y, z)[i] * size[i] for i in range(3)])
    triangles.extend([base + i for i in tri] for tri in _CUBE_TRIANGLES)
  return candidates.validate_mesh(vertices, triangles)


def _centered_box(size):
  return _boxes(([-s / 2 for s in size], size))


class TopDownGraspsTest(absltest.TestCase):
  def test_box_is_grasped_across_its_narrow_side(self):
    found = candidates.top_down_grasps(
      *_centered_box((0.04, 0.1, 0.03)), np.eye(4), pregrasp_distance_m=0.1
    )

    self.assertNotEmpty(found)
    best = found[0]
    self.assertAlmostEqual(best.width_m, 0.04, places=6)
    # Tool z points down, the jaws close along world x.
    np.testing.assert_allclose(best.grasp[:3, 2], [0, 0, -1], atol=1e-9)
    self.assertAlmostEqual(abs(best.grasp[0, 0]), 1.0, places=6)
    # The contacts are `contact_depth_m` below the tool frame, near the center.
    contact_center = best.grasp[:3, 3] + 0.022 * best.grasp[:3, 2]
    self.assertLess(np.linalg.norm(contact_center[:2]), 0.01)
    np.testing.assert_allclose(
      best.pregrasp[:3, 3], best.grasp[:3, 3] + [0, 0, 0.1], atol=1e-9
    )

  def test_object_pose_is_applied(self):
    yaw = math.radians(90)
    world_t_object = np.eye(4)
    world_t_object[:3, :3] = [
      [math.cos(yaw), -math.sin(yaw), 0],
      [math.sin(yaw), math.cos(yaw), 0],
      [0, 0, 1],
    ]
    world_t_object[:3, 3] = [0.3, 0.2, 0.7]

    best = candidates.top_down_grasps(
      *_centered_box((0.04, 0.1, 0.03)), world_t_object
    )[0]

    # The narrow side now spans world y.
    self.assertAlmostEqual(abs(best.grasp[1, 0]), 1.0, places=6)
    self.assertLess(np.linalg.norm(best.grasp[:2, 3] - [0.3, 0.2]), 0.01)

  def test_jaws_prefer_the_given_direction(self):
    found = candidates.top_down_grasps(
      *_centered_box((0.04, 0.1, 0.03)),
      np.eye(4),
      preferred_jaw_axis=(-1.0, 0.0, 0.0),
    )

    self.assertTrue(all(c.grasp[0, 0] < 0 for c in found))

  def test_part_wider_than_the_open_jaws_has_no_grasp(self):
    self.assertEmpty(
      candidates.top_down_grasps(*_centered_box((0.08, 0.1, 0.03)), np.eye(4))
    )

  def test_small_pose_estimate_tilt_keeps_the_grasps(self):
    # A few degrees of perception error tilts the gripped faces.
    tilt = math.radians(3)
    world_t_object = np.eye(4)
    world_t_object[:3, :3] = [
      [math.cos(tilt), 0, math.sin(tilt)],
      [0, 1, 0],
      [-math.sin(tilt), 0, math.cos(tilt)],
    ]

    self.assertNotEmpty(
      candidates.top_down_grasps(
        *_centered_box((0.04, 0.1, 0.03)), world_t_object
      )
    )

  def test_tilted_side_faces_have_no_top_down_grasp(self):
    tilt = math.radians(20)
    world_t_object = np.eye(4)
    world_t_object[:3, :3] = [
      [math.cos(tilt), 0, math.sin(tilt)],
      [0, 1, 0],
      [-math.sin(tilt), 0, math.cos(tilt)],
    ]

    self.assertEmpty(
      candidates.top_down_grasps(
        *_centered_box((0.04, 0.1, 0.03)), world_t_object, max_tilt_deg=2.0
      )
    )

  def test_part_in_a_closing_finger_path_blocks_the_grasp(self):
    plate = ((-0.006, -0.05, 0.0), (0.012, 0.1, 0.03))
    # Too thin to be grasped itself, but in the +x finger's path.
    wall = ((0.015, -0.05, 0.0), (0.005, 0.1, 0.03))

    self.assertNotEmpty(candidates.top_down_grasps(*_boxes(plate), np.eye(4)))
    self.assertEmpty(
      candidates.top_down_grasps(*_boxes(plate, wall), np.eye(4))
    )

  def test_planning_is_repeatable(self):
    mesh = _centered_box((0.04, 0.1, 0.03))
    first = candidates.top_down_grasps(*mesh, np.eye(4))
    second = candidates.top_down_grasps(*mesh, np.eye(4))

    self.assertEqual(len(first), len(second))
    for a, b in zip(first, second, strict=True):
      np.testing.assert_array_equal(a.grasp, b.grasp)


class ValidateMeshTest(absltest.TestCase):
  def test_rejects_an_open_mesh(self):
    vertices, triangles = _centered_box((0.04, 0.1, 0.03))
    with self.assertRaisesRegex(ValueError, "closed"):
      candidates.validate_mesh(vertices, np.delete(triangles, [2, 3], axis=0))

  def test_rejects_inconsistent_winding(self):
    vertices, triangles = _centered_box((0.04, 0.1, 0.03))
    triangles[4] = triangles[4, ::-1]
    with self.assertRaisesRegex(ValueError, "outward"):
      candidates.validate_mesh(vertices, triangles)

  def test_rejects_a_nonmanifold_edge(self):
    vertices, triangles = _centered_box((0.04, 0.1, 0.03))
    with self.assertRaisesRegex(ValueError, "closed"):
      candidates.validate_mesh(vertices, np.vstack([triangles, triangles[0]]))

  def test_rejects_a_degenerate_triangle(self):
    vertices, triangles = _centered_box((0.04, 0.1, 0.03))
    with self.assertRaisesRegex(ValueError, "degenerate"):
      candidates.validate_mesh(vertices, np.vstack([triangles, [0, 0, 1]]))

  def test_accepts_duplicated_vertices_at_mesh_seams(self):
    vertices, triangles = _centered_box((0.04, 0.1, 0.03))
    split_vertices = vertices[triangles].reshape(-1, 3)
    split_triangles = np.arange(len(split_vertices)).reshape(-1, 3)

    actual_vertices, actual_triangles = candidates.validate_mesh(
      split_vertices, split_triangles
    )

    np.testing.assert_array_equal(actual_vertices, split_vertices)
    np.testing.assert_array_equal(actual_triangles, split_triangles)

  def test_rejects_inward_facing_triangles(self):
    vertices, triangles = _centered_box((0.04, 0.1, 0.03))
    with self.assertRaisesRegex(ValueError, "outward"):
      candidates.validate_mesh(vertices, triangles[:, ::-1])


if __name__ == "__main__":
  absltest.main()
