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

"""Top-down parallel-jaw grasp candidates from a triangle mesh.

Samples surface points, casts a ray from each one straight through the part to
find the opposite contact, and keeps pairs that a top-down parallel-jaw
gripper can close on. Reachability and collisions are not checked here; the
`mesh_plan_grasp` skill checks each candidate with the motion planner.
"""

import dataclasses

import numpy as np

MAX_TRIANGLES = 10000


@dataclasses.dataclass(frozen=True)
class GripperModel:
  """Parallel-jaw geometry that matters for choosing contacts.

  Attributes:
    min_width_m: Smallest jaw gap the fingers can close to.
    max_width_m: Jaw gap when fully open.
    contact_depth_m: Distance along the tool z axis from the tool frame to the
      point where the finger pads meet the part.
  """

  min_width_m: float
  max_width_m: float
  contact_depth_m: float


# Robotiq Hand-E with OMTS's finger offset patch, from the SDF colliders: the
# fingertip spheres close from 60 to 10 mm, 22 mm below `tool_frame`.
HANDE = GripperModel(min_width_m=0.01, max_width_m=0.06, contact_depth_m=0.022)


@dataclasses.dataclass(frozen=True)
class Candidate:
  """A top-down grasp, expressed in the world frame.

  Attributes:
    grasp: 4x4 pose of the tool frame at the contacts, jaws open.
    pregrasp: 4x4 pose of the tool frame above `grasp`.
    width_m: Distance between the two contacts.
  """

  grasp: np.ndarray
  pregrasp: np.ndarray
  width_m: float


def validate_mesh(vertices, triangles):
  """Returns the mesh as (N, 3) float and (M, 3) int arrays, or raises."""
  vertices = np.asarray(vertices, dtype=float)
  triangles = np.asarray(triangles, dtype=int)
  if vertices.ndim != 2 or vertices.shape[1] != 3:
    raise ValueError("vertices must be a list of [x, y, z] points.")
  if triangles.ndim != 2 or triangles.shape[1] != 3:
    raise ValueError("triangles must be a list of vertex index triples.")
  if not 0 < len(triangles) <= MAX_TRIANGLES:
    raise ValueError(f"Expected 1 to {MAX_TRIANGLES} triangles.")
  if triangles.min() < 0 or triangles.max() >= len(vertices):
    raise ValueError("A triangle references a missing vertex.")
  if not np.isfinite(vertices).all():
    raise ValueError("vertices must be finite.")
  corners = vertices[triangles]
  if np.any(
    np.linalg.norm(
      np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]),
      axis=1,
    )
    == 0
  ):
    raise ValueError("The mesh contains a degenerate triangle.")
  # GLB seams can store the same point with different vertex indices.
  _, welded = np.unique(vertices, axis=0, return_inverse=True)
  faces = welded[triangles]
  edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
  _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
  # Every edge must occur exactly twice, once in each direction.
  if np.any(counts != 2) or len(np.unique(edges, axis=0)) != len(edges):
    raise ValueError("The mesh must be closed with outward-facing triangles.")
  volume = np.einsum(
    "ij,ij->i", corners[:, 0], np.cross(corners[:, 1], corners[:, 2])
  ).sum()
  if volume <= 0:
    raise ValueError("The mesh must be closed with outward-facing triangles.")
  return vertices, triangles


def _ray_hits(origins, directions, corners):
  """Möller-Trumbore distances (rays x triangles); inf where there is no hit."""
  edge1 = corners[:, 1] - corners[:, 0]
  edge2 = corners[:, 2] - corners[:, 0]
  h = np.cross(directions[:, None, :], edge2[None])
  det = np.einsum("tk,rtk->rt", edge1, h)
  with np.errstate(divide="ignore", invalid="ignore"):
    inv = 1.0 / det
    s = origins[:, None, :] - corners[None, :, 0]
    u = np.einsum("rtk,rtk->rt", s, h) * inv
    q = np.cross(s, edge1[None])
    v = np.einsum("rk,rtk->rt", directions, q) * inv
    t = np.einsum("tk,rtk->rt", edge2, q) * inv
    hit = (
      (np.abs(det) > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
    )
  return np.where(hit, t, np.inf)


def top_down_grasps(
  vertices,
  triangles,
  world_t_object,
  gripper=HANDE,
  *,
  pregrasp_distance_m=0.1,
  max_tilt_deg=10.0,
  friction_cone_deg=15.0,
  num_samples=2000,
  max_candidates=20,
  preferred_jaw_axis=(1.0, 0.0, 0.0),
  seed=42,
):
  """Returns grasp candidates in the world frame, best first.

  Candidates are ranked by how close the grasp center is to the part's
  centroid (a stable, central hold), then by how directly the two surface
  normals oppose each other.

  Args:
    vertices: (N, 3) mesh vertices in the object frame.
    triangles: (M, 3) vertex indices, closed and outward facing.
    world_t_object: 4x4 pose of the object in the world.
    gripper: Jaw geometry.
    pregrasp_distance_m: Height of the pregrasp above the grasp.
    max_tilt_deg: Largest angle between a contact face's normal and the
      horizontal plane, which also absorbs tilt in `world_t_object`.
    friction_cone_deg: Largest angle between the jaw axis and either normal.
    num_samples: Surface points to sample.
    max_candidates: Maximum number of distinct candidates to return.
    preferred_jaw_axis: World direction the tool x axis should be near, such as
      the current tool x axis, to avoid unnecessary wrist rotation.
    seed: Random seed, so planning is repeatable.
  """
  rotation, translation = world_t_object[:3, :3], world_t_object[:3, 3]
  corners = vertices[triangles] @ rotation.T + translation
  normals = np.cross(
    corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]
  )
  areas = np.linalg.norm(normals, axis=1)
  normals /= np.where(areas > 0, areas, 1.0)[:, None]

  # Only faces whose normal is close to horizontal can carry a jaw contact.
  side = np.abs(normals[:, 2]) <= np.sin(np.radians(max_tilt_deg))
  if not side.any():
    return []
  rng = np.random.default_rng(seed)
  weights = np.where(side, areas, 0.0)
  faces = rng.choice(len(corners), size=num_samples, p=weights / weights.sum())
  a, b = rng.random((2, num_samples))
  flip = a + b > 1
  a, b = np.where(flip, 1 - a, a), np.where(flip, 1 - b, b)
  points = (
    corners[faces, 0]
    + a[:, None] * (corners[faces, 1] - corners[faces, 0])
    + b[:, None] * (corners[faces, 2] - corners[faces, 0])
  )
  inward = -normals[faces]

  distances = np.concatenate(
    [
      _ray_hits(points[i : i + 64], inward[i : i + 64], corners)
      for i in range(0, num_samples, 64)
    ]
  )
  first = np.argmin(distances, axis=1)
  width = distances[np.arange(num_samples), first]
  # The far face of a good pair faces along the ray, away from the near face.
  opposing = np.einsum("ij,ij->i", normals[first], inward)
  keep = (
    np.isfinite(width)
    & (width >= gripper.min_width_m)
    & (width <= gripper.max_width_m)
    & (opposing >= np.cos(np.radians(friction_cone_deg)))
  )

  centroid = corners.reshape(-1, 3).mean(axis=0)
  centers = points + inward * width[:, None] / 2
  offset = np.linalg.norm((centers - centroid)[:, :2], axis=1)
  order = np.lexsort((-opposing, offset))
  preferred = np.asarray(preferred_jaw_axis, dtype=float)

  candidates, seen = [], set()
  for i in order[keep[order]]:
    axis = np.array([inward[i, 0], inward[i, 1], 0.0])
    axis /= np.linalg.norm(axis)
    if axis @ preferred < 0:
      axis = -axis
    yaw = round(np.degrees(np.arctan2(axis[1], axis[0])) / 10) % 36
    key = (*np.round(centers[i] / 0.002).astype(int), yaw)
    if key in seen:
      continue
    seen.add(key)
    if not _closing_path_clear(
      points[i], inward[i], width[i], gripper, corners
    ):
      continue
    down = np.array([0.0, 0.0, -1.0])
    grasp = np.eye(4)
    grasp[:3, :3] = np.column_stack([axis, np.cross(down, axis), down])
    grasp[:3, 3] = centers[i] - gripper.contact_depth_m * down
    pregrasp = grasp.copy()
    pregrasp[2, 3] += pregrasp_distance_m
    candidates.append(Candidate(grasp, pregrasp, float(width[i])))
    if len(candidates) == max_candidates:
      break
  return candidates


def _closing_path_clear(contact, inward, width, gripper, corners):
  """True if each open fingertip reaches its contact without hitting the part.

  The fingertips start `max_width_m` apart, centered on the contacts, and
  close along the jaw axis. A surface between a fingertip and its contact
  would stop the finger before it reaches the planned contact.
  """
  travel = (gripper.max_width_m - width) / 2
  if travel <= 1e-6:
    return True
  far = contact + inward * width
  starts = np.stack([contact - inward * travel, far + inward * travel])
  directions = np.stack([inward, -inward])
  hits = _ray_hits(starts, directions, corners).min(axis=1)
  return bool((hits >= travel - 1e-4).all())
