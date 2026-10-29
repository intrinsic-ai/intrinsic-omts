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

"""Every mesh workpiece has grasp candidates on its mesh in its scene pose."""

import json
import math
import pathlib
import struct

import numpy as np
from absl.testing import absltest
from google.protobuf import text_format
from intrinsic.math.python import data_types
from intrinsic.world.proto import object_world_updates_pb2

from src.core.config import load_app_config
from src.grasping import candidates

# (config, scene updates placing the workpiece, its visual mesh)
_WORKPIECES = (
  (
    "configs/lab_bb_01/app_config.yaml",
    "configs/lab_bb_01/scene.updates.pbtxt",
    "models/raw_stock_2x3x5/base_visual.glb",
  ),
  (
    "configs/lab_bb_01_banana/app_config.yaml",
    "configs/lab_bb_01_banana/scene.updates.pbtxt",
    "models/banana/banana.glb",
  ),
  (
    "configs/lab_bb_01_bracket/app_config.yaml",
    "configs/lab_bb_01_bracket/scene.updates.pbtxt",
    "models/bracket/bracket.glb",
  ),
)

_INDEX_TYPES = {5121: "<u1", 5123: "<u2", 5125: "<u4"}


def _read_glb(path):
  """Returns (vertices, triangles) of a .glb holding one triangle mesh."""
  data = pathlib.Path(path).read_bytes()
  json_length = struct.unpack_from("<I", data, 12)[0]
  gltf = json.loads(data[20 : 20 + json_length])
  binary = data[28 + json_length :]

  def read(index, dtype, width):
    accessor = gltf["accessors"][index]
    view = gltf["bufferViews"][accessor["bufferView"]]
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    values = np.frombuffer(binary, dtype, accessor["count"] * width, start)
    return values.reshape(-1, width)

  (primitive,) = gltf["meshes"][0]["primitives"]
  vertices = read(primitive["attributes"]["POSITION"], "<f4", 3)
  if "indices" not in primitive:
    return candidates.validate_mesh(
      vertices, np.arange(len(vertices)).reshape(-1, 3)
    )
  index_type = _INDEX_TYPES[
    gltf["accessors"][primitive["indices"]]["componentType"]
  ]
  triangles = read(primitive["indices"], index_type, 1).reshape(-1, 3)
  return candidates.validate_mesh(vertices, triangles)


def _scene_pose(scene_path, object_name):
  updates = text_format.Parse(
    pathlib.Path(scene_path).read_text(encoding="utf-8"),
    object_world_updates_pb2.ObjectWorldUpdates(),
  )
  for update in updates.updates:
    transform = update.update_transform
    if transform.node_to_update.by_name.object.object_name == object_name:
      q = transform.a_t_b.orientation
      norm = math.hypot(q.x, q.y, q.z, q.w)
      p = transform.a_t_b.position
      return data_types.Pose3(
        data_types.Rotation3(
          data_types.Quaternion(
            [q.x / norm, q.y / norm, q.z / norm, q.w / norm]
          )
        ),
        [p.x, p.y, p.z],
      ).matrix4x4()
  raise AssertionError(f"{object_name} is not placed in {scene_path}")


class MeshWorkpiecesTest(absltest.TestCase):
  def test_configured_workpieces_have_grasp_candidates(self):
    for config_path, scene_path, mesh_path in _WORKPIECES:
      with self.subTest(config=config_path):
        config = load_app_config(config_path)
        workpiece = config.cycle.workpiece_id
        self.assertEqual(
          config.vision.scene_object_id, f"ai.intrinsic.{workpiece}"
        )
        found = candidates.top_down_grasps(
          *_read_glb(mesh_path),
          _scene_pose(scene_path, workpiece),
          pregrasp_distance_m=config.grasp.mesh_pregrasp_distance_m,
          max_tilt_deg=config.grasp.mesh_max_approach_tilt_deg,
        )
        self.assertNotEmpty(found)
        for candidate in found:
          self.assertBetween(
            candidate.width_m,
            candidates.HANDE.min_width_m,
            candidates.HANDE.max_width_m,
          )


if __name__ == "__main__":
  absltest.main()
