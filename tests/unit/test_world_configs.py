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

"""Parses every checked-in world update config as the proto it claims to be.

A missing brace in configs/omts/scene.updates.pbtxt survived three commits
because nothing in the build ever parsed the file. The app, omts_ctl.sh and
//tools/world:apply_scene_updates all consume these at runtime, on the cell,
which is a poor place to discover a syntax error.
"""

import glob
import os

from absl.testing import absltest
from google.protobuf import text_format
from intrinsic.world.proto import object_world_updates_pb2

# Every world update config is expected to be an ObjectWorldUpdates. Configs
# that are not world updates, such as calibration_waypoints.pbtxt, are excluded
# by the naming convention rather than by an explicit list. The leading "*" is
# the robot cell directory, or "common".
_UPDATES_GLOB = "configs/*/*.updates.pbtxt"

# The frame names are a published interface: omts_ctl.sh, the app's frame flags
# and tools/jogging/move_to_frame.py all refer to them by name. lab_bb_01 has no
# vise, so it declares no vise_* frames.
_EXPECTED_SCENE_FRAMES = {
  "configs/omts/scene.updates.pbtxt": [
    "vise_pre_place",
    "vise_place",
    "machine_approach",
    "work_view",
    "view",
    "transit",
    "vise_view",
  ],
  "configs/lab_bb_01/scene.updates.pbtxt": [
    "machine_approach",
    "work_view",
    "view",
    "transit",
  ],
}


def _config_paths() -> list[str]:
  """Returns the world update configs, resolved from the runfiles root."""
  return sorted(glob.glob(_UPDATES_GLOB))


def _load_updates_proto(
  path: str,
) -> object_world_updates_pb2.ObjectWorldUpdates | None:
  """Loads and parses an ObjectWorldUpdates proto from the given path."""
  if not os.path.exists(path):
    return None
  with open(path, encoding="utf-8") as f:
    return text_format.Parse(
      f.read(), object_world_updates_pb2.ObjectWorldUpdates()
    )


class WorldUpdateConfigsTest(absltest.TestCase):
  def test_configs_are_present(self):
    # Guards against the glob silently matching nothing, which would make
    # every other assertion in this file vacuous.
    self.assertNotEmpty(
      _config_paths(),
      f"No configs matched {_UPDATES_GLOB} from {os.getcwd()}.",
    )

  def test_every_config_parses_as_object_world_updates(self):
    for path in _config_paths():
      with self.subTest(path=path):
        with open(path, encoding="utf-8") as f:
          contents = f.read()
        updates = object_world_updates_pb2.ObjectWorldUpdates()
        text_format.Parse(contents, updates)
        self.assertNotEmpty(
          updates.updates, f"{path} parsed but declares no updates."
        )

  def test_scene_declares_the_expected_frames(self):
    for path, expected in _EXPECTED_SCENE_FRAMES.items():
      with self.subTest(path=path):
        updates = _load_updates_proto(path)
        self.assertIsNotNone(updates, f"{path} is missing from {os.getcwd()}.")
        created = [
          update.create_frame.new_frame_name
          for update in updates.updates
          if update.HasField("create_frame")
        ]
        self.assertCountEqual(created, expected)


if __name__ == "__main__":
  absltest.main()
