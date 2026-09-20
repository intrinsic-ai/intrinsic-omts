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

"""Unit tests for the apply_scene_updates developer CLI tool."""

import os
import tempfile
from unittest import mock

from absl.testing import absltest
from google.protobuf import text_format
from intrinsic.world.proto import object_world_updates_pb2

from tools.world.apply_scene_updates import (
  DEFAULT_UPDATE_FILES,
  adapt_updates_for_live_world,
  apply_pbtxt_file,
  find_file,
  main,
  parse_args,
)


class ApplySceneUpdatesTest(absltest.TestCase):
  """Tests for apply_scene_updates script."""

  def test_parse_args_defaults(self) -> None:
    """Tests default command-line argument values."""
    args = parse_args([])
    self.assertEqual(args.address, "localhost:17080")
    self.assertEqual(args.files, DEFAULT_UPDATE_FILES)
    self.assertTrue(args.reset_sim)

  def test_parse_args_no_reset_sim(self) -> None:
    """Tests disabling simulation reset via --no-reset_sim flag."""
    args = parse_args(["--no-reset_sim"])
    self.assertFalse(args.reset_sim)

  def test_parse_args_explicit_reset_sim(self) -> None:
    """Tests explicitly enabling simulation reset via --reset_sim flag."""
    args = parse_args(["--reset_sim"])
    self.assertTrue(args.reset_sim)

  @mock.patch("tools.world.apply_scene_updates.deployments.connect")
  @mock.patch("tools.world.apply_scene_updates.apply_pbtxt_file")
  def test_main_triggers_sim_reset_when_simulated(
    self, mock_apply_file: mock.MagicMock, mock_connect: mock.MagicMock
  ) -> None:
    """Tests that simulation reset is called when running in simulation mode."""
    mock_solution = mock.MagicMock()
    mock_solution.is_simulated = True
    mock_solution.simulator = mock.MagicMock()
    mock_connect.return_value = mock_solution

    main(["--files", "test.pbtxt"])

    mock_apply_file.assert_called_once_with(
      world=mock_solution.world, filepath="test.pbtxt"
    )
    mock_solution.simulator.reset.assert_called_once()

  @mock.patch("tools.world.apply_scene_updates.deployments.connect")
  @mock.patch("tools.world.apply_scene_updates.apply_pbtxt_file")
  def test_main_skips_sim_reset_when_disabled(
    self, mock_apply_file: mock.MagicMock, mock_connect: mock.MagicMock
  ) -> None:
    """Tests that simulation reset is skipped when --no-reset_sim is provided."""
    mock_solution = mock.MagicMock()
    mock_solution.is_simulated = True
    mock_solution.simulator = mock.MagicMock()
    mock_connect.return_value = mock_solution

    main(["--no-reset_sim", "--files", "test.pbtxt"])

    mock_apply_file.assert_called_once_with(
      world=mock_solution.world, filepath="test.pbtxt"
    )
    mock_solution.simulator.reset.assert_not_called()

  @mock.patch("tools.world.apply_scene_updates.deployments.connect")
  @mock.patch("tools.world.apply_scene_updates.apply_pbtxt_file")
  def test_main_skips_sim_reset_on_real_hardware(
    self, mock_apply_file: mock.MagicMock, mock_connect: mock.MagicMock
  ) -> None:
    """Tests that simulation reset is not triggered on real hardware."""
    mock_solution = mock.MagicMock()
    mock_solution.is_simulated = False
    mock_solution.simulator = None
    mock_connect.return_value = mock_solution

    main(["--files", "test.pbtxt"])

    mock_apply_file.assert_called_once_with(
      world=mock_solution.world, filepath="test.pbtxt"
    )

  def setUp(self):
    super().setUp()
    self.temp_dir = tempfile.TemporaryDirectory()

  def tearDown(self):
    self.temp_dir.cleanup()
    super().tearDown()

  def test_find_file_direct_exists(self):
    test_file = os.path.join(self.temp_dir.name, "test.pbtxt")
    with open(test_file, "w", encoding="utf-8") as f:
      f.write("updates {}")

    resolved = find_file(test_file)
    self.assertEqual(resolved, test_file)

  def test_find_file_build_working_directory(self):
    configs_dir = os.path.join(self.temp_dir.name, "configs", "omts")
    os.makedirs(configs_dir, exist_ok=True)
    test_file = os.path.join(configs_dir, "cnc_enclosure.updates.pbtxt")
    with open(test_file, "w", encoding="utf-8") as f:
      f.write("updates {}")

    with mock.patch.dict(
      os.environ, {"BUILD_WORKING_DIRECTORY": self.temp_dir.name}
    ):
      resolved = find_file("configs/omts/cnc_enclosure.updates.pbtxt")
      self.assertEqual(resolved, test_file)

  def test_find_file_build_workspace_directory(self):
    configs_dir = os.path.join(self.temp_dir.name, "configs", "omts")
    os.makedirs(configs_dir, exist_ok=True)
    test_file = os.path.join(configs_dir, "cnc_enclosure.updates.pbtxt")
    with open(test_file, "w", encoding="utf-8") as f:
      f.write("updates {}")

    with mock.patch.dict(
      os.environ,
      {
        "BUILD_WORKING_DIRECTORY": "/some/other/dir",
        "BUILD_WORKSPACE_DIRECTORY": self.temp_dir.name,
      },
    ):
      resolved = find_file("configs/omts/cnc_enclosure.updates.pbtxt")
      self.assertEqual(resolved, test_file)

  def test_adapt_updates_for_live_world_converts_existing_frames(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = ["existing_frame"]
    mock_world.root = mock_root

    pbtxt = """
    updates {
      create_frame {
        parent_object_with_filter {
          reference {
            by_name {
              object_name: "root"
            }
          }
        }
        new_frame_name: "existing_frame"
        parent_t_new_frame {
          position { x: 1.0 y: 2.0 z: 3.0 }
          orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }
        }
      }
    }
    updates {
      create_frame {
        parent_object_with_filter {
          reference {
            by_name {
              object_name: "root"
            }
          }
        }
        new_frame_name: "brand_new_frame"
        parent_t_new_frame {
          position { x: 4.0 y: 5.0 z: 6.0 }
          orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }
        }
      }
    }
    """
    raw_updates = object_world_updates_pb2.ObjectWorldUpdates()
    text_format.Parse(pbtxt, raw_updates)

    adapted = adapt_updates_for_live_world(
      world=mock_world, updates=raw_updates
    )
    self.assertEqual(len(adapted.updates), 2)
    # First update should be converted to update_transform
    self.assertTrue(adapted.updates[0].HasField("update_transform"))
    ut_node = adapted.updates[0].update_transform.node_to_update.by_name.frame
    self.assertEqual(ut_node.frame_name, "existing_frame")
    # Second update should remain create_frame
    self.assertTrue(adapted.updates[1].HasField("create_frame"))
    self.assertEqual(
      adapted.updates[1].create_frame.new_frame_name, "brand_new_frame"
    )

  def test_adapt_updates_skips_update_transform_for_missing_object(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    mock_world.root = mock_root
    mock_world.list_objects.return_value = [mock_root]

    pbtxt = """
    updates {
      update_transform {
        node_a { by_name { object { object_name: "root" } } }
        node_b { by_name { object { object_name: "missing_stock" } } }
        node_to_update { by_name { object { object_name: "missing_stock" } } }
        a_t_b {
          position { x: 0.1 y: 0.2 z: 0.3 }
          orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }
        }
      }
    }
    """
    raw_updates = object_world_updates_pb2.ObjectWorldUpdates()
    text_format.Parse(pbtxt, raw_updates)

    adapted = adapt_updates_for_live_world(
      world=mock_world, updates=raw_updates
    )
    self.assertEqual(len(adapted.updates), 0)

  def test_apply_pbtxt_file(self):
    test_file = os.path.join(self.temp_dir.name, "scene.updates.pbtxt")
    with open(test_file, "w", encoding="utf-8") as f:
      f.write("""
      updates {
        create_frame {
          parent_object_with_filter {
            reference {
              by_name {
                object_name: "root"
              }
            }
          }
          new_frame_name: "test_frame"
          parent_t_new_frame {
            position { x: 0.1 y: 0.2 z: 0.3 }
            orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }
          }
        }
      }
      """)

    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    mock_world.root = mock_root

    apply_pbtxt_file(world=mock_world, filepath=test_file)
    mock_world.batch_update.assert_called_once()

  @mock.patch("tools.world.inspect_world.deployments.connect")
  def test_inspect_world_lists_world_objects_and_strict_resources(
    self, mock_connect
  ):
    import io
    import sys

    from tools.world import inspect_world

    class _FakeKinematicObj:
      def __init__(self, name: str) -> None:
        self.name = name
        cfg = mock.MagicMock()
        cfg.joint_position = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        self.joint_configurations = {"home": cfg}

    class _StrictResourcesWithoutUpdate:
      """Raises KeyError on unknown attributes (including .update) like Resources.__getattr__."""

      def __init__(self) -> None:
        self._h = mock.MagicMock()
        self._h.name = "orbbec_camera"

      def __getitem__(self, key: str):
        if key == "orbbec_camera":
          return self._h
        raise KeyError(f"Resource {key} not registered")

      def __getattr__(self, key: str):
        if key == "orbbec_camera":
          return self._h
        raise KeyError(f"Resource {key} not registered")

      def __dir__(self) -> list[str]:
        return ["orbbec_camera"]

    mock_solution = mock.MagicMock()
    mock_solution.world.list_objects.return_value = [
      _FakeKinematicObj("ur_module")
    ]
    mock_solution.resources = _StrictResourcesWithoutUpdate()
    mock_connect.return_value = mock_solution

    buf = io.StringIO()
    with mock.patch.object(sys, "stdout", buf):
      inspect_world.main(["--address", "localhost:17080"])

    output = buf.getvalue()
    self.assertIn("- ur_module", output)
    self.assertIn("* home: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]", output)
    self.assertIn("- orbbec_camera", output)
    self.assertNotIn("Error listing resources", output)


if __name__ == "__main__":
  absltest.main()
