"""Unit tests for the apply_scene_updates developer CLI tool."""

import os
import tempfile
from unittest import mock

from absl.testing import absltest
from google.protobuf import text_format
from intrinsic.world.public.proto import object_world_updates_pb2
from tools.world.apply_scene_updates import adapt_updates_for_live_world
from tools.world.apply_scene_updates import apply_pbtxt_file
from tools.world.apply_scene_updates import find_file


class ApplySceneUpdatesTest(absltest.TestCase):

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
    configs_dir = os.path.join(self.temp_dir.name, "configs")
    os.makedirs(configs_dir, exist_ok=True)
    test_file = os.path.join(configs_dir, "cnc_enclosure.updates.pbtxt")
    with open(test_file, "w", encoding="utf-8") as f:
      f.write("updates {}")

    with mock.patch.dict(
        os.environ, {"BUILD_WORKING_DIRECTORY": self.temp_dir.name}
    ):
      resolved = find_file("configs/cnc_enclosure.updates.pbtxt")
      self.assertEqual(resolved, test_file)

  def test_find_file_build_workspace_directory(self):
    configs_dir = os.path.join(self.temp_dir.name, "configs")
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
      resolved = find_file("configs/cnc_enclosure.updates.pbtxt")
      self.assertEqual(resolved, test_file)

  def test_adapt_updates_for_live_world_converts_existing_frames(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = ["existing_frame"]
    setattr(mock_world, "root", mock_root)

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
    ut_node = (
        adapted.updates[0].update_transform.node_to_update.by_name.frame
    )
    self.assertEqual(ut_node.frame_name, "existing_frame")
    # Second update should remain create_frame
    self.assertTrue(adapted.updates[1].HasField("create_frame"))
    self.assertEqual(
        adapted.updates[1].create_frame.new_frame_name, "brand_new_frame"
    )

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
    setattr(mock_world, "root", mock_root)

    apply_pbtxt_file(world=mock_world, filepath=test_file)
    mock_world.batch_update.assert_called_once()


if __name__ == "__main__":
  absltest.main()
