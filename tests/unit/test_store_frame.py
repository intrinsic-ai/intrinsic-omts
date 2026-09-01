"""Unit tests for the store_frame developer CLI tool."""

import os
import tempfile
from unittest import mock

from absl.testing import absltest
from google.protobuf import text_format
from intrinsic.math.python import data_types
from intrinsic.world.public.proto import object_world_updates_pb2
from tools.jogging.store_frame import get_current_tool_pose
from tools.jogging.store_frame import parse_args
from tools.jogging.store_frame import save_frame_to_scene_updates
from tools.jogging.store_frame import update_live_world_frame


class StoreFrameTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.temp_dir = tempfile.TemporaryDirectory()
    self.pbtxt_path = os.path.join(self.temp_dir.name, "scene.updates.pbtxt")

  def tearDown(self):
    self.temp_dir.cleanup()
    super().tearDown()

  def test_save_frame_to_scene_updates_new_frame(self):
    pos = (0.15, 0.25, 0.71)
    ori = (1.0, 0.0, 0.0, 0.0)

    saved_path = save_frame_to_scene_updates(
        frame_name="test_grasp",
        position=pos,
        orientation=ori,
        parent_object_name="root",
        filepath=self.pbtxt_path,
    )

    self.assertEqual(saved_path, self.pbtxt_path)
    self.assertTrue(os.path.exists(self.pbtxt_path))

    updates = object_world_updates_pb2.ObjectWorldUpdates()
    with open(self.pbtxt_path, "r", encoding="utf-8") as f:
      content = f.read()
      text_format.Parse(content, updates)

    self.assertEqual(len(updates.updates), 1)
    cf = updates.updates[0].create_frame
    self.assertEqual(cf.new_frame_name, "test_grasp")
    self.assertEqual(
        cf.parent_object_with_filter.reference.by_name.object_name, "root"
    )
    self.assertAlmostEqual(cf.parent_t_new_frame.position.x, 0.15)
    self.assertAlmostEqual(cf.parent_t_new_frame.position.y, 0.25)
    self.assertAlmostEqual(cf.parent_t_new_frame.position.z, 0.71)
    self.assertAlmostEqual(cf.parent_t_new_frame.orientation.x, 1.0)
    self.assertAlmostEqual(cf.parent_t_new_frame.orientation.w, 0.0)

  def test_save_frame_to_scene_updates_overwrite_existing(self):
    initial_pbtxt = """
# proto-file: intrinsic/world/public/proto/object_world_updates.proto
# proto-message: intrinsic_proto.world.ObjectWorldUpdates

updates: {
  create_frame: {
    parent_object_with_filter: { reference: {
      by_name: {
        object_name: "root"
      }
    } }
    parent_t_new_frame: {
      position: {
        x: 0.34
        y: 0.37
        z: 0.85
      }
      orientation: {
        x: 1
        y: 0
        z: 0
        w: 0
      }
    }
    new_frame_name: "view"
  }
}
"""
    with open(self.pbtxt_path, "w", encoding="utf-8") as f:
      f.write(initial_pbtxt)

    new_pos = (0.36, 0.40, 0.88)
    new_ori = (0.7071, 0.0, 0.0, 0.7071)

    save_frame_to_scene_updates(
        frame_name="view",
        position=new_pos,
        orientation=new_ori,
        parent_object_name="root",
        filepath=self.pbtxt_path,
    )

    updates = object_world_updates_pb2.ObjectWorldUpdates()
    with open(self.pbtxt_path, "r", encoding="utf-8") as f:
      content = f.read()
      text_format.Parse(content, updates)

    self.assertEqual(len(updates.updates), 1)
    cf = updates.updates[0].create_frame
    self.assertEqual(cf.new_frame_name, "view")
    self.assertAlmostEqual(cf.parent_t_new_frame.position.x, 0.36)
    self.assertAlmostEqual(cf.parent_t_new_frame.position.y, 0.40)
    self.assertAlmostEqual(cf.parent_t_new_frame.position.z, 0.88)
    self.assertAlmostEqual(cf.parent_t_new_frame.orientation.x, 0.7071)
    self.assertAlmostEqual(cf.parent_t_new_frame.orientation.w, 0.7071)

  def test_get_current_tool_pose(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_gripper = mock.MagicMock()
    mock_tool_frame = mock.MagicMock()

    setattr(mock_world, "root", mock_root)
    setattr(mock_world, "gripper", mock_gripper)
    setattr(mock_gripper, "tool_frame", mock_tool_frame)

    mock_pose = data_types.Pose3(
        data_types.Rotation3(data_types.Quaternion([0.0, 0.0, 0.7071, 0.7071])),
        [0.12, 0.34, 0.56],
    )
    mock_world.get_transform.return_value = mock_pose

    pos, ori = get_current_tool_pose(
        world=mock_world,
        parent_object_name="root",
        tool_object_name="gripper",
        tool_frame_name="tool_frame",
    )

    self.assertAlmostEqual(pos[0], 0.12)
    self.assertAlmostEqual(pos[1], 0.34)
    self.assertAlmostEqual(pos[2], 0.56)
    self.assertAlmostEqual(ori[0], 0.0)
    self.assertAlmostEqual(ori[1], 0.0)
    self.assertAlmostEqual(ori[2], 0.7071)
    self.assertAlmostEqual(ori[3], 0.7071)

  def test_update_live_world_frame_existing(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = ["view"]
    setattr(mock_world, "root", mock_root)
    setattr(mock_root, "view", mock.MagicMock())

    update_live_world_frame(
        world=mock_world,
        frame_name="view",
        position=(0.34, 0.37, 0.85),
        orientation=(1.0, 0.0, 0.0, 0.0),
        parent_object_name="root",
    )

    mock_world.update_transform.assert_called_once()

  def test_update_live_world_frame_new(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    setattr(mock_world, "root", mock_root)

    update_live_world_frame(
        world=mock_world,
        frame_name="new_frame",
        position=(0.15, 0.25, 0.71),
        orientation=(1.0, 0.0, 0.0, 0.0),
        parent_object_name="root",
    )

    mock_world.batch_update.assert_called_once()

  def test_parse_args(self):
    args = parse_args(["view", "--address", "localhost:17080"])
    self.assertEqual(args.name, "view")
    self.assertEqual(args.address, "localhost:17080")
    self.assertEqual(args.parent_object, "root")
    self.assertEqual(args.tool_object, "gripper")
    self.assertEqual(args.tool_frame, "tool_frame")


if __name__ == "__main__":
  absltest.main()
