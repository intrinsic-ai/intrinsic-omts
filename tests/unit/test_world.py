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

"""Unit tests for World facade and MockWorld in src/core/world.py."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.core.types import JointPosition
from src.core.world import MockWorld, World


class MockWorldTest(absltest.TestCase):
  def test_object_registration_and_lifecycle(self):
    fake = MockWorld()
    obj = mock.MagicMock(spec=["name"])
    obj.name = "block"
    fake.add_object("block", obj)

    self.assertEqual(fake.get_object("block"), obj)
    self.assertIn(obj, fake.list_objects())
    self.assertEqual(fake.block, obj)

    fake.delete_object(obj)
    self.assertIsNone(fake.get_object("block"))
    self.assertNotIn(obj, fake.list_objects())
    self.assertFalse(hasattr(fake, "block"))

  def test_transforms_and_joints(self):
    fake = MockWorld()
    fake.update_transform(
      "root", "tool", ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    )
    self.assertIsNotNone(fake.get_transform("root", "tool"))
    self.assertIsNone(fake.get_transform("root", "other"))

    fake.update_joint_positions("door", [0.4])
    self.assertEqual(fake.joint_positions["door"], [0.4])

    fake.reparent_object("stock", "gripper")
    self.assertEqual(fake.reparented, [("stock", "gripper")])


class WorldFacadeTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.fake_world = MockWorld()
    self.world = World(self.fake_world)

  def test_find_object_exact_and_alias(self):
    obj = mock.MagicMock(spec=["name"])
    obj.name = "raw_stock_2x3x5"
    self.fake_world.add_object("ai.intrinsic.raw_stock_2x3x5", obj)

    self.assertEqual(
      self.world.find_object("ai.intrinsic.raw_stock_2x3x5"), obj
    )
    self.assertEqual(self.world.find_object("raw_stock_2x3x5"), obj)

  def test_find_object_not_found(self):
    with self.assertRaises(ValueError):
      self.world.find_object("nonexistent")

  def test_build_attach_and_detach_tasks_use_native_skills(self):
    mock_sol = mock.MagicMock()
    mock_sol.skills.ai.intrinsic.attach_object_to_robot.return_value = (
      bt.PythonScript(function_body="pass")
    )
    mock_sol.skills.ai.intrinsic.detach_object.return_value = bt.PythonScript(
      function_body="pass"
    )
    world = World(self.fake_world, solution=mock_sol)
    self.fake_world.add_object("gripper", mock.MagicMock(name="gripper_obj"))
    self.fake_world.add_object("raw_stock", mock.MagicMock(name="stock_obj"))

    attach_task = world.build_attach_to_gripper_task(
      object_name="raw_stock", gripper_name="gripper"
    )
    self.assertIsInstance(attach_task, bt.Task)
    mock_sol.skills.ai.intrinsic.attach_object_to_robot.assert_called_once()

    detach_task = world.build_detach_from_gripper_task(
      object_name="raw_stock", gripper_name="gripper"
    )
    self.assertIsInstance(detach_task, bt.Task)
    mock_sol.skills.ai.intrinsic.detach_object.assert_called_once()

  def test_build_update_grasp_frames_task_signature(self):
    mock_sol = mock.MagicMock()
    world = World(self.fake_world, solution=mock_sol)
    task = world.build_update_grasp_frames_task(estimates="mock_estimates")
    self.assertIsInstance(task, bt.Task)
    mock_builder = mock_sol.proto_builder.create_signature_with_args
    mock_builder.assert_called_once()
    fields = mock_builder.call_args.kwargs["parameters"].fields
    field_dict = {f.name: f for f in fields}
    self.assertEqual(
      {name: f.number for name, f in field_dict.items()},
      {
        "approach_offset_z": 1,
        "parent_object": 2,
        "pregrasp_frame_name": 3,
        "grasp_frame_name": 4,
        "camera_name": 5,
        "target_scene_object_id": 6,
        "estimates": 7,
        "min_safe_z": 8,
      },
    )

  def test_build_joint_update_task_uses_update_world_skill(self):
    mock_sol = mock.MagicMock()
    mock_sol.skills.ai.intrinsic.update_world.return_value = bt.PythonScript(
      function_body="pass"
    )
    world = World(self.fake_world, solution=mock_sol)
    self.fake_world.add_object("cnc_enclosure", mock.MagicMock())
    self.fake_world.add_object("schunk_egp_64nnb", mock.MagicMock())

    door_task = world.build_joint_update_task(
      object_name="cnc_enclosure", joints=JointPosition(positions=[0.4])
    )
    self.assertIsInstance(door_task, bt.Task)
    mock_sol.skills.ai.intrinsic.update_world.assert_called()

    vise_task = world.build_joint_update_task(
      object_name="schunk_egp_64nnb", joints=[0.01, 0.01]
    )
    self.assertIsInstance(vise_task, bt.Task)

  def test_clear_stale_frames(self):
    raw_mock = mock.MagicMock()
    world = World(raw_mock)
    world.clear_stale_frames(["infeed_grasp", "infeed_pre_grasp"])
    self.assertEqual(raw_mock.delete_frame.call_count, 2)

  def test_reset_world_state(self):
    mock_robot = mock.MagicMock()
    mock_solution = mock.MagicMock()
    self.world.reset(
      robot=mock_robot,
      solution=mock_solution,
      workpiece_name="raw_stock_2x3x5",
    )
    mock_robot.clear_faults.assert_not_called()

    self.world.reset(
      robot=mock_robot,
      solution=mock_solution,
      workpiece_name="raw_stock_2x3x5",
      clear_faults=True,
    )
    mock_robot.clear_faults.assert_called_once()


if __name__ == "__main__":
  absltest.main()
