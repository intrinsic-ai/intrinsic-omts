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

"""Unit tests for the move_to_frame developer CLI tool."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import execution
from tools.jogging.move_to_frame import list_available_frames
from tools.jogging.move_to_frame import move_robot_to_frame
from tools.jogging.move_to_frame import parse_args
from tools.jogging.move_to_frame import prompt_for_frame


class MoveToFrameTest(absltest.TestCase):

  def test_list_available_frames_with_list_frames(self):
    mock_world = mock.MagicMock()
    mock_world.root.list_frames.return_value = ["view", "grasp"]
    mock_world.list_objects.return_value = ["root", "ur_module", "ksp3_160_vise"]

    mock_vise = mock.MagicMock()
    mock_vise.list_frames.return_value = ["approach_vise", "clamped_vise"]
    setattr(mock_world, "ksp3_160_vise", mock_vise)

    frames = list_available_frames(mock_world)
    expected = [
        ("root", "view"),
        ("root", "grasp"),
        ("ksp3_160_vise", "approach_vise"),
        ("ksp3_160_vise", "clamped_vise"),
    ]
    self.assertEqual(frames, expected)

  def test_list_available_frames_fallback(self):
    mock_world = mock.MagicMock(spec=[])

    frames = list_available_frames(mock_world)
    self.assertIn(("root", "view"), frames)
    self.assertIn(("root", "grasp"), frames)
    self.assertIn(("root", "machine_approach"), frames)

  def test_prompt_for_frame_valid_selection(self):
    available_frames = [("root", "view"), ("root", "grasp")]
    with mock.patch("builtins.input", side_effect=["2"]):
      selected = prompt_for_frame(available_frames)
      self.assertEqual(selected, ("root", "grasp"))

  def test_prompt_for_frame_retry_then_valid(self):
    available_frames = [("root", "view"), ("root", "grasp")]
    with mock.patch("builtins.input", side_effect=["invalid", "99", "1"]):
      selected = prompt_for_frame(available_frames)
      self.assertEqual(selected, ("root", "view"))

  def test_prompt_for_frame_quit(self):
    available_frames = [("root", "view"), ("root", "grasp")]
    with mock.patch("builtins.input", return_value="q"):
      selected = prompt_for_frame(available_frames)
      self.assertIsNone(selected)

  def test_move_robot_to_frame_success(self):
    mock_solution = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.move_robot.return_value = (
        bt.PythonScript(function_body="pass")
    )
    mock_solution.executive.run = mock.MagicMock()

    move_robot_to_frame(
        solution=mock_solution,
        target_frame_name="view",
        target_object_name="root",
        motion_type="ANY",
    )

    mock_solution.executive.run.assert_called_once()

  def test_move_robot_to_frame_execution_error_handled(self):
    mock_solution = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.move_robot.return_value = (
        bt.PythonScript(function_body="pass")
    )
    mock_solution.executive.run.side_effect = execution.ExecutionFailedError(
        "Trajectory planning failed"
    )

    # Should handle ExecutionFailedError without crashing
    move_robot_to_frame(
        solution=mock_solution,
        target_frame_name="invalid_frame",
        target_object_name="root",
        motion_type="LINEAR",
    )

  def test_parse_args_defaults(self):
    args = parse_args([])
    self.assertEqual(args.address, "localhost:17080")
    self.assertIsNone(args.frame)
    self.assertEqual(args.parent_object, "root")
    self.assertEqual(args.motion_type, "ANY")
    self.assertEqual(args.arm_part_name, "ur_module")
    self.assertEqual(args.tool_object_name, "gripper")
    self.assertEqual(args.tool_frame_name, "tool_frame")

  def test_parse_args_custom_values(self):
    args = parse_args([
        "--address",
        "192.168.1.50:17080",
        "--frame",
        "view",
        "--parent_object",
        "root",
        "--motion_type",
        "LINEAR",
        "--arm_part_name",
        "robot_arm",
        "--tool_object_name",
        "custom_gripper",
        "--tool_frame_name",
        "custom_tcp",
    ])
    self.assertEqual(args.address, "192.168.1.50:17080")
    self.assertEqual(args.frame, "view")
    self.assertEqual(args.parent_object, "root")
    self.assertEqual(args.motion_type, "LINEAR")
    self.assertEqual(args.arm_part_name, "robot_arm")
    self.assertEqual(args.tool_object_name, "custom_gripper")
    self.assertEqual(args.tool_frame_name, "custom_tcp")


if __name__ == "__main__":
  absltest.main()
