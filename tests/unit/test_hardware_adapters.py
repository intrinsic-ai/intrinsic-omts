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

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.hardware.gripper import DioGripper, MockGripper, RobotiqGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot
from src.hardware.vision import MockVision, OrbbecVision


class HardwareAdaptersTest(absltest.TestCase):
  def test_mock_robot(self):
    robot = MockRobot()
    robot.build_move_joint_task("home")
    robot.build_move_cartesian_task(target_frame_name="pre_grasp")
    robot.build_move_relative_cartesian_task(
      translation=(0.0, 0.0, -0.03), motion_type="LINEAR"
    )
    robot.build_move_to_contact_task(direction=(0.0, 0.0, 1.0))
    self.assertEqual(len(robot.executed_commands), 4)
    self.assertEqual(robot.executed_commands[0], "move_joint:home")
    self.assertEqual(
      robot.executed_commands[1],
      "move_cartesian:root/pre_grasp:ANY:z_rot=False",
    )
    self.assertEqual(
      robot.executed_commands[2],
      "move_relative_cartesian:(0.0, 0.0, -0.03):LINEAR",
    )
    self.assertIn("move_to_contact", robot.executed_commands[3])

  def test_mock_gripper(self):
    gripper = MockGripper()
    gripper.build_open_task()
    self.assertEqual(gripper.state, "open")
    gripper.build_close_task()
    self.assertEqual(gripper.state, "closed")
    self.assertEqual(gripper.command_log, ["open", "close"])

  def test_dio_gripper(self):
    mock_solution = mock.MagicMock()
    dio_set_mock = mock.MagicMock()
    dio_set_mock.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.dio_set_output = dio_set_mock

    gripper = DioGripper(
      solution=mock_solution,
      open_pin=0,
      close_pin=1,
      device_name="ur_module",
    )

    open_task = gripper.build_open_task()
    self.assertEqual(open_task.name, "Open Gripper (DIO)")
    dio_set_mock.assert_called_with(pin=0, state=True, device_name="ur_module")

    close_task = gripper.build_close_task()
    self.assertEqual(close_task.name, "Close Gripper (DIO)")
    dio_set_mock.assert_called_with(pin=1, state=True, device_name="ur_module")

  def test_robotiq_gripper_defaults(self):
    mock_solution = mock.MagicMock()
    gripper_cmd_mock = mock.MagicMock()
    gripper_cmd_mock.return_value = bt.PythonScript(function_body="pass")
    mock_joint_state_cls = mock.MagicMock()
    gripper_cmd_mock.ai.intrinsic.JointState = mock_joint_state_cls
    mock_solution.skills.ai.intrinsic.gripper_cmd_skill = gripper_cmd_mock

    gripper = RobotiqGripper(solution=mock_solution)

    open_task = gripper.build_open_task()
    self.assertEqual(open_task.name, "Open Robotiq Gripper")
    mock_joint_state_cls.assert_called_with(
      name=["robotiq_hande_left_finger_joint"],
      position=[0.025],
    )
    gripper_cmd_mock.assert_called_with(
      command=mock_joint_state_cls.return_value
    )

    close_task = gripper.build_close_task()
    self.assertEqual(close_task.name, "Close Robotiq Gripper")
    mock_joint_state_cls.assert_called_with(
      name=["robotiq_hande_left_finger_joint"],
      position=[0.0],
    )
    gripper_cmd_mock.assert_called_with(
      command=mock_joint_state_cls.return_value
    )

  def test_robotiq_gripper_custom_params(self):
    mock_solution = mock.MagicMock()
    gripper_cmd_mock = mock.MagicMock()
    gripper_cmd_mock.return_value = bt.PythonScript(function_body="pass")
    mock_joint_state_cls = mock.MagicMock()
    gripper_cmd_mock.ai.intrinsic.JointState = mock_joint_state_cls
    mock_solution.skills.ai.intrinsic.gripper_cmd_skill = gripper_cmd_mock

    gripper = RobotiqGripper(
      solution=mock_solution,
      joint_name="custom_finger_joint",
      open_position=0.005,
      close_position=0.020,
      action_name="/custom/action",
    )

    open_task = gripper.build_open_task(name="Custom Open")
    self.assertEqual(open_task.name, "Custom Open")
    mock_joint_state_cls.assert_called_with(
      name=["custom_finger_joint"],
      position=[0.005],
    )
    gripper_cmd_mock.assert_called_with(
      command=mock_joint_state_cls.return_value,
      action_name="/custom/action",
    )

  def test_mock_cnc_machine(self):
    machine = MockCncMachine()
    machine.build_open_door_task()
    self.assertTrue(machine.door_open)
    machine.build_open_vise_task()
    self.assertTrue(machine.vise_open)
    machine.build_close_vise_task()
    self.assertFalse(machine.vise_open)
    machine.build_close_door_task()
    self.assertFalse(machine.door_open)
    machine.build_trigger_cycle_task()
    self.assertTrue(machine.cycle_triggered)
    machine.build_wait_cycle_complete_task()
    self.assertIn("wait_cycle_complete", machine.command_log)

  def test_mock_vision(self):
    vision = MockVision()
    vision.build_capture_image_task()
    self.assertEqual(vision.capture_count, 1)
    vision.build_perception_and_spawn_task()
    self.assertEqual(vision.pipeline_count, 1)

  def test_orbbec_vision_build_perception_and_spawn_task(self):
    mock_solution = mock.MagicMock()
    mock_camera_resource = mock.MagicMock()
    mock_camera_resource.types = ["CameraConfig"]
    mock_perception_resource = mock.MagicMock()
    mock_perception_resource.types = [
      "intrinsic_proto.perception.v1.PoseEstimationService"
    ]
    mock_solution.resources = {
      "orbbec_camera": mock_camera_resource,
      "pose_estimator_service": mock_perception_resource,
    }
    mock_capture_action = mock.MagicMock(spec=bt.ActionBase)
    mock_capture_action.proto = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.capture_images.return_value = (
      mock_capture_action
    )
    mock_estimate_action = mock.MagicMock(spec=bt.ActionBase)
    mock_estimate_action.proto = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.estimate_pose_multi_view.return_value = (
      mock_estimate_action
    )
    vision = OrbbecVision(solution=mock_solution)
    task = vision.build_perception_and_spawn_task()
    self.assertIsInstance(task, bt.Sequence)
    self.assertEqual(len(task.children), 3)
    self.assertEqual(task.children[0].name, "1. Capture RGB-D Images")
    self.assertEqual(task.children[1].name, "2. Estimate 6D Workpiece Poses")
    self.assertEqual(
      task.children[2].name,
      "3. Calculate & Update Dynamic Grasp & Pre-Grasp Frames",
    )


if __name__ == "__main__":
  absltest.main()
