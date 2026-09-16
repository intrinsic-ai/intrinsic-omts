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

"""Unit tests for Mock hardware adapters and interfaces."""

import dataclasses
import types
from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.core.types import JointPosition
from src.hardware.gripper import (
  DioGripper,
  GripperConfig,
  GripperInterface,
  MockGripper,
  RobotiqGripper,
)
from src.hardware.machine import (
  CncMachineInterface,
  DioCncMachine,
  MachineConfig,
  MockCncMachine,
  run_initial_machine_prep,
)
from src.hardware.robot import (
  MockRobot,
  MotionConfig,
  RobotInterface,
  UrRobot,
  normalize_motion_types,
)
from src.hardware.vision import (
  MockVision,
  OrbbecVision,
  PerceptionConfig,
  VisionInterface,
)
from src.utils import dynamic_frame_calculator


class NormalizeMotionTypesTest(absltest.TestCase):
  """Tests the per-segment motion type expansion."""

  def test_scalar_applies_to_every_segment(self):
    self.assertEqual(normalize_motion_types("ANY", 3), ["ANY", "ANY", "ANY"])

  def test_sequence_passes_through(self):
    self.assertEqual(
      normalize_motion_types(["LINEAR", "ANY"], 2), ["LINEAR", "ANY"]
    )

  def test_length_mismatch_raises(self):
    with self.assertRaises(ValueError):
      normalize_motion_types(["LINEAR"], 2)

  def test_blended_task_pins_each_segment(self):
    robot = MockRobot()
    robot.build_move_blended_cartesian_task(
      target_frames=[("root", "a"), ("root", "b")],
      motion_type=["LINEAR", "ANY"],
    )
    self.assertEqual(
      robot.executed_commands[-1],
      "move_blended_cartesian:root/a->root/b:LINEAR/ANY",
    )


class HardwareAdaptersTest(absltest.TestCase):
  def test_mock_robot(self):
    robot = MockRobot()
    robot.build_move_joint_task("home")
    robot.build_move_to_joint_position_task(
      JointPosition((0.0, -1.57, 1.57, -1.57, -1.57, 0.0))
    )
    robot.build_move_cartesian_task(target_frame_name="pre_grasp")
    robot.build_move_relative_cartesian_task(
      translation=(0.0, 0.0, -0.03), motion_type="LINEAR"
    )
    robot.build_move_to_contact_task(direction=(0.0, 0.0, 1.0))
    self.assertEqual(len(robot.executed_commands), 5)
    self.assertEqual(robot.executed_commands[0], "move_joint:home")
    self.assertEqual(
      robot.executed_commands[1],
      "move_joint:[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]",
    )
    self.assertEqual(
      robot.executed_commands[2],
      "move_cartesian:root/pre_grasp:ANY",
    )
    self.assertEqual(
      robot.executed_commands[3],
      "move_relative_cartesian:(0.0, 0.0, -0.03):LINEAR",
    )
    self.assertIn("move_to_contact", robot.executed_commands[4])

  def test_mock_gripper(self):
    gripper = MockGripper()
    gripper.build_open_task()
    gripper.build_close_task()
    self.assertEqual(gripper.command_log, ["open", "close"])

  def test_dio_gripper(self):
    mock_solution = mock.MagicMock()
    dio_set_mock = mock.MagicMock()
    dio_set_mock.return_value = bt.PythonScript(function_body="pass")
    mock_block_cls = mock.MagicMock()
    dio_set_mock.intrinsic_proto.skills.DioOutputBlock = mock_block_cls
    mock_solution.skills.ai.intrinsic.dio_set_output = dio_set_mock

    gripper = DioGripper(
      solution=mock_solution,
      open_pin=0,
      close_pin=1,
      output_block_name="standard_out",
    )

    open_task = gripper.build_open_task()
    self.assertEqual(open_task.name, "Open Gripper (DIO)")
    mock_block_cls.assert_called_with(
      block_name="standard_out", indices=[0], values=[True]
    )
    dio_set_mock.assert_called_with(
      dio_output_blocks=[mock_block_cls.return_value]
    )

    close_task = gripper.build_close_task()
    self.assertEqual(close_task.name, "Close Gripper (DIO)")
    mock_block_cls.assert_called_with(
      block_name="standard_out", indices=[1], values=[True]
    )
    dio_set_mock.assert_called_with(
      dio_output_blocks=[mock_block_cls.return_value]
    )

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
      position=[0.024],
    )
    gripper_cmd_mock.assert_called_with(
      command=mock_joint_state_cls.return_value,
      action_name="/gripper/gripper_action_controller/gripper_cmd",
    )

    close_task = gripper.build_close_task()
    self.assertEqual(close_task.name, "Close Robotiq Gripper")
    mock_joint_state_cls.assert_called_with(
      name=["robotiq_hande_left_finger_joint"],
      position=[0.01],
    )
    gripper_cmd_mock.assert_called_with(
      command=mock_joint_state_cls.return_value,
      action_name="/gripper/gripper_action_controller/gripper_cmd",
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
    machine.build_open_vise_task()
    machine.build_close_vise_task()
    machine.build_close_door_task()
    machine.build_trigger_cycle_task()
    machine.build_wait_cycle_complete_task()
    self.assertEqual(
      machine.command_log,
      [
        "open_door",
        "open_vise",
        "close_vise",
        "close_door",
        "trigger_cycle",
        "wait_cycle_complete:30",
      ],
    )

  def _mock_dio_solution(self):
    """Returns a solution whose dio skills and output block class are mocks."""
    solution = mock.MagicMock()
    dio_set = mock.MagicMock()
    dio_set.return_value = bt.PythonScript(function_body="pass")
    block_cls = mock.MagicMock()
    dio_set.intrinsic_proto.skills.DioOutputBlock = block_cls
    dio_read = mock.MagicMock()
    dio_read.return_value = bt.PythonScript(function_body="pass")
    solution.skills.ai.intrinsic.dio_set_output = dio_set
    solution.skills.ai.intrinsic.dio_read_input = dio_read
    solution.skills.ai.intrinsic.update_world.return_value = bt.PythonScript(
      function_body="pass"
    )
    return solution, dio_set, dio_read, block_cls

  def test_dio_cnc_machine(self):
    solution, _, dio_read, block_cls = self._mock_dio_solution()

    machine = DioCncMachine(solution=solution, config=MachineConfig())

    open_door_task = machine.build_open_door_task()
    self.assertEqual(open_door_task.name, "Open CNC Door (DIO)")
    block_cls.assert_called_with(
      block_name="standard_out", indices=[2, 3], values=[True, False]
    )

    close_door_task = machine.build_close_door_task()
    self.assertEqual(close_door_task.name, "Close CNC Door (DIO)")
    block_cls.assert_called_with(
      block_name="standard_out", indices=[2, 3], values=[False, True]
    )

    open_vise_task = machine.build_open_vise_task()
    self.assertEqual(open_vise_task.name, "Open CNC Vise (DIO)")
    block_cls.assert_called_with(
      block_name="standard_out", indices=[4, 5], values=[True, False]
    )

    close_vise_task = machine.build_close_vise_task()
    self.assertEqual(close_vise_task.name, "Clamp CNC Vise (DIO)")
    block_cls.assert_called_with(
      block_name="standard_out", indices=[4, 5], values=[False, True]
    )

    trigger_task = machine.build_trigger_cycle_task()
    self.assertEqual(trigger_task.name, "Trigger CNC Machining Cycle (DIO)")
    block_cls.assert_called_with(
      block_name="standard_out", indices=[6], values=[True]
    )

    wait_task = machine.build_wait_cycle_complete_task()
    self.assertEqual(wait_task.name, "Wait for CNC Cycle Complete")
    dio_read.assert_called_with(block_name="standard_in", timeout=30.0)

  def test_dio_cnc_machine_single_pin_and_device_name(self):
    solution, dio_set, dio_read, block_cls = self._mock_dio_solution()
    icon_resource = mock.MagicMock()
    solution.resources.icon = icon_resource
    single_pin = MachineConfig(door_close_pin=None, vise_close_pin=None)

    # "ur_module" pins live on the robot, so adio is omitted and SBL resolves
    # the device itself.
    machine_ur = DioCncMachine(
      solution=solution,
      config=dataclasses.replace(single_pin, device_name="ur_module"),
    )
    machine_ur.build_open_door_task()
    block_cls.assert_called_with(
      block_name="standard_out", indices=[2], values=[True]
    )
    dio_set.assert_called_with(
      dio_output_blocks=[block_cls.return_value],
    )

    machine_ur.build_close_door_task()
    block_cls.assert_called_with(
      block_name="standard_out", indices=[2], values=[False]
    )

    machine_ur.build_wait_cycle_complete_task(timeout_seconds=10.0)
    dio_read.assert_called_with(block_name="standard_in", timeout=10.0)

    # A named ADIO resource is passed through instead.
    machine_icon = DioCncMachine(
      solution=solution,
      config=dataclasses.replace(single_pin, device_name="icon"),
    )
    machine_icon.build_open_door_task()
    dio_set.assert_called_with(
      dio_output_blocks=[block_cls.return_value],
      adio=icon_resource,
    )

  def test_mock_vision(self):
    vision = MockVision()
    cap, cap_data = vision.build_capture_image_task()
    est, estimates = vision.build_estimate_pose_task(capture_data=cap_data)
    self.assertIsInstance(cap, bt.Node)
    self.assertIsInstance(est, bt.Node)
    self.assertEqual(vision.capture_count, 1)
    self.assertEqual(vision.pipeline_count, 1)
    self.assertIsNotNone(estimates)

  def test_mock_vision_records_retry_parameters(self):
    vision = MockVision()
    vision.build_capture_image_task(
      max_tries=5,
      retry_delay_sec=2.5,
    )
    self.assertEqual(vision.last_max_tries, 5)
    self.assertEqual(vision.last_retry_delay_sec, 2.5)

  def test_orbbec_vision_build_capture_with_retries(self):
    mock_solution = mock.MagicMock()
    mock_action = mock.MagicMock(spec=bt.ActionBase)
    mock_solution.skills.ai.intrinsic.capture_images.return_value = mock_action
    mock_solution.skills.ai.intrinsic.estimate_pose_multi_view.return_value = (
      mock_action
    )
    mock_solution.resources = {
      "orbbec_camera": mock.MagicMock(),
      "pose_estimator_service": mock.MagicMock(),
    }
    vision = OrbbecVision(
      solution=mock_solution,
      camera_name="orbbec_camera",
      perception_service_name="pose_estimator_service",
    )
    retry_node, cap_data = vision.build_capture_image_task(
      max_tries=3,
      retry_delay_sec=1.5,
    )
    self.assertIsInstance(retry_node, bt.Retry)
    self.assertEqual(retry_node.max_tries, 3)
    self.assertIn("Capture", retry_node.child.name)
    self.assertIsNotNone(retry_node.recovery)
    self.assertIn("Dwell", retry_node.recovery.name)
    est_node, _ = vision.build_estimate_pose_task(capture_data=cap_data)
    self.assertIsInstance(est_node, bt.Task)

  def test_orbbec_vision_build_capture_without_retries(self):
    mock_solution = mock.MagicMock()
    mock_action = mock.MagicMock(spec=bt.ActionBase)
    mock_solution.skills.ai.intrinsic.capture_images.return_value = mock_action
    mock_solution.skills.ai.intrinsic.estimate_pose_multi_view.return_value = (
      mock_action
    )
    mock_solution.resources = {
      "orbbec_camera": mock.MagicMock(),
      "pose_estimator_service": mock.MagicMock(),
    }
    vision = OrbbecVision(
      solution=mock_solution,
      camera_name="orbbec_camera",
      perception_service_name="pose_estimator_service",
    )
    cap_node, cap_data = vision.build_capture_image_task(max_tries=1)
    self.assertNotIsInstance(cap_node, bt.Retry)
    self.assertIsInstance(cap_node, bt.Task)
    est_node, _ = vision.build_estimate_pose_task(capture_data=cap_data)
    self.assertIsInstance(est_node, bt.Task)

  def test_ur_robot_build_tasks(self):
    mock_solution = mock.MagicMock()
    mock_move_robot = mock.MagicMock(
      return_value=bt.PythonScript(function_body="pass")
    )
    mock_solution.skills.ai.intrinsic.move_robot = mock_move_robot
    mock_solution.skills.ai.intrinsic.move_to_contact = mock.MagicMock(
      return_value=bt.PythonScript(function_body="pass")
    )
    robot = UrRobot(solution=mock_solution)
    joint_task = robot.build_move_joint_task("home")
    self.assertIsInstance(joint_task, bt.Task)
    joint_pos_task = robot.build_move_joint_task(
      [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    )
    self.assertIsInstance(joint_pos_task, bt.Task)
    joint_position_task = robot.build_move_to_joint_position_task(
      JointPosition((0.0, -1.57, 1.57, -1.57, -1.57, 0.0))
    )
    self.assertIsInstance(joint_position_task, bt.Task)
    cartesian_task = robot.build_move_cartesian_task("view")
    self.assertIsInstance(cartesian_task, bt.Task)
    contact_task = robot.build_move_to_contact_task()
    self.assertIsInstance(contact_task, bt.Task)

  def test_ur_robot_disable_collision_checking(self):
    mock_solution = mock.MagicMock()
    mock_move_robot = mock.MagicMock(
      return_value=bt.PythonScript(function_body="pass")
    )
    mock_solution.skills.ai.intrinsic.move_robot = mock_move_robot
    robot = UrRobot(solution=mock_solution, disable_collision_checking=True)
    joint_task = robot.build_move_joint_task("home")
    self.assertIsInstance(joint_task, bt.Task)
    mock_move_robot.intrinsic_proto.skills.MotionSegment.assert_called()
    call_kwargs = (
      mock_move_robot.intrinsic_proto.skills.MotionSegment.call_args.kwargs
    )
    self.assertIn("collision_settings", call_kwargs)

  def test_ur_robot_build_move_blended_cartesian_task(self):
    mock_solution = mock.MagicMock()
    mock_move_robot = mock.MagicMock(
      return_value=bt.PythonScript(function_body="pass")
    )
    mock_solution.skills.ai.intrinsic.move_robot = mock_move_robot
    robot = UrRobot(solution=mock_solution)
    task = robot.build_move_blended_cartesian_task(
      target_frames=[("root", "transit"), ("root", "machine_approach")],
      motion_type="ANY",
    )
    self.assertIsInstance(task, bt.Task)
    call_kwargs = mock_move_robot.call_args.kwargs
    self.assertIn("motion_segments", call_kwargs)
    self.assertLen(call_kwargs["motion_segments"], 2)

  def test_mock_robot_build_move_blended_cartesian_task(self):
    robot = MockRobot()
    task = robot.build_move_blended_cartesian_task(
      target_frames=[("root", "transit"), ("root", "machine_approach")],
      motion_type="ANY",
    )
    self.assertIsInstance(task, bt.Node)
    self.assertIn(
      "move_blended_cartesian:root/transit->root/machine_approach:ANY",
      robot.executed_commands,
    )

  def test_calculate_and_update_dynamic_frames_with_target_object_sync(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = ["infeed_pre_grasp", "infeed_grasp"]
    mock_pregrasp = mock.MagicMock()
    mock_grasp = mock.MagicMock()
    mock_root.infeed_pre_grasp = mock_pregrasp
    mock_root.infeed_grasp = mock_grasp
    mock_block = mock.MagicMock()
    mock_world.root = mock_root
    setattr(mock_world, "ai.intrinsic.raw_stock_2x3x5", mock_block)
    mock_world.get_transform.return_value = types.SimpleNamespace(
      position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
      rotation=types.SimpleNamespace(
        quaternion=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
      ),
    )

    context = mock.MagicMock()
    context.object_world = mock_world

    est = types.SimpleNamespace(
      score=-10.0,
      pose_t_target=types.SimpleNamespace(
        position=types.SimpleNamespace(x=0.15, y=0.25, z=0.71),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
      ),
    )
    params = types.SimpleNamespace(
      parent_object="root",
      camera_name="orbbec_camera",
      estimates=[est],
      approach_offset_z=0.08,
      pregrasp_frame_name="infeed_pre_grasp",
      grasp_frame_name="infeed_grasp",
      target_scene_object_id="ai.intrinsic.raw_stock_2x3x5",
    )

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      context, params
    )

    self.assertGreaterEqual(mock_world.update_transform.call_count, 3)
    block_updates = [
      call
      for call in mock_world.update_transform.call_args_list
      if call.kwargs.get("node_b") == mock_block
    ]
    self.assertEqual(len(block_updates), 1)

  def test_factories_mock_mode(self):
    mock_solution = mock.MagicMock()
    robot = RobotInterface.from_config(
      mock_solution, MotionConfig(), mock_hardware=True
    )
    self.assertIsInstance(robot, MockRobot)

    gripper = GripperInterface.from_config(
      mock_solution, GripperConfig(), mock_hardware=True
    )
    self.assertIsInstance(gripper, MockGripper)

    vision = VisionInterface.from_config(
      mock_solution, PerceptionConfig(), mock_hardware=True
    )
    self.assertIsInstance(vision, MockVision)

    machine = CncMachineInterface.from_config(
      mock_solution, MachineConfig(), mock_hardware=True
    )
    self.assertIsInstance(machine, MockCncMachine)

  def test_factories_live_mode(self):
    mock_solution = mock.MagicMock()
    robot = RobotInterface.from_config(
      mock_solution,
      MotionConfig(disable_collision_checking=False),
      mock_hardware=False,
    )
    self.assertIsInstance(robot, UrRobot)
    self.assertFalse(robot._disable_collision_checking)

    gripper = GripperInterface.from_config(
      mock_solution,
      GripperConfig(hardware_type="robotiq"),
      mock_hardware=False,
    )
    self.assertIsInstance(gripper, RobotiqGripper)
    self.assertEqual(
      gripper._action_name,
      "/gripper/gripper_action_controller/gripper_cmd",
    )

    machine_none = CncMachineInterface.from_config(
      mock_solution,
      MachineConfig(machine_type="none"),
      mock_hardware=False,
    )
    self.assertIsNone(machine_none)

    machine_dio = CncMachineInterface.from_config(
      mock_solution,
      MachineConfig(machine_type="dio"),
      mock_hardware=False,
    )
    self.assertIsInstance(machine_dio, DioCncMachine)

  def test_run_initial_machine_prep(self):
    mock_solution = mock.MagicMock()
    machine = MockCncMachine()
    run_initial_machine_prep(mock_solution, machine, close_door_and_vise=True)
    mock_solution.run.assert_called_once()
    tree = mock_solution.run.call_args[0][0]
    self.assertIsInstance(tree, bt.Sequence)
    self.assertLen(tree.children, 2)

  def test_run_initial_machine_prep_with_robot_retract(self):
    mock_solution = mock.MagicMock()
    machine = MockCncMachine()
    robot = MockRobot()
    run_initial_machine_prep(
      mock_solution,
      machine=machine,
      robot=robot,
      view_frame="view",
      close_door_and_vise=True,
    )
    mock_solution.run.assert_called_once()
    tree = mock_solution.run.call_args[0][0]
    self.assertIsInstance(tree, bt.Sequence)
    self.assertLen(tree.children, 3)
    # Child 0 must be arm retract to view before door close
    self.assertIn("Retract Arm to view", tree.children[0].name)
    self.assertIn("Close CNC Door", tree.children[1].name)
    self.assertIn("Close CNC Vise", tree.children[2].name)

  def test_run_initial_machine_prep_disabled(self):
    mock_solution = mock.MagicMock()
    machine = MockCncMachine()
    run_initial_machine_prep(mock_solution, machine, close_door_and_vise=False)
    mock_solution.run.assert_not_called()


if __name__ == "__main__":
  absltest.main()
