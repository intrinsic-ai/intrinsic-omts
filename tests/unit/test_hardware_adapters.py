"""Unit tests for Mock hardware adapters and interfaces."""

from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt
from src.hardware.gripper import DioGripper
from src.hardware.gripper import MockGripper
from src.hardware.gripper import RobotiqGripper
from src.hardware.gripper import SideloadedGripperCmd
from src.hardware.machine import DioCncMachine
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot
from src.hardware.robot import UrRobot
from src.hardware.vision import MockVision
from src.hardware.vision import OrbbecVision


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

  def test_dio_cnc_machine(self):
    mock_solution = mock.MagicMock()
    dio_set_mock = mock.MagicMock()
    dio_set_mock.return_value = bt.PythonScript(function_body="pass")
    mock_block_cls = mock.MagicMock()
    dio_set_mock.intrinsic_proto.skills.DioOutputBlock = mock_block_cls
    dio_read_mock = mock.MagicMock()
    dio_read_mock.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.dio_set_output = dio_set_mock
    mock_solution.skills.ai.intrinsic.dio_read_input = dio_read_mock

    machine = DioCncMachine(
        solution=mock_solution,
        door_open_pin=2,
        door_close_pin=3,
        vise_open_pin=4,
        vise_close_pin=5,
        cycle_start_pin=6,
        cycle_done_input_pin=0,
        output_block_name="standard_out",
        input_block_name="standard_in",
    )

    open_door_task = machine.build_open_door_task()
    self.assertEqual(open_door_task.name, "Open CNC Door (DIO)")
    mock_block_cls.assert_called_with(
        block_name="standard_out", indices=[2, 3], values=[True, False]
    )

    close_door_task = machine.build_close_door_task()
    self.assertEqual(close_door_task.name, "Close CNC Door (DIO)")
    mock_block_cls.assert_called_with(
        block_name="standard_out", indices=[2, 3], values=[False, True]
    )

    open_vise_task = machine.build_open_vise_task()
    self.assertEqual(open_vise_task.name, "Open CNC Vise (DIO)")
    mock_block_cls.assert_called_with(
        block_name="standard_out", indices=[4, 5], values=[True, False]
    )

    close_vise_task = machine.build_close_vise_task()
    self.assertEqual(close_vise_task.name, "Clamp CNC Vise (DIO)")
    mock_block_cls.assert_called_with(
        block_name="standard_out", indices=[4, 5], values=[False, True]
    )

    trigger_task = machine.build_trigger_cycle_task()
    self.assertEqual(
        trigger_task.name, "Trigger CNC Machining Cycle (DIO)"
    )
    mock_block_cls.assert_called_with(
        block_name="standard_out", indices=[6], values=[True]
    )

    wait_task = machine.build_wait_cycle_complete_task()
    self.assertEqual(wait_task.name, "Wait for CNC Cycle Complete")
    dio_read_mock.assert_called_with(
        block_name="standard_in", timeout=30.0
    )

  def test_dio_cnc_machine_single_pin_and_device_name(self):
    mock_solution = mock.MagicMock()
    dio_set_mock = mock.MagicMock()
    dio_set_mock.return_value = bt.PythonScript(function_body="pass")
    mock_block_cls = mock.MagicMock()
    dio_set_mock.intrinsic_proto.skills.DioOutputBlock = mock_block_cls
    dio_read_mock = mock.MagicMock()
    dio_read_mock.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.dio_set_output = dio_set_mock
    mock_solution.skills.ai.intrinsic.dio_read_input = dio_read_mock
    mock_icon_resource = mock.MagicMock()
    mock_solution.resources.icon = mock_icon_resource

    # When device_name is "ur_module", adio should be omitted to let SBL auto-resolve
    machine_ur = DioCncMachine(
        solution=mock_solution,
        door_open_pin=2,
        door_close_pin=None,
        vise_open_pin=4,
        vise_close_pin=None,
        device_name="ur_module",
    )
    machine_ur.build_open_door_task()
    mock_block_cls.assert_called_with(
        block_name="standard_out", indices=[2], values=[True]
    )
    dio_set_mock.assert_called_with(
        dio_output_blocks=[mock_block_cls.return_value],
    )

    machine_ur.build_close_door_task()
    mock_block_cls.assert_called_with(
        block_name="standard_out", indices=[2], values=[False]
    )

    machine_ur.build_wait_cycle_complete_task(timeout_seconds=10.0)
    dio_read_mock.assert_called_with(
        block_name="standard_in", timeout=10.0
    )

    # When device_name is an ADIO resource like "icon", adio should be passed
    machine_icon = DioCncMachine(
        solution=mock_solution,
        door_open_pin=2,
        door_close_pin=None,
        vise_open_pin=4,
        vise_close_pin=None,
        device_name="icon",
    )
    machine_icon.build_open_door_task()
    dio_set_mock.assert_called_with(
        dio_output_blocks=[mock_block_cls.return_value],
        adio=mock_icon_resource,
    )

  def test_mock_vision(self):
    vision = MockVision()
    vision.build_capture_image_task()
    self.assertEqual(vision.capture_count, 1)
    vision.build_perception_and_spawn_task()
    self.assertEqual(vision.pipeline_count, 1)

  def test_mock_vision_estimate_and_update_pose(self):
    vision = MockVision()
    task = vision.build_estimate_and_update_pose_task(
        target_object="ai.intrinsic.raw_stock_2x3x5",
        pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator",
    )
    self.assertIsInstance(task, bt.Node)
    self.assertEqual(vision.pipeline_count, 1)

  def test_orbbec_vision_estimate_and_update_pose(self):
    mock_solution = mock.MagicMock()
    mock_cam = mock.MagicMock()
    mock_svc = mock.MagicMock()
    mock_solution.resources = {
        "orbbec_camera": mock_cam,
        "pose_estimator_service": mock_svc,
    }
    mock_estimate_skill = mock.MagicMock(
        return_value=bt.PythonScript(function_body="pass")
    )
    mock_solution.skills.ai.intrinsic.estimate_and_update_pose = (
        mock_estimate_skill
    )

    vision = OrbbecVision(
        solution=mock_solution,
        camera_name="orbbec_camera",
        perception_service_name="pose_estimator_service",
    )
    task = vision.build_estimate_and_update_pose_task(
        target_object="ai.intrinsic.raw_stock_2x3x5",
        pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator",
        name="Custom Estimate Task",
    )
    self.assertIsInstance(task, bt.Task)
    self.assertEqual(task.name, "Custom Estimate Task")
    mock_estimate_skill.assert_called_once()
    call_kwargs = mock_estimate_skill.call_args.kwargs
    self.assertEqual(call_kwargs.get("camera"), mock_cam)
    self.assertEqual(call_kwargs.get("perception"), mock_svc)
    self.assertEqual(call_kwargs.get("object"), "ai.intrinsic.raw_stock_2x3x5")

  def test_sideloaded_gripper_cmd_tasks(self):
    mock_solution = mock.MagicMock()
    mock_cmd_skill = mock.MagicMock(
        return_value=bt.PythonScript(function_body="pass")
    )
    mock_joint_state = mock.MagicMock()
    mock_cmd_skill.ai.intrinsic.JointState.return_value = mock_joint_state
    mock_solution.skills.ai.intrinsic.gripper_cmd_skill = mock_cmd_skill

    gripper = SideloadedGripperCmd(
        solution=mock_solution,
        action_name="/gripper/gripper_action_controller/gripper_cmd",
        joint_name="robotiq_hande_left_finger_joint",
        open_position=0.025,
        close_position=0.000,
    )

    open_task = gripper.build_open_task()
    self.assertIsInstance(open_task, bt.Task)
    self.assertEqual(open_task.name, "Open Gripper (gripper_cmd)")
    self.assertEqual(mock_joint_state.name, ["robotiq_hande_left_finger_joint"])
    self.assertEqual(mock_joint_state.position, [0.025])
    mock_cmd_skill.assert_called_with(
        action_name="/gripper/gripper_action_controller/gripper_cmd",
        command=mock_joint_state,
    )

    close_task = gripper.build_close_task()
    self.assertIsInstance(close_task, bt.Task)
    self.assertEqual(close_task.name, "Close Gripper (gripper_cmd)")
    self.assertEqual(mock_joint_state.position, [0.000])
    mock_cmd_skill.assert_called_with(
        action_name="/gripper/gripper_action_controller/gripper_cmd",
        command=mock_joint_state,
    )

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

  def test_ur_robot_settling_timeout(self):
    mock_solution = mock.MagicMock()
    mock_move_robot = mock.MagicMock(
        return_value=bt.PythonScript(function_body="pass")
    )
    mock_solution.skills.ai.intrinsic.move_robot = mock_move_robot
    robot = UrRobot(
        solution=mock_solution,
        default_settling_timeout_seconds=10.0,
    )
    # Default settling timeout passed to skill
    robot.build_move_joint_task("home")
    exec_params = mock_move_robot.intrinsic_proto.skills.ExecutionParameters
    exec_params.assert_called_with(settling_timeout_seconds=10.0)
    call_kwargs = mock_move_robot.call_args.kwargs
    self.assertIn("execution_parameters", call_kwargs)

    # Override settling timeout per call
    robot.build_move_cartesian_task("view", settling_timeout_seconds=15.0)
    exec_params.assert_called_with(settling_timeout_seconds=15.0)
    call_kwargs_cart = mock_move_robot.call_args.kwargs
    self.assertIn("execution_parameters", call_kwargs_cart)


if __name__ == "__main__":
  absltest.main()
