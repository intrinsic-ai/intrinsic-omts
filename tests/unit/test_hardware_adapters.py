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

"""Unit tests for stateless hardware adapters (UrRobot, Grippers, DioCncMachine, OrbbecVision)."""

import inspect
from unittest import mock

from absl.testing import absltest
from intrinsic.solutions import behavior_tree as bt

from src.core.config import (
  GripperConfig,
  MachineConfig,
  RobotConfig,
  VisionConfig,
)
from src.core.types import JointPosition
from src.hardware.gripper import DioGripper, RobotiqGripper
from src.hardware.machine import DioCncMachine
from src.hardware.robot import UrRobot
from src.hardware.vision import OrbbecVision


class HardwareAdaptersTest(absltest.TestCase):
  def test_ur_robot_has_no_disable_collision_checking(self):
    sig = inspect.signature(UrRobot.__init__)
    self.assertNotIn("disable_collision_checking", sig.parameters)

  def test_ur_robot_motion_and_object_attachment(self):
    mock_solution = mock.MagicMock()
    mock_move_robot = mock.MagicMock()
    mock_move_robot.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.move_robot = mock_move_robot
    mock_move_to_contact = mock.MagicMock()
    mock_move_to_contact.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.move_to_contact = mock_move_to_contact
    mock_attach = mock.MagicMock()
    mock_attach.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.attach_object_to_robot = mock_attach
    mock_detach = mock.MagicMock()
    mock_detach.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.detach_object = mock_detach

    robot_config = RobotConfig(
      arm_part_name="ur_module",
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
    )
    robot = UrRobot(
      solution=mock_solution,
      config=robot_config,
    )

    # Joint move by named configuration
    joint_task = robot.build_move_joint_task("home")
    self.assertIsInstance(joint_task, bt.Task)
    self.assertEqual(joint_task.name, "Move to home")

    # Joint move by JointPosition
    jp_task = robot.build_move_to_joint_position_task(
      JointPosition((0.0, -1.57, 1.57, 0.0, 0.0, 0.0))
    )
    self.assertIsInstance(jp_task, bt.Task)

    # Cartesian move
    cart_task = robot.build_move_cartesian_task(
      target_frame_name="view",
      target_object_name="root",
      motion_type="ANY",
    )
    self.assertIsInstance(cart_task, bt.Task)
    self.assertEqual(cart_task.name, "Move to root/view (ANY)")
    self.assertNotIn(
      "collision_settings",
      mock_move_robot.intrinsic_proto.skills.MotionSegment.call_args.kwargs,
    )

    # Cartesian move with excluded_collision_pairs
    mock_move_robot.intrinsic_proto.skills.MotionSegment.reset_mock()
    cart_exclude_task = robot.build_move_cartesian_task(
      target_frame_name="pre_place",
      target_object_name="root",
      motion_type="LINEAR",
      excluded_collision_pairs=[("raw_stock_2x3x5", "schunk_egp_64nnb")],
    )
    self.assertIsInstance(cart_exclude_task, bt.Task)
    cart_segment_kwargs = (
      mock_move_robot.intrinsic_proto.skills.MotionSegment.call_args.kwargs
    )
    cart_collision_settings = cart_segment_kwargs["collision_settings"]
    self.assertFalse(cart_collision_settings.disable_collision_checking)
    self.assertLen(cart_collision_settings.collision_rules, 1)
    cart_rule = cart_collision_settings.collision_rules[0]
    self.assertEqual(
      cart_rule.left[0].object.by_name.object_name, "raw_stock_2x3x5"
    )
    self.assertEqual(
      cart_rule.right[0].object.by_name.object_name, "schunk_egp_64nnb"
    )
    self.assertTrue(cart_rule.collision_action.is_excluded)

    # Blended Cartesian move
    blended_task = robot.build_move_blended_cartesian_task(
      target_frames=[("root", "transit"), ("root", "machine_approach")],
      motion_type=["ANY", "LINEAR"],
    )
    self.assertIsInstance(blended_task, bt.Task)
    self.assertIn("ANY/LINEAR", blended_task.name)

    # Relative Cartesian move
    rel_task = robot.build_move_relative_cartesian_task(
      translation=(0.0, 0.0, -0.03), motion_type="LINEAR"
    )
    self.assertIsInstance(rel_task, bt.Task)
    self.assertNotIn(
      "collision_settings",
      mock_move_robot.intrinsic_proto.skills.MotionSegment.call_args.kwargs,
    )

    # Relative Cartesian move with collision exclusion
    mock_move_robot.intrinsic_proto.skills.MotionSegment.reset_mock()
    rel_exclude_task = robot.build_move_relative_cartesian_task(
      translation=(0.0, 0.0, -0.05),
      motion_type="LINEAR",
      exclude_collision=True,
      excluded_collision_objects=["raw_stock_2x3x5"],
    )
    self.assertIsInstance(rel_exclude_task, bt.Task)
    segment_kwargs = (
      mock_move_robot.intrinsic_proto.skills.MotionSegment.call_args.kwargs
    )
    collision_settings = segment_kwargs["collision_settings"]
    self.assertFalse(collision_settings.disable_collision_checking)
    self.assertLen(collision_settings.collision_rules, 1)
    rule = collision_settings.collision_rules[0]
    self.assertTrue(rule.collision_action.is_excluded)
    self.assertEqual(
      [ref.object.by_name.object_name for ref in rule.left],
      ["gripper", "raw_stock_2x3x5"],
    )
    self.assertEmpty(rule.right)

    # Move to contact
    contact_task = robot.build_move_to_contact_task(
      direction=(0.0, 0.0, 1.0),
      contact_force_newtons=15.0,
      timeout_seconds=15.0,
    )
    self.assertIsInstance(contact_task, bt.Task)

    # Attach object to robot
    attach_task = robot.build_attach_object_task("raw_stock_2x3x5")
    self.assertIsInstance(attach_task, bt.Task)
    mock_attach.assert_called_once()
    attach_kwargs = mock_attach.call_args.kwargs
    self.assertEqual(
      attach_kwargs["gripper_entity"].by_name.object_name, "gripper"
    )
    self.assertEqual(
      attach_kwargs["object_entity"].by_name.object_name, "raw_stock_2x3x5"
    )

    # Detach object from robot
    detach_task = robot.build_detach_object_task("raw_stock_2x3x5")
    self.assertIsInstance(detach_task, bt.Task)
    mock_detach.assert_called_once()
    detach_kwargs = mock_detach.call_args.kwargs
    self.assertEqual(
      detach_kwargs["gripper_entity"].by_name.object_name, "gripper"
    )
    self.assertEqual(
      detach_kwargs["object_entity"].by_name.object_name, "raw_stock_2x3x5"
    )

  def test_dio_gripper(self):
    mock_solution = mock.MagicMock()
    dio_set_mock = mock.MagicMock()
    dio_set_mock.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.dio_set_output = dio_set_mock

    gripper_config = GripperConfig(
      type="dio",
      dio_open_pin=0,
      dio_close_pin=1,
      dio_device_name="ur_module",
      dio_output_block_name="standard_out",
    )
    gripper = DioGripper(
      solution=mock_solution,
      config=gripper_config,
    )

    open_task = gripper.build_open_task()
    self.assertEqual(open_task.name, "Open Gripper (DIO)")
    dio_set_mock.assert_called()

    close_task = gripper.build_close_task()
    self.assertEqual(close_task.name, "Close Gripper (DIO)")

  def test_robotiq_gripper_explicit_params(self):
    mock_solution = mock.MagicMock()
    gripper_cmd_mock = mock.MagicMock()
    gripper_cmd_mock.return_value = bt.PythonScript(function_body="pass")
    mock_joint_state_cls = mock.MagicMock()
    gripper_cmd_mock.ai.intrinsic.JointState = mock_joint_state_cls
    mock_solution.skills.ai.intrinsic.gripper_cmd_skill = gripper_cmd_mock

    gripper_config = GripperConfig(
      type="robotiq",
      joint_name="robotiq_hande_left_finger_joint",
      open_position=0.025,
      close_position=0.0,
    )
    gripper = RobotiqGripper(
      solution=mock_solution,
      config=gripper_config,
    )

    open_task = gripper.build_open_task()
    self.assertEqual(open_task.name, "Open Robotiq Gripper")
    mock_joint_state_cls.assert_called_with(
      name=["robotiq_hande_left_finger_joint"],
      position=[0.025],
    )

    close_task = gripper.build_close_task()
    self.assertEqual(close_task.name, "Close Robotiq Gripper")
    mock_joint_state_cls.assert_called_with(
      name=["robotiq_hande_left_finger_joint"],
      position=[0.0],
    )

  def test_robotiq_gripper_custom_params(self):
    mock_solution = mock.MagicMock()
    gripper_cmd_mock = mock.MagicMock()
    gripper_cmd_mock.return_value = bt.PythonScript(function_body="pass")
    mock_joint_state_cls = mock.MagicMock()
    gripper_cmd_mock.ai.intrinsic.JointState = mock_joint_state_cls
    mock_solution.skills.ai.intrinsic.gripper_cmd_skill = gripper_cmd_mock

    gripper_config = GripperConfig(
      type="robotiq",
      joint_name="custom_finger_joint",
      open_position=0.005,
      close_position=0.020,
      action_name="/custom/action",
    )
    gripper = RobotiqGripper(
      solution=mock_solution,
      config=gripper_config,
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

  def test_dio_cnc_machine_with_world_joint_sync(self):
    mock_solution = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.dio_set_output.return_value = (
      bt.PythonScript(function_body="pass")
    )
    mock_dio_read = mock.MagicMock()
    mock_dio_read.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.dio_read_input = mock_dio_read
    mock_solution.skills.ai.intrinsic.dio_wait_for_input = None
    mock_update_world = mock.MagicMock()
    mock_update_world.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.update_world = mock_update_world

    machine_config = MachineConfig(
      door_open_pin=2,
      door_close_pin=3,
      vise_open_pin=4,
      vise_close_pin=5,
      cycle_start_pin=6,
      cycle_complete_input_pin=0,
      device_name="ur_module",
      enclosure_object_name="cnc_enclosure",
      vise_object_name="schunk_egp_64nnb",
      door_open_joints=(0.4,),
      door_closed_joints=(0.0,),
      vise_open_joints=(0.01, 0.01),
      vise_closed_joints=(0.0, 0.0),
      output_block_name="standard_out",
      input_block_name="standard_in",
    )
    machine = DioCncMachine(
      solution=mock_solution,
      config=machine_config,
    )

    open_door_task = machine.build_open_door_task()
    self.assertIsInstance(open_door_task, bt.Sequence)
    self.assertEqual(len(open_door_task.children), 3)
    self.assertEqual(
      open_door_task.children[1].name, "Wait for CNC Door Open (10.0s)"
    )
    mock_update_world.assert_called()

    close_door_task = machine.build_close_door_task()
    self.assertIsInstance(close_door_task, bt.Sequence)
    self.assertEqual(len(close_door_task.children), 3)
    self.assertEqual(
      close_door_task.children[1].name, "Wait for CNC Door Closed (10.0s)"
    )

    close_vise_task = machine.build_close_vise_task()
    self.assertIsInstance(close_vise_task, bt.Sequence)
    self.assertEqual(len(close_vise_task.children), 2)

    trigger_task = machine.build_trigger_cycle_task()
    self.assertIsInstance(trigger_task, bt.Sequence)
    self.assertEqual(len(trigger_task.children), 3)

    wait_task = machine.build_wait_cycle_complete_task(timeout_seconds=12.5)
    self.assertIsInstance(wait_task, bt.Sequence)
    self.assertEqual(len(wait_task.children), 2)
    mock_dio_read.assert_called_once_with(
      block_name="standard_in", adio=mock_solution.resources["ur_module"]
    )

    # Verify fallback to timed dwell when cycle_complete_input_pin is None
    no_pin_config = MachineConfig(
      door_open_pin=2,
      door_close_pin=3,
      vise_open_pin=4,
      vise_close_pin=5,
      cycle_start_pin=6,
      cycle_complete_input_pin=None,
      device_name="ur_module",
      enclosure_object_name=None,
      vise_object_name=None,
      door_open_joints=(0.4,),
      door_closed_joints=(0.0,),
      vise_open_joints=(0.01, 0.01),
      vise_closed_joints=(0.0, 0.0),
      output_block_name="standard_out",
      input_block_name="standard_in",
    )
    machine_no_pin = DioCncMachine(
      solution=mock_solution,
      config=no_pin_config,
    )
    dwell_only_task = machine_no_pin.build_wait_cycle_complete_task(
      timeout_seconds=10.0
    )
    self.assertIsInstance(dwell_only_task, bt.Task)

  def test_dio_cnc_machine_skips_update_world_when_object_missing_in_world(
    self,
  ):
    mock_solution = mock.MagicMock()
    mock_solution.world.list_object_names.return_value = [
      "root",
      "ur_module",
      "gripper",
      "raw_stock_2x3x5",
    ]
    mock_solution.skills.ai.intrinsic.dio_set_output.return_value = (
      bt.PythonScript(function_body="pass")
    )
    mock_update_world = mock.MagicMock()
    mock_update_world.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.update_world = mock_update_world

    machine_config = MachineConfig(
      door_open_pin=2,
      door_close_pin=3,
      vise_open_pin=4,
      vise_close_pin=5,
      cycle_start_pin=6,
      cycle_complete_input_pin=None,
      device_name="ur_module",
      enclosure_object_name="cnc_enclosure",
      vise_object_name="schunk_egp_64nnb",
      door_open_joints=(0.4,),
      door_closed_joints=(0.0,),
      vise_open_joints=(0.01, 0.01),
      vise_closed_joints=(0.0, 0.0),
      output_block_name="standard_out",
      input_block_name="standard_in",
    )
    machine = DioCncMachine(
      solution=mock_solution,
      config=machine_config,
    )

    open_door_task = machine.build_open_door_task()
    self.assertIsInstance(open_door_task, bt.Sequence)
    self.assertEqual(len(open_door_task.children), 2)
    self.assertEqual(
      open_door_task.children[1].name, "Wait for CNC Door Open (10.0s)"
    )
    mock_update_world.assert_not_called()

    open_vise_task = machine.build_open_vise_task()
    self.assertIsInstance(open_vise_task, bt.Task)
    mock_update_world.assert_not_called()

  def test_ur_robot_skips_collision_rules_for_missing_world_objects(self):
    mock_solution = mock.MagicMock()
    mock_solution.world.list_object_names.return_value = [
      "root",
      "ur_module",
      "gripper",
      "raw_stock_2x3x5",
    ]
    mock_move_robot = mock.MagicMock()
    mock_move_robot.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.move_robot = mock_move_robot

    robot = UrRobot(
      solution=mock_solution,
      config=RobotConfig(
        arm_part_name="ur_module",
        tool_object_name="gripper",
        tool_frame_name="tool_frame",
      ),
    )

    cart_task = robot.build_move_cartesian_task(
      target_frame_name="machine_approach",
      target_object_name="root",
      motion_type="ANY",
      excluded_collision_pairs=[
        ("raw_stock_2x3x5", "schunk_egp_64nnb"),
        ("gripper", "schunk_egp_64nnb"),
        ("gripper", "raw_stock_2x3x5"),
      ],
    )
    self.assertIsInstance(cart_task, bt.Task)
    cart_segment_kwargs = (
      mock_move_robot.intrinsic_proto.skills.MotionSegment.call_args.kwargs
    )
    cart_collision_settings = cart_segment_kwargs["collision_settings"]
    self.assertFalse(cart_collision_settings.disable_collision_checking)
    self.assertLen(cart_collision_settings.collision_rules, 1)
    cart_rule = cart_collision_settings.collision_rules[0]
    self.assertEqual(cart_rule.left[0].object.by_name.object_name, "gripper")
    self.assertEqual(
      cart_rule.right[0].object.by_name.object_name, "raw_stock_2x3x5"
    )

  def test_orbbec_vision_atomic_retryable_perception(self):
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
    vision_config = VisionConfig(
      camera_name="orbbec_camera",
      perception_service_name="pose_estimator_service",
      pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator",
      scene_object_id="ai.intrinsic.raw_stock_2x3x5",
      sensor_ids=(1, 4),
      min_num_instances=1,
      infeed_mode="perception",
      min_safe_z=0.95,
    )
    vision = OrbbecVision(
      solution=mock_solution,
      config=vision_config,
    )
    task = vision.build_perception_and_spawn_task(
      approach_offset_z=0.08,
      parent_object="root",
      pregrasp_frame_name="pre_grasp",
      grasp_frame_name="grasp",
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
      max_tries=3,
    )
    self.assertIsInstance(task, bt.Retry)
    inner_seq = task.child
    self.assertIsInstance(inner_seq, bt.Sequence)
    self.assertEqual(len(inner_seq.children), 3)
    self.assertEqual(inner_seq.children[0].name, "1. Capture RGB-D Images")
    self.assertEqual(
      inner_seq.children[1].name, "2. Estimate 6D Workpiece Poses"
    )
    self.assertEqual(
      inner_seq.children[2].name,
      "3. Calculate & Update Dynamic Grasp & Pre-Grasp Frames",
    )

  def test_orbbec_vision_signature_cleanups(self):
    sig = inspect.signature(OrbbecVision.build_perception_and_spawn_task)
    self.assertNotIn("target_scene_object_id", sig.parameters)
    self.assertNotIn("pose_estimator_id", sig.parameters)
    self.assertNotIn("min_num_instances", sig.parameters)
    self.assertNotIn("min_safe_z", sig.parameters)
    self.assertIn("tool_object_name", sig.parameters)
    self.assertIn("tool_frame_name", sig.parameters)

  def test_dynamic_frame_calculator_min_safe_z_and_camera_validation(self):
    from src.utils.dynamic_frame_calculator import (
      calculate_and_update_dynamic_frames,
    )

    mock_context = mock.MagicMock()
    mock_context.object_world = mock.MagicMock()
    mock_context.object_world.get_transform.return_value = None

    params = mock.MagicMock()
    params.parent_object = "root"
    params.camera_name = "orbbec_camera"
    params.pos_x = 0.3
    params.pos_y = -0.2
    params.pos_z = 0.6657
    params.ori_x = 0.0
    params.ori_y = 0.0
    params.ori_z = 0.0
    params.ori_w = 1.0
    params.approach_offset_z = 0.08
    params.pregrasp_frame_name = "pre_grasp"
    params.grasp_frame_name = "grasp"
    params.target_scene_object_id = "ai.intrinsic.raw_stock_2x3x5"
    params.tool_object_name = "gripper"
    params.tool_frame_name = "tool_frame"

    # OMTS threshold (0.95m) rejects 0.6657m
    params.min_safe_z = 0.95
    with self.assertRaisesRegex(
      ValueError, r"Calculated workpiece target Z \(0\.6657m\) is below"
    ):
      calculate_and_update_dynamic_frames(mock_context, params)

    # Lab BB-01 threshold (0.60m) accepts 0.6657m
    params.min_safe_z = 0.60
    calculate_and_update_dynamic_frames(mock_context, params)

    # Missing camera object raises ValueError directly
    mock_context.object_world.orbbec_camera = None
    with self.assertRaisesRegex(
      ValueError, r"Camera object 'orbbec_camera' not found in world\."
    ):
      calculate_and_update_dynamic_frames(mock_context, params)

  def test_resolve_adio_resource_with_strict_resources_container(self):
    from src.utils.math_utils import resolve_adio_resource

    class _FakeResourceHandle:
      def __init__(self, types: list[str]) -> None:
        self.types = types

    class _StrictResources:
      def __init__(self, items: dict[str, object]) -> None:
        self._resources = items

      def __getitem__(self, name: str) -> object:
        return self._resources[name]

      def __getattr__(self, name: str) -> object:
        return self._resources[name]

    fake_ur_handle = _FakeResourceHandle(
      ["intrinsic_proto.world.RobotCalibrationDataService"]
    )
    fake_adio_handle = _FakeResourceHandle(["Icon2AdioPart"])
    strict_resources = _StrictResources(
      {
        "ur_module": fake_ur_handle,
        "adio_device": fake_adio_handle,
      }
    )
    mock_solution = mock.MagicMock()
    mock_solution.resources = strict_resources

    # Note: hasattr(strict_resources, "__contains__") raises KeyError!
    with self.assertRaises(KeyError):
      hasattr(strict_resources, "__contains__")

    # Non-ADIO resource handle (RobotCalibrationDataService) must return None
    self.assertIsNone(resolve_adio_resource(mock_solution, "ur_module"))
    # ADIO-capable resource handle (Icon2AdioPart) is returned
    self.assertIs(
      resolve_adio_resource(mock_solution, "adio_device"),
      fake_adio_handle,
    )
    self.assertIsNone(resolve_adio_resource(mock_solution, "missing_device"))

  def test_ur_robot_rotation_cone_and_blended_validation(self):
    mock_solution = mock.MagicMock()
    mock_move_robot = mock.MagicMock()
    mock_move_robot.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.move_robot = mock_move_robot

    robot = UrRobot(
      solution=mock_solution,
      config=RobotConfig(
        arm_part_name="ur_module",
        tool_object_name="gripper",
        tool_frame_name="tool_frame",
      ),
    )

    cone_task = robot.build_move_cartesian_task(
      target_frame_name="grasp",
      target_object_name="root",
      motion_type="ANY",
      allow_tool_z_rotation=True,
      cone_opening_half_angle=0.15,
      moving_frame_offset=(0.0, 0.0, 0.01),
      target_frame_offset=((0.0, 0.0, 0.02), (0.0, 0.0, 0.0, 1.0)),
    )
    self.assertIsInstance(cone_task, bt.Task)
    segment_kwargs = (
      mock_move_robot.intrinsic_proto.skills.MotionSegment.call_args.kwargs
    )
    self.assertIn("constraint_intersection", segment_kwargs)
    constraints = segment_kwargs["constraint_intersection"].constraints
    self.assertLen(constraints, 2)
    self.assertAlmostEqual(
      constraints[0].position_equality.moving_frame_offset.z, 0.01
    )
    self.assertAlmostEqual(
      constraints[0].position_equality.target_frame_offset.z, 0.02
    )
    self.assertAlmostEqual(
      constraints[1].rotation_cone.cone_opening_half_angle, 0.15
    )

    with self.assertRaisesRegex(
      ValueError, "target_frames must contain at least one target frame"
    ):
      robot.build_move_blended_cartesian_task(target_frames=[])

    with self.assertRaisesRegex(ValueError, "motion_type has 1 entries"):
      robot.build_move_blended_cartesian_task(
        target_frames=[("root", "transit"), ("root", "view")],
        motion_type=["ANY"],
      )

  def test_dio_cnc_machine_wait_for_input_skill_and_fallback(self):
    mock_solution = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.dio_set_output.return_value = (
      bt.PythonScript(function_body="pass")
    )
    mock_wait_input = mock.MagicMock()
    mock_wait_input.return_value = bt.PythonScript(function_body="pass")
    mock_solution.skills.ai.intrinsic.dio_wait_for_input = mock_wait_input

    cfg = MachineConfig(
      door_open_pin=2,
      door_close_pin=3,
      vise_open_pin=4,
      vise_close_pin=5,
      cycle_start_pin=6,
      cycle_complete_input_pin=1,
      device_name="ur_module",
      enclosure_object_name=None,
      vise_object_name=None,
      door_open_joints=(0.4,),
      door_closed_joints=(0.0,),
      vise_open_joints=(0.01, 0.01),
      vise_closed_joints=(0.0, 0.0),
      output_block_name="standard_out",
      input_block_name="standard_in",
    )
    machine = DioCncMachine(solution=mock_solution, config=cfg)
    wait_task = machine.build_wait_cycle_complete_task(timeout_seconds=25.0)
    self.assertIsInstance(wait_task, bt.Task)
    mock_wait_input.assert_called_once()
    self.assertEqual(mock_wait_input.call_args.kwargs["indices"], [1])
    self.assertEqual(mock_wait_input.call_args.kwargs["timeout"], 25.0)

    # Fallback dwell when both dio_wait_for_input and dio_read_input are None
    mock_solution.skills.ai.intrinsic.dio_wait_for_input = None
    mock_solution.skills.ai.intrinsic.dio_read_input = None
    machine_fallback = DioCncMachine(solution=mock_solution, config=cfg)
    fb_task = machine_fallback.build_wait_cycle_complete_task(
      timeout_seconds=5.0
    )
    self.assertIsInstance(fb_task, bt.Task)
    self.assertIn("Fallback Dwell 5.0s", fb_task.name)

  def test_orbbec_vision_proto_builder_fields_and_missing_resources(self):
    class _RealProtoBuilder:
      def __init__(self) -> None:
        self.last_parameters = None

      def create_signature_with_args(self, parameters):
        self.last_parameters = parameters
        return None

    mock_solution = mock.MagicMock()
    real_pb = _RealProtoBuilder()
    mock_solution.proto_builder = real_pb
    mock_solution.resources = {
      "orbbec_camera": mock.MagicMock(types=["CameraConfig"]),
      "pose_estimator_service": mock.MagicMock(
        types=["intrinsic_proto.perception.v1.PoseEstimationService"]
      ),
    }
    mock_capture_action = mock.MagicMock(spec=bt.ActionBase)
    mock_capture_action.proto = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.capture_images.return_value = (
      mock_capture_action
    )
    mock_est_action = mock.MagicMock(spec=bt.ActionBase)
    mock_est_action.proto = mock.MagicMock()
    mock_solution.skills.ai.intrinsic.estimate_pose_multi_view.return_value = (
      mock_est_action
    )

    vision_cfg = VisionConfig(
      camera_name="orbbec_camera",
      perception_service_name="pose_estimator_service",
      pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator",
      scene_object_id="ai.intrinsic.raw_stock_2x3x5",
      sensor_ids=(1, 4),
      min_num_instances=1,
      infeed_mode="perception",
      min_safe_z=0.95,
    )
    vision = OrbbecVision(solution=mock_solution, config=vision_cfg)
    seq_task = vision.build_perception_and_spawn_task(
      approach_offset_z=0.08,
      parent_object="root",
      pregrasp_frame_name="pre_grasp",
      grasp_frame_name="grasp",
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
      max_tries=1,
    )
    # max_tries=1 returns bt.Sequence directly without bt.Retry wrapper
    self.assertIsInstance(seq_task, bt.Sequence)
    self.assertIsNotNone(real_pb.last_parameters)
    self.assertLen(real_pb.last_parameters.fields, 16)

    # Missing resources raise ValueError
    mock_solution.resources = {}
    with self.assertRaisesRegex(ValueError, "Camera resource 'orbbec_camera'"):
      OrbbecVision(solution=mock_solution, config=vision_cfg)


if __name__ == "__main__":
  absltest.main()
