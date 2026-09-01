"""Unit tests for Mock hardware adapters and interfaces."""

from absl.testing import absltest
from src.hardware.gripper import MockGripper
from src.hardware.machine import MockCncMachine
from src.hardware.robot import MockRobot
from src.hardware.vision import MockVision


class HardwareAdaptersTest(absltest.TestCase):

  def test_mock_robot(self):
    robot = MockRobot()
    robot.build_move_joint_task("home")
    robot.build_move_to_contact_task(direction=(0.0, 0.0, 1.0))
    self.assertEqual(len(robot.executed_commands), 2)
    self.assertEqual(robot.executed_commands[0], "move_joint:home")
    self.assertIn("move_to_contact", robot.executed_commands[1])

  def test_mock_gripper(self):
    gripper = MockGripper()
    gripper.build_open_task()
    self.assertEqual(gripper.state, "open")
    gripper.build_close_task()
    self.assertEqual(gripper.state, "closed")
    self.assertEqual(gripper.command_log, ["open", "close"])

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


if __name__ == "__main__":
  absltest.main()
