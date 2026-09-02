"""Unit tests for calibrate_camera_to_robot helper functions."""

import os
import tempfile
from absl.testing import absltest
from google.protobuf import text_format

from intrinsic.perception.public.proto.v1 import camera_to_robot_calibration_pb2 as calibration_pb2
from intrinsic.world.public.proto import object_world_updates_pb2
from tools.calibration import calibrate_camera_to_robot


class CalibrateCameraToRobotTest(absltest.TestCase):

  def test_build_moving_camera_world_updates(self):
    res_pose = (
        calibration_pb2.CameraToRobotCalibrationResult.MovingCameraResultPoses()
    )
    res_pose.flange_t_camera.position.x = 0.025
    res_pose.flange_t_camera.position.y = 0.100
    res_pose.flange_t_camera.position.z = 0.050
    res_pose.flange_t_camera.orientation.x = 0.0
    res_pose.flange_t_camera.orientation.y = 0.0
    res_pose.flange_t_camera.orientation.z = 0.0
    res_pose.flange_t_camera.orientation.w = 1.0

    updates = calibrate_camera_to_robot.build_camera_world_updates(
        camera_name="orbbec_camera",
        moving_camera=True,
        moving_camera_poses=res_pose,
        robot_module_name="ur_module",
        robot_flange_frame="flange",
        reparent_camera=True,
    )

    self.assertIsInstance(updates, object_world_updates_pb2.ObjectWorldUpdates)
    self.assertEqual(len(updates.updates), 2)

    # Check update_transform
    ut = updates.updates[0].update_transform
    self.assertEqual(ut.node_a.by_name.frame.object_name, "ur_module")
    self.assertEqual(ut.node_a.by_name.frame.frame_name, "flange")
    self.assertEqual(ut.node_b.by_name.object.object_name, "orbbec_camera")
    self.assertEqual(
        ut.node_to_update.by_name.object.object_name, "orbbec_camera"
    )
    self.assertAlmostEqual(ut.a_t_b.position.x, 0.025)
    self.assertAlmostEqual(ut.a_t_b.position.y, 0.100)
    self.assertAlmostEqual(ut.a_t_b.position.z, 0.050)
    self.assertAlmostEqual(ut.a_t_b.orientation.w, 1.0)

    # Check reparent_object
    ro = updates.updates[1].reparent_object
    self.assertEqual(ro.object.by_name.object_name, "orbbec_camera")
    self.assertEqual(ro.new_parent.reference.by_name.object_name, "ur_module")
    self.assertTrue(ro.new_parent.entity_filter.include_final_entity)

  def test_build_stationary_camera_world_updates(self):
    res_pose = (
        calibration_pb2.CameraToRobotCalibrationResult.StationaryCameraResultPoses()
    )
    res_pose.base_t_camera.position.x = 1.200
    res_pose.base_t_camera.position.y = 0.500
    res_pose.base_t_camera.position.z = 0.800
    res_pose.base_t_camera.orientation.w = 1.0

    updates = calibrate_camera_to_robot.build_camera_world_updates(
        camera_name="orbbec_camera",
        moving_camera=False,
        stationary_camera_poses=res_pose,
        robot_module_name="ur_module",
        robot_flange_frame="flange",
        reparent_camera=False,
    )

    self.assertEqual(len(updates.updates), 1)
    ut = updates.updates[0].update_transform
    self.assertEqual(ut.node_a.by_name.object.object_name, "root")
    self.assertEqual(ut.node_b.by_name.object.object_name, "orbbec_camera")
    self.assertEqual(
        ut.node_to_update.by_name.object.object_name, "orbbec_camera"
    )
    self.assertAlmostEqual(ut.a_t_b.position.x, 1.200)

  def test_save_updates_to_file(self):
    updates = object_world_updates_pb2.ObjectWorldUpdates()
    up = updates.updates.add()
    up.update_transform.node_a.by_name.object.object_name = "root"
    up.update_transform.node_b.by_name.object.object_name = "test_cam"

    with tempfile.TemporaryDirectory() as tmpdir:
      filepath = os.path.join(tmpdir, "test_updates.pbtxt")
      calibrate_camera_to_robot.save_updates_to_file(updates, filepath)

      self.assertTrue(os.path.exists(filepath))
      with open(filepath, "r") as f:
        loaded = object_world_updates_pb2.ObjectWorldUpdates()
        text_format.Parse(f.read(), loaded)
        self.assertEqual(len(loaded.updates), 1)
        self.assertEqual(
            loaded.updates[0].update_transform.node_b.by_name.object.object_name,
            "test_cam",
        )


if __name__ == "__main__":
  absltest.main()
