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

"""Unit tests for tools/calibration/sample_calibration_poses.py."""

from unittest import mock

from absl.testing import absltest

from tools.calibration import sample_calibration_poses


class SampleCalibrationPosesTest(absltest.TestCase):
  """Tests ICON part discovery, camera streaming, and waypoint recording."""

  def test_resolve_icon_arm_part_skips_icon_and_non_arm_parts(self) -> None:
    mock_client = mock.MagicMock()
    # Suppose ICON returns ["arm", "adio", "ft_sensor"] and --robot="icon"
    mock_client.list_parts.return_value = ["arm", "adio", "ft_sensor"]

    adio_cfg = mock.MagicMock()
    adio_cfg.name = "adio"
    adio_cfg.HasField.return_value = False

    arm_cfg = mock.MagicMock()
    arm_cfg.name = "arm"
    arm_cfg.HasField.return_value = True
    arm_cfg.generic_config.joint_position_config.num_joints = 6

    mock_client.get_config.return_value.part_configs = [adio_cfg, arm_cfg]

    part_name, ndof = sample_calibration_poses.resolve_icon_arm_part(
      mock_client, requested_robot="icon"
    )
    self.assertEqual(part_name, "arm")
    self.assertEqual(ndof, 6)

    mock_client.list_parts.return_value = ["icon"]
    with self.assertRaisesRegex(ValueError, "No controllable parts found"):
      sample_calibration_poses.resolve_icon_arm_part(
        mock_client, requested_robot="icon"
      )

  def test_camera_streamer_and_manual_waypoint_recording(self) -> None:
    mock_camera = mock.MagicMock()
    mock_camera.capture.return_value = "frame_01"
    streamer = sample_calibration_poses.CameraStreamer(
      camera=mock_camera, fps=20.0
    )
    self.assertEqual(streamer.trigger_capture(), "frame_01")
    self.assertEqual(streamer.get_latest_capture(), "frame_01")

    # Test run_manual_waypoint_loop fallback when robot_ref.proto ("icon")
    # fails get_kinematic_object and falls back to ur_module
    mock_world = mock.MagicMock()
    mock_robot_ref = mock.MagicMock()

    def _get_kinematic(ref):
      if ref is mock_robot_ref.proto:
        raise ValueError("Resource 'icon' is not a kinematic object")
      kin = mock.MagicMock()
      kin.joint_positions = [0.1, -1.2, 1.4, -1.5, -1.5, 0.2]
      return kin

    mock_world.get_kinematic_object.side_effect = _get_kinematic
    recorded = []

    with mock.patch.object(
      sample_calibration_poses,
      "read_input",
      side_effect=["r", "s", "y", "y"],
    ):
      sample_calibration_poses.run_manual_waypoint_loop(
        world=mock_world,
        robot_ref=mock_robot_ref,
        manual_waypoints=recorded,
        session=None,
        ndof=None,
        part_name=None,
        icon_client=None,
      )

    self.assertLen(recorded, 1)
    self.assertAlmostEqual(recorded[0].joint_position.joints[0], 0.1)
    self.assertAlmostEqual(recorded[0].joint_position.joints[5], 0.2)


if __name__ == "__main__":
  absltest.main()
