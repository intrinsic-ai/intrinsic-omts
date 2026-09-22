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

"""Parses every checked-in world update config as the proto it claims to be.

A missing brace in configs/omts/scene.updates.pbtxt survived three commits
because nothing in the build ever parsed the file. The application and
//tools/world:apply_scene_updates consume these at runtime on the cell,
which is a poor place to discover a syntax error.
"""

import glob
import math
import os

from absl.testing import absltest
from google.protobuf import any_pb2, text_format
from intrinsic.assets.services.proto import service_manifest_pb2
from intrinsic.icon.control.parts.hal.adio_part import (
  hal_adio_part_config_pb2,
)
from intrinsic.icon.control.parts.hal.arm_part import (
  hal_arm_part_config_pb2,
)
from intrinsic.icon.control.parts.hal.force_torque_sensor_part import (
  hal_force_torque_sensor_part_config_pb2,
)
from intrinsic.icon.hal.proto import hardware_module_config_pb2
from intrinsic.icon.hardware_modules.universal_robots import (
  config_pb2 as ur_module_config_pb2,
)
from intrinsic.icon.server.config import icon_main_config_pb2
from intrinsic.perception.proto.v1 import camera_config_pb2
from intrinsic.perception.skills.calibration import sample_calibration_poses_pb2
from intrinsic.world.proto import object_world_updates_pb2
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.proto import (
  ioc_service_config_pb2,
)

from src.core.config import load_app_config
from tools.world.apply_scene_updates import DEFAULT_UPDATE_FILES

# Ensure all google.protobuf.Any payload descriptors are registered in the pool.
_REGISTERED_ANY_DESCRIPTORS = (
  hal_adio_part_config_pb2.HalADIOPartConfig.DESCRIPTOR,
  hal_arm_part_config_pb2.HalArmPartConfig.DESCRIPTOR,
  hal_force_torque_sensor_part_config_pb2.HalForceTorqueSensorPartConfig.DESCRIPTOR,
  hardware_module_config_pb2.HardwareModuleConfig.DESCRIPTOR,
  ur_module_config_pb2.UniversalRobotsModuleConfig.DESCRIPTOR,
  icon_main_config_pb2.IconMainConfig.DESCRIPTOR,
  camera_config_pb2.CameraConfig.DESCRIPTOR,
  ioc_service_config_pb2.IocPoseEstimatorServiceConfig.DESCRIPTOR,
)

# Every world update config is expected to be an ObjectWorldUpdates. Configs
# that are not world updates, such as calibration_waypoints.pbtxt, are excluded
# by the naming convention rather than by an explicit list. The leading "*" is
# the robot cell directory, or "common".
_UPDATES_GLOB = "configs/*/*.updates.pbtxt"
_TEXTPROTO_GLOB = "configs/*/*.textproto"

# The frame names are a published interface: the application and
# tools/jogging/move_to_frame.py refer to them by name. lab_bb_01 has no
# vise, so it declares no vise_* frames.
_EXPECTED_SCENE_FRAMES = {
  "configs/omts/scene.updates.pbtxt": [
    "vise_pre_place",
    "vise_place",
    "machine_approach",
    "work_view",
    "view",
    "transit",
    "vise_view",
  ],
  "configs/lab_bb_01/scene.updates.pbtxt": [
    "grasp",
    "pre_grasp",
    "pre_place_vise",
    "place_vise",
    "machine_approach",
    "view",
  ],
}


def _config_paths() -> list[str]:
  """Returns the world update configs, resolved from the runfiles root."""
  return sorted(glob.glob(_UPDATES_GLOB))


def _textproto_paths() -> list[str]:
  """Returns all .textproto service configs and manifests under configs/."""
  return sorted(glob.glob(_TEXTPROTO_GLOB))


def _load_updates_proto(
  path: str,
) -> object_world_updates_pb2.ObjectWorldUpdates | None:
  """Loads and parses an ObjectWorldUpdates proto from the given path."""
  if not os.path.exists(path):
    return None
  with open(path, encoding="utf-8") as f:
    return text_format.Parse(
      f.read(), object_world_updates_pb2.ObjectWorldUpdates()
    )


class WorldUpdateConfigsTest(absltest.TestCase):
  def test_configs_are_present(self):
    # Guards against the glob silently matching nothing, which would make
    # every other assertion in this file vacuous.
    self.assertNotEmpty(
      _config_paths(),
      f"No configs matched {_UPDATES_GLOB} from {os.getcwd()}.",
    )

  def test_every_config_parses_as_object_world_updates(self):
    for path in _config_paths():
      with self.subTest(path=path):
        with open(path, encoding="utf-8") as f:
          contents = f.read()
        updates = object_world_updates_pb2.ObjectWorldUpdates()
        text_format.Parse(contents, updates)
        self.assertNotEmpty(
          updates.updates, f"{path} parsed but declares no updates."
        )

  def test_scene_declares_the_expected_frames(self):
    for path, expected in _EXPECTED_SCENE_FRAMES.items():
      with self.subTest(path=path):
        updates = _load_updates_proto(path)
        self.assertIsNotNone(updates, f"{path} is missing from {os.getcwd()}.")
        created = [
          update.create_frame.new_frame_name
          for update in updates.updates
          if update.HasField("create_frame")
        ]
        self.assertCountEqual(created, expected)

  def test_ur_module_initial_joint_positions(self):
    expected_joint0 = {
      "configs/omts/ur_module.attachments.updates.pbtxt": 3.140,
      "configs/lab_bb_01/ur_module.attachments.updates.pbtxt": 1.5707,
    }
    for path, expected_j0 in expected_joint0.items():
      with self.subTest(path=path):
        updates = _load_updates_proto(path)
        self.assertIsNotNone(updates, f"{path} is missing from {os.getcwd()}.")
        joint_updates = [
          update.update_object_joints
          for update in updates.updates
          if update.HasField("update_object_joints")
          and update.update_object_joints.object.by_name.object_name
          == "ur_module"
        ]
        self.assertLen(joint_updates, 1)
        self.assertAlmostEqual(joint_updates[0].joint_positions[0], expected_j0)

  def test_app_config_frames_exist_in_cell_scene_updates(self):
    """Every static frame referenced by app_config.yaml must exist in scene.updates.pbtxt."""
    for cell in ("omts", "lab_bb_01"):
      with self.subTest(cell=cell):
        app_cfg = load_app_config(f"configs/{cell}/app_config.yaml")
        scene_updates = _load_updates_proto(
          f"configs/{cell}/scene.updates.pbtxt"
        )
        self.assertIsNotNone(scene_updates)
        scene_frames = {
          update.create_frame.new_frame_name
          for update in scene_updates.updates
          if update.HasField("create_frame")
        }
        required_static_frames = [
          app_cfg.frames.view_frame,
          app_cfg.frames.machine_approach_frame,
          app_cfg.frames.preplace_vise_frame,
          app_cfg.frames.place_vise_frame,
        ]
        if app_cfg.frames.transit_frame:
          required_static_frames.append(app_cfg.frames.transit_frame)
        for frame_name in required_static_frames:
          self.assertIn(
            frame_name,
            scene_frames,
            f"Frame '{frame_name}' in configs/{cell}/app_config.yaml is not "
            f"defined in configs/{cell}/scene.updates.pbtxt",
          )

  def test_apply_scene_updates_default_files_exist(self):
    """Every default update file in apply_scene_updates must exist and parse."""
    for path in DEFAULT_UPDATE_FILES:
      with self.subTest(path=path):
        self.assertTrue(os.path.exists(path), f"Missing default file: {path}")
        self.assertIsNotNone(_load_updates_proto(path))

  def test_calibration_waypoints_parses_with_valid_6dof_joints(self):
    """Validates configs/omts/calibration_waypoints.pbtxt schema and joint vectors."""
    path = "configs/omts/calibration_waypoints.pbtxt"
    self.assertTrue(os.path.exists(path))
    result = sample_calibration_poses_pb2.SampleCalibrationPosesResult()
    with open(path, encoding="utf-8") as f:
      text_format.Parse(f.read(), result)
    self.assertGreaterEqual(len(result.sample_calibration_poses_result), 3)
    for idx, wp in enumerate(result.sample_calibration_poses_result):
      self.assertTrue(
        wp.HasField("joint_position"), f"Waypoint {idx} missing joint_position"
      )
      self.assertLen(wp.joint_position.joints, 6)
      for angle in wp.joint_position.joints:
        self.assertTrue(math.isfinite(angle))

  def test_every_textproto_service_config_and_manifest_parses(self):
    """Validates all checked-in .textproto service configs and manifests against their proto schemas."""
    textproto_files = _textproto_paths()
    self.assertNotEmpty(textproto_files)

    for path in textproto_files:
      with self.subTest(path=path):
        with open(path, encoding="utf-8") as f:
          contents = f.read()

        if path.endswith("_manifest.textproto"):
          manifest = service_manifest_pb2.ServiceManifest()
          text_format.Parse(contents, manifest)
          self.assertTrue(manifest.metadata.id.package)
          self.assertTrue(manifest.metadata.id.name)
        else:
          # Service configs are google.protobuf.Any textprotos with unpacked [type.googleapis.com/...]
          any_msg = any_pb2.Any()
          text_format.Parse(contents, any_msg)
          self.assertTrue(any_msg.type_url, f"{path} produced empty type_url")
          self.assertTrue(
            any_msg.value, f"{path} produced empty serialized payload"
          )

          if path.endswith("icon_config.textproto"):
            icon_cfg = icon_main_config_pb2.IconMainConfig()
            self.assertTrue(any_msg.Unpack(icon_cfg))
            self.assertGreater(icon_cfg.control_frequency_hz, 0)
            self.assertIn("arm", icon_cfg.realtime_control_config.parts_by_name)
          elif path.endswith("ur_module_config.textproto"):
            hw_cfg = hardware_module_config_pb2.HardwareModuleConfig()
            self.assertTrue(any_msg.Unpack(hw_cfg))
            ur_cfg = ur_module_config_pb2.UniversalRobotsModuleConfig()
            self.assertTrue(hw_cfg.module_config.Unpack(ur_cfg))
            self.assertTrue(ur_cfg.robot_ip)
          elif path.endswith("gemini_device_config.textproto"):
            cam_cfg = camera_config_pb2.CameraConfig()
            self.assertTrue(any_msg.Unpack(cam_cfg))
            self.assertEqual(cam_cfg.identifier.ros.driver_type, "orbbec")
          elif path.endswith("pose_estimator_config.textproto"):
            pe_cfg = ioc_service_config_pb2.IocPoseEstimatorServiceConfig()
            self.assertTrue(any_msg.Unpack(pe_cfg))
            self.assertEqual(pe_cfg.inference_service.name, "inference_service")
          else:
            self.fail(f"Unhandled .textproto config file: {path}")


if __name__ == "__main__":
  absltest.main()
