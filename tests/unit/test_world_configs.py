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
from intrinsic.math.python import data_types
from intrinsic.perception.proto.v1 import camera_config_pb2
from intrinsic.perception.skills.calibration import sample_calibration_poses_pb2
from intrinsic.world.proto import object_world_updates_pb2

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
    "grasp",
    "pre_grasp",
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


# UR5e and UR3e kinematic joint chains matching @intrinsic-core SceneObjects:
# (inboard_xyz, inboard_quat_xyzw, revolute_axis_xyz)
_UR_JOINT_CHAINS: dict[
  str,
  tuple[
    tuple[
      tuple[float, float, float],
      tuple[float, float, float, float],
      tuple[float, float, float],
    ],
    ...,
  ],
] = {
  "omts": (
    ((0.0, 0.0, 0.1625), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    ((0.425, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    ((0.3922, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.1333, -0.0997), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, -1.0)),
    (
      (0.0, 0.0996, 0.0),
      (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0),
      (0.0, 0.0, 1.0),
    ),
  ),
  "lab_bb_01": (
    ((0.0, 0.0, 0.15185), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    ((0.24355, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    ((0.2132, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.13105, -0.08535), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, -1.0)),
    (
      (0.0, 0.0921, 0.0),
      (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0),
      (0.0, 0.0, 1.0),
    ),
  ),
}

_UR3E_DEFAULT_MIN_LIMITS = (
  -6.2203534,
  -4.059,
  -2.772,
  -6.2203534,
  -6.2203534,
  -6.2203534,
)
_UR3E_DEFAULT_MAX_LIMITS = (
  6.2203534,
  0.9504,
  2.772,
  6.2203534,
  6.2203534,
  6.2203534,
)


def _pose3_from_proto(pose_proto) -> data_types.Pose3:
  """Converts a `Pose` proto into a normalized `data_types.Pose3`."""
  ori = pose_proto.orientation
  quat = [float(ori.x), float(ori.y), float(ori.z), float(ori.w)]
  if all(v == 0.0 for v in quat):
    quat = [0.0, 0.0, 0.0, 1.0]
  pos = pose_proto.position
  return data_types.Pose3(
    rotation=data_types.Rotation3.from_xyzw(quat, normalize=True),
    translation=[float(pos.x), float(pos.y), float(pos.z)],
  )


def _axis_angle_pose3(
  axis: tuple[float, float, float], angle_rad: float
) -> data_types.Pose3:
  """Builds a pure-rotation `Pose3` for `angle_rad` around unit `axis`."""
  half = 0.5 * angle_rad
  s = math.sin(half)
  c = math.cos(half)
  return data_types.Pose3(
    rotation=data_types.Rotation3.from_xyzw(
      [axis[0] * s, axis[1] * s, axis[2] * s, c], normalize=True
    ),
    translation=[0.0, 0.0, 0.0],
  )


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

  def test_ur_module_initial_joint_positions_align_tool_with_view_frame(self):
    """Initial ur_module joints must stay within limits and align tool_frame to view."""
    expected_joints = {
      "omts": (
        1.543918,
        -2.356140,
        1.667639,
        -0.587754,
        -1.437866,
        -2.425851,
      ),
      "lab_bb_01": (
        0.770987,
        -2.237038,
        1.658675,
        -1.513059,
        -1.414750,
        -0.351416,
      ),
    }
    for cell, expected_q in expected_joints.items():
      with self.subTest(cell=cell):
        app_cfg = load_app_config(f"configs/{cell}/app_config.yaml")
        attachments_path = f"configs/{cell}/ur_module.attachments.updates.pbtxt"
        attachments = _load_updates_proto(attachments_path)
        self.assertIsNotNone(attachments)

        joint_updates = [
          update.update_object_joints
          for update in attachments.updates
          if update.HasField("update_object_joints")
          and update.update_object_joints.object.by_name.object_name
          == "ur_module"
        ]
        self.assertLen(joint_updates, 1)
        joints = list(joint_updates[0].joint_positions)
        self.assertLen(joints, 6)
        for actual, expected in zip(joints, expected_q, strict=True):
          self.assertAlmostEqual(actual, expected, places=6)

        if cell == "omts":
          limits_proto = _load_updates_proto(
            "configs/omts/ur_module.limits.updates.pbtxt"
          )
          self.assertIsNotNone(limits_proto)
          app_limits = limits_proto.updates[
            0
          ].update_object_joints.joint_application_limits
          min_limits = list(app_limits.min_position.values)
          max_limits = list(app_limits.max_position.values)
        else:
          min_limits = list(_UR3E_DEFAULT_MIN_LIMITS)
          max_limits = list(_UR3E_DEFAULT_MAX_LIMITS)

        for idx, (q_val, lo, hi) in enumerate(
          zip(joints, min_limits, max_limits, strict=True)
        ):
          self.assertGreaterEqual(
            q_val, lo, f"{cell} joint {idx} below min limit"
          )
          self.assertLessEqual(q_val, hi, f"{cell} joint {idx} above max limit")

        align = _load_updates_proto(f"configs/{cell}/align_robot.updates.pbtxt")
        self.assertIsNotNone(align)
        root_t_robot = _pose3_from_proto(
          align.updates[0].update_transform.a_t_b
        )

        flange_t_gripper = data_types.Pose3()
        gripper_t_tool = data_types.Pose3()
        for upd in attachments.updates:
          if not upd.HasField("update_transform"):
            continue
          ut = upd.update_transform
          if (
            ut.node_to_update.by_name.HasField("object")
            and ut.node_to_update.by_name.object.object_name
            == app_cfg.robot.tool_object_name
          ):
            flange_t_gripper = _pose3_from_proto(ut.a_t_b)
          elif (
            ut.node_to_update.by_name.HasField("frame")
            and ut.node_to_update.by_name.frame.frame_name
            == app_cfg.robot.tool_frame_name
          ):
            gripper_t_tool = _pose3_from_proto(ut.a_t_b)

        pose = root_t_robot
        for (inboard_pos, inboard_quat, axis), q_val in zip(
          _UR_JOINT_CHAINS[cell], joints, strict=True
        ):
          inboard_tf = data_types.Pose3(
            rotation=data_types.Rotation3.from_xyzw(
              list(inboard_quat), normalize=True
            ),
            translation=list(inboard_pos),
          )
          pose = pose.multiply(inboard_tf).multiply(
            _axis_angle_pose3(axis, q_val)
          )
        root_t_tool = pose.multiply(flange_t_gripper).multiply(gripper_t_tool)

        scene = _load_updates_proto(f"configs/{cell}/scene.updates.pbtxt")
        self.assertIsNotNone(scene)
        view_frames = [
          upd.create_frame.parent_t_new_frame
          for upd in scene.updates
          if upd.HasField("create_frame")
          and upd.create_frame.new_frame_name == app_cfg.frames.view_frame
        ]
        self.assertLen(view_frames, 1)
        root_t_view = _pose3_from_proto(view_frames[0])

        for coord_idx in range(3):
          self.assertAlmostEqual(
            float(root_t_tool.translation[coord_idx]),
            float(root_t_view.translation[coord_idx]),
            places=4,
          )
        q_tool = root_t_tool.rotation.quaternion.xyzw
        q_view = root_t_view.rotation.quaternion.xyzw
        quat_dot = abs(
          sum(float(a) * float(b) for a, b in zip(q_tool, q_view, strict=True))
        )
        self.assertAlmostEqual(quat_dot, 1.0, places=5)

  def test_align_robot_is_single_source_of_ur_module_base_transform(self):
    """Only align_robot.updates.pbtxt may update the enclosure -> ur_module base transform."""
    for cell in ("omts", "lab_bb_01"):
      with self.subTest(cell=cell):
        align_path = f"configs/{cell}/align_robot.updates.pbtxt"
        align_updates = _load_updates_proto(align_path)
        self.assertIsNotNone(align_updates)
        align_targets = [
          u.update_transform.node_to_update.by_name.object.object_name
          for u in align_updates.updates
          if u.HasField("update_transform")
        ]
        self.assertEqual(align_targets, ["ur_module"])

        attachments_path = f"configs/{cell}/ur_module.attachments.updates.pbtxt"
        attachments_updates = _load_updates_proto(attachments_path)
        self.assertIsNotNone(attachments_updates)
        attachment_obj_targets = [
          u.update_transform.node_to_update.by_name.object.object_name
          for u in attachments_updates.updates
          if u.HasField("update_transform")
          and u.update_transform.node_to_update.by_name.HasField("object")
        ]
        self.assertNotIn(
          "ur_module",
          attachment_obj_targets,
          f"{attachments_path} must not duplicate the ur_module base transform "
          f"from {align_path}.",
        )

  def test_ur_module_application_limits(self):
    """wrist_3_joint of the omts UR5e is limited to [-210 deg, +220 deg]."""
    path = "configs/omts/ur_module.limits.updates.pbtxt"
    updates = _load_updates_proto(path)
    self.assertIsNotNone(updates, f"{path} is missing from {os.getcwd()}.")
    limit_updates = [
      update.update_object_joints
      for update in updates.updates
      if update.HasField("update_object_joints")
      and update.update_object_joints.HasField("joint_application_limits")
    ]
    self.assertLen(limit_updates, 1)
    limits = limit_updates[0].joint_application_limits
    # JointLimitsUpdate replaces a field as a whole, so a partial vector would
    # be rejected by the world service at runtime.
    self.assertLen(limits.min_position.values, 6)
    self.assertLen(limits.max_position.values, 6)
    self.assertAlmostEqual(
      limits.min_position.values[5], math.radians(-210), places=6
    )
    self.assertAlmostEqual(
      limits.max_position.values[5], math.radians(220), places=6
    )
    # Application limits must stay inside the UR5e system limits.
    for value in limits.min_position.values:
      self.assertGreaterEqual(value, -2 * math.pi)
    for value in limits.max_position.values:
      self.assertLessEqual(value, 2 * math.pi)

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
          else:
            self.fail(f"Unhandled .textproto config file: {path}")


if __name__ == "__main__":
  absltest.main()
