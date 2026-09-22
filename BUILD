load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@bazel_skylib//rules:copy_file.bzl", "copy_file")
load("@intrinsic-core//intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("@intrinsic-core//intrinsic/assets/build_defs:solution.bzl", "intrinsic_solution")
load("@intrinsic-core//intrinsic/assets/data/build_defs:data.bzl", "intrinsic_data")
load("@intrinsic-core//intrinsic/assets/scene_objects/build_defs:scene_object.bzl", "intrinsic_scene_object")
load("@intrinsic-core//intrinsic/scene/build_defs:sdf_scene_object.bzl", "sdf_scene_object")
load("@intrinsic-core//intrinsic_inference/assets/inference_service/bazel:intrinsic_mlmodel.bzl", "intrinsic_mlmodel")
load("//bazel:imported_asset.bzl", "imported_asset_bundle")
load("//bazel:solution_manifest.bzl", "solution_manifest_only")

package(default_visibility = ["//visibility:public"])

# Flag to parameterize hardware setup (omts vs lab_bb_01).
#
# configs/kr_10/ exists but is not a value here: the solution below is wired to
# the UR hardware modules throughout, so selecting a KUKA cell would produce a
# solution that cannot start. Add the value with the rest of the KUKA wiring.
string_flag(
    name = "setup",
    build_setting_default = "omts",
    values = [
        "omts",
        "lab_bb_01",
    ],
)

config_setting(
    name = "is_lab_bb_01",
    flag_values = {
        ":setup": "lab_bb_01",
    },
)

_OMTS_SKILL_ASSETS = [
    ":hand_e_gripper_cmd_skill_asset",
    "@intrinsic-core//intrinsic_control/intrinsic/icon/skills:dio_read_input_skill",
    "@intrinsic-core//intrinsic_control/intrinsic/icon/skills:dio_set_output_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills:capture_images_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:calibrate_camera_to_robot_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:collect_calibration_data_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:initialize_calibration_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:sample_calibration_poses_skill",
    "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/multi_view:estimate_pose_multi_view_skill",
    "@intrinsic-core//incode/motion_planning/skills:clear_motion_planner_service_cache_skill",
    "@intrinsic-core//incode/motion_planning/skills:move_robot_skill",
    "@intrinsic-core//incode/motion_planning/skills:preplan_motion_skill",
    "@intrinsic-core//intrinsic/manipulation/skills/force:move_to_contact_skill",
    "@intrinsic-core//intrinsic/skills/apps:attach_object_to_robot_skill",
    "@intrinsic-core//intrinsic/skills/apps:detach_object_skill",
    "@intrinsic-core//intrinsic/skills/apps:update_world_skill",
    "@intrinsic-core//intrinsic/world/skills/create_object:create_object_skill",
]

# Solution deployment definition
intrinsic_solution(
    name = "omts_solution",
    assets = _OMTS_SKILL_ASSETS + [
        "@intrinsic-core//intrinsic/resources/catalog/resourcedata/gripper:robotiq_pinch_gripper_resource_type",
        "@intrinsic-core//intrinsic/simulation/gazebo/asset:gazebo_simulator_type",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/calibration/charuco_boards:charuco_9x14_20mm_15mm_dict_5x5",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/calibration/charuco_boards:charuco_9x14_20mm_15mm_dict_5x5_estimator",
        ":charuco_9x12_30mm_22mm_dict_5x5_asset",
        ":charuco_9x12_30mm_22mm_dict_5x5_estimator",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/calibration/services/v1:calibration_service",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/cameras/hardware_devices:orbbec_gemini_335le_hardware_device",
        "@intrinsic-core//intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_asset",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/machines/common:generic_icon_mainloop_type",
        "@intrinsic-core//intrinsic_inference/assets/inference_service:inference_service_asset",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:ioc_pose_estimator_service_asset",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service:ioc_train_service_asset",
        ":flowstate_ros_bridge_asset",
        ":hand_e_gripper_service_asset",
        ":orbbec_gemini_driver_asset",
        "//src/foundationpose:foundationpose_mlmodel",
        ":rfdetr_mlmodel",
        "//models/raw_stock_2x3x5",
    ] + select({
        ":is_lab_bb_01": [
            "@intrinsic-core//intrinsic/apps/bluebird_caw/resources:caw_enclosure",
            "@intrinsic-core//intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur3e_hardware_module_core",
        ],
        "//conditions:default": [
            "@intrinsic-core//intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_core",
            "//models/camera_mount",
            "//models/cnc_enclosure",
            "//models/omts_enclosure",
            "//models/schunk_egp_64nnb",
        ],
    }),
    default_operation_mode = "real",
    instances = [
        ":enclosure",
        ":robotiq_pinch_gripper",
        ":raw_stock_2x3x5",
        ":gazebo_simulator",
        ":charuco_9x14_20mm_15mm_dict_5x5",
        ":charuco_9x12_30mm_22mm_dict_5x5",
        ":calibration_service_instance",
        ":icon",
        ":ur_module",
        ":orbbec_camera",
        ":orbbec_gemini_driver",
        ":motion_planner_service",
        ":inference_service",
        ":pose_estimator_service",
        ":train_service",
        ":flowstate_ros_bridge",
        ":hand_e_gripper_service",
    ] + select({
        ":is_lab_bb_01": [
        ],
        "//conditions:default": [
            ":camera_mount",
            ":cnc_enclosure",
            ":schunk_egp_64nnb",
        ],
    }),
    object_world_updates = select({
        ":is_lab_bb_01": [
            "//configs:lab_bb_01/ur_module.attachments.updates.pbtxt",
            "//configs:lab_bb_01/scene.updates.pbtxt",
            "//configs:lab_bb_01/align_robot.updates.pbtxt",
            "//configs:lab_bb_01/orbbec_gemini.updates.pbtxt",
        ],
        "//conditions:default": [
            "//configs:omts/ur_module.attachments.updates.pbtxt",
            "//configs:omts/scene.updates.pbtxt",
            "//configs:omts/align_robot.updates.pbtxt",
            "//configs:omts/cnc_enclosure.updates.pbtxt",
            "//configs:omts/schunk.updates.pbtxt",
            "//configs:omts/camera_mount.updates.pbtxt",
            "//configs:omts/orbbec_gemini.updates.pbtxt",
        ],
    }),
)

# Default aliases
alias(
    name = "omts",
    actual = ":omts_solution",
)

alias(
    name = "omts_app",
    actual = "//src:omts_app",
)

intrinsic_asset_instance(
    name = "enclosure",
    asset = select({
        ":is_lab_bb_01": "ai.intrinsic.caw_enclosure",
        "//conditions:default": "ai.intrinsic.omts_enclosure",
    }),
    instance_name = "enclosure",
)

intrinsic_asset_instance(
    name = "cnc_enclosure",
    asset = "ai.intrinsic.cnc_enclosure",
)

intrinsic_asset_instance(
    name = "camera_mount",
    asset = "ai.intrinsic.camera_mount",
)

intrinsic_asset_instance(
    name = "robotiq_pinch_gripper",
    asset = "ai.intrinsic.robotiq_gripper",
    instance_name = "gripper",
)

intrinsic_asset_instance(
    name = "raw_stock_2x3x5",
    asset = "ai.intrinsic.raw_stock_2x3x5",
)

intrinsic_asset_instance(
    name = "charuco_9x14_20mm_15mm_dict_5x5",
    asset = "ai.intrinsic.charuco_9x14_20mm_15mm_dict_5x5",
)

intrinsic_asset_instance(
    name = "charuco_9x12_30mm_22mm_dict_5x5",
    asset = "ai.intrinsic.charuco_9x12_30mm_22mm_dict_5x5",
)

genrule(
    name = "charuco_9x12_30mm_22mm_dict_5x5_sdf",
    outs = ["charuco_9x12_30mm_22mm_dict_5x5.sdf"],
    cmd = """cat <<'EOF' > $@
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="charuco_9x12_30mm_22mm_dict_5x5">
    <link name="body">
      <visual name="visual">
        <pose>0 0 -0.0015 0 0 0</pose>
        <geometry>
          <box>
            <size>0.4 0.3 0.003</size>
          </box>
        </geometry>
      </visual>
      <collision name="collision">
        <pose>0 0 -0.0015 0 0 0</pose>
        <geometry>
          <box>
            <size>0.4 0.3 0.003</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>0.1</mass>
        <pose>0 0 -0.0015 0 0 0</pose>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00001</iyy>
          <iyz>0</iyz>
          <izz>0.000015</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
EOF""",
)

genrule(
    name = "charuco_9x12_30mm_22mm_dict_5x5_resource_manifest",
    outs = ["charuco_9x12_30mm_22mm_dict_5x5_resource_manifest.textproto"],
    cmd = """cat <<'EOF' > $@
metadata {
  id {
    package: "ai.intrinsic"
    name: "charuco_9x12_30mm_22mm_dict_5x5"
  }
  vendor {
    display_name: "Intrinsic"
  }
  documentation {
    description: "A ChArUco calibration board."
  }
  display_name: "ChArUco 9x12 | 30mm | 22mm | DICT_5X5"
}
EOF""",
)

sdf_scene_object(
    name = "charuco_9x12_30mm_22mm_dict_5x5_scene_object",
    src = ":charuco_9x12_30mm_22mm_dict_5x5.sdf",
)

intrinsic_scene_object(
    name = "charuco_9x12_30mm_22mm_dict_5x5_asset",
    manifest = ":charuco_9x12_30mm_22mm_dict_5x5_resource_manifest.textproto",
    scene_object = ":charuco_9x12_30mm_22mm_dict_5x5_scene_object",
)

genrule(
    name = "charuco_9x12_30mm_22mm_dict_5x5_estimator_manifest",
    outs = ["charuco_9x12_30mm_22mm_dict_5x5_estimator_manifest.textproto"],
    cmd = """cat <<'EOF' > $@
metadata {
  id {
    package: "ai.intrinsic"
    name: "charuco_9x12_30mm_22mm_dict_5x5_estimator"
  }
  display_name: "charuco_9x12_30mm_22mm_dict_5x5_estimator"
  vendor {
    display_name: "Intrinsic"
  }
}
data {
  [type.googleapis.com/intrinsic_proto.perception.v1.PerceptionModel] {
    pose_estimation_config {
      targets {
        id: "charuco_9x12_30mm_22mm_dict_5x5"
        marker {
          charuco_pattern {
            squares_x: 12
            squares_y: 9
            square_length: 0.03
            marker_length: 0.022
            dictionary: DICT_5X5_100
          }
        }
      }
      params {
        [type.googleapis.com/intrinsic_proto.perception.v1.CharucoMarkerPoseEstimationConfig] {
        }
      }
    }
  }
}
EOF""",
)

intrinsic_data(
    name = "charuco_9x12_30mm_22mm_dict_5x5_estimator",
    manifest = ":charuco_9x12_30mm_22mm_dict_5x5_estimator_manifest.textproto",
    deps = [
        "@intrinsic_apis//intrinsic/perception/proto/pose_estimators/v1:charuco_marker_pose_estimation_config_proto",
        "@intrinsic_apis//intrinsic/perception/proto/v1:perception_model_proto",
    ],
)

intrinsic_asset_instance(
    name = "gazebo_simulator",
    asset = "ai.intrinsic.gazebo_simulator",
)

intrinsic_asset_instance(
    name = "calibration_service_instance",
    asset = "ai.intrinsic.calibration_service",
    instance_name = "calibration_service",
)

intrinsic_asset_instance(
    name = "icon",
    asset = "ai.intrinsic.generic_realtime_control_service",
    instance_name = "icon",
    service_config = select({
        ":is_lab_bb_01": "//configs:lab_bb_01/icon_config.textproto",
        "//conditions:default": "//configs:omts/icon_config.textproto",
    }),
)

intrinsic_asset_instance(
    name = "ur_module",
    asset = select({
        ":is_lab_bb_01": "ai.intrinsic.ur3e_hardware_module_core",
        "//conditions:default": "ai.intrinsic.ur5e_hardware_module_core",
    }),
    instance_name = "ur_module",
    service_config = select({
        ":is_lab_bb_01": "//configs:lab_bb_01/ur_module_config.textproto",
        "//conditions:default": "//configs:omts/ur_module_config.textproto",
    }),
)

intrinsic_asset_instance(
    name = "orbbec_camera",
    asset = "ai.intrinsic.orbbec_gemini_335le",
    instance_name = "orbbec_camera",
    service_config = select({
        ":is_lab_bb_01": "//configs:lab_bb_01/gemini_device_config.textproto",
        "//conditions:default": "//configs:omts/gemini_device_config.textproto",
    }),
)

intrinsic_asset_instance(
    name = "inference_service",
    asset = "ai.intrinsic.inference_service",
    instance_name = "inference_service",
)

intrinsic_asset_instance(
    name = "motion_planner_service",
    asset = "ai.intrinsic.motion_planner_service",
    instance_name = "motion_planner_service",
)

intrinsic_asset_instance(
    name = "pose_estimator_service",
    asset = "ai.intrinsic.ioc_pose_estimator_service",
    instance_name = "pose_estimator_service",
    service_config = "//configs:common/pose_estimator_config.textproto",
)

intrinsic_asset_instance(
    name = "train_service",
    asset = "ai.intrinsic.ioc_train_service",
    instance_name = "train_service",
)

imported_asset_bundle(
    name = "flowstate_ros_bridge_asset",
    bundle = "@flowstate_ros_bridge_bundle//:flowstate_ros_bridge.bundle.tar",
    manifest = "//configs:common/flowstate_ros_bridge_manifest.textproto",
)

intrinsic_asset_instance(
    name = "flowstate_ros_bridge",
    asset = "ai.intrinsic.flowstate_ros_bridge",
    instance_name = "flowstate_ros_bridge",
)

# assetlocalinfogen parses skill manifests as binary proto, so the manifest is
# taken straight out of the bundle rather than being maintained as a checked-in
# copy that could drift from the released tarball.
genrule(
    name = "hand_e_gripper_cmd_skill_manifest",
    srcs = ["@hand_e_gripper_cmd_skill_bundle//:hand_e_gripper_cmd_skill.bundle.tar"],
    outs = ["hand_e_gripper_cmd_skill_manifest.binpb"],
    cmd = "tar -xOf $< skill_manifest.binpb > $@",
)

genrule(
    name = "hand_e_gripper_cmd_skill_fds",
    srcs = ["@hand_e_gripper_cmd_skill_bundle//:hand_e_gripper_cmd_skill.bundle.tar"],
    outs = ["hand_e_gripper_cmd_skill_fds.binpb"],
    cmd = "tar -xOf $< descriptors-transitive-descriptor-set.proto.bin > $@",
)

imported_asset_bundle(
    name = "hand_e_gripper_cmd_skill_asset",
    asset_type = "ASSET_TYPE_SKILL",
    bundle = "@hand_e_gripper_cmd_skill_bundle//:hand_e_gripper_cmd_skill.bundle.tar",
    file_descriptor_set = ":hand_e_gripper_cmd_skill_fds.binpb",
    manifest = ":hand_e_gripper_cmd_skill_manifest.binpb",
)

solution_manifest_only(
    name = "omts_solution_manifest",
    extra_manifests = [
        ":BUILD",
        ":hand_e_gripper_cmd_skill_manifest",
    ],
    skills = _OMTS_SKILL_ASSETS,
)

imported_asset_bundle(
    name = "hand_e_gripper_service_asset",
    bundle = "@hand_e_gripper_service_bundle//:hand_e_gripper_service.bundle.tar",
    manifest = "//configs:common/hande_gripper_service_manifest.textproto",
)

intrinsic_asset_instance(
    name = "hand_e_gripper_service",
    asset = "ai.intrinsic.hande_gripper_aquarium_hande_gripper_launch_xml",
    instance_name = "hande_gripper",
)

imported_asset_bundle(
    name = "orbbec_gemini_driver_asset",
    bundle = "@orbbec_gemini_driver_bundle//:orbbec_gemini_driver.bundle.tar",
    manifest = "//configs:common/orbbec_gemini_driver_manifest.textproto",
)

intrinsic_asset_instance(
    name = "orbbec_gemini_driver",
    asset = "ai.intrinsic.orbbec_gemini_driver",
    instance_name = "orbbec_gemini_driver",
)

intrinsic_asset_instance(
    name = "schunk_egp_64nnb",
    asset = "ai.intrinsic.schunk_egp_64nnb",
    instance_name = "schunk_egp_64nnb",
)

copy_file(
    name = "rfdetr_segmentation_config_pbtxt",
    src = "@segmentation_weights_bundle//:config.pbtxt",
    out = "rfdetr_segmentation_config.pbtxt",
    allow_symlink = True,
)

copy_file(
    name = "rfdetr_segmentation_onnx_file",
    src = "@segmentation_weights_bundle//:segmentation.onnx",
    out = "segmentation.onnx",
    allow_symlink = True,
)

intrinsic_mlmodel(
    name = "rfdetr_mlmodel",
    backend = "triton",
    config = ":rfdetr_segmentation_config_pbtxt",
    description = "RF-DETR segmentation model for object detection and segmentation.",
    display_name = "RF-DETR Segmentation Model",
    id = "ai.intrinsic.ioc_pose_estimation.segmentor.rfdetr",
    model_files = {
        ":rfdetr_segmentation_onnx_file": "1/segmentation.onnx",
    },
)
