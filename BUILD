load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@ioc//google3/intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("@ioc//google3/intrinsic/config:def.bzl", "intrinsic_solution")
load("//bazel:imported_asset.bzl", "imported_asset_bundle")

package(default_visibility = ["//visibility:public"])

# Flag to parameterize hardware setup (omts vs lab_bb_01)
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

# Solution deployment definition
intrinsic_solution(
    name = "omts_solution",
    add_compose_world_test = False,
    assets = [
        "@ioc//google3/intrinsic/apps/bluebird_caw/resources:caw_enclosure",
        "@ioc//google3/intrinsic/resources/catalog/resourcedata/gripper:robotiq_pinch_gripper_resource_type",
        "@ioc//google3/intrinsic/resources/catalog/resourcedata/product/building_block",
        "@ioc//google3/intrinsic/resources/catalog/resourcedata/product/imts:raw_stock_2x3x5",
        "@ioc//google3/intrinsic/simulation/gazebo/asset:gazebo_simulator_type",
        "@ioc//google3/intrinsic/resources/catalog/resourcedata/calibration:charuco_9x14_20mm_15mm_dict_5x5",
        "@ioc//google3/intrinsic/apps/common/resources:charuco_9x14_20mm_15mm_dict_5x5_estimator",
        "@ioc//google3/intrinsic/perception/service/v1:calibration_service",
        "@ioc//google3/intrinsic/perception/skills:capture_images_skill",
        "@ioc//google3/intrinsic/perception/skills/calibration:calibrate_camera_to_robot_skill",
        "@ioc//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_skill",
        "@ioc//google3/intrinsic/perception/skills/calibration:collect_calibration_data_skill",
        "@ioc//google3/intrinsic/perception/skills/calibration:initialize_calibration_skill",
        "@ioc//google3/intrinsic/perception/public/hardware_devices:orbbec_gemini_335le_hardware_device",
        "@ioc//google3/intrinsic/motion_planning/service:motion_planner_service_asset",
        "@ioc//incode/motion_planning/skills:clear_motion_planner_service_cache_skill",
        "@ioc//incode/motion_planning/skills:move_robot_skill",
        "@ioc//incode/motion_planning/skills:preplan_motion_skill",
        "@ioc//google3/intrinsic/icon/machines/common:generic_icon_mainloop_type",
        "@ioc//google3/intrinsic/manipulation/skills/force:move_to_contact_skill",
        "@ioc//google3/intrinsic/icon/skills:dio_read_input_skill",
        "@ioc//google3/intrinsic/icon/skills:dio_set_output_skill",
        "@ioc//incode/intrinsic_inference/assets/inference_service:inference_service_asset",
        ":moveit_flowstate_ros_bridge_asset",
        ":orbbec_gemini_driver_asset",
    ] + select({
        ":is_lab_bb_01": ["@ioc//google3/intrinsic/icon/hardware_modules/universal_robots:ur3e_hardware_module_ioc"],
        "//conditions:default": ["@ioc//google3/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_ioc"],
    }),
    default_operation_mode = "real",
    instances = [
        ":enclosure",
        ":robotiq_pinch_gripper",
        ":building_block",
        ":raw_stock_2x3x5",
        ":gazebo_simulator",
        ":charuco_9x14_20mm_15mm_dict_5x5",
        ":calibration_service_instance",
        ":icon",
        ":ur_module",
        ":orbbec_camera",
        ":orbbec_gemini_driver",
        ":motion_planner_service",
        ":inference_service",
        ":moveit_ros_bridge",
    ],
    object_world_updates = [
        "//configs:ur_module.attachments.updates.pbtxt",
        "//configs:scene.updates.pbtxt",
        "//configs:align_robot.updates.pbtxt",
    ],
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
    asset = "ai.intrinsic.caw_enclosure",
)

intrinsic_asset_instance(
    name = "robotiq_pinch_gripper",
    asset = "ai.intrinsic.robotiq_gripper",
    instance_name = "gripper",
)

intrinsic_asset_instance(
    name = "building_block",
    asset = "ai.intrinsic.building_block",
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
    config = "//configs:icon_config.textproto",
    asset = "ai.intrinsic.generic_realtime_control_service",
    instance_name = "icon",
)

intrinsic_asset_instance(
    name = "ur_module",
    config = "//configs:ur_module_config.textproto",
    asset = select({
        ":is_lab_bb_01": "ai.intrinsic.ur3e_hardware_module_ioc",
        "//conditions:default": "ai.intrinsic.ur5e_hardware_module_ioc",
    }),
    instance_name = "ur_module",
)

intrinsic_asset_instance(
    name = "orbbec_camera",
    config = select({
        ":is_lab_bb_01": "//configs:lab_bb_01_gemini_device_config.textproto",
        "//conditions:default": "//configs:omts_gemini_device_config.textproto",
    }),
    asset = "ai.intrinsic.orbbec_gemini_335le",
    instance_name = "orbbec_camera",
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

imported_asset_bundle(
    name = "moveit_flowstate_ros_bridge_asset",
    bundle = "@moveit_ros_bridge_bundle//:moveit_flowstate_ros_bridge.bundle.tar",
    manifest = "//configs:moveit_service_manifest.textproto",
)

intrinsic_asset_instance(
    name = "moveit_ros_bridge",
    asset = "ai.intrinsic.moveit_flowstate_ros_bridge",
    instance_name = "moveit_ros_bridge",
)

imported_asset_bundle(
    name = "orbbec_gemini_driver_asset",
    bundle = "@orbbec_gemini_driver_bundle//:orbbec_gemini_driver.bundle.tar",
    manifest = "//configs:orbbec_gemini_driver_manifest.textproto",
)

intrinsic_asset_instance(
    name = "orbbec_gemini_driver",
    asset = "ai.intrinsic.orbbec_gemini_driver",
    instance_name = "orbbec_gemini_driver",
)

