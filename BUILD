load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@ioc//google3/intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("@ioc//google3/intrinsic/config:def.bzl", "intrinsic_solution")

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
    assets = [
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
        "@ioc//incode/perception/ioc_pose_estimator:ioc_pose_estimator_service_asset",
        "@ioc//incode/perception/ioc_train_service:ioc_train_service_asset",
        "@ioc//google3/intrinsic/perception/skills/multi_view:estimate_pose_multi_view_skill",
    ] + select({
        ":is_lab_bb_01": ["@ioc//google3/intrinsic/icon/hardware_modules/universal_robots:ur3e_hardware_module_ioc"],
        "//conditions:default": ["@ioc//google3/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_ioc"],
    }),
    default_operation_mode = "real",
    instances = [
        ":building_block",
        ":raw_stock_2x3x5",
        ":gazebo_simulator",
        ":charuco_9x14_20mm_15mm_dict_5x5",
        ":calibration_service_instance",
        ":icon",
        ":ur_module",
        ":orbbec_camera",
        ":motion_planner_service",
        ":inference_service",
        ":pose_estimator_service",
        ":train_service",
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
    name = "building_block",
    id = "ai.intrinsic.building_block",
)

intrinsic_asset_instance(
    name = "raw_stock_2x3x5",
    id = "ai.intrinsic.raw_stock_2x3x5",
)

intrinsic_asset_instance(
    name = "charuco_9x14_20mm_15mm_dict_5x5",
    id = "ai.intrinsic.charuco_9x14_20mm_15mm_dict_5x5",
)

intrinsic_asset_instance(
    name = "gazebo_simulator",
    id = "ai.intrinsic.gazebo_simulator",
)

intrinsic_asset_instance(
    name = "calibration_service_instance",
    id = "ai.intrinsic.calibration_service",
    instance_name = "calibration_service",
)

intrinsic_asset_instance(
    name = "icon",
    config = "//configs:icon_config.textproto",
    id = "ai.intrinsic.generic_realtime_control_service",
    instance_name = "icon",
)

intrinsic_asset_instance(
    name = "ur_module",
    config = "//configs:ur_module_config.textproto",
    id = select({
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
    id = "ai.intrinsic.orbbec_gemini_335le",
    instance_name = "orbbec_camera",
)

intrinsic_asset_instance(
    name = "inference_service",
    id = "ai.intrinsic.inference_service",
    instance_name = "inference_service",
)

intrinsic_asset_instance(
    name = "motion_planner_service",
    id = "ai.intrinsic.motion_planner_service",
    instance_name = "motion_planner_service",
)

intrinsic_asset_instance(
    name = "pose_estimator_service",
    asset = "ai.intrinsic.ioc_pose_estimator_service",
    config = "//configs:pose_estimator_config.textproto",
    instance_name = "pose_estimator_service",
)

intrinsic_asset_instance(
    name = "train_service",
    asset = "ai.intrinsic.ioc_train_service",
    instance_name = "train_service",
)
