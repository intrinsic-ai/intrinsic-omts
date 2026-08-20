load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("//bazel:python.bzl", "py_binary")
load("//google3/intrinsic/assets/build_defs:asset.bzl", "intrinsic_asset_instance")
load("//google3/intrinsic/config:def.bzl", "intrinsic_solution")

package(default_visibility = ["//visibility:public"])

string_flag(
    name = "robot_model",
    build_setting_default = "ur5e",
    values = [
        "ur3e",
        "ur5e",
    ],
)

config_setting(
    name = "is_ur3e",
    flag_values = {
        ":robot_model": "ur3e",
    },
)

intrinsic_solution(
    name = "omts",
    assets = [
        "//google3/intrinsic/resources/catalog/resourcedata/product/building_block",
        "//google3/intrinsic/resources/catalog/resourcedata/product/imts:raw_stock_2x3x5",
        "//google3/intrinsic/simulation/gazebo/asset:gazebo_simulator_type",
        "//google3/intrinsic/resources/catalog/resourcedata/calibration:charuco_9x14_20mm_15mm_dict_5x5",
        "//google3/intrinsic/apps/common/resources:charuco_9x14_20mm_15mm_dict_5x5_estimator",
        "//google3/intrinsic/perception/service/v1:calibration_service",
        "//google3/intrinsic/perception/skills:capture_images_skill",
        "//google3/intrinsic/perception/skills/calibration:calibrate_camera_to_robot_skill",
        "//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_skill",
        "//google3/intrinsic/perception/skills/calibration:collect_calibration_data_skill",
        "//google3/intrinsic/perception/skills/calibration:initialize_calibration_skill",
        "//google3/intrinsic/perception/public/hardware_devices:orbbec_gemini_335le_hardware_device",
        "//google3/intrinsic/motion_planning/service:motion_planner_service_asset",
        "//incode/motion_planning/skills:clear_motion_planner_service_cache_skill",
        "//incode/motion_planning/skills:move_robot_skill",
        "//incode/motion_planning/skills:preplan_motion_skill",
        "//google3/intrinsic/icon/machines/common:generic_icon_mainloop_type",
        "//google3/intrinsic/manipulation/skills/force:move_to_contact_skill",
        "//google3/intrinsic/icon/skills:dio_read_input_skill",
        "//google3/intrinsic/icon/skills:dio_set_output_skill",
        "//incode/intrinsic_inference/assets/inference_service:inference_service_asset",
    ] + select({
        ":is_ur3e": ["//google3/intrinsic/icon/hardware_modules/universal_robots:ur3e_hardware_module_ioc"],
        "//conditions:default": ["//google3/intrinsic/icon/hardware_modules/universal_robots:ur5e_hardware_module_ioc"],
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
    ],
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
    config = ":icon_config.textproto",
    id = "ai.intrinsic.generic_realtime_control_service",
    instance_name = "icon",
)

intrinsic_asset_instance(
    name = "ur_module",
    config = ":ur_module_config.textproto",
    id = select({
        ":is_ur3e": "ai.intrinsic.ur3e_hardware_module_ioc",
        "//conditions:default": "ai.intrinsic.ur5e_hardware_module_ioc",
    }),
    instance_name = "ur_module",
)

intrinsic_asset_instance(
    name = "orbbec_camera",
    config = ":gemini_device_config.textproto",
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

py_binary(
    name = "calibrate_camera_to_robot_with_jogging",
    srcs = ["calibrate_camera_to_robot_with_jogging.py"],
    deps = [
        "//google3/intrinsic/icon/proto:joint_space_py_pb2",
        "//google3/intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "//google3/intrinsic/icon/python:create_action_utils",
        "//google3/intrinsic/icon/python:icon",
        "//google3/intrinsic/math/python:proto_conversion",
        "//google3/intrinsic/motion_planning/public/proto/v1:geometric_constraints_py_pb2",
        "//google3/intrinsic/perception/public/proto/v1:camera_to_robot_calibration_py_pb2",
        "//google3/intrinsic/perception/skills/calibration:collect_calibration_data_py_pb2",
        "//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "//google3/intrinsic/skills/proto:skills_py_pb2",
        "//google3/intrinsic/solutions:behavior_tree",
        "//google3/intrinsic/solutions:deployments",
        "//google3/intrinsic/solutions:execution",
        "//google3/intrinsic/solutions:provided",
        "//google3/intrinsic/util/grpc:connection",
        "//google3/intrinsic/util/grpc:interceptor",
        "//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
        "//google3/intrinsic/world/python:object_world_ids",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
    ],
)
