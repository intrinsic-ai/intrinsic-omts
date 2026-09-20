load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@intrinsic-core//bazel:python.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

# buildifier: disable=LANG_proto_library-colocation
py_proto_library(
    name = "hal_adio_part_config_py_pb2",
    deps = ["@intrinsic_apis//intrinsic/icon/control/parts/hal/adio_part:hal_adio_part_config_proto"],
)

# buildifier: disable=LANG_proto_library-colocation
py_proto_library(
    name = "hal_arm_part_config_py_pb2",
    deps = ["@intrinsic_apis//intrinsic/icon/control/parts/hal/arm_part:hal_arm_part_config_proto"],
)

# buildifier: disable=LANG_proto_library-colocation
py_proto_library(
    name = "hal_force_torque_sensor_part_config_py_pb2",
    deps = ["@intrinsic_apis//intrinsic/icon/control/parts/hal/force_torque_sensor_part:hal_force_torque_sensor_part_config_proto"],
)

# buildifier: disable=LANG_proto_library-colocation
py_proto_library(
    name = "hardware_module_config_py_pb2",
    deps = ["@intrinsic_apis//intrinsic/icon/hal/proto:hardware_module_config_proto"],
)

# buildifier: disable=LANG_proto_library-colocation
py_proto_library(
    name = "ur_module_config_py_pb2",
    deps = ["@intrinsic-core//intrinsic_control/intrinsic/icon/hardware_modules/universal_robots:config_proto"],
)

py_test(
    name = "test_tray",
    srcs = ["test_tray.py"],
    deps = [
        "//src/core:tray",
        "//src/core:types",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_workpiece",
    srcs = ["test_workpiece.py"],
    deps = [
        "//src/core:types",
        "//src/core:workcell",
        "//src/core:workpiece",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_infeed",
    srcs = ["test_infeed.py"],
    deps = [
        "//src/core:config",
        "//src/core:infeed",
        "//src/core:tray",
        "//src/core:types",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_hardware_adapters",
    srcs = ["test_hardware_adapters.py"],
    deps = [
        "//src/core:config",
        "//src/core:types",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_behaviors",
    srcs = ["test_behaviors.py"],
    data = [
        "//:omts_solution_manifest",
        "//configs:all_configs",
    ],
    deps = [
        "//src/behaviors:machine_tending_bt",
        "//src/core:config",
        "//src/core:infeed",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
        "@com_google_protobuf//:protobuf_python",
        "@intrinsic-core//intrinsic/assets/build_defs:asset_py_pb2",
        "@intrinsic-core//intrinsic/executive/proto:behavior_tree_py_pb2",
        "@intrinsic-core//intrinsic/world/proto:object_world_service_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:proto_building",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:provided",
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/python:object_world_resources",
        "@intrinsic_apis//intrinsic/assets/proto:asset_type_py_pb2",
        "@intrinsic_apis//intrinsic/assets/proto:id_py_pb2",
        "@intrinsic_apis//intrinsic/executive/proto:proto_builder_py_pb2",
        "@intrinsic_apis//intrinsic/resources/proto:resource_handle_py_pb2",
        "@intrinsic_apis//intrinsic/skills/proto:equipment_py_pb2",
        "@intrinsic_apis//intrinsic/skills/proto:skill_manifest_py_pb2",
    ],
)

py_test(
    name = "test_move_to_frame",
    srcs = ["test_move_to_frame.py"],
    deps = [
        "//src/core:config",
        "//src/hardware:robot",
        "//tools/jogging:move_to_frame_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_move_to_joint",
    srcs = ["test_move_to_joint.py"],
    deps = [
        "//src/core:config",
        "//src/core:types",
        "//tools/jogging:jog_interactive_lib",
        "//tools/jogging:move_to_joint_lib",
        "//tools/jogging:store_joint_config_lib",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_store_frame",
    srcs = ["test_store_frame.py"],
    deps = [
        "//tools/jogging:store_frame_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_test(
    name = "test_script_utils",
    srcs = ["test_script_utils.py"],
    deps = [
        "//src/utils:dynamic_frame_calculator",
        "//src/utils:script_utils",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_control_gripper",
    srcs = ["test_control_gripper.py"],
    deps = [
        "//src/core:config",
        "//src/hardware:gripper",
        "//tools/gripper:control_gripper_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_control_machine",
    srcs = ["test_control_machine.py"],
    deps = [
        "//src/core:config",
        "//src/hardware:machine",
        "//tools/machine:control_machine_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_apply_scene_updates",
    srcs = ["test_apply_scene_updates.py"],
    deps = [
        "//tools/world:apply_scene_updates_lib",
        "//tools/world:inspect_world_lib",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_execution_utils",
    srcs = ["test_execution_utils.py"],
    deps = [
        "//src/core:types",
        "//src/utils:execution_utils",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_world_configs",
    srcs = ["test_world_configs.py"],
    data = ["//configs:all_configs"],
    deps = [
        ":hal_adio_part_config_py_pb2",
        ":hal_arm_part_config_py_pb2",
        ":hal_force_torque_sensor_part_config_py_pb2",
        ":hardware_module_config_py_pb2",
        ":ur_module_config_py_pb2",
        "//src/core:config",
        "//tools/world:apply_scene_updates_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/proto:ioc_service_config_py_pb2",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@intrinsic_apis//intrinsic/assets/services/proto:service_manifest_py_pb2",
        "@intrinsic_apis//intrinsic/icon/server/config:icon_main_config_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:camera_config_py_pb2",
    ],
)

py_test(
    name = "test_calibrate_camera",
    srcs = ["test_calibrate_camera.py"],
    deps = [
        "//tools/calibration:calibrate_camera_lib",
        "//tools/world:apply_scene_updates_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:camera_to_robot_calibration_py_pb2",
    ],
)

py_test(
    name = "test_sample_calibration_poses",
    srcs = ["test_sample_calibration_poses.py"],
    deps = [
        "//tools/calibration:sample_calibration_poses_lib",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_update_robot_kinematics",
    srcs = ["test_update_robot_kinematics.py"],
    deps = [
        "//tools/calibration:update_robot_kinematics_lib",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic/world/service/robot_calibration:robot_update_service_py_pb2",
        "@intrinsic-core//intrinsic/world/service/robot_calibration:robot_update_service_py_pb2_grpc",
    ],
)

py_test(
    name = "test_register_using_train_service",
    srcs = ["test_register_using_train_service.py"],
    deps = [
        "//tools/pose_estimation:register_using_train_service_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@com_google_googleapis//google/longrunning:operations_py_proto",
        "@com_google_googleapis//google/rpc:status_py_proto",
        "@intrinsic-core//intrinsic/assets/proto:installed_assets_py_pb2",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service/proto:ioc_pose_estimator_params_py_pb2",
        "@intrinsic_apis//intrinsic/assets/proto:view_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:train_service_py_pb2",
        "@intrinsic_apis//intrinsic/scene/proto/v1:scene_object_py_pb2",
    ],
)

py_test(
    name = "test_config",
    srcs = ["test_config.py"],
    data = ["//configs:all_configs"],
    deps = [
        "//src:omts_app",
        "//src/core:config",
        "//src/core:types",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_dynamic_frame_calculator",
    srcs = ["test_dynamic_frame_calculator.py"],
    deps = [
        "//src/utils:dynamic_frame_calculator",
        "//src/utils:math_utils",
        "//src/utils:script_utils",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_test(
    name = "test_run_pose_estimation",
    srcs = ["test_run_pose_estimation.py"],
    deps = [
        "//tools/pose_estimation:run_pose_estimation_lib",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)
