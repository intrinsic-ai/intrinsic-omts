load("@intrinsic-core//bazel:python.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

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
    data = ["//configs:all_configs"],
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
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
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
        "//tools/jogging:move_to_joint_lib",
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
    data = ["//configs:all_world_updates"],
    deps = [
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
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
        "//src/core:config",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)
