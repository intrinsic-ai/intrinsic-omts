load("@ioc//bazel:python.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "test_tray",
    srcs = ["test_tray.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:tray",
        "//src/core:types",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_workpiece",
    srcs = ["test_workpiece.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:types",
        "//src/core:workpiece",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_infeed",
    srcs = ["test_infeed.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:infeed",
        "//src/core:tray",
        "//src/core:types",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_hardware_adapters",
    srcs = ["test_hardware_adapters.py"],
    srcs_version = "PY3",
    deps = [
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_behaviors",
    srcs = ["test_behaviors.py"],
    srcs_version = "PY3",
    deps = [
        "//src/behaviors:building_block_bt",
        "//src/behaviors:machine_tending_bt",
        "//src/core:infeed",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_move_to_frame",
    srcs = ["test_move_to_frame.py"],
    srcs_version = "PY3",
    deps = [
        "//src/hardware:robot",
        "//tools/jogging:move_to_frame_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_store_frame",
    srcs = ["test_store_frame.py"],
    srcs_version = "PY3",
    deps = [
        "//tools/jogging:store_frame_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/math/python:data_types",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
    ],
)

py_test(
    name = "test_store_joint_config",
    srcs = ["test_store_joint_config.py"],
    srcs_version = "PY3",
    deps = [
        "//tools/jogging:store_joint_config_lib",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_move_to_joint",
    srcs = ["test_move_to_joint.py"],
    srcs_version = "PY3",
    deps = [
        "//tools/jogging:move_to_joint_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_calibrate_camera",
    srcs = ["test_calibrate_camera.py"],
    srcs_version = "PY3",
    deps = [
        "//tools/calibration:calibrate_camera_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/perception/public/proto/v1:camera_to_robot_calibration_py_pb2",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
    ],
)

py_test(
    name = "test_update_robot_kinematics",
    srcs = ["test_update_robot_kinematics.py"],
    srcs_version = "PY3",
    deps = [
        "//tools/calibration:update_robot_kinematics_lib",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/world/service/robot_calibration:robot_update_service_py_pb2",
        "@ioc//google3/intrinsic/world/service/robot_calibration:robot_update_service_py_pb2_grpc",
    ],
)

py_test(
    name = "test_evaluate_calibration",
    srcs = ["test_evaluate_calibration.py"],
    srcs_version = "PY3",
    deps = [
        "//tools/calibration:evaluate_calibration_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/perception/skills/calibration:calibration_evaluation_py_pb2",
    ],
)

py_test(
    name = "test_pick_and_place_building_block",
    srcs = ["test_pick_and_place_building_block.py"],
    srcs_version = "PY3",
    deps = [
        "//src:pick_and_place_building_block_lib",
        "//src/behaviors:building_block_bt",
        "//src/core:types",
        "//src/hardware:gripper",
        "//src/hardware:robot",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
    ],
)
