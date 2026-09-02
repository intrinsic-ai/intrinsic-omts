load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "calibrate_camera_lib",
    srcs = ["calibrate_camera_to_robot.py"],
    srcs_version = "PY3",
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@ioc//google3/intrinsic/icon/proto:joint_space_py_pb2",
        "@ioc//google3/intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@ioc//google3/intrinsic/icon/python:create_action_utils",
        "@ioc//google3/intrinsic/icon/python:icon",
        "@ioc//google3/intrinsic/math/python:proto_conversion",
        "@ioc//google3/intrinsic/perception/public/proto/v1:camera_to_robot_calibration_py_pb2",
        "@ioc//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:execution",
        "@ioc//google3/intrinsic/solutions:provided",
        "@ioc//google3/intrinsic/util/grpc:connection",
        "@ioc//google3/intrinsic/util/grpc:interceptor",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
        "@ioc//google3/intrinsic/world/python:object_world_ids",
    ],
)

py_binary(
    name = "calibrate_camera",
    srcs = ["calibrate_camera_to_robot.py"],
    main = "calibrate_camera_to_robot.py",
    srcs_version = "PY3",
    deps = [
        ":calibrate_camera_lib",
    ],
)

py_library(
    name = "update_robot_kinematics_lib",
    srcs = ["update_robot_kinematics.py"],
    srcs_version = "PY3",
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@ioc//google3/intrinsic/world/service/robot_calibration:robot_update_service_py_pb2",
        "@ioc//google3/intrinsic/world/service/robot_calibration:robot_update_service_py_pb2_grpc",
    ],
)

py_binary(
    name = "update_robot_kinematics",
    srcs = ["update_robot_kinematics.py"],
    main = "update_robot_kinematics.py",
    srcs_version = "PY3",
    deps = [
        ":update_robot_kinematics_lib",
    ],
)

py_binary(
    name = "sample_calibration_poses",
    srcs = ["sample_calibration_poses.py"],
    main = "sample_calibration_poses.py",
    srcs_version = "PY3",
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@ioc//google3/intrinsic/icon/proto:joint_space_py_pb2",
        "@ioc//google3/intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@ioc//google3/intrinsic/icon/python:create_action_utils",
        "@ioc//google3/intrinsic/icon/python:icon",
        "@ioc//google3/intrinsic/math/python:proto_conversion",
        "@ioc//google3/intrinsic/motion_planning/public/proto/v1:geometric_constraints_py_pb2",
        "@ioc//google3/intrinsic/perception/public/proto/v1:camera_to_robot_calibration_py_pb2",
        "@ioc//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@ioc//google3/intrinsic/skills/proto:skills_py_pb2",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:execution",
        "@ioc//google3/intrinsic/solutions:provided",
        "@ioc//google3/intrinsic/util/grpc:connection",
        "@ioc//google3/intrinsic/util/grpc:interceptor",
    ],
)

py_library(
    name = "evaluate_calibration_lib",
    srcs = ["evaluate_calibration.py"],
    srcs_version = "PY3",
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@ioc//google3/intrinsic/icon/proto:joint_space_py_pb2",
        "@ioc//google3/intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@ioc//google3/intrinsic/icon/python:create_action_utils",
        "@ioc//google3/intrinsic/icon/python:icon",
        "@ioc//google3/intrinsic/math/python:proto_conversion",
        "@ioc//google3/intrinsic/motion_planning/public/proto/v1:geometric_constraints_py_pb2",
        "@ioc//google3/intrinsic/perception/public/proto/v1:camera_to_robot_calibration_py_pb2",
        "@ioc//google3/intrinsic/perception/public/proto/v1:charuco_pattern_py_pb2",
        "@ioc//google3/intrinsic/perception/skills/calibration:calibration_evaluation_py_pb2",
        "@ioc//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:execution",
        "@ioc//google3/intrinsic/solutions:provided",
        "@ioc//google3/intrinsic/util/grpc:connection",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
        "@ioc//google3/intrinsic/world/python:object_world_ids",
    ],
)

py_binary(
    name = "evaluate_calibration",
    srcs = ["evaluate_calibration.py"],
    main = "evaluate_calibration.py",
    srcs_version = "PY3",
    deps = [
        ":evaluate_calibration_lib",
    ],
)
