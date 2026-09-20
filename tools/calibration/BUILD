load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "calibrate_camera_lib",
    srcs = ["calibrate_camera_to_robot.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@intrinsic-core//intrinsic/util/grpc:connection",
        "@intrinsic-core//intrinsic/util/grpc:interceptor",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/python:create_action_utils",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/python:icon",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:provided",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2",
        "@intrinsic_apis//intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@intrinsic_apis//intrinsic/perception/proto/v1:camera_to_robot_calibration_py_pb2",
    ],
)

py_binary(
    name = "calibrate_camera",
    srcs = ["calibrate_camera_to_robot.py"],
    main = "calibrate_camera_to_robot.py",
    deps = [
        ":calibrate_camera_lib",
    ],
)

py_library(
    name = "update_robot_kinematics_lib",
    srcs = ["update_robot_kinematics.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@intrinsic-core//intrinsic/world/service/robot_calibration:robot_update_service_py_pb2",
        "@intrinsic-core//intrinsic/world/service/robot_calibration:robot_update_service_py_pb2_grpc",
    ],
)

py_binary(
    name = "update_robot_kinematics",
    srcs = ["update_robot_kinematics.py"],
    main = "update_robot_kinematics.py",
    deps = [
        ":update_robot_kinematics_lib",
    ],
)

py_library(
    name = "sample_calibration_poses_lib",
    srcs = ["sample_calibration_poses.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@intrinsic-core//intrinsic/util/grpc:connection",
        "@intrinsic-core//intrinsic/util/grpc:interceptor",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/python:create_action_utils",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/python:icon",
        "@intrinsic-core//intrinsic_perception/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:provided",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2",
        "@intrinsic_apis//intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:camera_to_robot_calibration_py_pb2",
        "@intrinsic_apis//intrinsic/skills/proto:skills_py_pb2",
    ],
)

py_binary(
    name = "sample_calibration_poses",
    srcs = ["sample_calibration_poses.py"],
    main = "sample_calibration_poses.py",
    deps = [
        ":sample_calibration_poses_lib",
    ],
)
