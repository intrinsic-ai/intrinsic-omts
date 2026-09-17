load("@intrinsic-core//bazel:python.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "calibrate_camera",
    srcs = ["calibrate_camera_to_robot.py"],
    main = "calibrate_camera_to_robot.py",
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@intrinsic-core//incode/intrinsic_control/intrinsic/icon/python:create_action_utils",
        "@intrinsic-core//incode/intrinsic_control/intrinsic/icon/python:icon",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:provided",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/world/python:object_world_ids",
        "@intrinsic-core//intrinsic/util/grpc:connection",
        "@intrinsic-core//intrinsic/util/grpc:interceptor",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2",
        "@intrinsic_apis//intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@intrinsic_apis//intrinsic/perception/proto/v1:camera_to_robot_calibration_py_pb2",
    ],
)

py_binary(
    name = "sample_calibration_poses",
    srcs = ["sample_calibration_poses.py"],
    main = "sample_calibration_poses.py",
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@intrinsic-core//incode/intrinsic_control/intrinsic/icon/python:create_action_utils",
        "@intrinsic-core//incode/intrinsic_control/intrinsic/icon/python:icon",
        "@intrinsic-core//incode/intrinsic_perception/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:provided",
        "@intrinsic-core//intrinsic/util/grpc:connection",
        "@intrinsic-core//intrinsic/util/grpc:interceptor",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2",
        "@intrinsic_apis//intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:camera_to_robot_calibration_py_pb2",
        "@intrinsic_apis//intrinsic/skills/proto:skills_py_pb2",
    ],
)
