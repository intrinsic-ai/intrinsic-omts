load("@ioc//bazel:python.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "calibrate_camera",
    srcs = ["calibrate_camera_to_robot.py"],
    main = "calibrate_camera_to_robot.py",
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@ioc//google3/intrinsic/math/python:proto_conversion",
        "@ioc//google3/intrinsic/perception/public/proto/v1:camera_to_robot_calibration_py_pb2",
        "@ioc//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@ioc//google3/intrinsic/util/grpc:connection",
        "@ioc//google3/intrinsic/util/grpc:interceptor",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
        "@ioc//google3/intrinsic/world/python:object_world_ids",
        "@ioc//incode/intrinsic_control/intrinsic/icon/proto:joint_space_py_pb2",
        "@ioc//incode/intrinsic_control/intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@ioc//incode/intrinsic_control/intrinsic/icon/python:create_action_utils",
        "@ioc//incode/intrinsic_control/intrinsic/icon/python:icon",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:provided",
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
        "@ioc//google3/intrinsic/math/python:proto_conversion",
        "@ioc//google3/intrinsic/perception/public/proto/v1:camera_to_robot_calibration_py_pb2",
        "@ioc//google3/intrinsic/perception/skills/calibration:sample_calibration_poses_py_pb2",
        "@ioc//google3/intrinsic/skills/proto:skills_py_pb2",
        "@ioc//google3/intrinsic/util/grpc:connection",
        "@ioc//google3/intrinsic/util/grpc:interceptor",
        "@ioc//incode/intrinsic_control/intrinsic/icon/proto:joint_space_py_pb2",
        "@ioc//incode/intrinsic_control/intrinsic/icon/proto/v1:service_py_pb2_grpc",
        "@ioc//incode/intrinsic_control/intrinsic/icon/python:create_action_utils",
        "@ioc//incode/intrinsic_control/intrinsic/icon/python:icon",
        "@ioc//incode/intrinsic_motion_planning/intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:provided",
    ],
)
