load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "move_to_frame_lib",
    srcs = ["move_to_frame.py"],
    deps = [
        "//src/hardware:robot",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_binary(
    name = "move_to_frame",
    srcs = ["move_to_frame.py"],
    main = "move_to_frame.py",
    deps = [
        ":move_to_frame_lib",
    ],
)

py_binary(
    name = "jog_interactive",
    srcs = ["jog_interactive.py"],
    main = "jog_interactive.py",
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@intrinsic-core//incode/intrinsic_control/intrinsic/icon/actions:joint_jogging_py_pb2",
        "@intrinsic-core//incode/intrinsic_control/intrinsic/icon/python:icon",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic/util/grpc:connection",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_py_pb2",
    ],
)

py_library(
    name = "store_frame_lib",
    srcs = ["store_frame.py"],
    data = [
        "//configs:scene.updates.pbtxt",
    ],
    deps = [
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
    ],
)

py_binary(
    name = "store_frame",
    srcs = ["store_frame.py"],
    data = [
        "//configs:scene.updates.pbtxt",
    ],
    main = "store_frame.py",
    deps = [
        ":store_frame_lib",
    ],
)

py_binary(
    name = "store_joint_config",
    srcs = ["store_joint_config.py"],
    main = "store_joint_config.py",
    deps = [
        "//src/utils:math_utils",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:worlds",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
    ],
)
