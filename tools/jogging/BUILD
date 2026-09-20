load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "move_to_frame_lib",
    srcs = ["move_to_frame.py"],
    deps = [
        "//src/core:config",
        "//src/hardware:robot",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
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

py_library(
    name = "move_to_joint_lib",
    srcs = ["move_to_joint.py"],
    deps = [
        "//src/core:config",
        "//src/core:types",
        "//src/hardware:robot",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_binary(
    name = "move_to_joint",
    srcs = ["move_to_joint.py"],
    main = "move_to_joint.py",
    deps = [
        ":move_to_joint_lib",
    ],
)

py_library(
    name = "jog_interactive_lib",
    srcs = ["jog_interactive.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@intrinsic-core//intrinsic/util/grpc:connection",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/actions:joint_jogging_py_pb2",
        "@intrinsic-core//intrinsic_control/intrinsic/icon/python:icon",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_py_pb2",
    ],
)

py_binary(
    name = "jog_interactive",
    srcs = ["jog_interactive.py"],
    main = "jog_interactive.py",
    deps = [
        ":jog_interactive_lib",
    ],
)

py_library(
    name = "store_frame_lib",
    srcs = ["store_frame.py"],
    data = [
        "//configs:all_world_updates",
    ],
    deps = [
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_binary(
    name = "store_frame",
    srcs = ["store_frame.py"],
    data = [
        "//configs:all_world_updates",
    ],
    main = "store_frame.py",
    deps = [
        ":store_frame_lib",
    ],
)

py_library(
    name = "store_joint_config_lib",
    srcs = ["store_joint_config.py"],
    deps = [
        "//src/utils:math_utils",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:worlds",
    ],
)

py_binary(
    name = "store_joint_config",
    srcs = ["store_joint_config.py"],
    main = "store_joint_config.py",
    deps = [
        ":store_joint_config_lib",
    ],
)
