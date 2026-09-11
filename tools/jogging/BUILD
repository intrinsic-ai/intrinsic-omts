load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "move_to_frame_lib",
    srcs = ["move_to_frame.py"],
    deps = [
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
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
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_py_pb2",
        "@ioc//google3/intrinsic/util/grpc:connection",
        "@ioc//incode/intrinsic_control/intrinsic/icon/actions:joint_jogging_py_pb2",
        "@ioc//incode/intrinsic_control/intrinsic/icon/proto:joint_space_py_pb2",
        "@ioc//incode/intrinsic_control/intrinsic/icon/python:icon",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_library(
    name = "store_frame_lib",
    srcs = ["store_frame.py"],
    data = [
        "//configs:scene.updates.pbtxt",
    ],
    deps = [
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/math/python:data_types",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
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
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:worlds",
    ],
)
