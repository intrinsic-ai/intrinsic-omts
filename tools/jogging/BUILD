load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "move_to_frame_lib",
    srcs = ["move_to_frame.py"],
    deps = [
        "//src/hardware:robot",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:execution",
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
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@ioc//google3/intrinsic/icon/python:create_action_utils",
        "@ioc//google3/intrinsic/icon/python:icon",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/util/grpc:connection",
    ],
)

py_library(
    name = "store_frame_lib",
    srcs = ["store_frame.py"],
    data = [
        "//configs:scene.updates.pbtxt",
    ],
    deps = [
        "@ioc//google3/intrinsic/math/python:data_types",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
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
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:worlds",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
    ],
)
