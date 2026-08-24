load("@ioc//bazel:python.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "jog_interactive",
    srcs = ["jog_interactive.py"],
    main = "jog_interactive.py",
    srcs_version = "PY3",
    deps = [
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@ioc//google3/intrinsic/icon/python:create_action_utils",
        "@ioc//google3/intrinsic/icon/python:icon",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/util/grpc:connection",
    ],
)

py_binary(
    name = "store_joint_config",
    srcs = ["store_joint_config.py"],
    main = "store_joint_config.py",
    srcs_version = "PY3",
    deps = [
        "//src/utils:math_utils",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:worlds",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
    ],
)
