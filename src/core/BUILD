load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "types",
    srcs = ["types.py"],
    deps = [
        "@intrinsic_apis//intrinsic/math/proto:point_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:pose_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_py_pb2",
    ],
)

py_library(
    name = "world",
    srcs = ["world.py"],
    deps = [
        ":types",
        "//src/utils:math_utils",
        "//src/utils:script_utils",
        "//tools/world:apply_scene_updates_lib",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:proto_building",
    ],
)

py_library(
    name = "solution",
    srcs = ["solution.py"],
    deps = [
        ":world",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "workpiece",
    srcs = ["workpiece.py"],
    deps = [
        ":types",
    ],
)

py_library(
    name = "tray",
    srcs = ["tray.py"],
    deps = [
        ":types",
        ":workpiece",
    ],
)

py_library(
    name = "infeed",
    srcs = ["infeed.py"],
    deps = [
        ":tray",
        ":types",
        ":workpiece",
    ],
)

py_library(
    name = "workcell",
    srcs = ["workcell.py"],
    deps = [
        ":tray",
        ":types",
        ":workpiece",
    ],
)

py_library(
    name = "core",
    srcs = ["__init__.py"],
    deps = [
        ":infeed",
        ":solution",
        ":tray",
        ":types",
        ":workcell",
        ":workpiece",
        ":world",
    ],
)
