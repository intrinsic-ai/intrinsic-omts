load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "inspect_world",
    srcs = ["inspect_world.py"],
    main = "inspect_world.py",
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_binary(
    name = "align_tool_frame",
    srcs = ["align_tool_frame.py"],
    main = "align_tool_frame.py",
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_library(
    name = "apply_scene_updates_lib",
    srcs = ["apply_scene_updates.py"],
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
    ],
)

py_binary(
    name = "apply_scene_updates",
    srcs = ["apply_scene_updates.py"],
    data = [
        "//configs:all_world_updates",
    ],
    main = "apply_scene_updates.py",
    deps = [
        ":apply_scene_updates_lib",
    ],
)
