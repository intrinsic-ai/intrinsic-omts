load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "inspect_world_lib",
    srcs = ["inspect_world.py"],
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_binary(
    name = "inspect_world",
    srcs = ["inspect_world.py"],
    main = "inspect_world.py",
    deps = [
        ":inspect_world_lib",
    ],
)

py_library(
    name = "apply_scene_updates_lib",
    srcs = ["apply_scene_updates.py"],
    deps = [
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:worlds",
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

py_binary(
    name = "change_ur_solver",
    srcs = ["change_ur_solver.py"],
    main = "change_ur_solver.py",
    deps = [
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/python:object_world_resources",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:worlds",
    ],
)

py_binary(
    name = "check_ur_solver",
    srcs = ["check_ur_solver.py"],
    main = "check_ur_solver.py",
    deps = [
        "@intrinsic-core//intrinsic_sdk/intrinsic/world/python:object_world_resources",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:worlds",
    ],
)
