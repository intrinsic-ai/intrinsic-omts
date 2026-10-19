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
        "//src/utils:world_utils",
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

py_library(
    name = "pbtxt_pose_store_lib",
    srcs = ["pbtxt_pose_store.py"],
    deps = [
        ":apply_scene_updates_lib",
        "//src/utils:world_utils",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
    ],
)

py_library(
    name = "rviz_marker_node_lib",
    srcs = ["rviz_marker_node.py"],
)

py_library(
    name = "rviz_marker_controller_lib",
    srcs = ["rviz_marker_controller.py"],
    deps = [
        ":apply_scene_updates_lib",
        ":pbtxt_pose_store_lib",
        ":rviz_marker_node_lib",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
    ],
)

py_library(
    name = "edit_frames_rviz_lib",
    srcs = ["edit_frames_rviz.py"],
    data = [
        "rviz_marker_node.py",
    ],
    deps = [
        ":apply_scene_updates_lib",
        ":pbtxt_pose_store_lib",
        ":rviz_marker_controller_lib",
        ":rviz_marker_node_lib",
        "//src/utils:world_utils",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:worlds",
    ],
)

py_binary(
    name = "edit_frames_rviz",
    srcs = ["edit_frames_rviz.py"],
    data = [
        "rviz_marker_node.py",
        "//configs:all_world_updates",
    ],
    main = "edit_frames_rviz.py",
    deps = [
        ":edit_frames_rviz_lib",
    ],
)
