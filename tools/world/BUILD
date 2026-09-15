load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "inspect_world",
    srcs = ["inspect_world.py"],
    main = "inspect_world.py",
    deps = [
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_binary(
    name = "align_tool_frame",
    srcs = ["align_tool_frame.py"],
    main = "align_tool_frame.py",
    deps = [
        "@ioc//incode/intrinsic_sdk/intrinsic/math/python:data_types",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_library(
    name = "apply_scene_updates_lib",
    srcs = ["apply_scene_updates.py"],
    deps = [
        "@ioc//intrinsic/world/public/proto:object_world_updates_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_binary(
    name = "apply_scene_updates",
    srcs = ["apply_scene_updates.py"],
    data = [
        "//configs:align_robot.updates.pbtxt",
        "//configs:lab_bb_01_orbbec_gemini.updates.pbtxt",
        "//configs:scene.updates.pbtxt",
        "//configs:ur_module.attachments.updates.pbtxt",
    ],
    main = "apply_scene_updates.py",
    deps = [
        ":apply_scene_updates_lib",
    ],
)
