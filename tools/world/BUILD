load("@ioc//bazel:python.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "inspect_world",
    srcs = ["inspect_world.py"],
    main = "inspect_world.py",
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/solutions:deployments",
    ],
)

py_binary(
    name = "align_tool_frame",
    srcs = ["align_tool_frame.py"],
    main = "align_tool_frame.py",
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/math/python:data_types",
        "@ioc//google3/intrinsic/solutions:deployments",
    ],
)

py_binary(
    name = "apply_scene_updates",
    srcs = ["apply_scene_updates.py"],
    data = [
        "//configs:align_robot.updates.pbtxt",
        "//configs:scene.updates.pbtxt",
        "//configs:ur_module.attachments.updates.pbtxt",
    ],
    main = "apply_scene_updates.py",
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
    ],
)
