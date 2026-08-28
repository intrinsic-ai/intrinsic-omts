load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "robot",
    srcs = ["robot.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:types",
        "@ioc//google3/intrinsic/manipulation/skills/force:move_to_contact_proto_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:vector3_py_pb2",
        "@ioc//google3/intrinsic/motion_planning/public/proto/v1:geometric_constraints_py_pb2",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/world/public/proto:object_world_refs_py_pb2",
    ],
)

py_library(
    name = "gripper",
    srcs = ["gripper.py"],
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "machine",
    srcs = ["machine.py"],
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "vision",
    srcs = ["vision.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:types",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "hardware",
    srcs = ["__init__.py"],
    srcs_version = "PY3",
    deps = [
        ":gripper",
        ":machine",
        ":robot",
        ":vision",
    ],
)
