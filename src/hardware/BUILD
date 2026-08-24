load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "robot",
    srcs = ["robot.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:types",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
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
