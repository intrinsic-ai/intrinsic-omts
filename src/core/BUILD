load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "types",
    srcs = ["types.py"],
    srcs_version = "PY3",
)

py_library(
    name = "workpiece",
    srcs = ["workpiece.py"],
    srcs_version = "PY3",
    deps = [
        ":types",
    ],
)

py_library(
    name = "tray",
    srcs = ["tray.py"],
    srcs_version = "PY3",
    deps = [
        ":types",
        ":workpiece",
    ],
)

py_library(
    name = "infeed",
    srcs = ["infeed.py"],
    srcs_version = "PY3",
    deps = [
        ":tray",
        ":types",
        ":workpiece",
    ],
)

py_library(
    name = "workcell",
    srcs = ["workcell.py"],
    srcs_version = "PY3",
    deps = [
        ":tray",
        ":types",
        ":workpiece",
    ],
)

py_library(
    name = "core",
    srcs = ["__init__.py"],
    srcs_version = "PY3",
    deps = [
        ":infeed",
        ":tray",
        ":types",
        ":workcell",
        ":workpiece",
    ],
)
