load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "types",
    srcs = ["types.py"],
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
        ":tray",
        ":types",
        ":workcell",
        ":workpiece",
    ],
)
