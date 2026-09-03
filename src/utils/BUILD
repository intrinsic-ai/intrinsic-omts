load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "math_utils",
    srcs = ["math_utils.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:types",
    ],
)

py_library(
    name = "script_utils",
    srcs = ["script_utils.py"],
    srcs_version = "PY3",
)

py_library(
    name = "dynamic_frame_calculator",
    srcs = ["dynamic_frame_calculator.py"],
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/math/python:data_types",
    ],
)

py_library(
    name = "utils",
    srcs = ["__init__.py"],
    srcs_version = "PY3",
    deps = [
        ":dynamic_frame_calculator",
        ":math_utils",
        ":script_utils",
    ],
)


