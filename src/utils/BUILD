load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "math_utils",
    srcs = ["math_utils.py"],
    deps = [
        "//src/core:types",
    ],
)

py_library(
    name = "script_utils",
    srcs = ["script_utils.py"],
)

py_library(
    name = "dynamic_frame_calculator",
    srcs = ["dynamic_frame_calculator.py"],
    deps = [
        "@ioc//incode/intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_library(
    name = "execution_utils",
    srcs = ["execution_utils.py"],
    deps = [
        "//src/core:types",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_library(
    name = "utils",
    srcs = ["__init__.py"],
    deps = [
        ":dynamic_frame_calculator",
        ":execution_utils",
        ":math_utils",
        ":script_utils",
    ],
)
