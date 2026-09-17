load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "math_utils",
    srcs = ["math_utils.py"],
)

py_library(
    name = "script_utils",
    srcs = ["script_utils.py"],
)

py_library(
    name = "dynamic_frame_calculator",
    srcs = ["dynamic_frame_calculator.py"],
    deps = [
        ":math_utils",
        "@ioc//incode/intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_library(
    name = "execution_utils",
    srcs = ["execution_utils.py"],
    deps = [
        "//src/core:types",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_library(
    name = "tree_diagnostics",
    srcs = ["tree_diagnostics.py"],
)

py_library(
    name = "utils",
    srcs = [
        "__init__.py",
    ],
    deps = [
        ":dynamic_frame_calculator",
        ":execution_utils",
        ":math_utils",
        ":script_utils",
        ":tree_diagnostics",
    ],
)
