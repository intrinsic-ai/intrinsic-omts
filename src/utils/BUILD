load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "math_utils",
    srcs = ["math_utils.py"],
    deps = [
        "//src/core:types",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
    ],
)

py_library(
    name = "script_utils",
    srcs = ["script_utils.py"],
    deps = [
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "dynamic_frame_calculator",
    srcs = ["dynamic_frame_calculator.py"],
    deps = [
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_library(
    name = "execution_utils",
    srcs = ["execution_utils.py"],
    deps = [
        "//src/core:types",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:execution",
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
