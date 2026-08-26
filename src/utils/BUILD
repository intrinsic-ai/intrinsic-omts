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
    name = "utils",
    srcs = ["__init__.py"],
    srcs_version = "PY3",
    deps = [
        ":math_utils",
    ],
)
