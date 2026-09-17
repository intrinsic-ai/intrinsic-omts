load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "cli",
    srcs = ["cli.py"],
)

py_library(
    name = "common",
    srcs = ["__init__.py"],
    deps = [":cli"],
)
