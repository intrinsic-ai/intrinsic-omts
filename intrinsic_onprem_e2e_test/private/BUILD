load("@bazel_skylib//:bzl_library.bzl", "bzl_library")

bzl_library(
    name = "wrapped_executable_bzl",
    srcs = ["wrapped_executable.bzl"],
    visibility = ["//:__pkg__"],
    deps = ["@bazel_lib//lib:paths"],
)
