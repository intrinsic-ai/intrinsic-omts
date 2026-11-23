load("@bazel_skylib//:bzl_library.bzl", "bzl_library")

package(default_visibility = ["//visibility:public"])

bzl_library(
    name = "defs_bzl",
    srcs = ["defs.bzl"],
    deps = ["//private:wrapped_executable_bzl"],
)

exports_files(
    # Used by defs.bzl
    srcs = ["vmpool_test_wrapper.sh"],
)
