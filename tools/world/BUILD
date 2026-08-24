load("@ioc//bazel:python.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "inspect_world",
    srcs = ["inspect_world.py"],
    main = "inspect_world.py",
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/solutions:deployments",
    ],
)
