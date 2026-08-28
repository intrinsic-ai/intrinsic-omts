load("@ioc//bazel:python.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "omts_app",
    srcs = ["main.py"],
    main = "main.py",
    srcs_version = "PY3",
    deps = [
        "//src/behaviors",
        "//src/core",
        "//src/hardware",
        "//src/utils",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:execution",
    ],
)
