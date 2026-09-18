load("@intrinsic-core//bazel:python.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "omts_app",
    srcs = ["main.py"],
    data = ["//configs:all_configs"],
    main = "main.py",
    deps = [
        "//src/behaviors",
        "//src/core",
        "//src/hardware",
        "//src/utils",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)
