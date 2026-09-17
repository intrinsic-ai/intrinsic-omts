load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "omts_app_lib",
    srcs = ["main.py"],
    data = ["//configs:app_configs"],
    deps = [
        "//src/behaviors",
        "//src/core",
        "//src/hardware",
        "//src/utils",
        "@ai_intrinsic_sdks_pip_deps//pyyaml",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
    ],
)

py_binary(
    name = "omts_app",
    srcs = ["main.py"],
    main = "main.py",
    deps = [
        ":omts_app_lib",
    ],
)
