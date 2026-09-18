load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "control_machine_lib",
    srcs = ["control_machine.py"],
    deps = [
        "//src/core:config",
        "//src/hardware:machine",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_binary(
    name = "control_machine",
    srcs = ["control_machine.py"],
    main = "control_machine.py",
    deps = [
        ":control_machine_lib",
    ],
)
