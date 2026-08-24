load("@ioc//bazel:python.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "test_tray",
    srcs = ["test_tray.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:tray",
        "//src/core:types",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_workpiece",
    srcs = ["test_workpiece.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:types",
        "//src/core:workpiece",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_infeed",
    srcs = ["test_infeed.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:infeed",
        "//src/core:tray",
        "//src/core:types",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_hardware_adapters",
    srcs = ["test_hardware_adapters.py"],
    srcs_version = "PY3",
    deps = [
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_behaviors",
    srcs = ["test_behaviors.py"],
    srcs_version = "PY3",
    deps = [
        "//src/behaviors:machine_tending_bt",
        "//src/core:infeed",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)
