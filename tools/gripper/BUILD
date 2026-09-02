load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "control_gripper_lib",
    srcs = ["control_gripper.py"],
    deps = [
        "//src/hardware:gripper",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:execution",
    ],
)

py_binary(
    name = "control_gripper",
    srcs = ["control_gripper.py"],
    main = "control_gripper.py",
    deps = [
        ":control_gripper_lib",
    ],
)
