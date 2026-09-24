load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "motions",
    srcs = ["motions.py"],
    deps = [
        "//src/core:config",
        "//src/core:types",
        "//src/hardware:robot",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "pick",
    srcs = ["pick.py"],
    deps = [
        ":motions",
        "//src/core:config",
        "//src/core:infeed",
        "//src/hardware:grasping",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "load_machine",
    srcs = ["load_machine.py"],
    deps = [
        ":motions",
        "//src/core:config",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "machining",
    srcs = ["machining.py"],
    deps = [
        ":motions",
        "//src/core:config",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "unload_machine",
    srcs = ["unload_machine.py"],
    deps = [
        ":motions",
        "//src/core:config",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "return_infeed",
    srcs = ["return_infeed.py"],
    deps = [
        ":motions",
        "//src/core:config",
        "//src/hardware:gripper",
        "//src/hardware:robot",
        "//src/utils:random_placement",
        "//src/utils:script_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "machine_tending_bt",
    srcs = ["machine_tending_bt.py"],
    deps = [
        ":load_machine",
        ":machining",
        ":pick",
        ":return_infeed",
        ":unload_machine",
        "//src/core:config",
        "//src/core:infeed",
        "//src/hardware:grasping",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "behaviors",
    srcs = ["__init__.py"],
    deps = [
        ":load_machine",
        ":machine_tending_bt",
        ":machining",
        ":motions",
        ":pick",
        ":return_infeed",
        ":unload_machine",
    ],
)
