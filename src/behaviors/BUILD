load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "motions",
    srcs = ["motions.py"],
    srcs_version = "PY3",
    deps = [
        "//src/hardware:robot",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "pick",
    srcs = ["pick.py"],
    srcs_version = "PY3",
    deps = [
        ":motions",
        "//src/core:infeed",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "load_machine",
    srcs = ["load_machine.py"],
    srcs_version = "PY3",
    deps = [
        ":motions",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "machining",
    srcs = ["machining.py"],
    srcs_version = "PY3",
    deps = [
        ":motions",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "unload_machine",
    srcs = ["unload_machine.py"],
    srcs_version = "PY3",
    deps = [
        ":motions",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "return_infeed",
    srcs = ["return_infeed.py"],
    srcs_version = "PY3",
    deps = [
        ":motions",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:robot",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "machine_tending_bt",
    srcs = ["machine_tending_bt.py"],
    srcs_version = "PY3",
    deps = [
        ":load_machine",
        ":machining",
        ":pick",
        ":return_infeed",
        ":unload_machine",
        "//src/core:infeed",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "building_block_bt",
    srcs = ["building_block_bt.py"],
    srcs_version = "PY3",
    deps = [
        ":motions",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "behaviors",
    srcs = ["__init__.py"],
    srcs_version = "PY3",
    deps = [
        ":building_block_bt",
        ":load_machine",
        ":machine_tending_bt",
        ":machining",
        ":motions",
        ":pick",
        ":return_infeed",
        ":unload_machine",
    ],
)
