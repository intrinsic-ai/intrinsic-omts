load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "motions",
    srcs = ["motions.py"],
    deps = [
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "grasp_planning",
    srcs = ["grasp_planning.py"],
    deps = [
        ":motions",
        "//src/hardware:grasp_planner",
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "pick",
    srcs = ["pick.py"],
    deps = [
        ":motions",
        "//src/core:infeed",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "load_machine",
    srcs = ["load_machine.py"],
    deps = [
        ":motions",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "machining",
    srcs = ["machining.py"],
    deps = [
        ":motions",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "unload_machine",
    srcs = ["unload_machine.py"],
    deps = [
        ":motions",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "return_infeed",
    srcs = ["return_infeed.py"],
    deps = [
        ":motions",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
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
        "//src/core:infeed",
        "//src/core:workpiece",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "behaviors",
    srcs = ["__init__.py"],
    deps = [
        ":grasp_planning",
        ":load_machine",
        ":machine_tending_bt",
        ":machining",
        ":motions",
        ":pick",
        ":return_infeed",
        ":unload_machine",
    ],
)
