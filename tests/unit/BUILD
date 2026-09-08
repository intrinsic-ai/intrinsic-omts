load("@ioc//bazel:python.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "test_tray",
    srcs = ["test_tray.py"],
    deps = [
        "//src/core:tray",
        "//src/core:types",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_workpiece",
    srcs = ["test_workpiece.py"],
    deps = [
        "//src/core:types",
        "//src/core:workpiece",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_infeed",
    srcs = ["test_infeed.py"],
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
    deps = [
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_behaviors",
    srcs = ["test_behaviors.py"],
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

py_test(
    name = "test_move_to_frame",
    srcs = ["test_move_to_frame.py"],
    deps = [
        "//src/hardware:robot",
        "//tools/jogging:move_to_frame_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_store_frame",
    srcs = ["test_store_frame.py"],
    deps = [
        "//tools/jogging:store_frame_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/math/python:data_types",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
    ],
)

py_test(
    name = "test_script_utils",
    srcs = ["test_script_utils.py"],
    deps = [
        "//src/utils:dynamic_frame_calculator",
        "//src/utils:script_utils",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_control_gripper",
    srcs = ["test_control_gripper.py"],
    deps = [
        "//src/hardware:gripper",
        "//tools/gripper:control_gripper_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//google3/intrinsic/solutions:execution",
    ],
)
