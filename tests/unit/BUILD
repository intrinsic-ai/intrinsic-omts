load("@ioc//bazel:python.bzl", "py_test")
load("@rules_shell//shell:sh_test.bzl", "sh_test")

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
        "//src/core:workcell",
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
        "//src/core:world",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_behaviors",
    srcs = ["test_behaviors.py"],
    deps = [
        "//src:omts_app_lib",
        "//src/behaviors:machine_tending_bt",
        "//src/behaviors:motions",
        "//src/core:infeed",
        "//src/core:types",
        "//src/core:workpiece",
        "//src/core:world",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_pick_subtree",
    srcs = ["test_pick_subtree.py"],
    deps = [
        "//src/behaviors:pick",
        "//src/core:infeed",
        "//src/core:tray",
        "//src/core:workpiece",
        "//src/core:world",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "//src/hardware:vision",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_load_machine_subtree",
    srcs = ["test_load_machine_subtree.py"],
    deps = [
        "//src/behaviors:load_machine",
        "//src/core:workpiece",
        "//src/core:world",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_unload_machine_subtree",
    srcs = ["test_unload_machine_subtree.py"],
    deps = [
        "//src/behaviors:unload_machine",
        "//src/core:workpiece",
        "//src/core:world",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_return_infeed_subtree",
    srcs = ["test_return_infeed_subtree.py"],
    deps = [
        "//src/behaviors:return_infeed",
        "//src/core:workpiece",
        "//src/core:world",
        "//src/hardware:gripper",
        "//src/hardware:robot",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_machining_subtree",
    srcs = ["test_machining_subtree.py"],
    deps = [
        "//src/behaviors:machining",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_move_to_frame",
    srcs = ["test_move_to_frame.py"],
    deps = [
        "//src/hardware:robot",
        "//tools/jogging:move_to_frame_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_store_frame",
    srcs = ["test_store_frame.py"],
    deps = [
        "//tools/jogging:store_frame_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/math/python:data_types",
        "@ioc//intrinsic/world/proto:object_world_updates_py_pb2",
    ],
)

py_test(
    name = "test_script_math",
    srcs = ["test_script_math.py"],
    deps = [
        "//src/utils:math_utils",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_test(
    name = "test_script_utils",
    srcs = ["test_script_utils.py"],
    deps = [
        "//src/core:world",
        "//src/utils:dynamic_frame_calculator",
        "//src/utils:math_utils",
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
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_store_joint_config",
    srcs = ["test_store_joint_config.py"],
    deps = [
        "//tools/jogging:store_joint_config_lib",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_move_to_joint",
    srcs = ["test_move_to_joint.py"],
    deps = [
        "//tools/jogging:move_to_joint_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_apply_scene_updates",
    srcs = ["test_apply_scene_updates.py"],
    deps = [
        "//tools/world:apply_scene_updates_lib",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

py_test(
    name = "test_execution_utils",
    srcs = ["test_execution_utils.py"],
    deps = [
        "//src/core:types",
        "//src/utils:execution_utils",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_world_configs",
    srcs = ["test_world_configs.py"],
    data = ["//configs:all_world_updates"],
    deps = [
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//intrinsic/world/proto:object_world_updates_py_pb2",
    ],
)

py_test(
    name = "test_calibrate_camera",
    srcs = ["test_calibrate_camera.py"],
    deps = [
        "//tools/calibration:calibrate_camera_lib",
        "//tools/world:apply_scene_updates_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@intrinsic_apis//intrinsic/perception/proto/v1:camera_to_robot_calibration_py_pb2",
        "@ioc//intrinsic/world/proto:object_world_updates_py_pb2",
    ],
)

py_test(
    name = "test_update_robot_kinematics",
    srcs = ["test_update_robot_kinematics.py"],
    deps = [
        "//tools/calibration:update_robot_kinematics_lib",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//intrinsic/world/service/robot_calibration:robot_update_service_py_pb2",
        "@ioc//intrinsic/world/service/robot_calibration:robot_update_service_py_pb2_grpc",
    ],
)

py_test(
    name = "test_control_machine",
    srcs = ["test_control_machine.py"],
    deps = [
        "//src/hardware:machine",
        "//tools/machine:control_machine_lib",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_test(
    name = "test_dynamic_frame_calculator",
    srcs = ["test_dynamic_frame_calculator.py"],
    deps = [
        "//src/utils:dynamic_frame_calculator",
        "//src/utils:math_utils",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/math/python:data_types",
    ],
)

py_test(
    name = "test_world",
    srcs = ["test_world.py"],
    deps = [
        "//src/core:types",
        "//src/core:world",
        "@com_google_absl_py//absl/testing:absltest",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_test(
    name = "test_tree_diagnostics",
    srcs = ["test_tree_diagnostics.py"],
    deps = [
        "//src/utils:tree_diagnostics",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)

sh_test(
    name = "test_omts_ctl",
    size = "small",
    srcs = ["test_omts_ctl.sh"],
    data = [
        "//tools:omts_ctl.sh",
        "//tools/ctl:ctl_libs",
    ],
)
