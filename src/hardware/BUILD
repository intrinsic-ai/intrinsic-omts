load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "robot",
    srcs = ["robot.py"],
    deps = [
        "//src/core:config",
        "//src/core:types",
        "//src/utils:math_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic/manipulation/skills/force:move_to_contact_proto_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:point_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:pose_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:vector3_py_pb2",
        "@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:collision_action_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
    ],
)

py_library(
    name = "gripper",
    srcs = ["gripper.py"],
    deps = [
        "//src/core:config",
        "//src/utils:math_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "machine",
    srcs = ["machine.py"],
    deps = [
        "//src/core:config",
        "//src/utils:math_utils",
        "//src/utils:script_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic/world/proto:object_world_updates_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
    ],
)

py_library(
    name = "vision",
    srcs = ["vision.py"],
    deps = [
        "//src/core:config",
        "//src/utils:dynamic_frame_calculator",
        "//src/utils:script_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:proto_building",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:provided",
        "@intrinsic-core//intrinsic/assets:id_utils_py",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_estimator_id_py_pb2",
    ],
)

py_library(
    name = "hardware",
    srcs = ["__init__.py"],
    deps = [
        ":gripper",
        ":machine",
        ":robot",
        ":vision",
    ],
)
