load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "robot",
    srcs = [
        "gripper.py",
        "machine.py",
        "robot.py",
        "vision.py",
    ],
    deps = [
        "//src/core:types",
        "//src/core:world",
        "//src/utils:execution_utils",
        "//src/utils:script_utils",
        "@intrinsic_apis//intrinsic/math/proto:point_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:pose_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:vector3_py_pb2",
        "@intrinsic_apis//intrinsic/motion_planning/proto/v1:geometric_constraints_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_estimator_id_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:provided",
        "@ioc//intrinsic/assets:id_utils_py",
        "@ioc//intrinsic/manipulation/skills/force:move_to_contact_proto_py_pb2",
        "@ioc//intrinsic/perception/skills/multi_view:estimate_pose_multi_view_py_pb2",
    ],
)

py_library(
    name = "gripper",
    deps = [":robot"],
)

py_library(
    name = "machine",
    deps = [":robot"],
)

py_library(
    name = "vision",
    deps = [":robot"],
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
