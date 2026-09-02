load("@ioc//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "robot",
    srcs = ["robot.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:types",
        "@intrinsic_apis//intrinsic/math/proto:point_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:pose_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_py_pb2",
        "@intrinsic_apis//intrinsic/math/proto:vector3_py_pb2",
        "@ioc//google3/intrinsic/manipulation/skills/force:move_to_contact_proto_py_pb2",
        "@ioc//google3/intrinsic/motion_planning/public/proto/v1:geometric_constraints_py_pb2",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/world/public/proto:object_world_refs_py_pb2",
    ],
)

py_library(
    name = "gripper",
    srcs = ["gripper.py"],
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "machine",
    srcs = ["machine.py"],
    srcs_version = "PY3",
    deps = [
        "@ioc//google3/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "vision",
    srcs = ["vision.py"],
    srcs_version = "PY3",
    deps = [
        "//src/core:types",
        "//src/utils:dynamic_frame_calculator",
        "//src/utils:script_utils",
        "@com_google_absl_py//absl/logging",
        "@ioc//google3/intrinsic/assets:id_utils_py",
        "@ioc//google3/intrinsic/assets/proto:id_py_pb2",
        "@ioc//google3/intrinsic/math/python:proto_conversion",
        "@ioc//google3/intrinsic/perception/public/proto/v1:pose_estimator_id_py_pb2",
        "@ioc//google3/intrinsic/perception/skills/multi_view:estimate_pose_multi_view_py_pb2",
        "@ioc//google3/intrinsic/solutions:behavior_tree",
        "@ioc//google3/intrinsic/solutions:cel",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:proto_building",
        "@ioc//google3/intrinsic/solutions:provided",
        "@ioc//google3/intrinsic/world/public/proto:object_world_refs_py_pb2",
    ],
)

py_library(
    name = "hardware",
    srcs = ["__init__.py"],
    srcs_version = "PY3",
    deps = [
        ":gripper",
        ":machine",
        ":robot",
        ":vision",
    ],
)
