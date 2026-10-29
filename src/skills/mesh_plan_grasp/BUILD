load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@intrinsic-core//bazel:container.bzl", "container_image")
load("@intrinsic-core//bazel:python.bzl", "py_library")
load("@intrinsic-core//intrinsic_sdk/intrinsic/skills/build_defs:skill.bzl", "py_skill", "skill_manifest")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "mesh_plan_grasp_proto",
    srcs = ["mesh_plan_grasp.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/assets/proto:field_metadata_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:resolved_dependency_proto",
    ],
)

py_proto_library(
    name = "mesh_plan_grasp_py_pb2",
    deps = [":mesh_plan_grasp_proto"],
)

skill_manifest(
    name = "mesh_plan_grasp_manifest",
    src = "mesh_plan_grasp_manifest.textproto",
    deps = [":mesh_plan_grasp_proto"],
)

py_library(
    name = "mesh_plan_grasp",
    srcs = ["mesh_plan_grasp.py"],
    deps = [
        ":mesh_plan_grasp_py_pb2",
        "//src/grasping:candidates",
        "//src/utils:math_utils",
        "@ai_intrinsic_sdks_pip_deps//numpy",
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl/logging",
        "@intrinsic-core//intrinsic_sdk/intrinsic/geometry/proto:geometry_service_py_pb2",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:data_types",
        "@intrinsic-core//intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//intrinsic_sdk/intrinsic/motion_planning:motion_planner_client",
        "@intrinsic-core//intrinsic_sdk/intrinsic/skills/python:skill_interface",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_py_pb2",
        "@intrinsic_apis//intrinsic/motion_planning/proto:motion_target_py_pb2",
        "@intrinsic_apis//intrinsic/skills/proto:footprint_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2",
    ],
)

# Same base as @intrinsic-core's Python skills (see MODULE.bazel).
container_image(
    name = "python3_base_image",
    base = "@distroless_python3",
)

py_skill(
    name = "mesh_plan_grasp_skill",
    base_image = ":python3_base_image",
    manifest = ":mesh_plan_grasp_manifest",
    deps = [
        ":mesh_plan_grasp",
        ":mesh_plan_grasp_py_pb2",
    ],
)
