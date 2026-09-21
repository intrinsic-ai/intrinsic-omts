load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//third_party/intrinsic_moveit:__subpackages__"])

py_library(
    name = "moveit_plan_grasp_and_move_lib",
    srcs = ["moveit_plan_grasp_and_move.py"],
    deps = [
        "//src/hardware:robot",
        "//third_party/intrinsic_moveit:moveit_grasp_planner",
        "//third_party/intrinsic_moveit:moveit_grasp_planning",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
    ],
)

py_binary(
    name = "moveit_plan_grasp_and_move",
    srcs = ["moveit_plan_grasp_and_move.py"],
    main = "moveit_plan_grasp_and_move.py",
    deps = [
        ":moveit_plan_grasp_and_move_lib",
    ],
)
