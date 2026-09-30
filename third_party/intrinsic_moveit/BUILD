load("@intrinsic-core//bazel:python.bzl", "py_library")

# Deliberately NOT //visibility:public. Only //src/hardware:grasp_planners, the
# factory that constructs the configured grasp planner, may depend on this
# integration; nothing else in src/ or tools/ may. See README.md in this
# directory.
package(default_visibility = ["//third_party/intrinsic_moveit:__subpackages__"])

py_library(
    name = "moveit_grasp_planner",
    srcs = ["moveit_grasp_planner.py"],
    visibility = [
        "//src/hardware:__pkg__",
        "//third_party/intrinsic_moveit:__subpackages__",
    ],
    deps = [
        "//src/hardware:grasping",
        "//src/utils:script_utils",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
    ],
)

py_library(
    name = "moveit_grasp_planning",
    srcs = ["moveit_grasp_planning.py"],
    deps = [
        ":moveit_grasp_planner",
        "//src/behaviors:motions",
        "//src/hardware:robot",
        "@intrinsic-core//intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)
