load("@ioc//bazel:python.bzl", "py_library")

# Deliberately NOT //visibility:public. Nothing in src/ or tools/ may depend on
# this integration: OMTS must keep building and running with intrinsic-moveit
# absent. See README.md in this directory.
package(default_visibility = ["//third_party/intrinsic_moveit:__subpackages__"])

py_library(
    name = "moveit_grasp_planner",
    srcs = ["moveit_grasp_planner.py"],
    deps = [
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "moveit_grasp_planning",
    srcs = ["moveit_grasp_planning.py"],
    deps = [
        ":moveit_grasp_planner",
        "//src/behaviors:motions",
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)

py_library(
    name = "moveit_grasp_tour",
    srcs = ["moveit_grasp_tour.py"],
    deps = [
        ":moveit_grasp_planner",
        ":moveit_grasp_planning",
        "//src/behaviors:motions",
        "//src/hardware:robot",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:behavior_tree",
    ],
)
