load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//third_party/intrinsic_moveit:__subpackages__"])

py_library(
    name = "moveit_plan_and_move_lib",
    srcs = ["moveit_plan_and_move.py"],
    deps = [
        "//src/hardware:robot",
        "//third_party/intrinsic_moveit:moveit_grasp_planner",
        "//third_party/intrinsic_moveit:moveit_grasp_planning",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_binary(
    name = "moveit_plan_and_move",
    srcs = ["moveit_plan_and_move.py"],
    main = "moveit_plan_and_move.py",
    deps = [
        ":moveit_plan_and_move_lib",
    ],
)

py_library(
    name = "moveit_grasp_tour_lib",
    srcs = ["moveit_grasp_tour.py"],
    deps = [
        ":moveit_plan_and_move_lib",
        "//src/hardware:robot",
        "//third_party/intrinsic_moveit:moveit_grasp_planner",
        "//third_party/intrinsic_moveit:moveit_grasp_tour",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_binary(
    name = "moveit_grasp_tour",
    srcs = ["moveit_grasp_tour.py"],
    main = "moveit_grasp_tour.py",
    deps = [
        ":moveit_grasp_tour_lib",
    ],
)
