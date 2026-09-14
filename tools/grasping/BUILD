load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "plan_and_move_lib",
    srcs = ["plan_and_move.py"],
    deps = [
        "//src/behaviors:grasp_planning",
        "//src/hardware:grasp_planner",
        "//src/hardware:robot",
        "@ioc//google3/intrinsic/world/public/proto:object_world_refs_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
    ],
)

py_binary(
    name = "plan_and_move",
    srcs = ["plan_and_move.py"],
    main = "plan_and_move.py",
    deps = [
        ":plan_and_move_lib",
    ],
)
