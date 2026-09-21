load("@ioc//bazel:python.bzl", "py_test")

package(default_visibility = ["//third_party/intrinsic_moveit:__subpackages__"])

py_test(
    name = "test_moveit_grasp_planning",
    srcs = ["test_moveit_grasp_planning.py"],
    deps = [
        "//src/hardware:robot",
        "//third_party/intrinsic_moveit:moveit_grasp_planner",
        "//third_party/intrinsic_moveit:moveit_grasp_planning",
        "//third_party/intrinsic_moveit/tools:moveit_plan_grasp_and_move_lib",
        "@com_google_absl_py//absl/testing:absltest",
    ],
)
