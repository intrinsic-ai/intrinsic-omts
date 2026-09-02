load("@ioc//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "omts_app",
    srcs = ["main.py"],
    main = "main.py",
    srcs_version = "PY3",
    deps = [
        "//src/behaviors",
        "//src/core",
        "//src/hardware",
        "//src/utils",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/solutions:execution",
    ],
)

py_library(
    name = "pick_and_place_building_block_lib",
    srcs = ["pick_and_place_building_block.py"],
    srcs_version = "PY3",
    deps = [
        "//src/behaviors:building_block_bt",
        "//src/behaviors:motions",
        "//src/core:types",
        "//src/hardware:gripper",
        "//src/hardware:machine",
        "//src/hardware:robot",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_absl_py//absl/logging",
        "@ioc//google3/intrinsic/assets:id_utils_py",
        "@ioc//google3/intrinsic/math/python:data_types",
        "@ioc//google3/intrinsic/math/python:proto_conversion",
        "@ioc//google3/intrinsic/perception/public/proto/v1:pose_estimator_id_py_pb2",
        "@ioc//google3/intrinsic/solutions:deployments",
        "@ioc//google3/intrinsic/world/public/proto:object_world_updates_py_pb2",
    ],
)

py_binary(
    name = "omts_pp_app",
    srcs = ["pick_and_place_building_block.py"],
    main = "pick_and_place_building_block.py",
    srcs_version = "PY3",
    deps = [
        ":pick_and_place_building_block_lib",
    ],
)

alias(
    name = "pick_and_place_building_block",
    actual = ":omts_pp_app",
)
