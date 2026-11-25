load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")

# Scoped exclusively to the root solution asset definition and unit config
# validation; nothing in src/ or tools/ may depend on this package.
package(default_visibility = [
    "//:__pkg__",
    "//tests/unit:__pkg__",
])

proto_library(
    name = "moveit_planning_service_proto",
    srcs = ["moveit_planning_service.proto"],
)

py_proto_library(
    name = "moveit_planning_service_py_pb2",
    deps = [":moveit_planning_service_proto"],
)
