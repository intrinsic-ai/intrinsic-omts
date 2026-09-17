load("@intrinsic-core//bazel:python.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "run_pose_estimation",
    srcs = ["run_pose_estimation.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/math/python:proto_conversion",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:execution",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:provided",
        "@intrinsic-core//intrinsic/assets:id_utils_py",
        "@intrinsic-core//intrinsic/perception/skills/multi_view:estimate_pose_multi_view_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_estimator_id_py_pb2",
    ],
)

py_library(
    name = "register_using_train_service_lib",
    srcs = ["register_using_train_service.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_googleapis//google/longrunning:operations_py_proto",
        "@com_google_protobuf//:protobuf_python",
        "@intrinsic-core//incode/intrinsic_perception/intrinsic/perception/service/ioc_train_service:ioc_train_service_lib",
        "@intrinsic-core//incode/intrinsic_perception/intrinsic/perception/service/ioc_train_service/proto:ioc_pose_estimator_params_py_pb2",
        "@intrinsic-core//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@intrinsic-core//intrinsic/assets:id_utils_py",
        "@intrinsic-core//intrinsic/assets/proto:installed_assets_pb2_grpc",
        "@intrinsic-core//intrinsic/assets/proto:installed_assets_py_pb2",
        "@intrinsic_apis//intrinsic/assets/data/proto/v1:data_asset_py_pb2",
        "@intrinsic_apis//intrinsic/assets/proto:id_py_pb2",
        "@intrinsic_apis//intrinsic/assets/proto:view_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_estimation_config_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_range_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:target_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:train_service_py_pb2",
        "@intrinsic_apis//intrinsic/perception/proto/v1:train_service_py_pb2_grpc",
        "@intrinsic_apis//intrinsic/scene/proto/v1:scene_object_py_pb2",
    ],
)

py_binary(
    name = "register_using_train_service",
    srcs = ["register_using_train_service.py"],
    main = "register_using_train_service.py",
    deps = [
        ":register_using_train_service_lib",
    ],
)
