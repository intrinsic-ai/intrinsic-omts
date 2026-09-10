load("@ioc//bazel:python.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "run_pose_estimation",
    srcs = ["run_pose_estimation.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@ioc//google3/intrinsic/assets:id_utils_py",
        "@ioc//google3/intrinsic/math/python:proto_conversion",
        "@ioc//google3/intrinsic/perception/public/proto/v1:pose_estimator_id_py_pb2",
        "@ioc//google3/intrinsic/perception/skills/multi_view:estimate_pose_multi_view_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:execution",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:provided",
    ],
)

py_binary(
    name = "register_using_train_service",
    srcs = ["register_using_train_service.py"],
    deps = [
        "@com_github_grpc_grpc//src/python/grpcio/grpc:grpcio",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_googleapis//google/longrunning:operations_py_proto",
        "@com_google_protobuf//:protobuf_python",
        "@intrinsic_apis//intrinsic/perception/proto/v1:pose_range_py_pb2",
        "@ioc//google3/intrinsic/assets/data/proto/v1:data_asset_py_pb2",
        "@ioc//google3/intrinsic/assets/proto:id_py_pb2",
        "@ioc//google3/intrinsic/assets/proto:installed_assets_pb2_grpc",
        "@ioc//google3/intrinsic/assets/proto:installed_assets_py_pb2",
        "@ioc//google3/intrinsic/assets/proto:view_py_pb2",
        "@ioc//google3/intrinsic/perception/proto/v1:pose_estimation_config_py_pb2",
        "@ioc//google3/intrinsic/perception/proto/v1:target_py_pb2",
        "@ioc//google3/intrinsic/perception/proto/v1:train_service_py_pb2",
        "@ioc//google3/intrinsic/perception/proto/v1:train_service_py_pb2_grpc",
        "@ioc//google3/intrinsic/scene/proto/v1:scene_object_py_pb2",
        "@ioc//incode/intrinsic_sdk/intrinsic/solutions:deployments",
        "@ioc//incode/perception/ioc_train_service:ioc_train_service_lib",
        "@ioc//incode/perception/ioc_train_service/proto:ioc_pose_estimator_params_py_pb2",
    ],
)
