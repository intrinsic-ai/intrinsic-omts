load("@intrinsic-core//bazel:container.bzl", "container_layer")
load("@intrinsic-core//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "foundation_pose_nvidia_lib",
    srcs = glob(
        [
            "foundation-pose-inference-library/python/src/foundation_pose_nvidia/**/*.py",
        ],
        allow_empty = True,
    ),
    imports = [
        "foundation-pose-inference-library/python/src",
    ],
)

py_library(
    name = "perception_pipeline_lib",
    srcs = glob(
        ["foundationpose_perception_pipeline/src/foundationpose_perception_pipeline/**/*.py"],
        allow_empty = True,
    ),
    data = glob(
        ["foundationpose_perception_pipeline/config/*.yaml"],
        allow_empty = True,
    ),
    imports = ["foundationpose_perception_pipeline/src"],
    deps = [
        ":foundation_pose_nvidia_lib",
    ],
)

container_layer(
    name = "fp_library_layer",
    data_path = "build/foundation-pose-inference-library/build",
    directory = "/usr/lib/x86_64-linux-gnu",
    files = glob(
        [
            "build/foundation-pose-inference-library/build/*.so*",
        ],
        allow_empty = True,
    ),
)

container_layer(
    name = "models_layer",
    data_path = "build/models",
    directory = "/models",
    files = glob(
        [
            "build/models/*.onnx",
            "build/models/*.txt.gz",
        ],
        allow_empty = True,
    ),
)

container_layer(
    name = "engine_cache_layer",
    data_path = "build/models/engine_cache_bundle",
    directory = "/models/engine_cache",
    files = glob(
        [
            "build/models/engine_cache_bundle/*.plan",
            "build/models/engine_cache_bundle/*.plan.json",
        ],
        allow_empty = True,
    ),
)

container_layer(
    name = "bindings_layer",
    data_path = "build/bindings",
    directory = "/bindings",
    files = glob(
        ["build/bindings/**"],
        allow_empty = True,
        exclude = ["build/bindings/**/__pycache__/**"],
    ),
)

alias(
    name = "nvidia_pose_estimator_service_asset",
    actual = "@intrinsic-core//intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator:nvidia_pose_estimator_service_asset",
)
