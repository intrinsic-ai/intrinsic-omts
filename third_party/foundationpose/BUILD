# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("@rules_license//rules:license.bzl", "license")
load("@rules_license//rules:package_info.bzl", "package_info")
load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_uv//uv:pip.bzl", "pip_compile")
load(":build_defs.bzl", "triton_python_env_tar")

package(
    default_applicable_licenses = [":license"],
    default_visibility = ["//visibility:public"],
)

license(
    name = "license",
    license_kinds = [
        "@rules_license//licenses/spdx:Apache-2.0",
    ],
    license_text = "LICENSE",
)

package_info(
    name = "package_info",
    package_name = "FoundationPose",
    package_url = "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation",
    package_version = "release-3.2",
)

exports_files([
    "LICENSE",
    "isaac_ros_pose_estimation.BUILD",
    "model.py",
    "nvdiffrast.patch",
    "requirements.in",
    "requirements.txt",
])

pip_compile(
    name = "requirements",
    extra_args = [
        "--python-version=3.12",
    ],
    python_platform = "x86_64-unknown-linux-gnu",
    requirements_in = ":requirements.in",
    requirements_txt = ":requirements.txt",
    # Disable the automatically generated test target as it is slow, requires
    # network access and adds unnecessary dependencies on external services we
    # do not control.
    tags = ["manual"],
)

py_binary(
    name = "build_triton_env",
    srcs = ["build_triton_env.py"],
    visibility = ["//visibility:private"],
)

cuda_library(
    name = "foundationpose_render_cuda",
    srcs = [
        "cpp/foundationpose_render.cu",
    ],
    hdrs = [
        "cpp/foundationpose_render.cu.hpp",
        "cpp/foundationpose_utils.hpp",
    ],
    copts = ["-DPYBIND11"],
    includes = ["cpp"],
    deps = [
        "@com_gitlab_libeigen_eigen//:eigen",
        "@cuda//:cuda_headers",
        "@isaac_ros_pose_estimation//:nvdiffrast",
    ],
    alwayslink = True,
)

cc_binary(
    name = "foundationpose_cpp.so",
    srcs = [
        "cpp/foundationpose_binding.cpp",
        "cpp/foundationpose_render.cu.hpp",
        "cpp/foundationpose_render_helpers.cpp",
        "cpp/foundationpose_render_helpers.hpp",
        "cpp/foundationpose_sampling_helpers.cpp",
        "cpp/foundationpose_sampling_helpers.hpp",
        "cpp/foundationpose_utils.hpp",
    ],
    copts = [
        "-DPYBIND11",
        "-fexceptions",
        "-fvisibility=hidden",
    ],
    features = [
        "-use_header_modules",
        "-parse_headers",
    ],
    includes = ["cpp"],
    linkopts = ["-Wl,-Bsymbolic"],
    linkshared = True,
    visibility = ["//visibility:private"],
    deps = [
        ":foundationpose_render_cuda",
        "@com_gitlab_libeigen_eigen//:eigen",
        "@cuda//:cuda_runtime_static",
        "@isaac_ros_pose_estimation//:nvdiffrast",
        "@pybind11",
    ],
)

triton_python_env_tar(
    name = "env_tar_gz",
    out = "env.tar.gz",
    so_file = ":foundationpose_cpp.so",
)
