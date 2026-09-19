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

load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("@rules_license//rules:license.bzl", "license")
load("@rules_license//rules:package_info.bzl", "package_info")

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
    package_name = "isaac_ros_pose_estimation",
    package_url = "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation",
    package_version = "release-3.2",
)

cuda_library(
    name = "nvdiffrast",
    srcs = [
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/common.cpp",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/Buffer.cpp",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/CudaRaster.cpp",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/RasterImpl.cpp",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/RasterImpl.cu",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/interpolate.cu",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/rasterize.cu",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/texture.cu",
    ],
    hdrs = glob([
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/**/*.h",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/**/*.hpp",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/**/*.cuh",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/**/*.inl",
    ]),
    copts = [
        "-x",
        "cuda",
    ],
    includes = [
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose",
        "isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common",
    ],
    deps = [
        "@cuda//:cuda_headers",
    ],
    alwayslink = True,
)
