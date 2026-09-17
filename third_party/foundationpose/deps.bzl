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

"""Module extension for FoundationPose third-party C++/CUDA dependencies and tools."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _foundationpose_deps_impl(_ctx):
    http_archive(
        name = "isaac_ros_pose_estimation",
        build_file = "//third_party/isaac_ros_pose_estimation:isaac_ros_pose_estimation.BUILD",
        patch_cmds = [
            "sed -i 's/%lanemask_/%%lanemask_/g' isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/Util.inl",
            "sed -i 's/^void \\(.*RasterKernel\\|triangleSetupKernel\\)/__global__ void \\1/' isaac_ros_gxf_extensions/gxf_isaac_foundationpose/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/RasterImpl.cpp",
        ],
        sha256 = "5339cf3cfe04a53785926058032a9dac293879cb9f0812dc98f06610cbc4dd93",
        strip_prefix = "isaac_ros_pose_estimation-9caca619bcc9d637b3107e17c1a77132c9d7863b",
        urls = [
            "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation/archive/9caca619bcc9d637b3107e17c1a77132c9d7863b.tar.gz",
        ],
    )

    http_archive(
        name = "patchelf",
        build_file = "//third_party/isaac_ros_pose_estimation:patchelf.BUILD",
        sha256 = "ce84f2447fb7a8679e58bc54a20dc2b01b37b5802e12c57eece772a6f14bf3f0",
        urls = [
            "https://github.com/NixOS/patchelf/releases/download/0.18.0/patchelf-0.18.0-x86_64.tar.gz",
        ],
    )

foundationpose_deps = module_extension(
    implementation = _foundationpose_deps_impl,
)
