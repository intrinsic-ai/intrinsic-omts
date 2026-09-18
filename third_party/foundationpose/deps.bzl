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

"""Module extension for FoundationPose third-party dependencies and Python 3.12 wheels."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

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

    http_file(
        name = "fp_py312_numpy",
        downloaded_file_path = "numpy-1.26.4-cp312-cp312-manylinux_2_17_x86_64.manylinux2014_x86_64.whl",
        sha256 = "675d61ffbfa78604709862923189bad94014bef562cc35cf61d3a07bba02a7ed",
        urls = [
            "https://files.pythonhosted.org/packages/0f/50/de23fde84e45f5c4fda2488c759b69990fd4512387a8632860f3ac9cd225/numpy-1.26.4-cp312-cp312-manylinux_2_17_x86_64.manylinux2014_x86_64.whl",
        ],
    )

    http_file(
        name = "fp_py312_scipy",
        downloaded_file_path = "scipy-1.17.1-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl",
        sha256 = "02ae3b274fde71c5e92ac4d54bc06c42d80e399fec704383dcd99b301df37458",
        urls = [
            "https://files.pythonhosted.org/packages/01/8e/1e35281b8ab6d5d72ebe9911edcdffa3f36b04ed9d51dec6dd140396e220/scipy-1.17.1-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl",
        ],
    )

    http_file(
        name = "fp_py312_onnxruntime_gpu",
        downloaded_file_path = "onnxruntime_gpu-1.29.0-cp312-cp312-manylinux_2_28_x86_64.whl",
        sha256 = "8ccd052ca4de21b2a335f3503a283126163b6fab9dba29d1d9cbf3fa55c51d28",
        urls = [
            "https://files.pythonhosted.org/packages/87/7b/764082683bbc3b3351a3864ffb87be11aa13642913f78569fa72b934e319/onnxruntime_gpu-1.29.0-cp312-cp312-manylinux_2_28_x86_64.whl",
        ],
    )

    http_file(
        name = "fp_py312_opencv_python_headless",
        downloaded_file_path = "opencv_python_headless-4.11.0.86-cp37-abi3-manylinux_2_17_x86_64.manylinux2014_x86_64.whl",
        sha256 = "0e0a27c19dd1f40ddff94976cfe43066fbbe9dfbb2ec1907d66c19caef42a57b",
        urls = [
            "https://files.pythonhosted.org/packages/dd/5c/c139a7876099916879609372bfa513b7f1257f7f1a908b0bdc1c2328241b/opencv_python_headless-4.11.0.86-cp37-abi3-manylinux_2_17_x86_64.manylinux2014_x86_64.whl",
        ],
    )

    http_file(
        name = "fp_py312_pillow",
        downloaded_file_path = "pillow-12.3.0-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl",
        sha256 = "78cb2c6865a35ab8ff8b75fd122f6033b92a62c82801110e48ddd6c936a45d91",
        urls = [
            "https://files.pythonhosted.org/packages/84/21/a35af28dcc61f37ed850a2d64c65c701321dfbf25085e469d5559360cbbf/pillow-12.3.0-cp312-cp312-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl",
        ],
    )

    http_file(
        name = "fp_py312_trimesh",
        downloaded_file_path = "trimesh-5.0.0-py3-none-any.whl",
        sha256 = "51ec67d7f9f74b918f2a695da5fd511b9c084e96307648ae83f45b58f13f8009",
        urls = [
            "https://files.pythonhosted.org/packages/37/82/19a03ba344ecb66ea8caab697b3059e0fbea576420f99945944c479caa78/trimesh-5.0.0-py3-none-any.whl",
        ],
    )

    http_file(
        name = "fp_py312_flatbuffers",
        downloaded_file_path = "flatbuffers-25.12.19-py2.py3-none-any.whl",
        sha256 = "7634f50c427838bb021c2d66a3d1168e9d199b0607e6329399f04846d42e20b4",
        urls = [
            "https://files.pythonhosted.org/packages/e8/2d/d2a548598be01649e2d46231d151a6c56d10b964d94043a335ae56ea2d92/flatbuffers-25.12.19-py2.py3-none-any.whl",
        ],
    )

    http_file(
        name = "fp_py312_protobuf",
        downloaded_file_path = "protobuf-7.35.1-cp310-abi3-manylinux2014_x86_64.whl",
        sha256 = "74758715c53d7158fb76caf4f0cfdacc5329a4b1bb994f865d6cf302d413a1c4",
        urls = [
            "https://files.pythonhosted.org/packages/e4/be/5b3cfe508bfab6761414ff944e3366eb13be4fd71efcd69450f89ba39f43/protobuf-7.35.1-cp310-abi3-manylinux2014_x86_64.whl",
        ],
    )

foundationpose_deps = module_extension(
    implementation = _foundationpose_deps_impl,
)
