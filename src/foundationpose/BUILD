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

load("@bazel_skylib//rules:copy_file.bzl", "copy_file")
load("@intrinsic-core//incode/intrinsic_inference/assets/inference_service/bazel:intrinsic_mlmodel.bzl", "intrinsic_mlmodel")

package(default_visibility = ["//visibility:public"])

exports_files([
    "config.pbtxt",
])

[
    copy_file(
        name = name,
        src = src,
        out = out,
        allow_symlink = True,
    )
    for name, src, out in [
        ("refine_onnx", "@foundationpose_refine_onnx//file", "foundationpose_refine.onnx"),
        ("score_onnx", "@foundationpose_score_onnx//file", "foundationpose_score.onnx"),
        ("model_py", "//third_party/foundationpose:model.py", "model.py"),
        ("env_tar_gz", "//third_party/foundationpose:env_tar_gz", "env.tar.gz"),
    ]
]

intrinsic_mlmodel(
    name = "foundationpose_mlmodel",
    backend = "triton",
    config = ":config.pbtxt",
    description = "FoundationPose 6D object pose estimation model.",
    display_name = "FoundationPose model weights",
    id = "ai.intrinsic.ioc_pose_estimation.pose_estimator.foundationpose",
    model_files = {
        ":refine_onnx": "1/foundationpose_refine.onnx",
        ":score_onnx": "1/foundationpose_score.onnx",
        ":model_py": "1/model.py",
        ":env_tar_gz": "env.tar.gz",
    },
)
