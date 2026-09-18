// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef FOUNDATIONPOSE_RENDER_HELPERS_HPP_
#define FOUNDATIONPOSE_RENDER_HELPERS_HPP_

#include <Eigen/Dense>
#include <vector>

namespace nvidia {
namespace isaac_ros {

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;

Eigen::Matrix4f ProjectMatrixFromIntrinsics(
    const Eigen::Matrix3f& K, int height, int width, float znear = 0.1f, float zfar = 100.0f);

RowMajorMatrix ComputeTF(
    float left, float right, float top, float bottom, Eigen::Vector2i out_size);

std::vector<RowMajorMatrix> ComputeCropWindowTF(
    const std::vector<Eigen::MatrixXf>& poses,
    const Eigen::Matrix3f& K,
    Eigen::Vector2i out_size,
    float crop_ratio,
    float mesh_diameter);

}  // namespace isaac_ros
}  // namespace nvidia

#endif  // FOUNDATIONPOSE_RENDER_HELPERS_HPP_
