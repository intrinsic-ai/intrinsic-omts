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

#ifndef FOUNDATIONPOSE_SAMPLING_HELPERS_HPP_
#define FOUNDATIONPOSE_SAMPLING_HELPERS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace nvidia {
namespace isaac_ros {

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix8u;

bool GuessTranslation(
    const Eigen::MatrixXf& depth, const RowMajorMatrix8u& mask, const Eigen::Matrix3f& K,
    float min_depth, Eigen::Vector3f& center);

std::vector<Eigen::Matrix4f> MakeRotationGrid(const std::vector<std::string>& symmetry_axes, 
                                             const std::vector<std::string>& fixed_axis_angles, 
                                             unsigned int n_views = 40, 
                                             double inplane_step = 60.0);

}  // namespace isaac_ros
}  // namespace nvidia

#endif  // FOUNDATIONPOSE_SAMPLING_HELPERS_HPP_
