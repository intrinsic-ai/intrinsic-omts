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

#include "foundationpose_render_helpers.hpp"
#include <cmath>
#include <iostream>

namespace nvidia {
namespace isaac_ros {

Eigen::Matrix4f ProjectMatrixFromIntrinsics(
    const Eigen::Matrix3f& K, int height, int width, float znear, float zfar) {
  int w = width;
  int h = height;
  float nc = znear;
  float fc = zfar;

  float depth = fc - nc;
  float q = -(fc + nc) / depth;
  float qn = -2.0f * (fc * nc) / depth;

  Eigen::Matrix4f proj_output = Eigen::Matrix4f::Zero();
  // Using y_down (standard OpenCV camera to clip space conversion in Isaac ROS)
  proj_output << 2.0f * K(0, 0) / w, -2.0f * K(0, 1) / w, (-2.0f * K(0, 2) + w) / w, 0.0f,
                 0.0f,               2.0f * K(1, 1) / h, (2.0f * K(1, 2) - h) / h,   0.0f,
                 0.0f,               0.0f,               q,                         qn,
                 0.0f,               0.0f,              -1.0f,                      0.0f;
  return proj_output;
}

RowMajorMatrix ComputeTF(
    float left, float right, float top, float bottom, Eigen::Vector2i out_size) {
  left = std::round(left);
  right = std::round(right);
  top = std::round(top);
  bottom = std::round(bottom);

  RowMajorMatrix tf = Eigen::MatrixXf::Identity(3, 3);
  tf(0, 2) = -left;
  tf(1, 2) = -top;

  RowMajorMatrix new_tf = Eigen::MatrixXf::Identity(3, 3);
  new_tf(0, 0) = static_cast<float>(out_size(0)) / (right - left);
  new_tf(1, 1) = static_cast<float>(out_size(1)) / (bottom - top);

  return new_tf * tf;
}

std::vector<RowMajorMatrix> ComputeCropWindowTF(
    const std::vector<Eigen::MatrixXf>& poses,
    const Eigen::Matrix3f& K,
    Eigen::Vector2i out_size,
    float crop_ratio,
    float mesh_diameter) {
  int B = poses.size();
  float r = mesh_diameter * crop_ratio / 2.0f;
  Eigen::MatrixXf offsets(5, 3);
  offsets << 0, 0, 0,
             r, 0, 0,
            -r, 0, 0,
             0, r, 0,
             0, -r, 0;

  std::vector<RowMajorMatrix> tfs;
  tfs.reserve(B);
  for (int i = 0; i < B; i++) {
    auto block = poses[i].block<3, 1>(0, 3).transpose();
    Eigen::MatrixXf pts = block.replicate(offsets.rows(), 1).array() + offsets.array();
    Eigen::MatrixXf projected = (K * pts.transpose()).transpose();
    Eigen::MatrixXf uvs =
        projected.leftCols(2).array() / projected.rightCols(1).replicate(1, 2).array();
    Eigen::Vector2f center = uvs.row(0);

    float radius = 0.0f;
    for (int k = 0; k < 5; ++k) {
      float dist = std::abs(uvs(k, 1) - center(1));
      if (dist > radius) radius = dist;
    }

    float left = center(0) - radius;
    float right = center(0) + radius;
    float top = center(1) - radius;
    float bottom = center(1) + radius;

    tfs.push_back(ComputeTF(left, right, top, bottom, out_size));
  }
  return tfs;
}

}  // namespace isaac_ros
}  // namespace nvidia
