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

#include "foundationpose_sampling_helpers.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>

namespace nvidia {
namespace isaac_ros {

namespace {

int AddVertex(const Eigen::Vector3f& p, std::vector<Eigen::Vector3f>& vertices) {
  vertices.push_back(p.normalized());
  return vertices.size() - 1;
}

int GetMiddlePoint(int p1, int p2, std::vector<Eigen::Vector3f>& vertices,
                   std::map<int64_t, int>& middle_point_index_cache) {
  int64_t smaller_index = std::min(p1, p2);
  int64_t greater_index = std::max(p1, p2);
  int64_t key = (smaller_index << 32) + greater_index;

  auto it = middle_point_index_cache.find(key);
  if (it != middle_point_index_cache.end()) {
    return it->second;
  }

  Eigen::Vector3f point1 = vertices[p1];
  Eigen::Vector3f point2 = vertices[p2];
  Eigen::Vector3f middle = (point1 + point2) / 2.0;

  int i = AddVertex(middle, vertices);
  middle_point_index_cache[key] = i;
  return i;
}

std::vector<Eigen::Vector3f> GenerateIcosphere(int recursion_level) {
  std::vector<Eigen::Vector3f> vertices;
  std::map<int64_t, int> middle_point_index_cache;

  float t = (1.0 + std::sqrt(5.0)) / 2.0;

  AddVertex(Eigen::Vector3f(-1, t, 0), vertices);
  AddVertex(Eigen::Vector3f(1, t, 0), vertices);
  AddVertex(Eigen::Vector3f(-1, -t, 0), vertices);
  AddVertex(Eigen::Vector3f(1, -t, 0), vertices);

  AddVertex(Eigen::Vector3f(0, -1, t), vertices);
  AddVertex(Eigen::Vector3f(0, 1, t), vertices);
  AddVertex(Eigen::Vector3f(0, -1, -t), vertices);
  AddVertex(Eigen::Vector3f(0, 1, -t), vertices);

  AddVertex(Eigen::Vector3f(t, 0, -1), vertices);
  AddVertex(Eigen::Vector3f(t, 0, 1), vertices);
  AddVertex(Eigen::Vector3f(-t, 0, -1), vertices);
  AddVertex(Eigen::Vector3f(-t, 0, 1), vertices);

  struct Tri { int v1, v2, v3; };
  std::vector<Tri> faces = {
      {0, 11, 5}, {0, 5, 1},  {0, 1, 7},   {0, 7, 10}, {0, 10, 11},
      {1, 5, 9},  {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
      {3, 9, 4},  {3, 4, 2},  {3, 2, 6},   {3, 6, 8},  {3, 8, 9},
      {4, 9, 5},  {2, 4, 11}, {6, 2, 10},  {8, 6, 7},  {9, 8, 1}
  };

  for (int i = 0; i < recursion_level; i++) {
    std::vector<Tri> faces2;
    for (auto tri : faces) {
      int a = GetMiddlePoint(tri.v1, tri.v2, vertices, middle_point_index_cache);
      int b = GetMiddlePoint(tri.v2, tri.v3, vertices, middle_point_index_cache);
      int c = GetMiddlePoint(tri.v3, tri.v1, vertices, middle_point_index_cache);

      faces2.push_back({tri.v1, a, c});
      faces2.push_back({tri.v2, b, a});
      faces2.push_back({tri.v3, c, b});
      faces2.push_back({a, b, c});
    }
    faces = faces2;
  }

  return vertices;
}

}  // namespace

bool GuessTranslation(
    const Eigen::MatrixXf& depth, const RowMajorMatrix8u& mask, const Eigen::Matrix3f& K,
    float min_depth, Eigen::Vector3f& center) {
  std::vector<int> vs, us;
  for (int i = 0; i < mask.rows(); i++) {
    for (int j = 0; j < mask.cols(); j++) {
      if (mask(i, j) > 0) {
        vs.push_back(i);
        us.push_back(j);
      }
    }
  }
  if (us.empty()) {
    return false;
  }

  float uc = (*std::min_element(us.begin(), us.end()) + *std::max_element(us.begin(), us.end())) / 2.0;
  float vc = (*std::min_element(vs.begin(), vs.end()) + *std::max_element(vs.begin(), vs.end())) / 2.0;

  std::vector<float> valid_depth;
  for (int i = 0; i < mask.rows(); i++) {
    for (int j = 0; j < mask.cols(); j++) {
      if (mask(i, j) > 0 && depth(i, j) >= min_depth) {
        valid_depth.push_back(depth(i, j));
      }
    }
  }

  if (valid_depth.empty()) {
    return false;
  }

  std::sort(valid_depth.begin(), valid_depth.end());
  int n = valid_depth.size();
  float zc = (n % 2 == 0) ? (valid_depth[n / 2 - 1] + valid_depth[n / 2]) / 2.0 : valid_depth[n / 2];

  center = K.inverse() * Eigen::Vector3f(uc, vc, 1.0f) * zc;
  return true;
}

std::vector<Eigen::Matrix4f> MakeRotationGrid(const std::vector<std::string>& symmetry_axes, 
                                             const std::vector<std::string>& fixed_axis_angles, 
                                             unsigned int n_views, 
                                             double inplane_step) {
  auto cam_in_obs = GenerateIcosphere(2);
  if (cam_in_obs.size() > n_views) {
    cam_in_obs.resize(n_views);
  }

  double inplane_step_rad = inplane_step / 180.0 * M_PI;
  std::vector<Eigen::Matrix4f> rot_grid;

  for (unsigned int i = 0; i < cam_in_obs.size(); i++) {
    Eigen::Matrix4f cam_in_ob = Eigen::Matrix4f::Identity();
    cam_in_ob.block<3, 1>(0, 3) = cam_in_obs[i];

    Eigen::Vector3f up(0, 0, 1);
    Eigen::Vector3f z_axis = -cam_in_ob.block<3, 1>(0, 3).normalized();
    Eigen::Vector3f x_axis = up.cross(z_axis);
    if (x_axis.isZero()) {
      x_axis << 1, 0, 0;
    }
    x_axis.normalize();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();

    cam_in_ob.block<3, 1>(0, 0) = x_axis;
    cam_in_ob.block<3, 1>(0, 1) = y_axis;
    cam_in_ob.block<3, 1>(0, 2) = z_axis;

    for (double inplane_rot = 0; inplane_rot < 2.0 * M_PI; inplane_rot += inplane_step_rad) {
      Eigen::Affine3f R_inplane = Eigen::Affine3f::Identity();
      R_inplane.rotate(Eigen::AngleAxisf(inplane_rot, Eigen::Vector3f::UnitZ()));

      Eigen::Matrix4f cur_cam_in_ob = cam_in_ob * R_inplane.matrix();
      Eigen::Matrix4f ob_in_cam = cur_cam_in_ob.inverse();
      rot_grid.push_back(ob_in_cam);
    }
  }

  return rot_grid;
}

}  // namespace isaac_ros
}  // namespace nvidia
