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

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <cmath>
#include "cuda_runtime.h"
#include "foundationpose_render.cu.hpp"
#include "nvdiffrast/common/cudaraster/CudaRaster.hpp"
#include <algorithm>
#include <iostream>

namespace py = pybind11;

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;

int AddVertex(const Eigen::Vector3f& p, std::vector<Eigen::Vector3f>& vertices) {
  vertices.push_back(p.normalized());
  return vertices.size() - 1;
}

void AddFace(int i, int j, int k, std::vector<Eigen::Vector3i>& faces) {
  faces.emplace_back(i, j, k);
}

int GetMiddlePoint(
    int i, int j, std::vector<Eigen::Vector3f>& vertices, std::map<int64_t, int>& cache) {
  bool first_is_smaller = i < j;
  int64_t smaller = first_is_smaller ? i : j;
  int64_t greater = first_is_smaller ? j : i;
  int64_t key = (smaller << 32) + greater;

  auto it = cache.find(key);
  if (it != cache.end()) {
    return it->second;
  }

  Eigen::Vector3f p1 = vertices[i];
  Eigen::Vector3f p2 = vertices[j];
  Eigen::Vector3f pm = (p1 + p2) / 2.0;
  int index = AddVertex(pm, vertices);
  cache[key] = index;
  return index;
}

std::vector<Eigen::Vector3f> GenerateIcosphere(unsigned int n_views) {
  std::map<int64_t, int> cache;
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector3i> faces;

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

  AddFace(0, 11, 5, faces);
  AddFace(0, 5, 1, faces);
  AddFace(0, 1, 7, faces);
  AddFace(0, 7, 10, faces);
  AddFace(0, 10, 11, faces);
  AddFace(1, 5, 9, faces);
  AddFace(5, 11, 4, faces);
  AddFace(11, 10, 2, faces);
  AddFace(10, 7, 6, faces);
  AddFace(7, 1, 8, faces);
  AddFace(3, 9, 4, faces);
  AddFace(3, 4, 2, faces);
  AddFace(3, 2, 6, faces);
  AddFace(3, 6, 8, faces);
  AddFace(3, 8, 9, faces);
  AddFace(4, 9, 5, faces);
  AddFace(2, 4, 11, faces);
  AddFace(6, 2, 10, faces);
  AddFace(8, 6, 7, faces);
  AddFace(9, 8, 1, faces);

  while (vertices.size() < n_views) {
    std::vector<Eigen::Vector3i> new_faces;
    for (const auto& face : faces) {
      int a = face[0];
      int b = face[1];
      int c = face[2];

      int ab = GetMiddlePoint(a, b, vertices, cache);
      int bc = GetMiddlePoint(b, c, vertices, cache);
      int ca = GetMiddlePoint(c, a, vertices, cache);

      AddFace(a, ab, ca, new_faces);
      AddFace(b, bc, ab, new_faces);
      AddFace(c, ca, bc, new_faces);
      AddFace(ab, bc, ca, new_faces);
    }
    faces = new_faces;
  }
  return vertices;
}

#include "foundationpose_sampling_helpers.hpp"

py::array_t<float> sample_initial_poses_cpp(
    py::array_t<uint8_t> mask_array,
    py::array_t<float> depth_array,
    py::array_t<float> K_array,
    unsigned int n_views,
    float inplane_step_deg) {

    auto r_mask = mask_array.unchecked<2>();
    auto r_depth = depth_array.unchecked<2>();
    auto r_K = K_array.unchecked<2>();

    int H = r_mask.shape(0);
    int W = r_mask.shape(1);

    Eigen::Matrix3f K;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            K(r, c) = r_K(r, c);
        }
    }

    Eigen::MatrixXf depth_eigen(H, W);
    nvidia::isaac_ros::RowMajorMatrix8u mask_eigen(H, W);

    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
            depth_eigen(r, c) = r_depth(r, c);
            mask_eigen(r, c) = r_mask(r, c);
        }
    }

    Eigen::Vector3f center(0, 0, 0.8f);
    nvidia::isaac_ros::GuessTranslation(depth_eigen, mask_eigen, K, 0.1f, center);

    auto poses_out = nvidia::isaac_ros::MakeRotationGrid({}, {}, n_views, inplane_step_deg);
    for (auto& pose : poses_out) {
        pose.block<3, 1>(0, 3) = center;
    }

    size_t B = poses_out.size();
    std::vector<ssize_t> shape = {static_cast<ssize_t>(B), 4, 4};
    py::array_t<float> result(shape);
    auto r_res = result.mutable_unchecked<3>();

    for (size_t i = 0; i < B; ++i) {
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                r_res(i, r, c) = poses_out[i](r, c);
            }
        }
    }

    return result;
}

RowMajorMatrix ComputeTF_CPP(float left, float right, float top, float bottom, Eigen::Vector2i out_size) {
  left = std::round(left);
  right = std::round(right);
  top = std::round(top);
  bottom = std::round(bottom);

  RowMajorMatrix tf = RowMajorMatrix::Zero(3, 3);
  tf(0, 0) = static_cast<float>(out_size.x()) / (right - left);
  tf(0, 2) = -left * static_cast<float>(out_size.x()) / (right - left);
  tf(1, 1) = static_cast<float>(out_size.y()) / (bottom - top);
  tf(1, 2) = -top * static_cast<float>(out_size.y()) / (bottom - top);
  tf(2, 2) = 1.0f;
  return tf;
}

py::array_t<float> compute_crop_window_tf_cpp(
    py::array_t<float> poses_array,
    py::array_t<float> K_array,
    int out_h, int out_w,
    float crop_ratio, float mesh_diameter) {

    auto r_poses = poses_array.unchecked<3>();
    size_t B = r_poses.shape(0);

    auto r_K = K_array.unchecked<2>();
    Eigen::Matrix3f K;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            K(r, c) = r_K(r, c);
        }
    }

    std::vector<Eigen::MatrixXf> poses(B);
    for (size_t i = 0; i < B; ++i) {
        Eigen::Matrix4f P;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                P(r, c) = r_poses(i, r, c);
            }
        }
        poses[i] = P;
    }

    Eigen::Vector2i out_size(out_w, out_h);
    float radius_val = mesh_diameter * crop_ratio / 2.0f;
    Eigen::MatrixXf offsets(5, 3);
    offsets << 0, 0, 0,
               radius_val, 0, 0,
               -radius_val, 0, 0,
               0, radius_val, 0,
               0, -radius_val, 0;

    std::vector<ssize_t> shape = {static_cast<ssize_t>(B), 3, 3};
    py::array_t<float> result(shape);
    auto r_res = result.mutable_unchecked<3>();

    for (size_t i = 0; i < B; i++) {
        Eigen::Vector3f block = poses[i].block<3, 1>(0, 3);
        Eigen::MatrixXf pts(5, 3);
        for (int k = 0; k < 5; ++k) {
            pts.row(k) = block.transpose() + offsets.row(k);
        }

        Eigen::MatrixXf projected = (K * pts.transpose()).transpose();
        Eigen::MatrixXf uvs(5, 2);
        for (int k = 0; k < 5; ++k) {
            uvs(k, 0) = projected(k, 0) / projected(k, 2);
            uvs(k, 1) = projected(k, 1) / projected(k, 2);
        }

        Eigen::Vector2f center = uvs.row(0);
        float max_r = 0.0f;
        for (int k = 0; k < 5; ++k) {
            float dist = std::abs(uvs(k, 1) - center(1));
            if (dist > max_r) max_r = dist;
        }

        float left = center(0) - max_r;
        float right = center(0) + max_r;
        float top = center(1) - max_r;
        float bottom = center(1) + max_r;

        RowMajorMatrix tf = ComputeTF_CPP(left, right, top, bottom, out_size);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                r_res(i, r, c) = tf(r, c);
            }
        }
    }

    return result;
}

py::array_t<float> update_refined_poses_cpp(
    py::array_t<float> poses_array,
    py::array_t<float> trans_delta_array,
    py::array_t<float> rot_delta_array,
    float mesh_diameter,
    float rot_normalizer) {

    auto r_poses = poses_array.unchecked<3>();
    auto r_trans = trans_delta_array.unchecked<2>();
    auto r_rot = rot_delta_array.unchecked<2>();

    size_t N = r_poses.shape(0);
    std::vector<ssize_t> shape = {static_cast<ssize_t>(N), 4, 4};
    py::array_t<float> result(shape);
    auto r_res = result.mutable_unchecked<3>();

    for (size_t i = 0; i < N; ++i) {
        Eigen::Matrix4f cur_pose;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                cur_pose(r, c) = r_poses(i, r, c);
            }
        }

        Eigen::Vector3f t_delta(r_trans(i, 0), r_trans(i, 1), r_trans(i, 2));
        Eigen::Vector3f t_delta_scaled = t_delta * (mesh_diameter / 2.0f);
        cur_pose.block<3, 1>(0, 3) += t_delta_scaled;

        Eigen::Vector3f r_delta(r_rot(i, 0), r_rot(i, 1), r_rot(i, 2));
        Eigen::Array3f tanh_val = r_delta.array().tanh() * rot_normalizer;
        Eigen::Vector3f norm_vec = tanh_val.matrix();

        float norm = norm_vec.norm();
        Eigen::Matrix3f rot_mat_delta = Eigen::Matrix3f::Identity();
        if (norm > 1e-6f) {
            Eigen::AngleAxisf rot_axis(norm, norm_vec.normalized());
            rot_mat_delta = rot_axis.toRotationMatrix().transpose();
        }

        Eigen::Matrix3f top_left_3x3 = cur_pose.block<3, 3>(0, 0);
        cur_pose.block<3, 3>(0, 0) = rot_mat_delta * top_left_3x3;

        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                r_res(i, r, c) = cur_pose(r, c);
            }
        }
    }
    return result;
}

py::array_t<float> apply_mesh_center_offset_cpp(
    py::array_t<float> pose_array,
    py::array_t<float> mesh_center_array) {

    auto r_pose = pose_array.unchecked<2>();
    auto r_center = mesh_center_array.unchecked<1>();

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            pose(r, c) = r_pose(r, c);
        }
    }

    Eigen::Matrix4f tf_to_center = Eigen::Matrix4f::Identity();
    tf_to_center(0, 3) = r_center(0);
    tf_to_center(1, 3) = r_center(1);
    tf_to_center(2, 3) = r_center(2);

    Eigen::Matrix4f final_pose = pose * tf_to_center;

    std::vector<ssize_t> shape = {4, 4};
    py::array_t<float> result(shape);
    auto r_res = result.mutable_unchecked<2>();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            r_res(r, c) = final_pose(r, c);
        }
    }
    return result;
}

#include "foundationpose_render_helpers.hpp"

py::array_t<float> compute_projection_matrix_cpp(
    py::array_t<float> K_array,
    int height, int width,
    float znear, float zfar) {

    auto r_K = K_array.unchecked<2>();
    Eigen::Matrix3f K;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            K(r, c) = r_K(r, c);
        }
    }

    Eigen::Matrix4f proj = nvidia::isaac_ros::ProjectMatrixFromIntrinsics(K, height, width, znear, zfar);

    std::vector<ssize_t> shape = {4, 4};
    py::array_t<float> result(shape);
    auto r_res = result.mutable_unchecked<2>();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            r_res(r, c) = proj(r, c);
        }
    }
    return result;
}

#define CHECK_CUDA_FP(call)                                                   \
  do {                                                                        \
    cudaError_t err__ = (call);                                               \
    if (err__ != cudaSuccess) {                                               \
      std::string msg = std::string("[foundationpose_cpp CUDA ERROR] ") +     \
                        __FILE__ + ":" + std::to_string(__LINE__) +           \
                        " in " + #call + ": " + cudaGetErrorString(err__) +   \
                        " (code=" + std::to_string(static_cast<int>(err__)) + \
                        ")";                                                  \
      std::cerr << msg << std::endl;                                          \
      throw std::runtime_error(msg);                                          \
    }                                                                         \
  } while (0)

class RasterizeCudaContext {
public:
    RasterizeCudaContext(int width, int height, int max_images) {
        int dev = 0;
        CHECK_CUDA_FP(cudaGetDevice(&dev));
        cudaDeviceProp prop;
        CHECK_CUDA_FP(cudaGetDeviceProperties(&prop, dev));
        std::cerr << "[foundationpose_cpp] RasterizeCudaContext initialized on GPU "
                  << dev << " (" << prop.name << ", Compute SM " << prop.major << "." << prop.minor
                  << ", totalGlobalMem=" << (prop.totalGlobalMem / (1024 * 1024)) << " MB)"
                  << ", viewport=" << width << "x" << height << ", max_images=" << max_images << std::endl;

        cr_ = new CR::CudaRaster();
        cr_->setViewportSize(width, height, max_images);
        CHECK_CUDA_FP(cudaStreamCreate(&stream_));
    }
    ~RasterizeCudaContext() {
        if (cr_) {
            delete cr_;
            cr_ = nullptr;
        }
        if (stream_) {
            cudaStreamDestroy(stream_);
            stream_ = nullptr;
        }
    }

    py::array_t<float> rasterize(
        py::array_t<float> v_clip_array,
        py::array_t<int32_t> f_array,
        int H, int W) {

        auto r_clip = v_clip_array.unchecked<3>(); // shape: (N, V, 4)
        auto r_f = f_array.unchecked<2>();         // shape: (F, 3)
        int N = r_clip.shape(0);
        int V = r_clip.shape(1);
        int F = r_f.shape(0);

        if (N <= 0 || V <= 0 || F <= 0 || H <= 0 || W <= 0) {
            std::string err_msg = "[foundationpose_cpp::rasterize ERROR] Invalid dimensions: N=" +
                                  std::to_string(N) + ", V=" + std::to_string(V) + ", F=" +
                                  std::to_string(F) + ", H=" + std::to_string(H) + ", W=" + std::to_string(W);
            std::cerr << err_msg << std::endl;
            throw std::invalid_argument(err_msg);
        }

        // Validate triangle indices
        int min_idx = r_f(0, 0);
        int max_idx = r_f(0, 0);
        for (int i = 0; i < F; ++i) {
            for (int j = 0; j < 3; ++j) {
                int idx = r_f(i, j);
                if (idx < min_idx) min_idx = idx;
                if (idx > max_idx) max_idx = idx;
            }
        }
        if (min_idx < 0 || max_idx >= V) {
            std::string err_msg = "[foundationpose_cpp::rasterize ERROR] Mesh face index out of bounds: min_idx=" +
                                  std::to_string(min_idx) + ", max_idx=" + std::to_string(max_idx) +
                                  ", but vertex count V=" + std::to_string(V);
            std::cerr << err_msg << std::endl;
            throw std::runtime_error(err_msg);
        }

        float* d_pos = nullptr;
        int32_t* d_tri = nullptr;
        float* d_out = nullptr;
        CHECK_CUDA_FP(cudaMalloc(&d_pos, N * V * 4 * sizeof(float)));
        CHECK_CUDA_FP(cudaMalloc(&d_tri, F * 3 * sizeof(int32_t)));
        CHECK_CUDA_FP(cudaMalloc(&d_out, N * H * W * 4 * sizeof(float)));

        CHECK_CUDA_FP(cudaMemcpyAsync(d_pos, r_clip.data(0, 0, 0), N * V * 4 * sizeof(float), cudaMemcpyHostToDevice, stream_));
        CHECK_CUDA_FP(cudaMemcpyAsync(d_tri, r_f.data(0, 0), F * 3 * sizeof(int32_t), cudaMemcpyHostToDevice, stream_));

        nvidia::isaac_ros::rasterize(stream_, cr_, d_pos, d_tri, d_out, V, F, H, W, N);
        CHECK_CUDA_FP(cudaGetLastError());
        CHECK_CUDA_FP(cudaStreamSynchronize(stream_));

        std::vector<ssize_t> shape = {N, H, W, 4};
        py::array_t<float> result(shape);
        CHECK_CUDA_FP(cudaMemcpy(result.mutable_data(), d_out, N * H * W * 4 * sizeof(float), cudaMemcpyDeviceToHost));

        CHECK_CUDA_FP(cudaFree(d_pos));
        CHECK_CUDA_FP(cudaFree(d_tri));
        CHECK_CUDA_FP(cudaFree(d_out));
        return result;
    }

    py::array_t<float> interpolate(
        py::array_t<float> v_cam_array,
        py::array_t<float> rast_array,
        py::array_t<int32_t> f_array,
        int H, int W) {

        auto r_cam = v_cam_array.unchecked<3>(); // shape: (N, V, 3)
        auto r_rast = rast_array.unchecked<4>(); // shape: (N, H, W, 4)
        auto r_f = f_array.unchecked<2>();       // shape: (F, 3)
        int N = r_cam.shape(0);
        int V = r_cam.shape(1);
        int F = r_f.shape(0);

        if (N <= 0 || V <= 0 || F <= 0 || H <= 0 || W <= 0) {
            std::string err_msg = "[foundationpose_cpp::interpolate ERROR] Invalid dimensions: N=" +
                                  std::to_string(N) + ", V=" + std::to_string(V) + ", F=" +
                                  std::to_string(F) + ", H=" + std::to_string(H) + ", W=" + std::to_string(W);
            std::cerr << err_msg << std::endl;
            throw std::invalid_argument(err_msg);
        }

        float* d_attr = nullptr;
        float* d_rast = nullptr;
        int32_t* d_tri = nullptr;
        float* d_out = nullptr;
        CHECK_CUDA_FP(cudaMalloc(&d_attr, N * V * 3 * sizeof(float)));
        CHECK_CUDA_FP(cudaMalloc(&d_rast, N * H * W * 4 * sizeof(float)));
        CHECK_CUDA_FP(cudaMalloc(&d_tri, F * 3 * sizeof(int32_t)));
        CHECK_CUDA_FP(cudaMalloc(&d_out, N * H * W * 3 * sizeof(float)));

        CHECK_CUDA_FP(cudaMemcpyAsync(d_attr, r_cam.data(0, 0, 0), N * V * 3 * sizeof(float), cudaMemcpyHostToDevice, stream_));
        CHECK_CUDA_FP(cudaMemcpyAsync(d_rast, r_rast.data(0, 0, 0, 0), N * H * W * 4 * sizeof(float), cudaMemcpyHostToDevice, stream_));
        CHECK_CUDA_FP(cudaMemcpyAsync(d_tri, r_f.data(0, 0), F * 3 * sizeof(int32_t), cudaMemcpyHostToDevice, stream_));

        nvidia::isaac_ros::interpolate(stream_, d_attr, d_rast, d_tri, d_out, V, F, 3, H, W, N, 0);
        CHECK_CUDA_FP(cudaGetLastError());
        CHECK_CUDA_FP(cudaStreamSynchronize(stream_));

        std::vector<ssize_t> shape = {N, H, W, 3};
        py::array_t<float> result(shape);
        CHECK_CUDA_FP(cudaMemcpy(result.mutable_data(), d_out, N * H * W * 3 * sizeof(float), cudaMemcpyDeviceToHost));

        CHECK_CUDA_FP(cudaFree(d_attr));
        CHECK_CUDA_FP(cudaFree(d_rast));
        CHECK_CUDA_FP(cudaFree(d_tri));
        CHECK_CUDA_FP(cudaFree(d_out));
        return result;
    }

private:
    CR::CudaRaster* cr_ = nullptr;
    cudaStream_t stream_ = nullptr;
};

PYBIND11_MODULE(foundationpose_cpp, m) {
    m.doc() = "Native Isaac ROS FoundationPose C++ Extensions";
    m.def("sample_initial_poses", &sample_initial_poses_cpp, "Generate candidate pose grid matching C++ FoundationposeSampling");
    m.def("compute_crop_window_tf", &compute_crop_window_tf_cpp, "Compute pose-centered crop transform matrices M_tf in C++");
    m.def("update_refined_poses", &update_refined_poses_cpp, "Update candidate poses matching C++ FoundationposeTransformation");
    m.def("apply_mesh_center_offset", &apply_mesh_center_offset_cpp, "Apply CAD mesh center offset matching C++ FoundationposeDecoder");
    m.def("compute_projection_matrix", &compute_projection_matrix_cpp, "Compute 4x4 perspective projection matrix matching C++ FoundationposeRender");
    py::class_<RasterizeCudaContext>(m, "RasterizeCudaContext")
        .def(py::init<int, int, int>(), py::arg("width") = 160, py::arg("height") = 160, py::arg("max_images") = 64)
        .def("rasterize", &RasterizeCudaContext::rasterize, "Rasterize clip-space vertices in C++/CUDA")
        .def("interpolate", &RasterizeCudaContext::interpolate, "Interpolate vertex attributes in C++/CUDA");
}
