# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(foundationpose_cpp_standalone LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES "75;80;86;89;90;100;120")
endif()

find_package(CUDAToolkit REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(OpenCV REQUIRED)

if(NOT DEFINED ISAAC_ROS_DIR)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../src/isaac_ros_pose_estimation")
    set(ISAAC_ROS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src/isaac_ros_pose_estimation")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../src/isaac_ros_pose_estimation")
    set(ISAAC_ROS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src/isaac_ros_pose_estimation")
  else()
    set(ISAAC_ROS_DIR "/workspace/src/isaac_ros_pose_estimation")
  endif()
endif()

set(GXF_FP_DIR "${ISAAC_ROS_DIR}/isaac_ros_gxf_extensions/gxf_isaac_foundationpose")

pybind11_add_module(foundationpose_cpp
    foundationpose_binding.cpp
    foundationpose_render_helpers.cpp
    foundationpose_sampling_helpers.cpp
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/Buffer.cpp
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/CudaRaster.cpp
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/RasterImpl.cpp
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common/cudaraster/impl/RasterImpl.cu
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common/common.cpp
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common/interpolate.cu
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common/rasterize.cu
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common/texture.cu
    foundationpose_render.cu
)

target_include_directories(foundationpose_cpp
  BEFORE PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  PUBLIC
    ${CUDAToolkit_INCLUDE_DIRS}
    ${GXF_FP_DIR}/gxf/foundationpose
    ${GXF_FP_DIR}/gxf/foundationpose/nvdiffrast/common
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(foundationpose_cpp
  PUBLIC
    PYBIND11
)

target_link_libraries(foundationpose_cpp
  PRIVATE
    CUDA::cudart_static
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
