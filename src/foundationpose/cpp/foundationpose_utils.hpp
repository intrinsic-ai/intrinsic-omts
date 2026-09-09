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

#pragma once
#include <iostream>
#include <string>
#include <cuda_runtime.h>

#define CHECK_CUDA_ERRORS(result) \
  { CheckCudaErrors(result, __FILE__, __LINE__); }
inline void CheckCudaErrors(cudaError_t result, const char* filename, int line_number) {
  if (result != cudaSuccess) {
    std::cout << "CUDA Error: " + std::string(cudaGetErrorString(result)) +
                     " (error code: " + std::to_string(result) + ") at " + std::string(filename) +
                     " in line " + std::to_string(line_number) << std::endl;
  }
}
