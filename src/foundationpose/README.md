# FoundationPose Build & Packaging

This directory contains the Triton Inference Server model configuration (`config.pbtxt`) for FoundationPose (Python 3.12 / CUDA).

## Overview

The FoundationPose deployment asset (`//:foundationpose_mlmodel`) is built hermetically from source using Bazel:
1. **`//third_party/foundationpose:foundationpose_cpp_so`**: Compiled via `@rules_cuda` and `@pybind11_bazel` against Python 3.12 headers from `//third_party/foundationpose/cpp` and `nvdiffrast` sources from `@isaac_ros_pose_estimation`.
2. **`//third_party/foundationpose:env_tar_gz`**: Assembled hermetically from Python 3.12 wheels managed by `@rules_python`'s `pip.parse` (`@foundationpose_pip_deps//...`) and `foundationpose_cpp.so`, with `$ORIGIN`-relative `RPATH` applied via `patchelf`.
3. **`//:foundationpose_mlmodel`**: Directly packages `//src/foundationpose:config.pbtxt`, `//third_party/foundationpose:model.py`, `//third_party/foundationpose:foundationpose_cpp_so`, `//third_party/foundationpose:env_tar_gz`, and the ONNX model weights (`foundationpose_refine.onnx` and `foundationpose_score.onnx`) into the Intrinsic MLModel asset.

## Building with Bazel

To build the Triton Python 3.12 environment tarball (`env.tar.gz`) or shared library (`foundationpose_cpp.so`):
```bash
bazel build //third_party/foundationpose:env_tar_gz //third_party/foundationpose:foundationpose_cpp_so
```

To build the complete Intrinsic MLModel asset (`//:foundationpose_mlmodel`):
```bash
bazel build //:foundationpose_mlmodel
```

## Updating Python Dependencies

Python 3.12 dependencies are declared in `//third_party/foundationpose:requirements.in` and locked with SHA-256 hashes in `//third_party/foundationpose:requirements_lock.txt`. To update the lockfile:
```bash
bazel run //third_party/foundationpose:requirements.update
```

## Code Organization & Licensing

- **`//src/foundationpose/`**: Contains Intrinsic-authored Triton configuration (`config.pbtxt`).
- **`//third_party/foundationpose/`**: Contains NVIDIA-copyrighted C++/CUDA sources (`cpp/`), Python orchestration code (`model.py`), and Bazel build rules, derived from [Isaac ROS FoundationPose](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation) and licensed under Apache-2.0.
- **`//third_party/isaac_ros_pose_estimation/`**: Contains the Bazel `BUILD` overlay for upstream `isaac_ros_pose_estimation` (`release-3.2`).
