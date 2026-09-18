# FoundationPose Build & Packaging

This directory contains the Triton Inference Server model configuration (`config.pbtxt`) and `intrinsic_mlmodel` asset definition (`//src/foundationpose:foundationpose_mlmodel`) for FoundationPose (Python 3.12 / CUDA).

## Overview

The FoundationPose deployment asset (`//src/foundationpose:foundationpose_mlmodel`) is built hermetically from source using Bazel:
1. **`//third_party/foundationpose:foundationpose_cpp.so`**: Compiled via `@rules_cuda` and `@pybind11_bazel` against Python 3.12 headers from `//third_party/foundationpose/cpp` and `nvdiffrast` sources from `@isaac_ros_pose_estimation`.
2. **`//third_party/foundationpose:env_tar_gz`**: Assembled hermetically from all Python 3.12 wheels locked in `@foundationpose_pip_deps` (`all_whl_requirements`) and `foundationpose_cpp.so` (installed in `site-packages/`), with `$ORIGIN`-relative `RPATH` applied via `patchelf`.
3. **`//src/foundationpose:foundationpose_mlmodel`**: Packages `config.pbtxt`, `//third_party/foundationpose:model.py`, `//third_party/foundationpose:env_tar_gz`, and the ONNX model weights (`@foundationpose_refine_onnx` and `@foundationpose_score_onnx`) into the Intrinsic MLModel asset.

## Building with Bazel

To build the complete Intrinsic MLModel asset:
```bash
bazel build //src/foundationpose:foundationpose_mlmodel
```

## Updating Python Dependencies

Python 3.12 dependencies are declared in `//third_party/foundationpose:requirements.in` and locked with SHA-256 hashes in `//third_party/foundationpose:requirements.txt`. To update the lockfile (no `BUILD` edits required):
```bash
bazel run //third_party/foundationpose:requirements
```

## Code Organization & Licensing

- **`//src/foundationpose/`**: Contains Intrinsic-authored Triton configuration (`config.pbtxt`) and `intrinsic_mlmodel` packaging (`BUILD`).
- **`//third_party/foundationpose/`**: Contains all NVIDIA-copyrighted C++/CUDA sources (`cpp/`), Python orchestration code (`model.py`), external dependency module extensions (`deps.bzl`), build overlays (`isaac_ros_pose_estimation.BUILD`), and Python lockfiles (`requirements.in`, `requirements.txt`), licensed under Apache-2.0.
