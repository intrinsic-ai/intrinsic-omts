# FoundationPose Build Resources

This directory contains standalone C++/CUDA sources and build scripts to build deployment artifacts for FoundationPose running on Triton Inference Server.

## Overview

The build scripts compile the CUDA/C++ binding (`foundationpose_cpp.so`), construct a Python virtual environment archive (`env.tar.gz`), and package them alongside `config.pbtxt` and `model.py` into the target tarballs:
- `foundationpose_py311.tar.gz` (for Triton 25.06 / Python 3.11 / CUDA 12)
- `foundationpose_py312.tar.gz` (for Triton 26.07 / Python 3.12 / CUDA 13)

By default, the resulting tarballs are written to:
`src/foundationpose/dist/`

### Target Archive Structure

Each generated tarball has the following root-level structure:
```
foundationpose_py31*.tar.gz
├── config.pbtxt
├── model.py
├── foundationpose_cpp.so
└── env.tar.gz
```

Note: ONNX model weights (`foundationpose_refine.onnx` and `foundationpose_score.onnx`) are deployed separately as data assets and are not bundled into these environment archives.

## Supported Triton Versions

- **Version 25.06 (Python 3.11 / CUDA 12)**:
  ```bash
  bash src/foundationpose/scripts/build_triton_env_py311.sh
  ```

- **Version 26.07 (Python 3.12 / CUDA 13)**:
  ```bash
  bash src/foundationpose/scripts/build_triton_env_py312.sh
  ```

### Optional Arguments

Both scripts support:
- `--output-dir <path>`: Custom destination directory for the generated `.tar.gz` (defaults to `src/foundationpose/dist/`).
- `--image <docker-image>`: Custom Docker image for building.

## Implementation Details

The C++ library is directly derived from the [Isaac ROS FoundationPose implementation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation), with dependencies like Nitros and GXF stripped away. This standalone build uses PyBind11, Eigen3, OpenCV, and CUDA Rasterization (`nvdiffrast`), and inference is executed via ONNX Runtime with a CUDA execution provider.
The code in model.py is also derived from the Isaac ROS FoundationPose implementation and is used to orchestrate inference between the custom C++ library and the two ONNX models.
