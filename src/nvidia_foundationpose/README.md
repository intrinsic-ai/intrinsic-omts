# NVIDIA FoundationPose (`omts`)

Builds the external dependencies (`foundation-pose-inference-library` and `foundationpose_perception_pipeline`), ONNX models, and TensorRT/CUDA 13.0 runtime layers for the **NVIDIA Pose Estimator Service**, and packages the final service asset bundle in the `omts` solution.

---

## 1. Prerequisites & Manual Setup

- **NVIDIA Driver**: `>= 580` (CUDA 13.0)
- **Docker**: Installed and running
- **Manual Checkpoints**: Place the following files in `src/nvidia_foundationpose/checkpoints/`:
  - `sam3.pt`
  - `deployable_foundation_stereo_s_dynamic_v2.0.onnx`

---

## 2. Step-by-Step Execution & Expected Outputs

All commands below are run from the `omts` repository root.

### Step 1: Run `build.sh` (Build External Libraries, Bindings & Models)

**Command:**
```bash
./src/nvidia_foundationpose/scripts/build.sh
```

**Expected Console Output:**
```text
[0/4] Checking external git repositories...
[1/4] Checking manual model downloads in .../src/nvidia_foundationpose/checkpoints...
[2/4] Exporting SAM 3 checkpoint to ONNX in Docker (Dockerfile.sam3_export)...
[3/4] Building libfoundation_pose_nvidia.so in Docker (nvcr.io/nvidia/pytorch:26.05-py3)...
[4/4] Extracting TensorRT/CUDA .so libraries and Python 3.11 bindings (including ftfy) via Docker...
================================================================================
 Build of foundation-pose-inference-library & foundationpose_perception_pipeline
 completed successfully!
 Outputs ready in:
   - src/nvidia_foundationpose/build/models
   - src/nvidia_foundationpose/build/bindings
   - src/nvidia_foundationpose/build/foundation-pose-inference-library/build
================================================================================
```

**Expected Generated Files (`src/nvidia_foundationpose/build/`):**
- `build/models/`:
  - `refiner_net.onnx`, `score_net.onnx`
  - `deployable_foundation_stereo_s_dynamic_v2.0.onnx`
  - `sam3_vision_encoder.onnx`, `sam3_text_encoder.onnx`, `sam3_mask_decoder.onnx`, `sam3_box_decoder.onnx`, `bpe_simple_vocab_16e6.txt.gz`
- `build/foundation-pose-inference-library/build/`:
  - `libfoundation_pose_nvidia.so`
  - `libnvinfer.so.10.16.1`, `libnvonnxparser.so.10.16.1`, `libnvinfer_plugin.so.10.16.1`, `libnvinfer_vc_plugin.so.10.16.1`, `libnvinfer_builder_resource_sm*.so.10.16.1`
  - `libcudart.so.13`, `libnvrtc.so.13`, `libnvrtc-builtins.so.13.0` (CUDA 13.0 runtime)
- `build/bindings/`:
  - `tensorrt/`, `tensorrt_bindings/`, `cuda/`, `ftfy/`, `wcwidth/` (Python 3.11 wheels unpacked)

---

### Step 2: Build the Bazel Service Asset Bundle

**Command:**
```bash
bazel build @intrinsic-core//intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator:nvidia_pose_estimator_service_asset
```

**Expected Console Output:**
```text
Target @@intrinsic-core+//intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator:nvidia_pose_estimator_service_asset up-to-date:
  bazel-bin/external/intrinsic-core+/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/nvidia_pose_estimator_service_asset.bundle.tar
INFO: Build completed successfully
```
