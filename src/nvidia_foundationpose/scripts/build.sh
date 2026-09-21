#!/usr/bin/env bash
# Copyright 2026 Intrinsic Innovation LLC
#
# Automated setup & build script for the externalized NVIDIA FoundationPose
# repositories (foundation-pose-inference-library and foundationpose_perception_pipeline).
#
# Structure:
#   1. Manual model download check (checkpoints/sam3.pt & FoundationStereo ONNX)
#   2. SAM 3 export from ckpt to ONNX via Dockerfile.sam3_export (no uv required)
#   3. Build libfoundation_pose_nvidia.so inside Docker
#   4. Extract TensorRT/CUDA .so libraries and Python 3.11 bindings (including ftfy) via Docker
#
# Usage:
#   ./src/nvidia_foundationpose/scripts/build.sh [--build-asset] [--force-sam3-export] [--verify-docker-load]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
BASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
OMTS_ROOT="$(cd "${BASE_DIR}/../.." && pwd -P)"

NGC_IMAGE="${NGC_IMAGE:-nvcr.io/nvidia/pytorch:26.05-py3}"
SAM3_EXPORT_IMAGE="sam3_exporter:latest"

# Detect if --runtime=runc is needed/supported (e.g. on Cloudtop hosts without nvidia-container-toolkit)
if [[ -z "${DOCKER_RUNTIME:-}" ]]; then
  if docker info 2>/dev/null | grep -q "runc"; then
    DOCKER_RUNTIME="--runtime=runc"
  else
    DOCKER_RUNTIME=""
  fi
fi

# Detect GPU device flags for Docker containers that need CUDA
GPU_DEVICE_FLAGS="--privileged --ipc=host --ulimit memlock=-1 --ulimit stack=67108864"
if [[ -e /dev/nvidia0 ]]; then
  for dev in /dev/nvidia0 /dev/nvidiactl /dev/nvidia-uvm /dev/nvidia-uvm-tools; do
    [[ -e "$dev" ]] && GPU_DEVICE_FLAGS="${GPU_DEVICE_FLAGS} --device $dev"
  done
  for host_lib in /usr/lib/x86_64-linux-gnu/libcuda.so.1 /usr/lib/x86_64-linux-gnu/libnvidia-ptxjitcompiler.so.1; do
    [[ -e "$host_lib" ]] && GPU_DEVICE_FLAGS="${GPU_DEVICE_FLAGS} -v ${host_lib}:${host_lib}:ro"
  done
fi

BUILD_ASSET=0
FORCE_SAM3_EXPORT=0
VERIFY_DOCKER_LOAD=0

for arg in "$@"; do
  case "$arg" in
    --build-asset)
      BUILD_ASSET=1
      ;;
    --force-sam3-export)
      FORCE_SAM3_EXPORT=1
      ;;
    --verify-docker-load)
      VERIFY_DOCKER_LOAD=1
      ;;
    -h|--help)
      echo "Usage: $0 [--build-asset] [--force-sam3-export] [--verify-docker-load]"
      echo ""
      echo "Options:"
      echo "  --build-asset          Also run bazel build in omts to build the asset bundle"
      echo "  --force-sam3-export    Force re-exporting SAM 3 .pt checkpoint to ONNX in Docker"
      echo "  --verify-docker-load   Load the built image via 'docker load' and verify container startup"
      exit 0
      ;;
  esac
done

FP_LIB_DIR="${BASE_DIR}/foundation-pose-inference-library"
PIPELINE_DIR="${BASE_DIR}/foundationpose_perception_pipeline"
CHECKPOINTS_DIR="${BASE_DIR}/checkpoints"

BUILD_ROOT="${BASE_DIR}/build"
FP_BUILD_DIR="${BUILD_ROOT}/foundation-pose-inference-library/build"
BINDINGS_DIR="${BUILD_ROOT}/bindings"
MODELS_DIR="${BUILD_ROOT}/models"
ENGINE_CACHE_DIR="${MODELS_DIR}/engine_cache_bundle"

HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

echo "================================================================================"
echo " NVIDIA FoundationPose External Repositories Build"
echo " Base Directory : ${BASE_DIR}"
echo " OMTS Workspace : ${OMTS_ROOT}"
echo " NGC Container  : ${NGC_IMAGE}"
echo "================================================================================"

# ------------------------------------------------------------------------------
# Step 0: Clone or verify external git repositories
# ------------------------------------------------------------------------------
ensure_git_repositories() {
  echo "[0/4] Checking external git repositories..."

  if [[ ! -d "${FP_LIB_DIR}/.git" ]]; then
    echo "  -> Cloning foundation-pose-inference-library..."
    rm -rf "${FP_LIB_DIR}"
    git clone https://github.com/nvidia-isaac/foundation-pose-inference-library.git "${FP_LIB_DIR}"
  else
    echo "  -> Found foundation-pose-inference-library checkout."
  fi

  if [[ ! -d "${PIPELINE_DIR}/.git" ]]; then
    echo "  -> Cloning foundationpose_perception_pipeline (branch: experimental/gjamesgoenawan)..."
    rm -rf "${PIPELINE_DIR}"
    git clone -b experimental/gjamesgoenawan https://github.com/intrinsic-opensource/foundationpose_perception_pipeline.git "${PIPELINE_DIR}"
  else
    echo "  -> Found foundationpose_perception_pipeline checkout."
    CURRENT_BRANCH="$(git -C "${PIPELINE_DIR}" rev-parse --abbrev-ref HEAD || true)"
    if [[ "${CURRENT_BRANCH}" != "experimental/gjamesgoenawan" ]]; then
      echo "  -> Switching foundationpose_perception_pipeline to branch experimental/gjamesgoenawan..."
      git -C "${PIPELINE_DIR}" checkout experimental/gjamesgoenawan
    fi
  fi

  mkdir -p "${FP_BUILD_DIR}" "${BINDINGS_DIR}" "${MODELS_DIR}" "${ENGINE_CACHE_DIR}" "${CHECKPOINTS_DIR}"
}

# ------------------------------------------------------------------------------
# Step 1: Manual Model Download Check & Preparation
# ------------------------------------------------------------------------------
check_and_prepare_manual_models() {
  echo "[1/4] Checking manual model downloads in ${CHECKPOINTS_DIR}..."

  # Link any pre-existing models from checkpoints/ into build/models/
  for src_dir in "${MODELS_SRC_DIR:-}" "${CHECKPOINTS_DIR}"; do
    [[ -n "${src_dir}" && -d "${src_dir}" ]] || continue
    for f in "${src_dir}"/*.onnx "${src_dir}"/*.txt.gz; do
      [[ -e "$f" ]] || continue
      base_f="$(basename "$f")"
      if [[ ! -f "${MODELS_DIR}/${base_f}" ]]; then
        cp -al "$f" "${MODELS_DIR}/${base_f}" 2>/dev/null || cp -f "$f" "${MODELS_DIR}/${base_f}"
      fi
    done
  done

  # Download public FoundationPose weights (refiner_net.onnx, score_net.onnx) from Hugging Face if absent
  if [[ ! -f "${MODELS_DIR}/refiner_net.onnx" || ! -f "${MODELS_DIR}/score_net.onnx" ]]; then
    echo "  -> Downloading public FoundationPose weights (refiner_net.onnx, score_net.onnx) from Hugging Face..."
    "${FP_LIB_DIR}/scripts/download_weights.sh" "${MODELS_DIR}"
  fi

  # Check if FoundationStereo ONNX model is present
  if [[ ! -f "${MODELS_DIR}/deployable_foundation_stereo_s_dynamic_v2.0.onnx" ]]; then
    echo ""
    echo "================================================================================" >&2
    echo " ACTION REQUIRED: Manual Model Download" >&2
    echo " Missing: deployable_foundation_stereo_s_dynamic_v2.0.onnx" >&2
    echo " Please download deployable_foundation_stereo_s_dynamic_v2.0.onnx and place it in:" >&2
    echo "   ${CHECKPOINTS_DIR}/" >&2
    echo "================================================================================" >&2
    exit 1
  fi

  # Check if SAM 3 checkpoint (.pt) or pre-exported SAM 3 ONNX files exist
  SAM3_ONNX_PRESENT=1
  for f in sam3_vision_encoder.onnx sam3_text_encoder.onnx sam3_mask_decoder.onnx sam3_box_decoder.onnx bpe_simple_vocab_16e6.txt.gz; do
    if [[ ! -f "${MODELS_DIR}/${f}" ]]; then
      SAM3_ONNX_PRESENT=0
      break
    fi
  done

  SAM3_CKPT_PATH="${SAM3_CKPT:-${CHECKPOINTS_DIR}/sam3.pt}"
  if [[ "${SAM3_ONNX_PRESENT}" -eq 0 && ! -f "${SAM3_CKPT_PATH}" ]]; then
    echo ""
    echo "================================================================================" >&2
    echo " ACTION REQUIRED: Manual SAM 3 Checkpoint Download" >&2
    echo " Neither SAM 3 ONNX models nor the SAM 3 checkpoint (sam3.pt) were found." >&2
    echo " Please manually download sam3.pt and place it in:" >&2
    echo "   ${CHECKPOINTS_DIR}/sam3.pt" >&2
    echo " (Or set SAM3_CKPT=/path/to/sam3.pt before running this script.)" >&2
    echo "================================================================================" >&2
    exit 1
  fi
  echo "  -> Manual model inputs verified."
}

# ------------------------------------------------------------------------------
# Step 2: Export SAM 3 Checkpoint (.pt -> .onnx) inside Docker (Dockerfile.sam3_export)
# ------------------------------------------------------------------------------
export_sam3_to_onnx_in_docker() {
  echo "[2/4] Checking SAM 3 ONNX export..."

  SAM3_NEEDS_EXPORT=0
  if [[ "${FORCE_SAM3_EXPORT}" -eq 1 ]]; then
    SAM3_NEEDS_EXPORT=1
  else
    for f in sam3_vision_encoder.onnx sam3_text_encoder.onnx sam3_mask_decoder.onnx sam3_box_decoder.onnx bpe_simple_vocab_16e6.txt.gz; do
      if [[ ! -f "${MODELS_DIR}/${f}" ]]; then
        SAM3_NEEDS_EXPORT=1
        break
      fi
    done
  fi

  if [[ "${SAM3_NEEDS_EXPORT}" -eq 0 ]]; then
    echo "  -> SAM 3 ONNX models already present in ${MODELS_DIR} (pass --force-sam3-export to re-export from sam3.pt)."
  else
    SAM3_CKPT_PATH="${SAM3_CKPT:-${CHECKPOINTS_DIR}/sam3.pt}"
    SAM3_CKPT_REAL="$(readlink -f "${SAM3_CKPT_PATH}")"
    if [[ ! -f "${SAM3_CKPT_REAL}" ]]; then
      echo "ERROR: Cannot export SAM 3: checkpoint not found at ${SAM3_CKPT_PATH}" >&2
      exit 1
    fi

    echo "  -> Building Docker image ${SAM3_EXPORT_IMAGE} from Dockerfile.sam3_export..."
    docker build -t "${SAM3_EXPORT_IMAGE}" -f "${SCRIPT_DIR}/Dockerfile.sam3_export" "${SCRIPT_DIR}"

    echo "  -> Running export_sam3_to_onnx.py inside Docker container on GPU..."
    docker run ${DOCKER_RUNTIME} ${GPU_DEVICE_FLAGS} --rm \
      -v "${PIPELINE_DIR}:/pipeline:ro" \
      -v "${SAM3_CKPT_REAL}:/ckpt/sam3.pt:ro" \
      -v "${MODELS_DIR}:/out" \
      "${SAM3_EXPORT_IMAGE}" \
      bash -c "
        set -euo pipefail
        python3 /pipeline/tools/export_sam3_to_onnx.py --checkpoint /ckpt/sam3.pt --output-dir /out
        chown -R ${HOST_UID}:${HOST_GID} /out
      "
    echo "  -> SAM 3 ONNX export completed successfully."
  fi

  # Cache exported/verified models in .models_cache for fast clean rebuilds
  mkdir -p "${BASE_DIR}/.models_cache"
  for f in "${MODELS_DIR}"/*.onnx "${MODELS_DIR}"/*.txt.gz; do
    [[ -e "$f" ]] || continue
    base_f="$(basename "$f")"
    if [[ ! -f "${BASE_DIR}/.models_cache/${base_f}" ]]; then
      cp -al "$f" "${BASE_DIR}/.models_cache/${base_f}" 2>/dev/null || true
    fi
  done
}

# ------------------------------------------------------------------------------
# Step 3: Build C++ libfoundation_pose_nvidia.so in Docker container
# ------------------------------------------------------------------------------
build_foundationpose_so_in_docker() {
  echo "[3/4] Building libfoundation_pose_nvidia.so inside Docker container (${NGC_IMAGE})..."

  # Remove any existing build symlink inside foundation-pose-inference-library before mounting
  rm -rf "${FP_LIB_DIR}/build"

  docker run ${DOCKER_RUNTIME} ${GPU_DEVICE_FLAGS} --rm \
    -v "${FP_LIB_DIR}:/workspace" \
    -v "${FP_BUILD_DIR}:/out_build" \
    -w /workspace \
    "${NGC_IMAGE}" \
    bash -c "
      set -euo pipefail
      cmake -S /workspace -B /out_build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES='75;80;86;89;90'
      make -C /out_build -j\$(nproc)
      chown -R ${HOST_UID}:${HOST_GID} /out_build
    "

  echo "  -> Built C++ shared library: $(ls -lh "${FP_BUILD_DIR}/libfoundation_pose_nvidia.so")"
}

# ------------------------------------------------------------------------------
# Step 4: Copy TensorRT/CUDA .so libraries & Python 3.11 bindings (including ftfy)
# ------------------------------------------------------------------------------
copy_tensorrt_and_bindings_via_docker() {
  echo "[4/4] Extracting TensorRT/CUDA .so libraries and Python 3.11 bindings (including ftfy) via Docker..."

  # 4a. Copy shared libraries (TensorRT 10.16 + CUDA 13.0 runtime from PyPI) and create standard symlinks
  docker run ${DOCKER_RUNTIME} ${GPU_DEVICE_FLAGS} --rm \
    -v "${FP_BUILD_DIR}:/out_build" \
    "${NGC_IMAGE}" \
    bash -c "
      set -euo pipefail
      cp -f /usr/lib/x86_64-linux-gnu/libnvinfer.so.10.16.1 /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libnvonnxparser.so.10.16.1 /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so.10.16.1 /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libnvinfer_vc_plugin.so.10.16.1 /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libnvinfer_builder_resource_sm75.so.10.16.1 /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libnvinfer_builder_resource_sm80.so.10.16.1 /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libnvinfer_builder_resource_sm86.so.10.16.1 /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libnvinfer_builder_resource_sm89.so.10.16.1 /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libnvinfer_builder_resource_sm90.so.10.16.1 /out_build/

      # Extract pure CUDA 13.0 runtime (13.0.88) so container runs natively on Driver >= 580 without cuda-compat
      pip download -q -d /tmp/cu13_wheels nvidia-cuda-runtime==13.0.88 nvidia-cuda-nvrtc==13.0.88
      unzip -q -j /tmp/cu13_wheels/nvidia_cuda_runtime*.whl 'nvidia/cu13/lib/libcudart.so.13' -d /out_build/
      unzip -q -j /tmp/cu13_wheels/nvidia_cuda_nvrtc*.whl 'nvidia/cu13/lib/libnvrtc.so.13' 'nvidia/cu13/lib/libnvrtc-builtins.so.13.0' -d /out_build/

      cp -f /usr/lib/x86_64-linux-gnu/libpng16.so.16.* /out_build/
      cp -f /usr/lib/x86_64-linux-gnu/libz.so.1.* /out_build/

      cd /out_build
      ln -sf libnvinfer.so.10.16.1 libnvinfer.so.10 && ln -sf libnvinfer.so.10 libnvinfer.so
      ln -sf libnvonnxparser.so.10.16.1 libnvonnxparser.so.10 && ln -sf libnvonnxparser.so.10 libnvonnxparser.so
      ln -sf libnvinfer_plugin.so.10.16.1 libnvinfer_plugin.so.10 && ln -sf libnvinfer_plugin.so.10 libnvinfer_plugin.so
      ln -sf libnvinfer_vc_plugin.so.10.16.1 libnvinfer_vc_plugin.so.10 && ln -sf libnvinfer_vc_plugin.so.10 libnvinfer_vc_plugin.so
      ln -sf libcudart.so.13 libcudart.so
      ln -sf libnvrtc.so.13 libnvrtc.so
      ln -sf libpng16.so.16.* libpng16.so.16 && ln -sf libpng16.so.16 libpng16.so
      ln -sf libz.so.1.* libz.so.1 && ln -sf libz.so.1 libz.so

      chown -R ${HOST_UID}:${HOST_GID} /out_build
    "

  # 4b. Download and extract Python 3.11 wheels (TensorRT, CUDA, ftfy, wcwidth)
  mkdir -p "${BINDINGS_DIR}/.wheels"

  docker run ${DOCKER_RUNTIME} ${GPU_DEVICE_FLAGS} --rm \
    --user "${HOST_UID}:${HOST_GID}" \
    -v "${BINDINGS_DIR}:/bindings" \
    "${NGC_IMAGE}" \
    pip download \
      tensorrt_cu13_bindings==10.16.1.11 \
      cuda-bindings==12.9.4 \
      cuda-pathfinder==1.8.1 \
      ftfy==6.3.1 \
      wcwidth==0.6.0 \
      --only-binary=:all: \
      --python-version 311 \
      --platform manylinux_2_28_x86_64 \
      --dest /bindings/.wheels

  python3 - "${BINDINGS_DIR}" << 'EOF'
import sys
import zipfile
from pathlib import Path

bindings_dir = Path(sys.argv[1])
wheels_dir = bindings_dir / ".wheels"
for whl in sorted(wheels_dir.glob("*.whl")):
    print(f"  -> Extracting {whl.name}...")
    with zipfile.ZipFile(whl, "r") as zf:
        zf.extractall(bindings_dir)

tensorrt_pkg = bindings_dir / "tensorrt"
tensorrt_pkg.mkdir(parents=True, exist_ok=True)
(tensorrt_pkg / "__init__.py").write_text(
    "from tensorrt_bindings import *\nfrom tensorrt_bindings import __version__\n"
)
EOF

  rm -rf "${BINDINGS_DIR}/.wheels"

  # Verify no Python 3.13 ABI symbols leaked
  BAD_SYMBOLS=$(nm -D -u "${BINDINGS_DIR}/tensorrt_bindings/tensorrt.so" 2>/dev/null | grep -c "PyThreadState_GetUnchecked" || true)
  if [[ "${BAD_SYMBOLS}" -ne 0 ]]; then
    echo "ERROR: Python 3.13 symbol PyThreadState_GetUnchecked detected in tensorrt.so!" >&2
    exit 1
  fi
}

# Execute all build steps
ensure_git_repositories
check_and_prepare_manual_models
export_sam3_to_onnx_in_docker
build_foundationpose_so_in_docker
copy_tensorrt_and_bindings_via_docker

echo "================================================================================"
echo " Build of foundation-pose-inference-library & foundationpose_perception_pipeline"
echo " completed successfully!"
echo " Outputs ready in:"
echo "   - ${MODELS_DIR}"
echo "   - ${BINDINGS_DIR}"
echo "   - ${FP_BUILD_DIR}"
echo "================================================================================"

if [[ "${BUILD_ASSET}" -eq 1 || "${VERIFY_DOCKER_LOAD}" -eq 1 ]]; then
  echo ""
  echo "Building Bazel asset bundle in ${OMTS_ROOT}..."
  mkdir -p /tmp/empty_docker_config && echo '{}' > /tmp/empty_docker_config/config.json
  cd "${OMTS_ROOT}"
  bazel build @intrinsic-core//intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator:nvidia_pose_estimator_service_asset
fi

if [[ "${VERIFY_DOCKER_LOAD}" -eq 1 ]]; then
  echo ""
  echo "Verifying asset bundle image with 'docker load'..."
  BUNDLE_TAR="${OMTS_ROOT}/bazel-bin/external/intrinsic-core+/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/nvidia_pose_estimator_service_asset.bundle.tar"
  LOAD_OUT="$(tar -xOf "${BUNDLE_TAR}" nvidia_pose_estimator_image.tar | docker load)"
  echo "${LOAD_OUT}"
  IMAGE_ID="$(echo "${LOAD_OUT}" | awk '/Loaded image/ {print $NF}')"
  echo "Testing container startup and module imports (ftfy, tensorrt, foundationpose_perception_pipeline)..."
  HERMETIC_PY="/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/nvidia_service_main.runfiles/rules_python++python+python_3_11_x86_64-unknown-linux-gnu/bin/python3"
  docker run ${DOCKER_RUNTIME} ${GPU_DEVICE_FLAGS} --rm \
    -e LD_LIBRARY_PATH="/usr/local/lib/foundation_pose:/usr/local/lib/foundation_pose/lib" \
    --entrypoint "${HERMETIC_PY}" "${IMAGE_ID}" -c "
import sys, glob
runfiles = '/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/nvidia_service_main.runfiles'
sys.path[:0] = ['/bindings'] + glob.glob(runfiles + '/*/site-packages') + [runfiles + '/_main/src/nvidia_foundationpose/foundationpose_perception_pipeline/src']
import ftfy
import tensorrt
import foundationpose_perception_pipeline.inference.sam3.tokenizer as tok
print('SUCCESS: Imported ftfy version:', ftfy.__version__)
print('SUCCESS: Imported tensorrt version:', tensorrt.__version__)
print('SUCCESS: SAM3 tokenizer basic_clean test:', tok._basic_clean('test &amp;amp; string'))
"
  echo "Testing service binary entrypoint (--help)..."
  SERVICE_HELP="$(docker run ${DOCKER_RUNTIME} --rm "${IMAGE_ID}" /intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/nvidia_service_main --help 2>&1 || true)"
  echo "${SERVICE_HELP}" | head -n 12
  if ! echo "${SERVICE_HELP}" | grep -q "Main entrypoint for the Nvidia Pose Estimator Service"; then
    echo "ERROR: Service entrypoint verification failed!" >&2
    exit 1
  fi
  echo "Docker load and container runtime verification succeeded!"
fi


