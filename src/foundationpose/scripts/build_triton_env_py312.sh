#!/usr/bin/env bash
# ==============================================================================
# build_triton_env_py312.sh
#
# Automated end-to-end build script for NVIDIA Isaac ROS FoundationPose Triton
# deployment artifact (foundationpose_py312.tar.gz) for Python 3.12.
#
# Target artifact:
#   <OUTPUT_DIR>/foundationpose_py312.tar.gz
#   containing:
#     - config.pbtxt
#     - model.py
#     - foundationpose_cpp.so
#     - env.tar.gz
#
# Designed for Triton Server 26.07 (Python 3.12 / CUDA 13).
# Uses pure CMake standalone build with ZERO patch files.
# ==============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(git -C "${PACKAGE_ROOT}" rev-parse --show-toplevel 2>/dev/null || (cd "${PACKAGE_ROOT}/../.." && pwd))"

DEFAULT_OUTPUT_DIR="${PACKAGE_ROOT}/dist"
OUTPUT_DIR="${OUTPUT_DIR:-${DEFAULT_OUTPUT_DIR}}"
TRITON_IMAGE="${TRITON_IMAGE:-nvcr.io/nvidia/tritonserver:26.07-py3}"

# Parse optional arguments
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --output-dir=*)
      OUTPUT_DIR="${1#*=}"
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift
      ;;
    --image=*)
      TRITON_IMAGE="${1#*=}"
      ;;
    --image)
      TRITON_IMAGE="$2"
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--output-dir <path>] [--image <docker-image>]"
      echo "  --output-dir  Target directory for foundationpose_py312.tar.gz"
      echo "                (default: ${DEFAULT_OUTPUT_DIR})"
      echo "  --image       Docker image to use for building"
      echo "                (default: ${TRITON_IMAGE})"
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1"
      echo "Run '$0 --help' for usage."
      exit 1
      ;;
  esac
  shift
done

echo "=========================================================================="
echo "  FoundationPose Triton Automated Build Script (Python 3.12 / CUDA 13)   "
echo "=========================================================================="
echo "Builder container: ${TRITON_IMAGE}"
echo "Target output dir: ${OUTPUT_DIR}"

# 1. Check Docker prerequisite
if ! command -v docker >/dev/null 2>&1; then
    echo "[ERROR] Docker is not installed or not in PATH. Please install Docker."
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    echo "[ERROR] Docker daemon is not running. Please start Docker."
    exit 1
fi
echo "[OK] Docker is running."

# 2. Download official NVIDIA Isaac ROS Pose Estimation (release-3.2) if not present
mkdir -p "${PACKAGE_ROOT}/src"
if [ ! -d "${PACKAGE_ROOT}/src/isaac_ros_pose_estimation/.git" ]; then
    echo "=========================================================================="
    echo "  Step 1: Downloading NVIDIA Isaac ROS Pose Estimation (release-3.2)     "
    echo "=========================================================================="
    git clone --depth 1 -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git "${PACKAGE_ROOT}/src/isaac_ros_pose_estimation"
else
    echo "[OK] ${PACKAGE_ROOT}/src/isaac_ros_pose_estimation already exists."
fi

# 3. Compile foundationpose_cpp.so and build env.tar.gz inside Triton container
echo "=========================================================================="
echo "  Step 2: Compiling foundationpose_cpp.so & Building Python 3.12 env.tar.gz"
echo "=========================================================================="

DIST_DIR="${PACKAGE_ROOT}/dist/py312"
rm -rf "${DIST_DIR}"
mkdir -p "${DIST_DIR}"

HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

docker run --rm \
  -v "${PACKAGE_ROOT}:/workspace:ro" \
  -v "${DIST_DIR}:/output" \
  "${TRITON_IMAGE}" \
  /bin/bash -c "apt-get update -qq && \
      apt-get install -y -qq cmake g++ libeigen3-dev libopencv-dev libassimp-dev python3-dev python3-pip python3-venv patchelf && \
      python3 -m pip install pybind11 --break-system-packages --quiet && \
      echo '----------------------------------------------------------------------' && \
      echo '  Compiling foundationpose_cpp.so (CMake standalone build for Py3.12)...' && \
      echo '----------------------------------------------------------------------' && \
      mkdir -p /tmp/build_cpp && \
      cmake -S /workspace/cpp -B /tmp/build_cpp -DCMAKE_BUILD_TYPE=Release \
        -DISAAC_ROS_DIR=/workspace/src/isaac_ros_pose_estimation \
        -DPYTHON_EXECUTABLE=\$(which python3) \
        -Dpybind11_DIR=\$(python3 -m pybind11 --cmakedir) && \
      cmake --build /tmp/build_cpp -j\$(nproc) && \
      cp /tmp/build_cpp/foundationpose_cpp.*.so /output/foundationpose_cpp.so && \
      echo '----------------------------------------------------------------------' && \
      echo '  Building Python 3.12 env.tar.gz with onnxruntime-gpu...              ' && \
      echo '----------------------------------------------------------------------' && \
      rm -rf /tmp/env_312 && \
      python3 -m venv /tmp/env_312 && \
      /tmp/env_312/bin/pip install --upgrade pip --quiet && \
      /tmp/env_312/bin/pip install numpy==1.26.4 onnxruntime-gpu opencv-python-headless trimesh scipy pillow --quiet && \
      cp /output/foundationpose_cpp.so /tmp/env_312/lib/python3.12/site-packages/foundationpose_cpp.so && \
      RPATH_STR='\$ORIGIN:\$ORIGIN/..:\$ORIGIN/../..:\$ORIGIN/../../..:\$ORIGIN/../../../..:\$ORIGIN/../../../../..:\$ORIGIN/numpy.libs:\$ORIGIN/scipy.libs:\$ORIGIN/opencv_python_headless.libs:\$ORIGIN/pillow.libs:\$ORIGIN/shapely.libs:\$ORIGIN/h5py.libs:\$ORIGIN/pyzmq.libs:\$ORIGIN/simsimd.libs:\$ORIGIN/../numpy.libs:\$ORIGIN/../scipy.libs:\$ORIGIN/../opencv_python_headless.libs:\$ORIGIN/../pillow.libs:\$ORIGIN/../shapely.libs:\$ORIGIN/../pyzmq.libs:\$ORIGIN/../simsimd.libs:\$ORIGIN/../../numpy.libs:\$ORIGIN/../../scipy.libs:\$ORIGIN/../../opencv_python_headless.libs:\$ORIGIN/../../pillow.libs:\$ORIGIN/../../shapely.libs:\$ORIGIN/../../pyzmq.libs:\$ORIGIN/../../simsimd.libs:\$ORIGIN/../../../numpy.libs:\$ORIGIN/../../../scipy.libs:\$ORIGIN/../../../opencv_python_headless.libs:\$ORIGIN/../../../pillow.libs:\$ORIGIN/../../../shapely.libs:\$ORIGIN/../../../pyzmq.libs:\$ORIGIN/../../../simsimd.libs:\$ORIGIN/../../../../numpy.libs:\$ORIGIN/../../../../scipy.libs:\$ORIGIN/../../../../opencv_python_headless.libs:\$ORIGIN/../../../../pillow.libs:\$ORIGIN/../../../../shapely.libs:\$ORIGIN/../../../../pyzmq.libs:\$ORIGIN/../../../../simsimd.libs:\$ORIGIN/../../../../../numpy.libs:\$ORIGIN/../../../../../scipy.libs:\$ORIGIN/../../../../../opencv_python_headless.libs:\$ORIGIN/../../../../../pillow.libs:\$ORIGIN/../../../../../shapely.libs:\$ORIGIN/../../../../../pyzmq.libs:\$ORIGIN/../../../../../simsimd.libs' && \
      find /tmp/env_312/lib/python3.12/site-packages -name '*.so*' -type f -exec patchelf --set-rpath \"\$RPATH_STR\" {} + 2>/dev/null || true; \
      cat << 'EOF_ACTIVATE' >> /tmp/env_312/bin/activate
_ENV_DIR=\"\$(cd \"\$(dirname \"\${BASH_SOURCE[0]}\")/..\" && pwd)\"
export PYTHONPATH=\"\${_ENV_DIR}/lib/python3.12/site-packages:\${PYTHONPATH}\"
export LD_LIBRARY_PATH=\"\${_ENV_DIR}/lib:\${_ENV_DIR}/lib/python3.12/site-packages/numpy.libs:\${LD_LIBRARY_PATH}\"
EOF_ACTIVATE
      cd /tmp/env_312 && tar -czf /output/env.tar.gz * && \
      chown -R ${HOST_UID}:${HOST_GID} /output && \
      echo 'SUCCESS: foundationpose_cpp.so and env.tar.gz built successfully for Python 3.12!'"

# 4. Package into final foundationpose_py312.tar.gz
echo "=========================================================================="
echo "  Step 3: Packaging final foundationpose_py312.tar.gz                     "
echo "=========================================================================="

mkdir -p "${OUTPUT_DIR}"
TARGET_TAR="${OUTPUT_DIR}/foundationpose_py312.tar.gz"

STAGE_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "${STAGE_DIR}"
}
trap cleanup EXIT

cp "${PACKAGE_ROOT}/config.pbtxt" "${STAGE_DIR}/config.pbtxt"
cp "${PACKAGE_ROOT}/model.py" "${STAGE_DIR}/model.py"
cp "${DIST_DIR}/foundationpose_cpp.so" "${STAGE_DIR}/foundationpose_cpp.so"
cp "${DIST_DIR}/env.tar.gz" "${STAGE_DIR}/env.tar.gz"

echo "Creating ${TARGET_TAR}..."
tar -czf "${TARGET_TAR}" -C "${STAGE_DIR}" config.pbtxt model.py foundationpose_cpp.so env.tar.gz

echo "=========================================================================="
echo "  SUCCESS! Python 3.12 FoundationPose Artifact Is Ready:                 "
echo "=========================================================================="
ls -lh "${TARGET_TAR}"
echo "Archive contents:"
tar -ztvf "${TARGET_TAR}"
echo "=========================================================================="
