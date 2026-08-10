#!/usr/bin/env bash
#
# Builds and installs skills and services listed in required_assets.txt
# into a target Intrinsic cluster.
#
# Usage:
#   ./install_assets.sh [options]
#
# Options:
#   -f, --file <path>             Path to required_assets.txt (default: ./required_assets.txt)
#   -a, --address <host:port>      Address of the cluster gateway (default: localhost:17080)
#   -w, --workspace <path>        Path to the Bazel workspace (default: ../ioc or current workspace)
#   -c, --compilation_mode <mode> Bazel compilation mode: fastbuild, dbg, opt (default: opt)
#   -n, --dry-run                 Print commands without executing them
#       --build-only              Only build bundle.tar assets without installing
#       --install-only            Only install existing bundle.tar assets without building
#   -h, --help                    Show this help message

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Defaults
ASSETS_FILE="${SCRIPT_DIR}/required_assets.txt"
ADDRESS="${ADDRESS:-localhost:17080}"
WORKSPACE_DIR="${IOC_DIR:-}"
COMPILATION_MODE="${COMPILATION_MODE:-opt}"
DRY_RUN=false
BUILD_ONLY=false
INSTALL_ONLY=false

usage() {
  sed -n '/^# Usage:/,/^#   -h, --help/p' "$0" | sed 's/^# \?//'
  exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    -f|--file)
      ASSETS_FILE="$2"
      shift 2
      ;;
    -a|--address)
      ADDRESS="$2"
      shift 2
      ;;
    -w|--workspace)
      WORKSPACE_DIR="$2"
      shift 2
      ;;
    -c|--compilation_mode)
      COMPILATION_MODE="$2"
      shift 2
      ;;
    -n|--dry-run)
      DRY_RUN=true
      shift
      ;;
    --build-only)
      BUILD_ONLY=true
      shift
      ;;
    --install-only)
      INSTALL_ONLY=true
      shift
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Error: Unknown argument: $1" >&2
      usage
      ;;
  esac
done

# Resolve Bazel workspace directory
if [[ -z "${WORKSPACE_DIR}" ]]; then
  if [[ -f "${SCRIPT_DIR}/MODULE.bazel" || -f "${SCRIPT_DIR}/WORKSPACE" ]]; then
    WORKSPACE_DIR="${SCRIPT_DIR}"
  elif [[ -f "${SCRIPT_DIR}/../ioc/MODULE.bazel" || -f "${SCRIPT_DIR}/../ioc/WORKSPACE" ]]; then
    WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../ioc" && pwd)"
  elif [[ -f "MODULE.bazel" || -f "WORKSPACE" ]]; then
    WORKSPACE_DIR="$(pwd)"
  else
    echo "Error: Could not locate Bazel workspace (ioc). Specify with -w <path> or set IOC_DIR." >&2
    exit 1
  fi
fi

if [[ ! -f "${ASSETS_FILE}" ]]; then
  echo "Error: Assets file not found: ${ASSETS_FILE}" >&2
  exit 1
fi

echo "============================================================"
echo "Asset Installer Configuration"
echo "============================================================"
echo "Assets file:      ${ASSETS_FILE}"
echo "Bazel workspace:  ${WORKSPACE_DIR}"
echo "Cluster address:  ${ADDRESS}"
echo "Compilation mode: ${COMPILATION_MODE}"
echo "Dry run:          ${DRY_RUN}"
echo "============================================================"
echo ""

# Parse target labels from file (ignoring comments and blank lines)
TARGETS=()
BUNDLE_PATHS=()

while IFS= read -r line || [[ -n "$line" ]]; do
  # Trim leading and trailing whitespace
  trimmed="$(echo "$line" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"

  # Skip empty lines or comment lines
  if [[ -z "$trimmed" || "$trimmed" =~ ^# ]]; then
    continue
  fi

  target="$trimmed"
  clean_target="${target#//}"

  if [[ "$clean_target" == *:* ]]; then
    pkg="${clean_target%%:*}"
    name="${clean_target##*:}"
  else
    pkg="$clean_target"
    name="$(basename "$clean_target")"
  fi

  bundle_path="bazel-bin/${pkg}/${name}.bundle.tar"

  TARGETS+=("$target")
  BUNDLE_PATHS+=("$bundle_path")
done < "${ASSETS_FILE}"

if [[ ${#TARGETS[@]} -eq 0 ]]; then
  echo "No valid targets found in ${ASSETS_FILE}."
  exit 0
fi

echo "Found ${#TARGETS[@]} asset(s) to process:"
for i in "${!TARGETS[@]}"; do
  echo "  [$((i + 1))] ${TARGETS[i]}"
  echo "      -> ${BUNDLE_PATHS[i]}"
done
echo ""

cd "${WORKSPACE_DIR}"

BUILD_FAILED=()
BUILD_SUCCEEDED=()

# Step 1: Bazel Build (target by target for fault tolerance and per-target reporting)
if [[ "${INSTALL_ONLY}" == false ]]; then
  echo "============================================================"
  echo "Building asset bundles in ${WORKSPACE_DIR}..."
  echo "============================================================"

  for i in "${!TARGETS[@]}"; do
    target="${TARGETS[i]}"
    echo "------------------------------------------------------------"
    echo "Building [$((i + 1))/${#TARGETS[@]}]: ${target}"
    echo "------------------------------------------------------------"

    BUILD_CMD=(bazel build -c "${COMPILATION_MODE}" "${target}")
    echo "+ ${BUILD_CMD[*]}"

    if [[ "${DRY_RUN}" == true ]]; then
      BUILD_SUCCEEDED+=("$i")
      continue
    fi

    set +e
    "${BUILD_CMD[@]}"
    build_exit=$?
    set -e

    if [[ $build_exit -eq 0 ]]; then
      BUILD_SUCCEEDED+=("$i")
    else
      echo "Warning: Build failed for target ${target} (exit code ${build_exit})." >&2
      BUILD_FAILED+=("$target")
    fi
    echo ""
  done
else
  for i in "${!TARGETS[@]}"; do
    BUILD_SUCCEEDED+=("$i")
  done
fi

# Step 2: Install via inctl
if [[ "${BUILD_ONLY}" == false ]]; then
  echo "============================================================"
  echo "Installing asset bundles to ${ADDRESS}..."
  echo "============================================================"

  INSTALLED_COUNT=0
  SKIPPED_COUNT=0
  FAILED_COUNT=0

  for i in "${BUILD_SUCCEEDED[@]}"; do
    target="${TARGETS[i]}"
    bundle_path="${BUNDLE_PATHS[i]}"

    echo "------------------------------------------------------------"
    echo "Installing [$((i + 1))/${#TARGETS[@]}]: ${target}"
    echo "Bundle: ${bundle_path}"
    echo "------------------------------------------------------------"

    if [[ "${DRY_RUN}" == false && ! -f "${bundle_path}" ]]; then
      echo "Error: Bundle tar file does not exist: ${bundle_path}" >&2
      FAILED_COUNT=$((FAILED_COUNT + 1))
      continue
    fi

    INSTALL_CMD=(
      bazel run -c "${COMPILATION_MODE}" //google3/intrinsic/tools/inctl:inctl_external --
      -alsologtostderr asset install
      --address "${ADDRESS}"
      "${bundle_path}"
    )

    echo "+ ${INSTALL_CMD[*]}"

    if [[ "${DRY_RUN}" == true ]]; then
      continue
    fi

    # Run installation and capture output to inspect for AlreadyExists
    set +e
    output="$("${INSTALL_CMD[@]}" 2>&1)"
    exit_code=$?
    set -e

    echo "$output"

    if [[ $exit_code -eq 0 ]]; then
      echo "Status: Successfully installed."
      INSTALLED_COUNT=$((INSTALLED_COUNT + 1))
    elif echo "$output" | grep -q "AlreadyExists"; then
      echo "Status: Asset is already installed. Skipping."
      SKIPPED_COUNT=$((SKIPPED_COUNT + 1))
    else
      echo "Status: Installation failed with exit code ${exit_code}." >&2
      FAILED_COUNT=$((FAILED_COUNT + 1))
    fi
    echo ""
  done

  echo "============================================================"
  echo "Installation Summary"
  echo "============================================================"
  echo "Total assets:      ${#TARGETS[@]}"
  echo "Build failures:    ${#BUILD_FAILED[@]}"
  echo "Installed:         ${INSTALLED_COUNT}"
  echo "Already present:   ${SKIPPED_COUNT}"
  echo "Install failures:  ${FAILED_COUNT}"
  echo "============================================================"

  if [[ ${#BUILD_FAILED[@]} -gt 0 || ${FAILED_COUNT} -gt 0 ]]; then
    exit 1
  fi
fi
