#!/usr/bin/env bash
# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Formats Python and Bazel code across the repository.
# Can be run locally by developers before submitting a PR.
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

echo "=== Formatting Bazel files (buildifier) ==="
if command -v buildifier >/dev/null 2>&1; then
  buildifier -r .
  buildifier --lint=fix -r .
  echo "✓ Bazel files formatted and lint-fixed."
else
  echo "⚠ Warning: 'buildifier' not found in PATH. Skipping Bazel formatting."
  echo "  Install via: https://github.com/bazelbuild/buildtools or 'sudo apt install buildifier'"
fi

echo ""
echo "=== Formatting and fixing Python files (ruff) ==="
RUFF_CMD=""
if command -v ruff >/dev/null 2>&1; then
  RUFF_CMD="ruff"
elif python3 -m ruff --version >/dev/null 2>&1; then
  RUFF_CMD="python3 -m ruff"
elif [ -x "${HOME}/.jetski-server/extensions/charliermarsh.ruff-2026.78.0-linux-x64/bundled/libs/bin/ruff" ]; then
  RUFF_CMD="${HOME}/.jetski-server/extensions/charliermarsh.ruff-2026.78.0-linux-x64/bundled/libs/bin/ruff"
fi

if [ -n "${RUFF_CMD}" ]; then
  ${RUFF_CMD} check --fix .
  ${RUFF_CMD} format .
  echo "✓ Python files formatted and linted."
else
  echo "⚠ Error: 'ruff' not found. Please install ruff with: pip install ruff"
  exit 1
fi

echo ""
echo "All formatting completed successfully."
