#!/usr/bin/env bash
#
# Checks Python and Bazel code formatting and linting across the repository.
# Mirrors the checks performed in GitHub Actions CI.
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

EXIT_CODE=0

echo "=== Checking Bazel formatting (buildifier) ==="
if command -v buildifier >/dev/null 2>&1; then
  if ! buildifier --mode=check -r .; then
    echo "❌ Buildifier format check failed. Run './tools/format.sh' to fix."
    EXIT_CODE=1
  else
    echo "✓ Bazel files formatting check passed."
  fi

  echo ""
  echo "=== Checking Bazel linting (buildifier) ==="
  if ! buildifier --lint=warn -r .; then
    echo "❌ Buildifier lint check failed. Run './tools/format.sh' to fix."
    EXIT_CODE=1
  else
    echo "✓ Bazel files lint check passed."
  fi
else
  echo "⚠ Warning: 'buildifier' not found in PATH."
  echo "  Install via: https://github.com/bazelbuild/buildtools or 'sudo apt install buildifier'"
  EXIT_CODE=1
fi

echo ""
echo "=== Checking Python linting and formatting (ruff) ==="
RUFF_CMD=""
if command -v ruff >/dev/null 2>&1; then
  RUFF_CMD="ruff"
elif python3 -m ruff --version >/dev/null 2>&1; then
  RUFF_CMD="python3 -m ruff"
elif [ -x "${HOME}/.jetski-server/extensions/charliermarsh.ruff-2026.78.0-linux-x64/bundled/libs/bin/ruff" ]; then
  RUFF_CMD="${HOME}/.jetski-server/extensions/charliermarsh.ruff-2026.78.0-linux-x64/bundled/libs/bin/ruff"
fi

if [ -n "${RUFF_CMD}" ]; then
  if ! ${RUFF_CMD} check .; then
    echo "❌ Ruff lint check failed. Run './tools/format.sh' to auto-fix where possible."
    EXIT_CODE=1
  else
    echo "✓ Python lint check passed."
  fi

  if ! ${RUFF_CMD} format --check .; then
    echo "❌ Ruff format check failed. Run './tools/format.sh' to fix."
    EXIT_CODE=1
  else
    echo "✓ Python format check passed."
  fi
else
  echo "⚠ Error: 'ruff' not found. Please install ruff with: pip install ruff"
  EXIT_CODE=1
fi

echo ""
if [ ${EXIT_CODE} -eq 0 ]; then
  echo "All CI lint and format checks passed successfully."
else
  echo "One or more lint/format checks failed."
fi

exit ${EXIT_CODE}
