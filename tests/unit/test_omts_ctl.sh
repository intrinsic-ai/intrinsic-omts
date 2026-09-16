#!/bin/bash
# Regression tests for omts_ctl.sh. Two bugs are guarded here:
#   1. A flag missing its argument must fail cleanly, not crash under `set -u`.
#   2. `world apply` must populate --files from positional paths.
# Plain string formatting of the emitted bazel command is not asserted; that is
# the tool's own business and churns with every flag change.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OMTS_CTL="$(cd "$SCRIPT_DIR/../.." && pwd)/omts_ctl.sh"
TEST_TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TEST_TMPDIR"' EXIT

# Stub bazel so subcommands record their argv instead of building anything.
mkdir -p "$TEST_TMPDIR/bin"
cat << 'MOCK_EOF' > "$TEST_TMPDIR/bin/bazel"
#!/bin/bash
echo "$@" >> "$BAZEL_LOG"
MOCK_EOF
chmod +x "$TEST_TMPDIR/bin/bazel"
export PATH="$TEST_TMPDIR/bin:$PATH"
export BAZEL_LOG="$TEST_TMPDIR/bazel.log"

assert_contains() {
  if ! grep -F -- "$1" "$2" > /dev/null; then
    echo "ASSERTION FAILED: '$1' not found in $2" >&2
    cat "$2" >&2
    exit 1
  fi
}

echo "=== Missing flag argument fails cleanly under set -u ==="
err_log="$TEST_TMPDIR/err.log"
if "$OMTS_CTL" gripper open --gripper_type 2> "$err_log"; then
  echo "ASSERTION FAILED: expected missing flag argument to fail" >&2
  exit 1
fi
assert_contains "Error: Flag '--gripper_type' requires an argument." "$err_log"
if grep -qi "unbound variable" "$err_log"; then
  echo "ASSERTION FAILED: unbound variable error:" >&2
  cat "$err_log" >&2
  exit 1
fi

echo "=== world apply populates --files ==="
: > "$BAZEL_LOG"
"$OMTS_CTL" world apply scene_a.updates.pbtxt scene_b.updates.pbtxt
assert_contains "--files scene_a.updates.pbtxt scene_b.updates.pbtxt" "$BAZEL_LOG"

# A flag preceding a path must not swallow it, and must still be forwarded.
: > "$BAZEL_LOG"
"$OMTS_CTL" world apply --some_flag scene_a.updates.pbtxt
assert_contains "--files scene_a.updates.pbtxt" "$BAZEL_LOG"
assert_contains "--some_flag" "$BAZEL_LOG"

# With no paths at all the empty arrays must not trip set -u either.
: > "$BAZEL_LOG"
"$OMTS_CTL" world apply
assert_contains "//tools/world:apply_scene_updates -- --address=" "$BAZEL_LOG"

echo "All tests passed."
