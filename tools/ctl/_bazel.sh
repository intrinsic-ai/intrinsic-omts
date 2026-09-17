#!/bin/bash
# Bazel invocation helper for omts_ctl.sh.
# This file is a library: source it, do not execute it.

# Runs a bazel target from the repository root with --address prepended to the
# target's own arguments. An optional literal "--" after the label is ignored,
# so call sites read like the command they emit.
run_target() {
  local label="$1"
  shift
  if [ "${1:-}" = "--" ]; then
    shift
  fi
  (cd "$SCRIPT_DIR" && bazel run -c opt "$label" -- \
    --address="$INCTL_ADDR" "$@")
}
