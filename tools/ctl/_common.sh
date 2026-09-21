#!/bin/bash
# Shared configuration and helpers for omts_ctl.sh.
# This file is a library: source it, do not execute it.

# Repository root. Derived from this file's own location so that library
# functions work regardless of the caller's working directory. An already
# exported value (set by omts_ctl.sh) wins.
SCRIPT_DIR="${SCRIPT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

HOME="${HOME:-/tmp}"
IOC_DIR="${IOC_DIR:-$(realpath "$SCRIPT_DIR/../ioc" 2>/dev/null || echo "$HOME/ioc")}"
INCTL_ADDR="${INCTL_ADDR:-localhost:17080}"
DEFAULT_APP="${DEFAULT_APP:-//:omts}"
APP_LOG="${APP_LOG:-$HOME/.omts_app.log}"
APP_PID="${APP_PID:-$HOME/.omts_app.pid}"
OPERATION_MODE="${OPERATION_MODE:-real}"
INCTL_BIN="${INCTL_BIN:-$IOC_DIR/bazel-bin/intrinsic/tools/inctl/inctl_external}"

# Single source of truth for on-disk config locations. Every configs/ path used
# by this tool is built from CONFIG_DIR, so a directory reshuffle is a one-line
# change here.
CONFIG_DIR="${CONFIG_DIR:-configs/omts}"

# Resolves a config file name to the path passed to the python tools.
config_path() {
  printf '%s/%s\n' "$CONFIG_DIR" "$1"
}

# Prints an error to stderr and exits non-zero.
die() {
  echo "$@" >&2
  exit 1
}

# Splits "host:port" into "<host> <port>", normalizing localhost to 127.0.0.1.
# $2 is an optional port to substitute when the address carries none; without
# it a port-less address yields the host as the port, as /dev/tcp probes expect.
resolve_address() {
  local addr="$1"
  local default_port="${2:-}"
  local host="${addr%:*}"
  local port="${addr##*:}"
  if [ "$host" = "$port" ] && [ -n "$default_port" ]; then
    port="$default_port"
  fi
  if [ "$host" = "localhost" ]; then
    host="127.0.0.1"
  fi
  printf '%s %s\n' "$host" "$port"
}

# Returns 0 if the given address accepts a TCP connection.
probe_port() {
  local host port
  read -r host port <<< "$(resolve_address "$1")"
  (echo > "/dev/tcp/$host/$port") 2>/dev/null
}

# Runs inctl from the prebuilt binary, falling back to a bazel run in IOC_DIR.
run_inctl() {
  if [ -x "$INCTL_BIN" ]; then
    "$INCTL_BIN" "$@"
  elif [ -d "$IOC_DIR" ]; then
    (cd "$IOC_DIR" && bazel run -c opt //intrinsic/tools/inctl:inctl_external -- -alsologtostderr "$@")
  else
    die "Error: inctl binary not found at $INCTL_BIN and IOC directory not found at $IOC_DIR"
  fi
}
