#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IOC_DIR="${IOC_DIR:-$(realpath "$SCRIPT_DIR/../ioc" 2>/dev/null || echo "$HOME/ioc")}"
INCTL_ADDR="${INCTL_ADDR:-localhost:17080}"
DEFAULT_APP="${DEFAULT_APP:-//:omts}"
APP_LOG="${APP_LOG:-$HOME/.omts_app.log}"
APP_PID="${APP_PID:-$HOME/.omts_app.pid}"
OPERATION_MODE="${OPERATION_MODE:-real}"
INCTL_BIN="${INCTL_BIN:-$IOC_DIR/bazel-bin/google3/intrinsic/tools/inctl/inctl_external}"

probe_port() {
  local addr="$1"
  local host="${addr%:*}"
  local port="${addr##*:}"
  [ "$host" = "localhost" ] && host="127.0.0.1"
  (echo > "/dev/tcp/$host/$port") 2>/dev/null
}

run_inctl() {
  if [ -x "$INCTL_BIN" ]; then
    "$INCTL_BIN" "$@"
  elif [ -d "$IOC_DIR" ]; then
    (cd "$IOC_DIR" && bazel run -c opt //google3/intrinsic/tools/inctl:inctl_external -- -alsologtostderr "$@")
  else
    echo "Error: inctl binary not found at $INCTL_BIN and IOC directory not found at $IOC_DIR" >&2
    exit 1
  fi
}

start_app() {
  local bg=false
  local target="$DEFAULT_APP"
  local op_mode="$OPERATION_MODE"

  while [ $# -gt 0 ]; do
    case "$1" in
      --bg|--background|-d)
        bg=true
        shift
        ;;
      --sim|--simulation)
        op_mode="sim"
        shift
        ;;
      --real)
        op_mode="real"
        shift
        ;;
      *)
        target="$1"
        shift
        ;;
    esac
  done

  if [ "$bg" = true ]; then
    if [ -s "$APP_PID" ]; then
      local pid
      pid="$(cat "$APP_PID" 2>/dev/null || true)"
      if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        echo "Solution deployer is already running (PID: $pid)."
        return 0
      fi
      rm -f "$APP_PID"
    fi
    echo "Deploying solution $target in background (mode: $op_mode, logging to $APP_LOG)..."
    (cd "$SCRIPT_DIR" && nohup bash -c "bazel run -c opt '$target' -- --address='$INCTL_ADDR' --operation_mode '$op_mode' && if [ '$op_mode' = 'real' ]; then bazel run -c opt //tools/calibration:update_robot_kinematics -- --address='$INCTL_ADDR'; fi" > "$APP_LOG" 2>&1 & echo $! > "$APP_PID")
    sleep 2
    local pid
    pid="$(cat "$APP_PID" 2>/dev/null || true)"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      echo "Solution deployment started with PID $pid. Follow logs with: tail -f $APP_LOG"
    else
      echo "Solution deployment finished or exited. Check logs with: cat $APP_LOG"
      rm -f "$APP_PID"
    fi
  else
    echo "Deploying solution $target (mode: $op_mode, Ctrl+C to cancel)..."
    (cd "$SCRIPT_DIR" && bazel run -c opt "$target" -- --address="$INCTL_ADDR" --operation_mode "$op_mode")
    if [ "$op_mode" = "real" ]; then
      update_kinematics || echo "Warning: Robot kinematic calibration update failed or timed out."
    fi
  fi
}

update_kinematics() {
  echo "Syncing robot kinematic calibration from controller on $INCTL_ADDR..."
  (cd "$SCRIPT_DIR" && bazel run -c opt //tools/calibration:update_robot_kinematics -- --address="$INCTL_ADDR" "$@")
}

stop_app() {
  echo "Stopping application on cluster (deploying empty application)..."
  (cd "$SCRIPT_DIR" && bazel run -c opt @ioc//google3/intrinsic/config:empty_application -- --address="$INCTL_ADDR" --operation_mode real || true)
  rm -f "$APP_PID"
  echo "Application stopped."
}

clear_faults() {
  echo "Clearing ICON faults on $INCTL_ADDR..."
  run_inctl icon --address="$INCTL_ADDR" --instance_name icon clear-faults
}

status_cell() {
  if probe_port "$INCTL_ADDR"; then
    echo "Cell gateway ($INCTL_ADDR): REACHABLE"
    echo "--- Active Services ---"
    run_inctl service state list --address "$INCTL_ADDR" 2>/dev/null || echo "No active services or unable to query."
  else
    echo "Cell gateway ($INCTL_ADDR): NOT REACHABLE"
  fi
}

install_asset() {
  if [ $# -lt 1 ]; then
    echo "Usage: $0 install <path-to-bundle.tar>" >&2
    exit 1
  fi
  if [ ! -f "$1" ]; then
    echo "Error: File '$1' not found." >&2
    exit 1
  fi
  local bundle_path
  bundle_path="$(realpath "$1")"
  echo "Installing asset: $bundle_path..."
  run_inctl asset install "$bundle_path" --address "$INCTL_ADDR"
}

usage() {
  cat <<EOF
Usage: $0 <command> [args]

Commands:
  status                              Check cell gateway and running app status
  start [--bg] [--sim|--real] [target] Deploy solution to cell (--real default, --sim for simulation)
  calibrate-robot                     Fetch and update robot kinematic calibration from UR controller
  stop                                Stop running app (deploy empty application)
  clear                               Clear ICON faults on $INCTL_ADDR
  install <path-to-bundle.tar>        Sideload / install asset or skill bundle via inctl
  help                                Show this help message
EOF
}

cmd="${1:-help}"
shift || true

case "$cmd" in
  status)                                           status_cell ;;
  start)                                            start_app "$@" ;;
  calibrate-robot|update-kinematics|sync-kinematics) update_kinematics "$@" ;;
  stop)                                             stop_app ;;
  clear|clear-faults)                               clear_faults ;;
  install)                                          install_asset "$@" ;;
  help|--help|-h)                                   usage ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage
    exit 1
    ;;
esac
