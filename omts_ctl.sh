#!/bin/bash
set -euo pipefail

# Entry point for OMTS cell control. Subcommand implementations live in
# tools/ctl/; this file owns the cell lifecycle, argument parsing, and dispatch.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/tools/ctl/_common.sh"
source "$SCRIPT_DIR/tools/ctl/_bazel.sh"
source "$SCRIPT_DIR/tools/ctl/machine.sh"
source "$SCRIPT_DIR/tools/ctl/world.sh"
source "$SCRIPT_DIR/tools/ctl/pose.sh"
source "$SCRIPT_DIR/tools/ctl/calibrate.sh"

# Deploys the solution to the cell, optionally in the background.
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

# Stops the running application by deploying the empty application.
stop_app() {
  echo "Stopping application on cluster (deploying empty application)..."
  if [ -s "$APP_PID" ]; then
    local pid
    pid="$(cat "$APP_PID" 2>/dev/null || true)"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      echo "Terminating background deployment process (PID: $pid)..."
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
      sleep 1
    fi
    rm -f "$APP_PID"
  fi
  (cd "$SCRIPT_DIR" && bazel run -c opt @ioc//intrinsic/config:empty_application -- --address="$INCTL_ADDR" --operation_mode real || true)
  echo "Application stopped."
}

# Clears ICON faults on the cell.
clear_faults() {
  echo "Clearing ICON faults on $INCTL_ADDR..."
  run_inctl icon --address="$INCTL_ADDR" --instance_name icon clear-faults
}

# Reports cell gateway reachability and active services.
status_cell() {
  if probe_port "$INCTL_ADDR"; then
    echo "Cell gateway ($INCTL_ADDR): REACHABLE"
    echo "--- Active Services ---"
    run_inctl service state list --address "$INCTL_ADDR" 2>/dev/null || echo "No active services or unable to query."
  else
    echo "Cell gateway ($INCTL_ADDR): NOT REACHABLE"
  fi
}

# Sideloads an asset or skill bundle via inctl.
install_asset() {
  if [ $# -lt 1 ]; then
    die "Usage: $0 install <path-to-bundle.tar>"
  fi
  if [ ! -f "$1" ]; then
    die "Error: File '$1' not found."
  fi
  local bundle_path
  bundle_path="$(realpath "$1")"
  echo "Installing asset: $bundle_path..."
  run_inctl asset install "$bundle_path" --address "$INCTL_ADDR"
}

# Runs the interactive keyboard jogging tool.
cmd_jog() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 jog [extra_args...]"
    exit 0
  fi
  local host port
  read -r host port <<< "$(resolve_address "$INCTL_ADDR" 17080)"
  (cd "$SCRIPT_DIR" && \
    bazel run -c opt //tools/jogging:jog_interactive -- \
      --host="$host" --port="$port" "$@")
}

# Launches the OMTS application.
cmd_app() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 app [extra_args...]"
    exit 0
  fi
  if [ $# -gt 0 ] && { [ "$1" = "full" ] || [ "$1" = "omts" ]; }; then
    shift
  fi
  run_target //:omts_app -- "$@"
}

usage() {
  cat <<EOF
Usage: $0 <command> [args]

Cell Lifecycle:
  status                              Check cell gateway and running app status
  clear                               Clear ICON faults on $INCTL_ADDR
  start [--bg] [--sim|--real] [target]
                                      Deploy solution to cell (--real default)
  stop                                Stop running app (deploy empty app)
  install <path-to-bundle.tar>        Sideload asset or skill bundle via inctl
  prep-machine                        Prep cell: reset world, retract, close CNC

Workcell Commands:
  jog                                 Launch interactive keyboard jogging tool
  move <frame|joint> [target]         Move robot to scene frame or joint config
  store <frame|joint> <name>          Store robot pose as frame or joint config
  world <apply|align> [files]         Apply scene updates or align tool frame
  world seed [infeed]                 Seed scene without sim reset
  sim seed [infeed]                   Seed sim scene with reset
  machine <action>                    Control CNC door, vise, or cycle
  gripper <open|close>                Control gripper (defaults to robotiq)
  calibrate <sample|run|robot>        Camera and robot calibration tools
  pose <capture|run|estimate|train>   Pose estimation, capture, or training
  app [extra_args...]                 Launch OMTS application
  help                                Show this help message
EOF
}

cmd="${1:-help}"
shift || true

case "$cmd" in
  help|--help|-h)     usage ;;
  status)             status_cell ;;
  clear|clear-faults) clear_faults ;;
  start)              start_app "$@" ;;
  stop)               stop_app ;;
  install)            install_asset "$@" ;;
  prep-machine|prep)  cmd_prep_machine "$@" ;;
  jog)                cmd_jog "$@" ;;
  move)               cmd_move "$@" ;;
  store)              cmd_store "$@" ;;
  world)              cmd_world "$@" ;;
  sim)                cmd_sim "$@" ;;
  machine)            cmd_machine "$@" ;;
  gripper)            cmd_gripper "$@" ;;
  calibrate)          cmd_calibrate "$@" ;;
  pose)               cmd_pose "$@" ;;
  app)                cmd_app "$@" ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage
    exit 1
    ;;
esac
