#!/bin/bash
# Gripper, CNC machine, and cell-prep subcommands for omts_ctl.sh.
# This file is a library: source it, do not execute it.

#######################################
# Controls the gripper to open or close.
# Globals:
#   INCTL_ADDR
# Arguments:
#   action: "open" or "close"
#   --gripper_type: Gripper hardware type (default: robotiq, supports robotiq, dio).
#   Additional arguments forwarded to control_gripper.
#######################################
cmd_gripper() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 gripper <open|close> [--gripper_type <robotiq|dio>]"
    exit 0
  fi
  if [ $# -lt 1 ]; then
    die "Usage: $0 gripper <open|close> [extra_args...]"
  fi
  local action="$1"
  shift
  case "$action" in
    open|close) ;;
    *)
      die "Error: Unknown gripper action '$action'." \
        "Must be 'open' or 'close'."
      ;;
  esac

  local gripper_type="robotiq"
  local forwarded_args=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --gripper_type=*)
        gripper_type="${1#*=}"
        shift
        ;;
      --gripper_type)
        if [ $# -lt 2 ]; then
          die "Error: Flag '--gripper_type' requires an argument."
        fi
        gripper_type="$2"
        shift 2
        ;;
      *)
        forwarded_args+=("$1")
        shift
        ;;
    esac
  done

  run_target //tools/gripper:control_gripper -- \
    --gripper_type "$gripper_type" \
    --open_position=0.024 \
    --close_position=0.01 \
    --action_name="/gripper/gripper_action_controller/gripper_cmd" \
    --action "$action" \
    ${forwarded_args[@]+"${forwarded_args[@]}"}
}

# Commands the CNC door and vise together. $1 is "open" or "close" and builds
# the two actions; $2 is the word used in the status message.
machine_door_and_vise() {
  local action="$1"
  local state="$2"
  shift 2
  echo "Commanding CNC door and vise $state on $INCTL_ADDR..."
  run_target //tools/machine:control_machine -- \
    --action="${action}_door" "$@"
  run_target //tools/machine:control_machine -- \
    --action="${action}_vise" "$@"
}

#######################################
# Controls CNC machine door, vise, and machining cycle.
# Globals:
#   INCTL_ADDR
# Arguments:
#   action: One of open, close, open-door, close-door, open-vise, close-vise,
#           trigger-cycle, wait-cycle.
#   extra_args: Trailing options forwarded to control_machine.
#######################################
cmd_machine() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 machine" \
      "<open|close|open-door|close-door|open-vise|close-vise|" \
      "trigger-cycle|wait-cycle> [extra_args...]"
    exit 0
  fi
  if [ $# -lt 1 ]; then
    die "Usage: $0 machine" \
      "<open|close|open-door|close-door|open-vise|close-vise|" \
      "trigger-cycle|wait-cycle> [extra_args...]"
  fi
  local action="$1"
  shift
  local normalized_action="${action//-/_}"
  local default_args=()

  case "$normalized_action" in
    open)
      machine_door_and_vise open "open" "$@"
      return 0
      ;;
    close)
      machine_door_and_vise close "closed" "$@"
      return 0
      ;;
    open_door|close_door|open_vise|close_vise|trigger_cycle) ;;
    wait_cycle)
      default_args+=(--timeout_seconds 30.0)
      ;;
    *)
      die "Error: Unknown machine action '$action'."
      ;;
  esac

  run_target //tools/machine:control_machine -- \
    --action "$normalized_action" \
    ${default_args[@]+"${default_args[@]}"} \
    "$@"
}

#######################################
# Prepares OMTS cell: resets world, retracts arm, and closes door/vise.
# Globals:
#   INCTL_ADDR
# Arguments:
#   extra_args: Options forwarded to control_machine.
#######################################
cmd_prep_machine() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 prep-machine [extra_args...]"
    exit 0
  fi
  echo "Preparing cell, applying scene updates, retracting robot, and" \
    "closing door/vise on $INCTL_ADDR..."
  run_target //tools/world:apply_scene_updates -- --no-reset_sim
  run_target //tools/jogging:move_to_frame -- --frame="view"
  machine_door_and_vise close "closed" "$@"
}
