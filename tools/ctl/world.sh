#!/bin/bash
# World, simulation, move, and store subcommands for omts_ctl.sh.
# This file is a library: source it, do not execute it.

# Config files this library references, resolved through config_path() so that
# a configs/ directory reshuffle is a one-line change in _common.sh.
readonly SCENE_UPDATES_FILE="scene.updates.pbtxt"

# Applies the scene seed updates. $1 is --reset_sim or --no-reset_sim, $2 is
# the word used in the unknown-target error ("seed" or "sim seed"), and the
# remainder is the optional target plus flags for apply_scene_updates.
seed_scene() {
  local reset_flag="$1"
  local context="$2"
  shift 2
  local target="${1:-infeed}"
  shift || true
  case "$target" in
    infeed)
      run_target //tools/world:apply_scene_updates -- \
        --files="$(config_path "$SCENE_UPDATES_FILE")" \
        "$reset_flag" "$@"
      ;;
    *)
      die "Unknown $context target: $target (expected: infeed)"
      ;;
  esac
}

#######################################
# Applies scene updates (.pbtxt) or seeds the scene.
# Globals:
#   INCTL_ADDR
# Arguments:
#   action: 'apply' or 'seed'.
#   extra_args: File paths or trailing tool options.
#######################################
cmd_world() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 world <apply|seed> [files|extra_args...]"
    exit 0
  fi
  if [ $# -lt 1 ]; then
    die "Usage: $0 world <apply|seed> [extra_args...]"
  fi
  local action="$1"
  shift
  case "$action" in
    apply)
      # apply_scene_updates takes --files with nargs="*", so every bare path is
      # simply appended to one --files list and every other flag is forwarded.
      local files=()
      local extra=()
      while [ $# -gt 0 ]; do
        case "$1" in
          --files) ;;
          --files=*) files+=("${1#--files=}") ;;
          --*) extra+=("$1") ;;
          *) files+=("$1") ;;
        esac
        shift
      done
      local file_args=()
      if [ ${#files[@]} -gt 0 ]; then
        file_args=(--files "${files[@]}")
      fi
      run_target //tools/world:apply_scene_updates -- \
        ${file_args[@]+"${file_args[@]}"} \
        ${extra[@]+"${extra[@]}"}
      ;;
    seed)
      seed_scene --no-reset_sim "seed" "$@"
      ;;
    *)
      die "Error: Unknown world action '$action'." \
        "Must be 'apply' or 'seed'."
      ;;
  esac
}

#######################################
# Seeds simulation scene state with simulator reset.
# Arguments:
#   action: 'seed'.
#   target: 'infeed'.
#   extra_args: Options forwarded to apply_scene_updates.
#######################################
cmd_sim() {
  if [ $# -lt 1 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ] || [ "$1" = "help" ]; then
    echo "Usage: $0 sim <seed> [infeed] [extra_args...]"
    exit 0
  fi
  local action="$1"
  shift
  case "$action" in
    seed)
      seed_scene --reset_sim "sim seed" "$@"
      ;;
    *)
      die "Unknown sim command: $action"
      ;;
  esac
}

#######################################
# Commands robot arm to move to a frame or joint configuration.
# Globals:
#   INCTL_ADDR
# Arguments:
#   kind: 'frame' or 'joint'.
#   target: Optional frame or joint name.
#   extra_args: Options forwarded to motion tool.
#######################################
cmd_move() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 move <frame|joint> [target] [extra_args...]"
    exit 0
  fi
  if [ $# -lt 1 ]; then
    die "Usage: $0 move <frame|joint> [target] [extra_args...]"
  fi
  local kind="$1"
  shift
  case "$kind" in
    frame)
      local frame_arg=()
      if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
        frame_arg=(--frame="$1")
        shift
      fi
      run_target //tools/jogging:move_to_frame -- \
        ${frame_arg[@]+"${frame_arg[@]}"} \
        "$@"
      ;;
    joint)
      local target_args=()
      if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
        target_args=("$1")
        shift
      fi
      run_target //tools/jogging:move_to_joint -- \
        ${target_args[@]+"${target_args[@]}"} \
        "$@"
      ;;
    *)
      die "Error: Unknown move target kind '$kind'." \
        "Must be 'frame' or 'joint'."
      ;;
  esac
}

#######################################
# Stores current robot pose as a named frame or joint configuration.
# Globals:
#   INCTL_ADDR
# Arguments:
#   kind: 'frame' or 'joint'.
#   name: Target name for the frame or joint configuration.
#   extra_args: Options forwarded to storage tool.
#######################################
cmd_store() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 store <frame|joint> <name> [extra_args...]"
    exit 0
  fi
  if [ $# -lt 2 ]; then
    die "Usage: $0 store <frame|joint> <name> [extra_args...]"
  fi
  local kind="$1"
  local name="$2"
  shift 2
  case "$kind" in
    frame)
      run_target //tools/jogging:store_frame -- "$name" "$@"
      ;;
    joint)
      run_target //tools/jogging:store_joint_config -- "$name" "$@"
      ;;
    *)
      die "Error: Unknown store target kind '$kind'." \
        "Must be 'frame' or 'joint'."
      ;;
  esac
}
