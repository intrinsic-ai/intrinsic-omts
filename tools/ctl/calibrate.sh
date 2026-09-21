#!/bin/bash
# Camera and robot calibration subcommands for omts_ctl.sh.
# This file is a library: source it, do not execute it.

# Config files this library references, resolved through config_path() so that
# a configs/ directory reshuffle is a one-line change in _common.sh.
readonly CALIBRATION_WAYPOINTS_FILE="calibration_waypoints.pbtxt"
readonly AUTO_CALIBRATION_WAYPOINTS_FILE="auto_calibration_waypoints.pbtxt"

#######################################
# Syncs robot kinematic calibration from the controller.
# Globals:
#   INCTL_ADDR
# Arguments:
#   extra_args: Options forwarded to update_robot_kinematics.
#######################################
update_kinematics() {
  echo "Syncing robot kinematic calibration from controller on $INCTL_ADDR..."
  run_target //tools/calibration:update_robot_kinematics -- "$@"
}

#######################################
# Commands camera-to-robot calibration workflows.
# Globals:
#   INCTL_ADDR
# Arguments:
#   action: 'sample', 'run', or 'robot'.
#   extra_args: Sub-modes (manual/auto) or tool flags.
#######################################
cmd_calibrate() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 calibrate <sample|run|robot> [args...]"
    exit 0
  fi
  if [ $# -lt 1 ]; then
    die "Usage: $0 calibrate <sample|run|robot> [args...]"
  fi
  local action="$1"
  shift
  case "$action" in
    sample)
      local mode="manual"
      if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
        mode="$1"
        shift
      fi
      local mode_args=()
      if [ "$mode" = "manual" ]; then
        mode_args=(
          --manual_waypoints=true
          --moving_camera=true
          --export_waypoints_file="$(config_path "$CALIBRATION_WAYPOINTS_FILE")"
        )
      elif [ "$mode" = "auto" ]; then
        mode_args=(
          --manual_waypoints=false
          --moving_camera=true
          --num_samples=20
          --sample_box_halfsize_x=0.04
          --sample_box_halfsize_y=0.10
          --sample_box_halfsize_z=0.08
          --rand_angle=15.0
          --rand_roll_angle=75.0
          --export_waypoints_file="$(config_path "$AUTO_CALIBRATION_WAYPOINTS_FILE")"
        )
      else
        mode_args=("$mode")
      fi
      # --calibration_object="charuco_11x15_35mm_26mm_dict_4x4"
      # --calibration_object="charuco_9x12_30mm_22mm_dict_5x5"
      run_target //tools/calibration:sample_calibration_poses -- \
        --robot="icon" \
        --camera="orbbec_camera" \
        --calibration_object="charuco_9x14_20mm_15mm_dict_5x5" \
        ${mode_args[@]+"${mode_args[@]}"} \
        "$@"
      ;;
    run)
      # --calibration_object="charuco_11x15_35mm_26mm_dict_4x4"
      # --pose_estimator="charuco_11x15_35mm_26mm_dict_4x4_estimator"
      # --calibration_object="charuco_9x12_30mm_22mm_dict_5x5"
      # --pose_estimator="charuco_9x12_30mm_22mm_dict_5x5_estimator"
      run_target //tools/calibration:calibrate_camera -- \
        --robot="icon" \
        --camera="orbbec_camera" \
        --calibration_object="charuco_9x14_20mm_15mm_dict_5x5" \
        --pose_estimator="charuco_9x14_20mm_15mm_dict_5x5_estimator" \
        --import_waypoints_file="$(config_path "$CALIBRATION_WAYPOINTS_FILE")" \
        --moving_camera=true \
        --translation_rms_threshold=0.02 \
        --rotation_rms_threshold=1.0 \
        "$@"
      ;;
    robot|kinematics)
      update_kinematics "$@"
      ;;
    *)
      die "Error: Unknown calibrate action '$action'." \
        "Must be 'sample', 'run', or 'robot'."
      ;;
  esac
}
