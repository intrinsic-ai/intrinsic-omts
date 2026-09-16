#!/bin/bash
# Pose estimation subcommands for omts_ctl.sh.
# This file is a library: source it, do not execute it.

#######################################
# Resolves model keyword to scene_object_id:pose_estimator_id.
# Arguments:
#   model_key: Model alias or id (raw_stock, 2x3x5, or an explicit id).
# Outputs:
#   Prints "<scene_object_id>:<pose_estimator_id>".
#######################################
resolve_pose_model() {
  local key="${1:-raw_stock}"
  local scene_id
  local estimator_id
  case "$key" in
    raw_stock|2x3x5|default)
      scene_id="ai.intrinsic.raw_stock_2x3x5"
      estimator_id="${scene_id}_estimator"
      ;;
    *)
      if [[ "$key" == *_estimator ]]; then
        scene_id="${key%_estimator}"
        estimator_id="$key"
      else
        scene_id="$key"
        estimator_id="${key}_estimator"
      fi
      ;;
  esac
  echo "${scene_id}:${estimator_id}"
}

#######################################
# Runs pose estimation, single-shot image capture, or train service
# registration.
# Globals:
#   INCTL_ADDR
# Arguments:
#   action: capture, run, estimate, or train.
#   model: Optional model alias (defaults to raw_stock).
#   extra_args: Trailing options forwarded to the bazel tool.
#######################################
cmd_pose() {
  if [ $# -gt 0 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
    echo "Usage: $0 pose <capture|run|estimate|train> [model] [extra_args...]"
    exit 0
  fi
  if [ $# -lt 1 ]; then
    die "Usage: $0 pose <capture|run|estimate|train>" \
      "[model] [extra_args...]"
  fi
  local action="$1"
  shift

  local model resolved
  case "$action" in
    capture)
      run_target //tools/pose_estimation:run_pose_estimation -- \
        --camera_name="orbbec_camera" \
        --sensor_ids="1,4" \
        --capture_only \
        "$@"
      ;;
    run|estimate)
      model="raw_stock"
      if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
        model="$1"
        shift
      fi
      resolved="$(resolve_pose_model "$model")"
      run_target //tools/pose_estimation:run_pose_estimation -- \
        --pose_estimator_id="${resolved#*:}" \
        --camera_name="orbbec_camera" \
        --sensor_ids="1,4" \
        --service_name="pose_estimator_service" \
        --min_num_instances=1 \
        "$@"
      ;;
    train)
      model="raw_stock"
      if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
        model="$1"
        shift
      fi
      resolved="$(resolve_pose_model "$model")"
      run_target //tools/pose_estimation:register_using_train_service -- \
        --scene_object_id="${resolved%:*}" \
        --pose_estimator_id="${resolved#*:}" \
        --refinement_iters=3 \
        --confidence_threshold=0.6 \
        --visibility_threshold=0.6 \
        "$@"
      ;;
    *)
      die "Error: Unknown pose action '$action'." \
        "Must be 'capture', 'run', 'estimate', or 'train'."
      ;;
  esac
}
