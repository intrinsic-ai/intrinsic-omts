# Pose Estimation Registration & Inference CLI (`tools/pose_estimation/`)

CLI utilities for registering CAD scene objects as FoundationPose estimators via
the Intrinsic Core Train Service (`IocTrainService`) and executing standalone 6D
pose estimation.

## CLI Targets

| Target | Source File | Description |
| :--- | :--- | :--- |
| `//tools/pose_estimation:register_using_train_service` | [`register_using_train_service.py`](register_using_train_service.py) | Generates and sideloads a FoundationPose estimator asset from an installed CAD scene object via `IocTrainService`. |
| `//tools/pose_estimation:run_pose_estimation` | [`run_pose_estimation.py`](run_pose_estimation.py) | Runs `capture_images` + `estimate_pose_multi_view` on `solution.executive` and prints 6D translations, quaternions, and RPY angles. |

## 1. Registering a Pose Estimator Data Asset

Creates and sideloads a pose estimator asset from an installed scene object:

```bash
bazel run //tools/pose_estimation:register_using_train_service -- \
  --address="localhost:17080" \
  --scene_object_id="ai.intrinsic.raw_stock_2x3x5" \
  --pose_estimator_id="ai.intrinsic.my_pose_estimator" \
  --refinement_iters=3 \
  --confidence_threshold=0.6 \
  --visibility_threshold=0.6
```

* `--address`: gRPC address of the running solution deployment.
* `--scene_object_id`: Asset ID of the target CAD scene object (`<package>.<name>`).
* `--pose_estimator_id`: Asset ID to assign to the generated pose estimator (`<package>.<name>`).
* `--refinement_iters`: Number of FoundationPose refinement iterations (default: `3`).
* `--confidence_threshold`: Minimum detector confidence score threshold (default: `0.6`).
* `--visibility_threshold`: Minimum unoccluded visibility fraction threshold (default: `0.6`).

## 2. Running Pose Estimation

Executes RGB-D capture and multi-view 6D pose estimation using a registered pose
estimator asset:

```bash
bazel run //tools/pose_estimation:run_pose_estimation -- \
  --address="localhost:17080" \
  --pose_estimator_id="ai.intrinsic.my_pose_estimator" \
  --camera_name="orbbec_camera" \
  --sensor_ids="1,4" \
  --service_name="pose_estimator_service" \
  --min_num_instances=1 \
  --log_debug_data=true
```

* `--address`: gRPC address of the running solution deployment.
* `--pose_estimator_id`: Asset ID of the registered pose estimator.
* `--camera_name`: Camera resource name (defaults to `orbbec_camera` or auto-detected `CameraConfig` resource).
* `--sensor_ids`: Comma-separated camera stream IDs (`1,4` for RGB + Depth on Orbbec Gemini 335Le).
* `--service_name`: Pose estimator service resource name (default: `pose_estimator_service`).
* `--min_num_instances`: Minimum required detected instances before skill failure (default: `1`).
* `--log_debug_data`: Whether to log debug point clouds and images during execution (default: `true`).
* `--timeout_sec`: Optional inference timeout in seconds.