# Pose Estimation Registration & Inference CLI (`tools/pose_estimation/`)

Pose estimator registration and standalone pose estimation CLI tools live in
[Intrinsic Core (`@intrinsic-core`)](https://github.com/intrinsic-ai/intrinsic-core)
and are invoked directly from `@intrinsic-core//intrinsic_perception/...`.

See the
[`nvidia_pose_estimator` README in `intrinsic-core`](https://github.com/intrinsic-ai/intrinsic-core/blob/main/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/README.md)
for SAM 3 weight export, model `DataAsset` packaging and sideloading,
`nvidia_pose_estimator_service` configuration, and TensorRT engine caching (note
that [`.bazelrc`](../../.bazelrc) in OMTS sets `build_full_image=True` and
`download_weights=True` by default).

## Upstream CLI Targets in `@intrinsic-core`

| Target | Source | Description |
| :--- | :--- | :--- |
| `@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service/tools:register_using_ioc_train_service` | [`register_using_ioc_train_service.py`](https://github.com/intrinsic-ai/intrinsic-core/blob/main/intrinsic_perception/intrinsic/perception/service/ioc_train_service/tools/register_using_ioc_train_service.py) | Generates and sideloads a FoundationPose + SAM 3 pose estimator `DataAsset` from an installed CAD scene object via `IocTrainService`. |
| `@intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:run_pose_estimation` | [`run_pose_estimation.py`](https://github.com/intrinsic-ai/intrinsic-core/blob/main/intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/run_pose_estimation.py) | Runs `capture_images` + `estimate_pose_multi_view` on `solution.executive` and prints 6D translations, quaternions, and RPY angles. |

## Invoking from the OMTS Workspace

### 1. Register the OMTS Workpiece Pose Estimator

```bash
bazel run @intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service/tools:register_using_ioc_train_service -- \
  --address="localhost:17080" \
  --scene_object_id="ai.intrinsic.raw_stock_2x3x5" \
  --pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator_text" \
  --refinement_iters=5 \
  --confidence_threshold=0.15 \
  --visibility_threshold=0.15 \
  --text_prompt="cuboid"
```

### 2. Run Standalone Pose Estimation

```bash
bazel run @intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:run_pose_estimation -- \
  --address="localhost:17080" \
  --service_name="nvidia_pose_estimator_service" \
  --pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator_text" \
  --camera_name="orbbec_camera" \
  --sensor_ids="1,4" \
  --min_num_instances=1 \
  --log_debug_data=true
```
