# Pose estimator usage
Usage of the pose estimator is split into two parts:

## Creating a pose estimator data asset
This creates a data asset based on a scene object in the solution. Threshold parameters for detection and the number of refinement iterations for pose estimation are additionally defined.

```bash
bazel run //tools/pose_estimation:register_using_train_service -- \
  --address="localhost:17080" \
  --scene_object_id="ai.intrinsic.scene_object_id" \
  --pose_estimator_id="ai.intrinsic.my_pose_estimator" \
  --refinement_iters=3 \
  --confidence_threshold=0.6 \
  --visibility_threshold=0.6
```

- `address` specifies the address of the running solution
- `scene_object_id` specifies the scene object which will be detected and whose pose will be estimated
- `pose_estimator_id` defines the id of the pose estimator that will be created
- `refinement_iters` defines the number of refinement iterations that foundationpose runs. More iterations can lead to more accurate pose estimates at the cost of higher runtime.
- `confidence_threshold` is the threshold on the confidence score of the detector
- `visibility_threshold` is the threshold on the visibility score of the detector, i.e. what fraction of the part should be unoccluded to pass as a valid detection

## Running pose estimation
This triggers the estimate pose multi view skill and pose estimator service to run pose estimation for a given pose estimator data asset using a given camera. Currently only RGB-D cameras such as the Orbbec Gemini 335le are supported.

```bash
bazel run //tools/pose_estimation:run_pose_estimation -- \
  --address="localhost:17080" \
  --pose_estimator_id="ai.intrinsic.my_pose_estimator" \
  --camera_name="orbbec_camera" \
  --sensor_ids="1,4" \
  --service_name="pose_estimator_service" \
  --min_num_instances=1
```

- `address` specifies the address of the running solution
- `pose_estimator_id` specifies the pose estimator data asset that will be used for pose estimation
- `camera_name` specifies the camera that will be used for pose estimation
- `sensor_ids` specifies which sensor ids contain the rgb and depth image data for the camera. The service expects exactly one rgb and one depth image
- `service_name` specifies the name of the installed pose estimator service
- `min_num_instances` specifies the minimum number of instances that should be detected in the scene