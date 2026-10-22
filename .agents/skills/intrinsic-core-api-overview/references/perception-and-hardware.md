# Perception, camera calibration, pose estimation, and hardware `I/O` reference

This document provides technical reference documentation, multi-RPC sequences, and API usage hints for interacting with cameras, camera-to-robot calibration workflows, 6D pose estimation, server-side image post-processing, hardware GPIO signals, and EOAT/gripper services.

---

## Paired guardrails for perception and hardware operations

To ensure operational safety, prevent hardware faults, and avoid system deadlocks, adhere strictly to the following five paired guardrails:

1. **Camera register write protection**: Do not attempt to mutate camera device registers (such as `AcquisitionMode` or trigger parameters) while continuous streaming or frame capture is active.
   * *Affirmative target action*: Explicitly pause or stop active streams or acquisitions before sending `UpdateCameraSetting` requests.
2. **Calibration target unit convention**: Do not specify `square_length` or `marker_length` on `CharucoPattern` in millimeters.
   * *Affirmative target action*: Always supply square and marker dimensions in meters (for example, `0.020` for 20 mm) satisfying `0 < marker_length < square_length`.
3. **Multi-view view count limit**: Do not pass more than 4 camera views simultaneously to multi-view 6DoF pose estimation (`estimate_pose_multi_view`).
   * *Affirmative target action*: Bound input views to <= 4 distinct camera poses with wide angular baselines to ensure well-conditioned ray midpoints and prevent combinatorial tensor broadcasting.
4. **Persistent calibration gating**: Do not sync the runtime belief world (`"world"`) to the initial world (`"init_world"`) or commit calibration assets without validating reprojection error.
   * *Affirmative target action*: Verify that 2D RMS reprojection error is < 1.0 px and 3D mean error is < 1.0 mm before calling `CalibrationService/Save` or updating `"init_world"`.
5. **Zenoh KV store buffer leaks**: Do not discard `capture_results/<UUID>` references without explicitly evicting stored image buffers.
   * *Affirmative target action*: Always wrap KV store capture consumption in a `try/finally` block that issues a delete request for every allocated capture key.

---

## System 2 reflection checkpoints for perception and hardware mutations

Before executing high-cost, state-mutating, or physically hazardous actions, pause and execute a brief System 2 self-critique:

1. **Pre-calibration checkpoint**:
   * *Verification*: Verify that `CharucoPattern` square and marker lengths are specified in meters (`0 < marker_length < square_length`), target calibration object is registered in `ObjectWorld`, at least 3 distinct pose pairs are sampled, and accumulated frames from prior sessions are cleared (< 100 stored captures).
2. **Pre-commit calibration checkpoint**:
   * *Verification*: Verify that 2D RMS error is < 1.0 px and 3D error is < 1.0 mm on validation poses before calling `CalibrationService/Save` or syncing `"world"` transforms back to `"init_world"`.
3. **Pre-GPIO write session checkpoint**:
   * *Verification*: Verify whether target signals belong to an OPC-UA array (where claiming an individual index locks the entire parent node), ensure Envoy routing metadata `x-resource-instance-name` is attached to the channel, and confirm that no other concurrent session holds active claims on that node.
4. **Pre-grasp payload transition checkpoint**:
   * *Verification*: Verify that `change_payload_index` is invoked while the robot is stationary before or after mechanical coupling/release to prevent collaborative force limit faults.
5. **Pre-model destruction checkpoint**:
   * *Verification*: Verify that `cleanup_model_on_destruction = false` is configured on short-lived training or evaluation clients to prevent 60-second GPU weight reload latency spikes on the model server.

---

## Anti-thrashing circuit breakers

To prevent infinite retry loops, action thrashing, and prompt rot during hardware or perception failures:

1. **Camera capture & streaming circuit breaker**: Cap retries at <= 2 attempts. If `DEADLINE_EXCEEDED` or dropped frames persist, branch to verifying GigE Vision MTU (enabling 9000 Jumbo Frames), switching `PixelFormat` from `RGB8` to `BayerRG8`, attaching `x-resource-instance-name: <camera_name>` to the gRPC channel, and waiting 10 seconds for WebRTC backend encoder teardown.
2. **Calibration pattern detection circuit breaker**: Cap retries at <= 2 attempts per waypoint. If `CAPTURE_DATA_STATUS_INSUFFICIENT_MARKER_POINTS` or `BLURRED_IMAGE` recurs, branch to verifying illumination, robot look-at orientation, and ensuring at least 10 non-collinear ChArUco corners are unoccluded (with >= 3 valid pose pairs collected).
3. **Pose estimation & model inference circuit breaker**: Cap retries at <= 2 attempts. If `RunPoseEstimation` returns `INVALID_ARGUMENT` or times out, branch to verifying the dual-sensor constraint (exactly 1 intensity image and 1 depth image in meters in a single `CaptureResult`), verifying `cleanup_model_on_destruction = false`, and checking GPU memory before repeating inference calls.
4. **GPIO wait-for-value circuit breaker**: Cap retries at <= 2 attempts. If `WaitForValue` hits its deadline, branch to checking backend response semantics (e.g. ICON ADIO leaves `SignalValueSet` empty and requires follow-up `ReadSignals`), signal polarity, or OPC-UA server connectivity rather than re-issuing `WaitForValue`.
5. **Gripper fault recovery circuit breaker**: Cap retries at <= 2 attempts. If `ServiceState/Enable` fails to clear faults, branch to checking physical E-stop switches, pneumatic supply pressure, or `inctl icon clear-faults --instance_name=icon` rather than repeatedly looping on `Enable`.

---

## Camera service, offline configuration, and camera settings

### Core camera `gRPC` services, `Envoy` routing, and data hierarchy

- `intrinsic_proto.perception.v1.CameraService`: Exposes sensor discovery (`ListAvailableCameras`, `DescribeCamera`), frame acquisition (`Capture`, `CaptureStream`), and GenICam Standard Features Naming Convention (SFNC) or ROS driver parameter inspection and mutation (`ReadCameraSettingAccess`, `ReadCameraSettingProperties`, `ReadCameraSetting`, `UpdateCameraSetting`).
- **Envoy ingress routing for camera drivers**:
  Camera drivers (such as `basler_camera` or `photoneo_camera`) are deployed Intrinsic asset instances. When connecting through the Envoy ingress gateway (`localhost:17080` externally or `istio-ingressgateway.app-ingress.svc.cluster.local:80` inside Kubernetes pods), all RPCs to `CameraService` **must attach the metadata header `x-resource-instance-name: <camera_instance_name>`**. Omitting this header causes Envoy to return `rpc error: code = Unimplemented`. In Python, use `intrinsic.util.grpc.connection.ConnectionParams(address=..., instance_name="<camera_name>", header="x-resource-instance-name")` with `HeaderAdderInterceptor`.
- Data hierarchy (`SensorConfig` -> `CameraParams` -> `IntrinsicParams` / `DistortionParams`):
  - `intrinsic_proto.perception.v1.SensorConfig`: Specifies calibration and mounting geometry (`id`, `camera_t_sensor`, and `camera_params`) for an individual sensor stream within a single- or multi-sensor camera device (for example, `id = 0` for single-sensor GenICam cameras; `id = 1` for RGB and `id = 4` for Depth on multi-sensor RGB-D cameras).
  - `intrinsic_proto.perception.v1.CameraParams`: Bundles `intrinsic_params` (pinhole projection matrix parameters `fx, fy, cx, cy` and image plane `dimensions` in pixels) and optional `distortion_params` (radial `k1..k6`, tangential `p1, p2`, thin prism `s1..s4`, and tilted sensor `tx, ty`).

### Offline camera configuration extraction via resource registry

Services that require a camera's `IntrinsicParams` without opening an active gRPC connection to the camera hardware driver query `intrinsic_proto.resources.ResourceRegistry/GetResourceInstance` by camera resource name:

1. Call `intrinsic_proto.resources.ResourceRegistry/GetResourceInstance` passing the camera resource instance name (for example, `"basler_camera"`).
2. Extract `resource_handle.resource_data["CameraConfig"].contents` and unpack `type.googleapis.com/intrinsic_proto.perception.v1.CameraConfig`.
3. Read `camera_config.sensor_configs[0].camera_params.intrinsic_params`. When `SensorConfig.camera_params` is omitted in the resource configuration, the runtime `Camera` wrapper falls back to `SensorInformation.factory_camera_params` returned by `intrinsic_proto.perception.v1.CameraService/DescribeCamera`.

### Camera settings, write-only commands, and SDK parameter behavior

- **Driver-specific setting conventions (GenICam vs. ROS drivers)**:
  - **GenICam cameras (`CameraIdentifier.genicam`, e.g., Basler, Photoneo)**: Use GenICam Standard Features Naming Convention (SFNC) PascalCase names and units. Exposure is configured via `name: "ExposureTime"` with `float_value` in **microseconds** (e.g., `50000.0` for 50 ms), auto-exposure via `name: "ExposureAuto"` (`enumeration_value: "Off"`), and gain via `name: "Gain"` (`float_value`).
  - **ROS driver cameras (`CameraIdentifier.ros`, e.g., Orbbec Gemini)**: Use lowercase driver-specific feature names and SI units. Exposure is configured via `name: "exposure"` with `float_value` in **seconds** (e.g., `0.001` for 1 ms).
- **Exclusive register access and acquisition mode transitions**:
  - GenICam and GigE Vision camera drivers (backed by `Aravis`) enforce exclusive write access to device registers over the GigE Vision Control Protocol (GVCP). While continuous streaming or single-frame acquisition is active, registers such as `AcquisitionMode` are write-protected on hardware.
  - Calling `intrinsic_proto.perception.v1.CameraService/CaptureStream` while single-frame acquisition is active fails with `PERMISSION_DENIED` or `ARV_DEVICE_ERROR_PROTOCOL_ERROR_WRITE_PROTECT`. Explicitly pause or stop prior acquisition sessions before mutating `AcquisitionMode` or initiating new streams.
- **Why `Camera.update_camera_setting` fails on write-only settings and how to update them directly**:
  - In `intrinsic.perception.client.v1.python.camera.cameras.Camera.update_camera_setting(name, value)`, the Python SDK wrapper performs a read-before-write round-trip: it calls `intrinsic_proto.perception.v1.CameraService/ReadCameraSetting(name)` first to inspect which `oneof value` field (`integer_value`, `float_value`, `bool_value`, `string_value`, `enumeration_value`, or `command_value`) is active before issuing `UpdateCameraSetting`.
  - Calling `Camera.update_camera_setting` on write-only settings (`CameraSettingAccess.Mode.WRITE`, such as hardware trigger or reset `ICommand` features where `ReadCameraSetting` is unsupported) fails during the initial read.
  - **API usage hint**: To execute a write-only command or update a write-only setting without reading it first, bypass `Camera.update_camera_setting` and call `CameraClient.update_camera_setting` directly with an explicitly constructed `CameraSetting` message:
    ```python
    from google.protobuf import empty_pb2
    from intrinsic.perception.proto.v1 import camera_settings_pb2

    camera_client.update_camera_setting(
        camera_settings_pb2.CameraSetting(name="TriggerSoftware", command_value=empty_pb2.Empty())
    )
    ```
- **Interdependent register access modes and one-shot auto-calculation timing**:
  - GenICam parameters exhibit dynamic access modes: when `ExposureAuto` or `GainAuto` is set to `Once` or `Continuous`, manual `ExposureTime` and `Gain` registers become read-only. Applying manual values while auto-mode is active fails.
  - Enabling software trigger mode (`TriggerMode = On`) fails on industrial cameras unless `AcquisitionFrameRateEnable` is disabled first. Manual exposure and gain values must be clamped to exact multiples of `FloatSettingProperties.increment`.
  - High-speed cameras complete a `Once` auto-calculation faster than an RPC round-trip. Await write confirmation of `Once` and immediately poll for the transition to `Off`, rather than polling to observe the transient `Once` state.
- **Thin-prism distortion coefficient order reversal (`s4, s3, s2, s1`) in `_camera_utils.extract_distortion_params`**:
  - In `intrinsic.perception.client.v1.python.camera._camera_utils.extract_distortion_params` (invoked by `Camera.distortion_params` and `CameraParams.distortion_params`), when thin-prism (`s1..s4`) or tilt (`tx, ty`) coefficients are present on `DistortionParams`, the helper returns `[k1, k2, p1, p2, k3, k4, k5, k6, s4, s3, s2, s1, ...]`, reversing the thin-prism coefficient order (`s4, s3, s2, s1`) relative to OpenCV's standard 12- and 14-parameter vector layout (`s1, s2, s3, s4`).
  - **API usage hint**: When passing 12- or 14-element distortion vectors to `cv2.undistort` or `cv2.projectPoints`, construct the array explicitly from the `DistortionParams` proto fields (`[dp.k1, dp.k2, dp.p1, dp.p2, dp.k3, dp.k4, dp.k5, dp.k6, dp.s1, dp.s2, dp.s3, dp.s4, dp.tx, dp.ty]`).
- **Channel receive size and KV store key isolation**:
  - Uncompressed RGB-D or high-resolution intensity `SensorImage` buffers exceed gRPC's default 4 MB receive limit. When calling `intrinsic_proto.perception.v1.CameraService/Capture` inline without `capture_result_location`, configure the gRPC channel with `options=[("grpc.max_receive_message_length", -1)]`.
  - When calling `ai.intrinsic.capture_images` across multiple robot waypoints, supply a unique `capture_result_location = StorageLocation(store="kv_store", key=f"waypoint_{i}_capture")` per waypoint to prevent overwriting the default key (`CanonicalString(camera_config.identifier())`).

### GigE vision `MTU` negotiation, packet formats, and network saturation

- **Network saturation across multi-camera workcells**:
  Streaming uncompressed `RGB8` images (3 bytes/pixel) across multiple high-resolution cameras saturates a 1 GbE network link (e.g. three 8 MP cameras at 5 FPS generate ~960 Mbit/s), causing dropped packets and `DEADLINE_EXCEEDED` timeouts.
- **Jumbo Frames and Bayer format**:
  1. Configure end-to-end Jumbo Frames (`9000` MTU / `GevSCPSPacketSize`) across the camera driver, network switch, and host interface. PoE industrial cameras often fail automatic MTU negotiation and fall back to 1400-byte packets.
  2. Switch `PixelFormat` from `RGB8` to `BayerRG8` to achieve a 3x reduction in network bandwidth.
  3. Ensure `Width` and `Height` are exact multiples of 8.
- **WebRTC transport renegotiation timeouts vs. capture errors**:
  Live camera previews streamed over WebRTC timeout after 10 seconds during network jitter. If renegotiation aborts and reconnects before backend encoder teardown completes, the retry fails with `video stream already exists for camera`. Distinguish transport renegotiation delays from physical sensor capture errors.

### Input-aware decision tree for camera acquisition and register mutation

- **`CaptureStream` write-protect error (`PERMISSION_DENIED`)**: Diagnostic check: check if asynchronous preview or single-frame capture is currently active. Targeted action: pause or stop active acquisition sessions before mutating `AcquisitionMode` or starting a stream.
- **`CameraService` RPC `Unimplemented` on port 17080**: Diagnostic check: check if channel includes `x-resource-instance-name` header with target camera name. Targeted action: attach `x-resource-instance-name: <camera_name>` via `ConnectionParams` and `HeaderAdderInterceptor`.
- **`Camera.update_camera_setting` fails on write-only commands (`Mode.WRITE`)**: Diagnostic check: check `ReadCameraSettingAccess` mode; confirm register is write-only (`TriggerSoftware`). Targeted action: call `CameraClient.update_camera_setting` directly with `CameraSetting(command_value=Empty())`.
- **Manual `ExposureTime` or `Gain` update rejected**: Diagnostic check: check if `ExposureAuto` or `GainAuto` is set to `Once` or `Continuous`. Targeted action: set `ExposureAuto = "Off"` before setting manual exposure; clamp values to setting increment.
- **Multi-camera frame drops or `DEADLINE_EXCEEDED` over GigE Vision**: Diagnostic check: inspect network MTU and bandwidth utilization across the 1 GbE link. Targeted action: enable 9000 MTU Jumbo Frames, switch `PixelFormat` to `BayerRG8`, and clamp image dimensions to multiples of 8.
- **WebRTC live preview reconnect collision**: Diagnostic check: check backend logs for active encoder teardown after connection timeout. Targeted action: wait 10 seconds for encoder teardown to complete before issuing reconnect.

---

## Camera calibration and behavior tree calibration skills

### 5-stage stateful calibration service workflow

Calibration follows a stateful 5-stage sequence on `intrinsic_proto.perception.v1.CalibrationService`:

1. **`intrinsic_proto.perception.v1.CalibrationService/Initialize`**:
   - Accepts `InitializeRequest` with `pattern_detection_config`, `repeated camera_resource_handles`, `optional arm_part` (`ObjectReference`), `optional calibration_object` (`ObjectReference`), `optional belief_world_id` (default `"world"`), and `optional edit_world_id` (default `"init_world"`).
   - Clears prior captures in server memory, deletes prior session keys under `calibration/sessions/**` from the cluster Key-Value store, and sets `skip_undistortion = true` on all sensor post-processing configurations so calibration operates on raw unrectified images.
2. **`intrinsic_proto.perception.v1.CalibrationService/CaptureData` (and `DeleteCapture`)**:
   - Captures frames from all initialized cameras at each robot pose, detects the calibration pattern (requiring at least 10 non-collinear ChArUco corners per capture; runtime PnP pose estimation requires only 6), evaluates image sharpness (`detect_blurred_image = true`) and redundancy (`detect_redundant_pattern = true`), saves JPEG-encoded annotated images with crosshairs to the KV store (`annotated_image_locations`), and returns a 4-digit string `capture_id` (e.g., `"0000"`).
   - Enforces a hard maximum of **100 stored captures** per session (`kMaxCalibrationDataPoints = 100`). Use `DeleteCapture` to remove outlier frames or `exclude_capture_ids` in `CalibrationRequest` to omit specific captures during solving.
3. **`intrinsic_proto.perception.v1.CalibrationService/Calibrate`**:
   - Computes intrinsic parameters and/or extrinsic transforms (`STATIONARY_CAMERA` or `MOVING_CAMERA` topologies; simultaneous `AUTO`/`NONLINEAR`/`SHAH`/`LI` solvers solve for both camera and target transforms, whereas classic AX = XB `TSAI`/`PARK`/`HORAUD`/`ANDREFF`/`DANIILIDIS` solvers solve only for `base_t_camera` or `flange_t_camera`).
   - Requires at least **3 pose pairs** (`INVALID_ARGUMENT: At least 3 pose pairs are required for calibration`).
   - For multi-camera extrinsic calibration (`calibrate_camera_to_camera = true`), requires at least **2 initialized cameras** and at least **3 captures** sharing >= 10 common marker points between camera pairs.
   - Immediately updates camera and calibration object transforms in the live belief world (`"world"`) via `intrinsic_proto.world.ObjectWorldService/UpdateTransform` for visual inspection, but **does not persist them to `"init_world"` or the asset registry**.
4. **`intrinsic_proto.perception.v1.CalibrationService/Validate`**:
   - Evaluates 2D RMS reprojection error (`rms_error_2d` in pixels) and 3D error (`mean_error_3d` in meters) against captured validation poses without re-running the optimization solver.
5. **`intrinsic_proto.perception.v1.CalibrationService/Save`**:
   - For single-sensor intrinsic results, updates the camera's `CameraConfig` asset via `intrinsic_proto.assets.AssetDeploymentService/UpdateResource`.
   - For extrinsic results (`base_t_camera`, `flange_t_camera`, or `cam_t_cam0`), writes the calibrated transforms to the initial world (`"init_world"`) in memory via `intrinsic_proto.world.ObjectWorldService/UpdateTransform`.

### The 5 behavior tree camera-to-robot calibration skills

In Behavior Tree workflows, camera-to-robot calibration is executed through five coordinated skills under `intrinsic_proto.skills`:

1. **`ai.intrinsic.sample_calibration_poses` (`SampleCalibrationPosesParams` -> `SampleCalibrationPosesResult`)**:
   Generates collision-free robot waypoints (`repeated intrinsic_proto.motion_planning.v1.GeometricConstraint`) using either `randomized_box_params` (sampling `num_samples` poses inside an axis-aligned box of `sample_box_halfsize` centered on `calibration_object` with random tilt and roll angles) or `pre_calibration_params` (small safe Cartesian offsets around the starting pose).
2. **`ai.intrinsic.initialize_calibration` (`InitializeCalibrationParams`)**:
   Calls `intrinsic_proto.perception.v1.CalibrationService/Initialize` after resolving `pattern_detection_config` and extracting the ICON position part from `robot`.
3. **`ai.intrinsic.collect_calibration_data` (`CollectCalibrationDataParams`)**:
   Queries ICON (`GetSinglePartStatus`) to synchronize the robot's initial joint configuration in `ObjectWorld`, visits each waypoint in `waypoints`, queries `ObjectWorld` for `base_t_flange`, invokes `intrinsic_proto.perception.v1.CalibrationService/CaptureData`, and returns the robot to its starting joint angles upon completion.
4. **`ai.intrinsic.calibrate_camera_to_robot` (`CalibrateCameraToRobotParams` -> `CalibrateCameraToRobotResult`)**:
   Calls `intrinsic_proto.perception.v1.CalibrationService/Calibrate` and validates RMS errors against `translation_root_mean_square_error_threshold` (in meters) and `rotation_root_mean_square_error_threshold` (in degrees). If either threshold is exceeded, the skill fails with `FAILED_PRECONDITION`.
5. **`ai.intrinsic.save_calibration_result` (`SaveCalibrationResultParams`)**:
   Commits the `CalibrationResult` to `"init_world"` via `intrinsic_proto.perception.v1.CalibrationService/Save`.

### Calibration data ceilings, motion filtering, and coordinate frames

- **100-frame ceiling per session**:
  `CalibrationService` enforces a strict ceiling of 100 stored captures (`kMaxCalibrationDataPoints = 100`). Once 100 captures accumulate, subsequent `CaptureData` calls fail with `FAILED_PRECONDITION`. Delete outlier frames using `DeleteCapture` or call `Initialize` to start a new session before collecting additional poses.
- **Belief world update vs. initial world gating**:
  Calling `CalibrationService/Calibrate` updates camera extrinsics in `"world"` immediately for visual inspection regardless of reprojection error. Automated pipelines must explicitly verify that 2D RMS error is < 1.0 px and 3D error is < 1.0 mm before calling `CalibrationService/Save` to persist transforms to `"init_world"`.
- **Motion planning transition filtering**:
  `collect_calibration_data` evaluates reachability between sequential waypoints. If the motion planner cannot find a collision-free trajectory between waypoint i and i+1, the skill silently skips waypoint i+1 without failing the overall execution, resulting in fewer collected data points than sampled.
- **Automatic look-at tracking behavior**:
  Setting random rotation around X/Y to `0` degrees in `sample_calibration_poses` does not lock the robot end-effector orientation. The skill automatically applies look-at tracking to keep the calibration target facing the camera across Cartesian positions; setting rotation to zero only removes additional random angular noise.

### `API` usage hints for calibration

- **Populating all 4 camera equipment slots (`camera_1..4`) and automatic deduplication**:
  Both `ai.intrinsic.initialize_calibration` and `ai.intrinsic.estimate_pose_multi_view` require all 4 camera equipment slots (`camera_1`, `camera_2`, `camera_3`, `camera_4`) to be populated when instantiated via the Python SDK. When calibrating fewer than 4 cameras (such as a single camera), pass the same camera resource handle to all 4 slots (`camera_1=cam, camera_2=cam, camera_3=cam, camera_4=cam`). `initialize_calibration` automatically deduplicates handles by resource name before calling `intrinsic_proto.perception.v1.CalibrationService/Initialize`.
- **ChArUco target validation when initializing from a `PoseEstimatorId`**:
  When `InitializeCalibrationParams.pose_estimator` (`intrinsic_proto.perception.v1.PoseEstimatorId`) is passed to `ai.intrinsic.initialize_calibration` instead of `pattern_detection_config`, the skill fetches the `PerceptionModel` from `intrinsic_proto.data.v1.DataAssets` and enforces `pose_estimation_config.targets(0).marker().has_charuco_pattern() == true`. Passing a non-ChArUco pose estimator fails with `FAILED_PRECONDITION: PoseEstimationConfig (v1) target is not a Charuco pattern`.
- **Strict meter unit convention on `CharucoPattern`**:
  Both `square_length` and `marker_length` in `intrinsic_proto.perception.v1.CharucoPattern` must be specified in **meters** (for example, `0.020` for a 20 mm square), never in millimeters, and must satisfy `0 < marker_length < square_length`. Specifying millimeter values scales computed camera-to-robot transforms by 1000x. Both `intrinsic_proto.perception.v1.CalibrationService` and runtime ChArUco pose estimation automatically apply an internal rigid offset (`corner_t_center`) so that `camera_t_target` aligns with the geometric center of the calibration object in `ObjectWorld` with Z pointing out of the board face.
- **Multi-sensor cameras reject intrinsic calibration**:
  If `DescribeCamera` reports more than one sensor stream (`describe_response.sensors().size() > 1`, such as RGB-D cameras), calling `intrinsic_proto.perception.v1.CalibrationService/Calibrate` with `calibrate_intrinsics = true` or `intrinsic_proto.perception.v1.CalibrationService/Save` with non-empty `intrinsic_calibration_results` fails with `FAILED_PRECONDITION: Intrinsic calibration of multi-sensor cameras is not supported`.
- **`base_t_flange` consistency validation in `CaptureData`**:
  If `arm_part` was provided during `intrinsic_proto.perception.v1.CalibrationService/Initialize`, `CaptureData` automatically queries `ObjectWorldService` for `base_t_flange`. If `CaptureDataRequest.base_t_flange` is also passed and deviates from the `ObjectWorld` transform by more than `1e-4`, `CaptureData` rejects the request with `INVALID_ARGUMENT`. Either omit `base_t_flange` from `CaptureDataRequest` when `arm_part` is initialized or ensure the digital twin joint state is updated prior to calling `CaptureData`.
- **Persisting calibration transforms to `world_updates.pbtxt`**:
  Because `intrinsic_proto.perception.v1.CalibrationService/Save` updates `"init_world"` in memory without modifying solution source files on disk, persisting calibrated camera transforms to a solution's `world_updates.pbtxt` requires constructing an `intrinsic_proto.world.ObjectWorldUpdates` message containing an `UpdateTransformRequest` (`node_a = camera_scene_object.parent.transform_node_reference`, `node_b = camera_scene_object.transform_node_reference`, `a_t_b = calibrated_pose`, `node_to_update = camera_scene_object.transform_node_reference`).

### Input-aware decision tree for camera calibration and coordinate validation

- **`Initialize` pattern failure (`FAILED_PRECONDITION`)**: Diagnostic check: check if `PoseEstimatorId` targets a non-ChArUco marker or `pattern_detection_config` is missing. Targeted action: supply `CharucoPattern` with meter units (`0 < marker_length < square_length`).
- **`CaptureData` insufficient corners (`CAPTURE_DATA_STATUS_INSUFFICIENT_MARKER_POINTS`)**: Diagnostic check: inspect raw and annotated image in KV store; check for glare, occlusion, or steep view angle. Targeted action: reposition robot waypoint to reduce tilt; ensure at least 10 non-collinear corners are visible.
- **`CaptureData` 100-frame ceiling (`FAILED_PRECONDITION`)**: Diagnostic check: count active stored captures in active calibration session. Targeted action: call `DeleteCapture` to prune outlier frames, or call `Initialize` to reset session.
- **`Calibrate` fails with `INVALID_ARGUMENT` (fewer than 3 pose pairs)**: Diagnostic check: verify number of successfully captured waypoints with valid marker detections. Targeted action: collect at least 3 non-degenerate waypoints before calling `Calibrate`.
- **`Calibrate` produces 1000x scaled camera transform**: Diagnostic check: check `square_length` and `marker_length` units on `CharucoPattern`. Targeted action: change units from millimeters (e.g. 20.0) to meters (e.g. 0.020).
- **`Calibrate` reprojection error exceeds threshold**: Diagnostic check: inspect per-capture reprojection errors in `Validate` response. Targeted action: delete highest-error capture IDs using `DeleteCapture`, re-run `Calibrate`, and verify RMS < 1.0 px before `Save`.

---

## Pose estimation and server-side vision post-processing

### Pose estimation service, model residency, and training pipeline

- `intrinsic_proto.perception.v1.PoseEstimationService/RunPoseEstimation`: Runs 6D pose estimation on an inline `CaptureResult` or KV-store `CaptureDataList` using a trained `PerceptionModel` DataAsset identified by `asset_id = Id(package="ai.intrinsic", name="<estimator_id>")`.
- `intrinsic_proto.perception.v1.PoseEstimationService/DeletePoseEstimation`: Evicts the cached CAD geometry and inference parameters for `asset_id` from server memory so subsequent `RunPoseEstimation` calls reload the updated DataAsset.
- `intrinsic_proto.perception.v1.TrainService` (`VerifyTrainingJob`, `CreateTrainingJob`, `GetTrainingJob`, `SaveTrainingJob`): Asynchronous training pipeline for creating one-shot pose estimation DataAssets.
- `intrinsic_proto.perception.v1.SymmetryService` (`CreateSymmetryDetectionJob`, `GetSymmetryDetectionJob`): Computes rotational and reflection symmetries for CAD models during training configuration.
- **Model residency and lifecycle coupling (`cleanup_model_on_destruction`)**:
  - Heavyweight vision models require ~60+ seconds to load neural network weights onto GPU memory. When short-lived clients construct a `ModelClientInterface` with default `cleanup_model_on_destruction = true`, exiting client scope unloads the model from the model server, causing a 25x–30x latency spike on the next call.
  - Set `cleanup_model_on_destruction = false` in short-lived scripts. To perform detection followed by refinement without loading two redundant GPU model instances, use a single multi-view pose estimation skill with `refinement_only: true`.

### Behavior tree pose estimation skills

In Behavior Tree workflows, 6D pose estimation is invoked through specialized skills under `intrinsic_proto.skills`:

- **`ai.intrinsic.estimate_pose` (`EstimatePoseParams` -> `EstimatePoseResult`)**:
  Accepts input `capture_data` (from `ai.intrinsic.capture_images`) and a `pose_estimator` (`intrinsic_proto.perception.v1.PoseEstimatorId`). Invokes `PoseEstimationService/RunPoseEstimation` and outputs candidate object poses (`repeated intrinsic_proto.Pose poses`) and detection confidence scores.
- **`ai.intrinsic.estimate_and_update_pose` (`EstimateAndUpdatePoseParams` -> `EstimateAndUpdatePoseResult`)**:
  Extends `estimate_pose` by automatically applying the highest-confidence estimated pose to the target scene object in `ObjectWorldService` (`UpdateTransformRequest`), reparenting or aligning the digital twin belief model to reality in a single execution step.
- **`ai.intrinsic.run_single_camera_multi_view_inference`**:
  Orchestrates multi-view 6D pose estimation using a single robot-mounted camera. Moves the robot to sequential inspection waypoints, acquires synchronized frames into distinct KV-store locations, and invokes multi-view pose estimation without requiring multiple physical cameras.
- **`ai.intrinsic.estimate_pose_multi_view`**:
  Runs multi-view 6DoF pose estimation across multiple synchronized static or moving cameras. Requires all 4 equipment slots (`camera_1..4`) populated in SDK instantiation. Bound input views to <= 4 distinct perspectives with wide angular baselines.

### Multi-view geometry, in-hand refinement, and coordinate math

- **Multi-view perspective bounds**:
  Multi-view 6DoF pose estimation (`estimate_pose_multi_view`) triangulates candidate 3D poses from ray intersections across views. Passing more than 4 views simultaneously triggers combinatorial tensor broadcasting (O(B^K)) and GPU memory exhaustion. Limit inputs to <= 4 distinct camera views with wide angular baselines.
- **In-hand object pose refinement coordinate frame math**:
  - Standard multi-view merging (`MergeResults` in `refine_pose`) computes relative camera extrinsics across captures as `cam_0_t_cam_i = cam_0_t_world * world_ts_camera[i]`, which assumes a moving camera and static world.
  - When an arm holds an in-hand workpiece and presents it to a stationary external camera across multiple configurations, `world_ts_camera[i]` is identical across captures, collapsing camera extrinsics to identity.
  - For in-hand refinement workflows, compute relative object transforms using the moving robot TCP frame at each capture timestamp (`root_t_object`), align camera object and sensor frames, and reparent the refined object to the robot TCP frame in `ObjectWorld`.

### Zenoh `KVStore` memory management and `REST` routing

- **Memory leaks from orphaned `capture_results/<UUID>` keys**:
  High-resolution camera captures passed by reference store multi-megabyte payloads in an in-memory Zenoh KV store under prefix `capture_results/<UUID>`. If perception skills fail to delete consumed keys, `zenoh-router` exhausts pod memory and restarts, terminating `/tf` transform streams (`Frame [root] does not exist`). Always delete consumed KV store keys inside `try/finally` blocks.
- **REST gateway route prefixing**:
  When inspecting the KV store via the HTTP REST gateway through ingress port 80, route paths through `/api/http-gateway` (e.g. `/api/http-gateway/api/kvstore/stores/<store>/keys/<key>`). Direct calls to `http-gateway.app-intrinsic-base:8080` are blocked by network security policies, and unprefixed paths return HTTP 404.
- **PCIe root complex bus contention**:
  On industrial PCs where discrete GPUs share a PCIe root complex with real-time network interfaces (`realtime_nic*` / `ICON`), heavy GPU DMA transfers during vision inference degrade PCIe link generation (Gen 4 dropping to Gen 1/3) and stall CPU interrupt handling, triggering real-time robot control disconnects. Inspect link status via `nvidia-smi -a | grep -A5 PCIe` and `lshw -class network -businfo`.

### `API` usage hints for pose estimation and vision post-processing

- **Dual discovery sources for `PoseEstimatorId` (`perception_model` resources vs. DataAssets)**:
  The Python SDK (`intrinsic.solutions.pose_estimation.PoseEstimators`) discovers available pose estimators by querying both:
  1. `intrinsic_proto.resources.ResourceRegistry/ListAllResourceInstances` with `resource_family_id = "perception_model"`.
  2. `intrinsic_proto.assets.v1.InstalledAssets/ListInstalledAssets` filtered by `AssetType.ASSET_TYPE_DATA` where the payload type URL is `type.googleapis.com/intrinsic_proto.perception.v1.PerceptionModel`.
  When saving trained DataAssets via `intrinsic_proto.perception.v1.TrainService/SaveTrainingJob`, set `package = "ai.intrinsic"` so `intrinsic_proto.perception.v1.PoseEstimationService/RunPoseEstimation` can resolve the installed asset by `intrinsic_proto.assets.Id`.
- **Dual-sensor and single-capture constraints for FoundationPose 6D pose estimation**:
  FoundationPose-based pose estimation (`intrinsic_proto.perception.v1.PoseEstimationService/RunPoseEstimation`) requires **both** of the following in a single `CaptureResult`:
  1. Exactly one intensity image (`PixelType.PIXEL_INTENSITY`, shape `(H, W, 3)` or broadcastable grayscale/RGBA) with valid `intrinsic_params`.
  2. Exactly one depth image (`PixelType.PIXEL_DEPTH`, 2D `float32` depth map in meters).
  When passing KV-store references via `RunPoseEstimationRequest.capture_data_list`, the list must contain **exactly one** `CaptureResult`, or `INVALID_ARGUMENT` is raised.
- **Mixed linear and angular unit conventions in `Target` and Fortran column-major `target_t_mesh`**:
  - `min_distance` and `max_distance` in `intrinsic_proto.perception.v1.PoseRange` are specified in **meters**.
  - All angular fields in `intrinsic_proto.perception.v1.LatitudeLongitudeViewSpace` (`min_longitude`, `max_longitude`, `min_latitude`, `max_latitude`, `min_camera_roll`, `max_camera_roll`, and `reference_rotation_x/y/z`) are specified in **degrees**, whereas symmetry angles in `intrinsic_proto.perception.v1.Symmetry` (`AxisAngle.angle`) are specified in **radians**.
  - `reference_rotation_x/y/z` in `LatitudeLongitudeViewSpace` orients the CAD model frame during synthetic template generation (controlling which faces of the object are sampled during training), but does **not** modify the coordinate frame of the 6D pose returned by `intrinsic_proto.perception.v1.PoseEstimationService/RunPoseEstimation` at inference time.
  - When `intrinsic_proto.perception.v1.TrainService/CreateTrainingJob` extracts a CAD mesh from `Target.scene_object`, it ignores the scene object's internal link transform (`parent_t_this` / `ref_t_shape`), defaulting `target_t_mesh` to identity unless `Target.mesh.target_t_mesh` is explicitly set. When populating `Target.mesh.target_t_mesh`, the 3x3 rotation matrix (`linear.values`) must be serialized in **column-major (Fortran) order (`order="F"`)**, not row-major order.
- **Column-major field ordering in `intrinsic_proto.perception.v1.Dimensions`**:
  Field `1` is `cols` (image width along X) and field `2` is `rows` (image height along Y). When converting between `Dimensions` and NumPy arrays (`(rows, cols, channels)`), map `shape[1] -> cols` and `shape[0] -> rows`.
- **Lightweight vector precision distinction (`Vector3f` vs. `Vector3`)**:
  `intrinsic_proto.perception.Vector2f`, `Vector3f`, and `Vector4f` belong to package `intrinsic_proto.perception` and use 32-bit `float` fields (`x`, `y`, `z`, `w`) for 2D/3D pattern detection coordinates, whereas `intrinsic_proto.Vector3` (used in `Symmetry` and math APIs) uses 64-bit `double` fields.
- **Server-side post-processing execution order (`SensorImagePostProcessing`)**:
  When `CaptureRequest.post_processing_by_sensor_id` is populated, `intrinsic_proto.perception.v1.CameraService/Capture` executes operations strictly in the order `Raw Image -> Undistortion -> Cropping -> Resizing`.
  When cropping or resizing is applied, the camera service automatically updates `SensorImage.camera_params.intrinsic_params` (shifting principal point `(cx, cy)` for crops and scaling `(fx, fy, cx, cy)` for resizes) so downstream consumers receive calibrated intrinsics matching the modified pixel buffer. Server-side undistortion is only supported for 2D intensity and depth images (not `PIXEL_POINT` or `PIXEL_NORMAL`).
- **Mandatory cache eviction when hot-patching pose estimator score thresholds**:
  When updating `min_score_threshold` or `confidence_threshold` inside an installed `PerceptionModel` DataAsset (`UpdatePolicy.UPDATE_POLICY_UPDATE_COMPATIBLE`), immediately call `intrinsic_proto.perception.v1.PoseEstimationService/DeletePoseEstimation(asset_id)` to evict the in-memory parameter cache.

### Multi-`RPC` sequence for one-shot pose estimator training and score threshold hot-patching

1. **Resolve scene object and camera intrinsics**: Retrieve part `SceneObject` model from `InstalledAssets/GetInstalledAsset` (`ASSET_VIEW_TYPE_FULL`). Query `ResourceRegistry/GetResourceInstance` for reference camera to read `sensor_configs[0].camera_params.intrinsic_params`.
2. **Compute or query part symmetries**: Call `SymmetryService/CreateSymmetryDetectionJob` with `SceneObject` and poll `GetSymmetryDetectionJob` until `done == true` to unpack `SymmetryDetectionResult.symmetry` (or specify `AxisAngle` in radians).
3. **Validate and launch training job**: Assemble `CreateTrainingJobRequest` with `POSE_ESTIMATOR_TYPE_ONE_SHOT`, target `PoseRange` (degrees/meters), symmetry in radians, and column-major `linear.values`. Call `TrainService/VerifyTrainingJob`, then `CreateTrainingJob`, and poll `GetTrainingJob` until `done == true`.
4. **Save trained estimator into a `DataAsset`**: Call `TrainService/SaveTrainingJob` with target `asset_id = Id(package="ai.intrinsic", name="<part>_ospe")`.
5. **Hot-patch score thresholds without retraining**: Call `InstalledAssets/GetInstalledAsset` (`ASSET_VIEW_TYPE_FULL`), unpack `PerceptionModel`, update `inference_params` (`min_score_threshold` or `confidence_threshold`), re-pack and call `InstalledAssets/CreateInstalledAsset` (`UPDATE_POLICY_UPDATE_COMPATIBLE`), then call `PoseEstimationService/DeletePoseEstimation(asset_id)` to evict cached parameters.

### Input-aware decision tree for pose estimation and vision memory management

- **`RunPoseEstimation` fails with `INVALID_ARGUMENT` on sensor image counts**: Inspect `sensor_images` map inside `CaptureResult`. Ensure capture contains exactly 1 intensity image and 1 depth image in meters; ensure `CaptureDataList` has length 1.
- **`RunPoseEstimation` exhibits unexpected 60+ second latency spike on every call**: Check model server logs for model unload/load cycles on client exit. Set `cleanup_model_on_destruction = false` in `ModelClientInterface`; use `refinement_only: true`.
- **In-hand workpiece multi-view refinement collapses camera poses to identity**: Check if `world_ts_camera` is identical across captures while arm moves object. Compute relative object transforms using moving robot `TCP` frame (`root_t_object`) across timestamps.
- **`zenoh-router` pod `OOM`s or restarts, causing `/tf` transform stream loss**: Query `KV` store keys under `capture_results/` via `/api/http-gateway/api/kvstore/stores/...`. Wrap perception capture consumers in `try`/`finally` blocks to delete consumed keys.
- **Updated confidence or score thresholds in `DataAsset` have no effect**: Check if `PoseEstimationService` is caching previous parameters in memory. Invoke `PoseEstimationService/DeletePoseEstimation(asset_id)` to evict cached parameters.

---

## Hardware `GPIO` service (`GpioService` / `GPIOService`)

The hardware GPIO layer exposes digital and analog I/O across OPC-UA modules, ICON real-time ADIO parts, and Gazebo simulation plugins via `intrinsic_proto.gpio.v1.GPIOService` (and legacy alias `intrinsic_proto.hardware.gpio.GpioService`):

- `intrinsic_proto.gpio.v1.GPIOService/GetSignalDescriptions`: Introspects registered signals (`can_read`, `can_write`, `SignalType`, `signal_name`, `alternate_signal_names`, and `pubsub_topic_name`).
- `intrinsic_proto.gpio.v1.GPIOService/ReadSignals`: Stateless point-in-time read of signal names returning `SignalValueSet`.
- `intrinsic_proto.gpio.v1.GPIOService/WaitForValue`: Blocks server-side until `all_of` or `any_of` signal conditions match. Always attach an explicit gRPC deadline.
- `intrinsic_proto.gpio.v1.GPIOService/OpenWriteSession`: Bidirectional streaming RPC required for all signal writes. Claims exclusive ownership of output signals for the duration of the active gRPC stream.
- **Envoy ingress routing for GPIO service instances**:
  Deployed GPIO service instances (e.g. `opcua_gpio`, `icon_gpio`) are Intrinsic asset instances. When connecting through `localhost:17080`, clients must attach the gRPC metadata header `x-resource-instance-name: <gpio_instance_name>`.

### Exclusive write session lifecycle and actuate-and-verify sequence

1. **Stream initialization and claim handshake**: Open `intrinsic_proto.gpio.v1.GPIOService/OpenWriteSession` stream; send initial `OpenWriteSessionRequest` with **only** `initial_session_data.signal_names`. Verify `response.status.code == 0` (`OK`) before sending writes (server returns `FAILED_PRECONDITION` or `PERMISSION_DENIED` if already claimed).
2. **Synchronous write loop**: Send `OpenWriteSessionRequest` messages with `write_signals.signal_values` populated and read the response after every write. If `response.status.code == 10` (`ABORTED`), close stream and establish a new session.
3. **Actuate-and-verify equipment control flow**: Keep `OpenWriteSession` stream active, issue write requests, and call `intrinsic_proto.gpio.v1.GPIOService/WaitForValue` with a gRPC deadline. Branch on status: execute `on_success` write on `OK`, or `on_failure` rollback on `DEADLINE_EXCEEDED`.
4. **Clean session teardown**: Close client write stream (`CloseSend()`), drain responses until EOF, and call `Finish()` to release signal locks.

### `OPC-UA` parent node locking, signal typing, and backend tolerances

- **OPC-UA array indexing, whole-node locking, and certificate configuration**:
  - Address array elements using dot-index notation (`<node_name>.<index>`). Providing an index for a scalar node or omitting an index for an array node returns `INVALID_ARGUMENT`.
  - In `OpcuaGPIOService`, write session claims are tracked on the parent OPC-UA node rather than per array index: claiming `my_array.0` locks the entire array node against other concurrent sessions (`FAILED_PRECONDITION`) while allowing the claiming session to write to any index of `my_array`.
  - When configuring `intrinsic_proto.gpio.OpcuaGpioServiceConfig` with TLS certificates (`cert_file`, `private_key_file`), `application_uri` must match the `urn:...` identifier in the certificate's X509v3 Subject Alternative Name extension (including `urn:`, omitting `URI:`). The `trusted_certificate_filepaths` field is ignored by the OPC-UA client.
- **Write-only signal types**:
  Write-only OPC-UA signals (`can_read == false`, `can_write == true`) report `type = SIGNAL_TYPE_UNKNOWN` in `GetSignalDescriptions`; calling `ReadSignals` on a write-only signal returns `PERMISSION_DENIED`.
- **Backend differences in `WaitForValueResponse`**:
  - **OPC-UA**: Populates `response.values` with observed values and sets `response.event_time`.
  - **Gazebo simulation**: Sets `response.values` to the requested condition values rather than measured values.
  - **ICON ADIO**: Leaves `response.values` completely empty (`SignalValueSet{}`) and `response.event_time` unset (`0`). Issue a follow-up `ReadSignals` call if signal values are needed after `WaitForValue` completes on ICON ADIO parts.
- **Floating-point comparison tolerances (`ReadSignals` vs. `WaitForValue`)**:
  Client-side matching via `SignalValueSet` equality (`operator==`) performs exact binary floating-point comparison (`==`). Server-side `WaitForValue` performs approximate comparison using internal tolerances (`1e-3` on OPC-UA and simulation services, `1e-6` on ICON ADIO services). On ICON ADIO services, `WaitForValue` accepts only `bool_value` and `double_value` expectations; other types return `INVALID_ARGUMENT`.
- **PubSub signal subscriptions (`SignalDescription.pubsub_topic_name`)**:
  On `OpcuaGPIOService`, `PublishSignals` only publishes scalar signals (`HoldsScalar() == true`). Array-backed OPC-UA nodes still report a non-empty `pubsub_topic_name` in `GetSignalDescriptions`, so `SubscribeToSignal` succeeds without error, but no messages are published to that topic. On `simulation::GPIOService`, `pubsub_topic_name` is always empty (`""`), causing `SubscribeToSignal` to return `NOT_FOUND`.
- **Empty ICON ADIO configuration (`EmptyIconGPIOService`)**:
  If `IconGpioServiceConfig.parts` is empty, `GetSignalDescriptions` returns `OK` with an empty list, but any non-empty `ReadSignals`, `WaitForValue`, or `OpenWriteSession` request returns `FAILED_PRECONDITION` (`"No ADIO part has been configured. Check the application configuration."`).
- **Trajectory bridge vs. ADIO part routing**:
  Hardware module trajectory bridges route joint trajectory and velocity commands. Toggling GPIO/ADIO signals through trajectory bridges without explicit ADIO streaming interfaces mapped in shared memory drops commands silently. Verify and command physical or simulated I/O through the dedicated `adio` part exposed by `intrinsic_proto.icon.v1.IconApi` (`inctl icon list-parts --instance_name=icon --address=localhost:17080`).

### Input-aware decision tree for hardware `GPIO` write sessions

- **`OpenWriteSession` stream terminated with `FAILED_PRECONDITION` on connect**: Check if requested signal (or sibling index on same parent node) is held by another session. Close competing session, or verify first request contains only `initial_session_data`.
- **`GPIO` service `RPC` fails with `rpc error: code = Unimplemented` on `localhost:17080`**: Check if channel includes `x-resource-instance-name` header with target `GPIO` instance name. Attach `x-resource-instance-name: <gpio_instance_name>` via `ConnectionParams`.
- **`WaitForValue` returns `OK` on `ICON` `ADIO`, but `response.values` is empty**: Check service backend type (`ICON` `ADIO` leaves values empty and `event_time=0`). Issue an explicit `ReadSignals` call immediately following `WaitForValue`.
- **Toggling digital `I/O` through trajectory bridge drops signals silently**: Inspect registered parts via `inctl icon list-parts --instance_name=icon --address=localhost:17080`. Route `I/O` through the dedicated `adio` part or `GPIOService`, not joint trajectory bridges.
- **Subscribing to array signal pubsub topic receives no messages**: Check if signal is array-backed on `OpcuaGPIOService`. Array signals do not publish to `PubSub`; use point-in-time `ReadSignals` or polling instead.

---

## `EOAT` and generic gripper services (`EoatService` / `GenericGripperService`)

The gripper layer provides specialized DIO services (`intrinsic_proto.eoat.PinchGripper`, `intrinsic_proto.eoat.SuctionGripper`, `intrinsic_proto.eoat.EoatService`) and adaptive/generic gripper services (`intrinsic_proto.gripper.PinchGripperServer`, `intrinsic_proto.gripper.GenericGripper`, `intrinsic_proto.hardware.gripper.GenericGripperService`):

| Use case | Recommended gRPC service / skill | Why |
| :--- | :--- | :--- |
| Discrete pneumatic/electric pinch gripper via GPIO/OPC-UA | `intrinsic_proto.eoat.PinchGripper` (via skill `ai.intrinsic.control_pinch_gripper`) | Standard skills bind to `grpc://intrinsic_proto.eoat.PinchGripper` and automatically synchronize digital twin joint positions after actuation. |
| Discrete pneumatic/electric pinch gripper via realtime ICON part | `intrinsic_proto.icon.v1.IconApi` + `Icon2GripperPart` (via skill `ai.intrinsic.simple_gripper`) | Controls ICON-managed gripper parts directly through realtime action descriptors (`SimpleGripperActionInfo`). |
| Vacuum suction gripper | `intrinsic_proto.eoat.SuctionGripper` (via skills `ai.intrinsic.control_suction_gripper` and `ai.intrinsic.suction_gripping_indicated`) | Provides dedicated RPCs for vacuum `Grasp`, `Release`, active air `BlowOff`, and vacuum sensor verification (`GrippingIndicated`). |
| Adaptive/servo pinch gripper (Robotiq, WSG32, Zimmer) | `intrinsic_proto.gripper.PinchGripperServer` (via skill `ai.intrinsic.control_adaptive_pinch_gripper`) | Supports continuous position, velocity, and force/effort control with closed-loop status feedback (`position_reached`, `object_detected`). |
| Hardware-agnostic gripper abstraction | `intrinsic_proto.gripper.GenericGripper` (`intrinsic_proto.hardware.gripper.GenericGripperService`) | Registered on the same gRPC port across both DIO (`PinchGripper`/`SuctionGripper`) and adaptive (`PinchGripperServer`) gripper servers. |

### `API` usage hints for grippers

1. **Envoy ingress routing for gripper services**:
   Grippers are deployed asset instances. When communicating through `localhost:17080`, always pass `x-resource-instance-name: <gripper_instance_name>` (e.g. `pinch_gripper`).
2. **Single-RPC fault recovery (`intrinsic_proto.services.v1.ServiceState/Enable` vs. `intrinsic_proto.resources.ResourceHealth/Enable`)**:
   - Every gripper server registers `intrinsic_proto.resources.ResourceHealth`, `intrinsic_proto.services.v1.ServiceState`, the specialized gripper service, and `intrinsic_proto.gripper.GenericGripper` on the same port.
   - Calling `intrinsic_proto.services.v1.ServiceState/Enable` (invoked by `ai.intrinsic.enable_gripper`) automatically checks whether `state_code == STATE_CODE_ERROR`; if faulted, it executes `ClearFaults()` followed immediately by `Enable()` in a single RPC. In contrast, calling legacy `intrinsic_proto.resources.ResourceHealth/Enable` while faulted fails unless `intrinsic_proto.resources.ResourceHealth/ClearFaults` was called first.
3. **Asymmetric `RequireIsEnabled()` enforcement on status reads**:
   - On DIO grippers (`PinchGripper`, `SuctionGripper`), calling `GrippingIndicated` while disabled or faulted returns `FAILED_PRECONDITION`.
   - On adaptive pinch grippers (`PinchGripperServer`), calling `GetPinchGripperStatus` or `GenericGripper/GrippingIndicated` does not check `RequireIsEnabled()` and succeeds even when the service state machine is disabled.
4. **Digital twin joint synchronization and `is_default_closed` kinematics**:
   - Calling `intrinsic_proto.eoat.PinchGripper/Grasp` or `intrinsic_proto.eoat.PinchGripper/Release` directly via gRPC actuates hardware/simulated I/O signals but **does not update `ObjectWorld`**.
   - `ai.intrinsic.control_pinch_gripper` updates the gripper's joint positions in the belief world (`intrinsic_proto.world.ObjectWorldService/UpdateObjectJoints`) to `joint_application_limits.min_position` on `grasp` and `joint_application_limits.max_position` on `release`.
   - Discrete pinch grippers whose URDF/SDF lower joint limit represents the closed state (such as Schunk pneumatic grippers) must set `is_default_closed: true` in `intrinsic_proto.eoat.PinchGripperConfig` so physics simulation (`ActuatedGripperPlugin`) and the belief world agree.
   - For adaptive pinch grippers (`PinchGripperServer`), the reported `position` is the total opening distance between both fingers in meters (`0.0` when closed). Update individual symmetric finger joints in `ObjectWorld` using `finger_joint_position = joint_limit_upper - 0.5 * reported_position`.
5. **Behavioral differences of `intrinsic_proto.gripper.GenericGripper` across backends**:
   - **DIO backends (`PinchGripperConfig` / `SuctionGripperConfig`)**: `intrinsic_proto.gripper.GenericGripper/Command` returns `UNIMPLEMENTED`. `intrinsic_proto.gripper.GenericGripper/Release(enable_blowoff=True)` on a suction gripper automatically chains `Release()` followed by `BlowOff(turn_on=True)`. `intrinsic_proto.gripper.GenericGripper/GrippingIndicated` performs a live hardware read of the configured GPIO inputs.
   - **Adaptive pinch backends (`PinchGripperServer`)**: `intrinsic_proto.gripper.GenericGripper/Grasp` commands `position_percentage: 20.0` (not `0.0%`) and `intrinsic_proto.gripper.GenericGripper/Release` commands `position_percentage: 100.0`.
   - **Cached `GrippingIndicated` status on adaptive grippers**: On adaptive pinch grippers, `intrinsic_proto.gripper.GenericGripper/GrippingIndicated` **does not query live hardware**; it returns `last_status_.object_detected()` cached from the most recent `GenericGripper` RPC on that server instance. If the gripper was actuated via `intrinsic_proto.gripper.PinchGripperServer/CommandPinchGripper` (or `ai.intrinsic.control_adaptive_pinch_gripper`), `intrinsic_proto.gripper.GenericGripper/GrippingIndicated` returns `false`. Query `intrinsic_proto.gripper.PinchGripperServer/GetPinchGripperStatus` directly and evaluate `object_detected || position_reached`.
6. **Python SDK `GripperClient.blow_off()` default parameter behavior**:
   - In `intrinsic.hardware.gripper.eoat.gripper_client.GripperClient`, calling `.blow_off()` sends a default `BlowOffRequest()` with `turn_on = False`, which deactivates blow-off rather than triggering an air pulse. To trigger an active blow-off pulse, call the gRPC stub directly with `BlowOffRequest(turn_on=True)` followed by `BlowOffRequest(turn_on=False)`, or invoke `intrinsic_proto.gripper.GenericGripper/Release(enable_blowoff=True)`.
7. **Heavy payload transitions and force-limit faults**:
   - When grasping or releasing heavy parts on collaborative arms triggers a collaborative force limit fault immediately after actuation, coordinate gripper commands with `change_payload_index`.
   - Update `change_payload_index` while the robot is stationary, or temporarily adjust controller safety settings during the physical coupling/release transition.
8. **Universal Robots F/T taring log interpretation**:
   - On `ur_module`, cyclic reader decrementation logs `Failed to tare within the specified number of cycles` on every request because the hardware driver lacks a taring completion signal, even when `ReadStatus` reports `taring_done = true`.
   - Treat this entry as benign unless accompanied by `FailedPreconditionError("Failed to send 'retare' command to ur_robot.")`. If `MoveToContact` times out after a protective stop, inspect motion settling and protective stop clearance rather than taring logs.

### Multi-`RPC` gripper sequences

- **Discrete pinch gripper pick-and-place sequence**: `intrinsic_proto.services.v1.ServiceState/Enable` -> `intrinsic_proto.eoat.PinchGripper/Release` (via `ai.intrinsic.control_pinch_gripper`) -> update `ObjectWorld` to `max_position` -> move to grasp -> `intrinsic_proto.eoat.PinchGripper/Grasp` -> update `ObjectWorld` to `min_position` -> verify `intrinsic_proto.eoat.PinchGripper/GrippingIndicated`.
- **Suction gripper pick, verify, and release with blow-off**: `intrinsic_proto.eoat.SuctionGripper/Grasp` -> verify `intrinsic_proto.eoat.SuctionGripper/GrippingIndicated` -> move to place -> `intrinsic_proto.gripper.GenericGripper/Release(enable_blowoff=True)` -> `intrinsic_proto.eoat.SuctionGripper/BlowOff(turn_on=False)`.
- **Adaptive pinch gripper session and command loop**: `intrinsic_proto.gripper.PinchGripperServer/CreatePinchGripper` -> `intrinsic_proto.gripper.PinchGripperServer/CommandPinchGripper` -> poll `GetPinchGripperStatus` until `position_reached || object_detected` -> update `ObjectWorld` finger joints to `joint_limit_upper - 0.5 * status.position`.

### Input-aware decision tree for gripper fault recovery and actuation

- **`Grasp` or `Release` fails with `FAILED_PRECONDITION` due to faulted state machine**: Check `ServiceState.state_code` via `inctl service state list --address=localhost:17080`. Call `ServiceState/Enable` (clears faults and enables in a single `RPC`).
- **Gripper `RPC` fails with `rpc error: code = Unimplemented` on `localhost:17080`**: Check if channel includes `x-resource-instance-name` header with target gripper name. Attach `x-resource-instance-name: <gripper_name>` via `ConnectionParams`.
- **`GenericGripper/GrippingIndicated` returns `false` on adaptive pinch gripper**: Check if actuation occurred via `PinchGripperServer/CommandPinchGripper`. Call `PinchGripperServer/GetPinchGripperStatus` directly; evaluate `object_detected || position_reached`.
- **Discrete pinch gripper simulation behaves inverted relative to belief world**: Check `URDF`/`SDF` lower joint limit convention in `model.sdf`. Set `is_default_closed: true` in `PinchGripperConfig` if lower limit represents closed.
- **Heavy payload grasp triggers collaborative force limit fault immediately**: Check timing between gripper grasp completion and `change_payload_index`. Update `change_payload_index` while stationary before or during grasp transition.
- **`GripperClient.blow_off()` does not pulse air**: Check `turn_on` parameter in `BlowOffRequest` (defaults to `False`). Pass `turn_on=True` to trigger air pulse, followed by `turn_on=False` to deactivate.
