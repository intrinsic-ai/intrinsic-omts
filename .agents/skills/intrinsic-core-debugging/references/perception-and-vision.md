# Debugging cameras, perception pipelines, calibration, and `KV` store memory

## 1. Camera drivers, `GenICam`/`GigE Vision` state machines, and streaming

- **Exclusive register access and acquisition mode transitions**:
  - `GenICam` and `GigE Vision` camera drivers (`Aravis`) enforce exclusive write access to device registers over the `GigE Vision` Control Protocol (`GVCP`). While continuous streaming or asynchronous single-frame capture is active, device registers such as `AcquisitionMode` become write-protected on hardware.
  - Guardrail: Do not mutate write-protected registers such as `AcquisitionMode` while acquisition is active; explicitly pause or stop active acquisition sessions before modifying acquisition modes or initiating new streams.
  - Decision tree:
    - *Precondition*: `intrinsic_proto.perception.v1.CameraService/CaptureStream` fails with `PERMISSION_DENIED` or `ARV_DEVICE_ERROR_PROTOCOL_ERROR_WRITE_PROTECT` on `AcquisitionMode`.
    - *Diagnostic check*: Inspect whether an asynchronous single-frame capture session (preview or one-shot skill) is currently active.
    - *Targeted action*: Call `PauseStream()` or `StopStream()` before mutating `AcquisitionMode` or starting a new stream.
- **Read-before-write verification on write-only registers**:
  - In the Python SDK, calling `Camera.update_camera_setting(name, value)` invokes `read_camera_setting(name)` first to inspect register access mode. On write-only registers (`Mode.WRITE`, such as software trigger or reset registers), this pre-read fails.
  - Decision tree:
    - *Precondition*: `Camera.update_camera_setting` raises an exception during pre-read on a write-only register (`Mode.WRITE`).
    - *Diagnostic check*: Inspect register access mode using `CameraClient.read_camera_setting_access(name)`.
    - *Targeted action*: Invoke `CameraClient.update_camera_setting` directly to execute the write without a pre-read.
- **Interdependent register access modes and auto-calculation timing**:
  - Setting `ExposureAuto` or `GainAuto` to `Once` or `Continuous` makes manual `ExposureTime` and `Gain` registers read-only on hardware. Disable `AcquisitionFrameRateEnable` before setting `TriggerMode = On`, and clamp manual exposure and gain values to exact multiples of the register increment.
  - High-speed cameras complete a `Once` auto-exposure or gain calculation and transition back to `Off` faster than a single read RPC round-trip. Await write confirmation of the `Once` setting and immediately poll for the transition to `Off`, rather than polling to observe the transient `Once` state.
- **Allied Vision Alvium firmware version ceiling for frame rate**:
  - Allied Vision Alvium cameras enforce a firmware version ceiling of `13.1.794391F9` for frame rate mutations; newer firmware versions reject frame rate updates with driver-level preconditions.
  - Decision tree:
    - *Precondition*: Setting `AcquisitionFrameRate` on an Allied Vision Alvium camera fails with `FAILED_PRECONDITION: The Allied Vision Alvium camera in use has a firmware bug for versions greater than 13.1.794391F9`.
    - *Diagnostic check*: Inspect camera device information and firmware version string.
    - *Targeted action*: Maintain camera firmware <= 13.1.794391F9 or adjust `ExposureTime` to regulate acquisition frame intervals.
- **Orbbec Gemini 3D camera network subnet alignment and firmware requirements**:
  - Orbbec Gemini 3D cameras default to IP `192.168.0.10/24` on TCP port 8090, which mismatches typical `192.168.1.0/24` IPC subnets. Furthermore, GigE Vision Control Channel Privilege (CCP) requires firmware >= 1.6.07, and multi-IP interface binding requires driver >= 0.0.9.
  - Decision tree:
    - *Precondition*: Orbbec camera driver starts but fails to stream frames or logs `VendorTCPClient: Connect to server failed! addr=192.168.0.10, port=8090`.
    - *Diagnostic check*: Inspect host network interface subnet against `192.168.0.10`, verify camera firmware version is >= 1.6.07, and confirm driver version is >= 0.0.9.
    - *Targeted action*: Assign a static IP on the IPC subnet (`192.168.1.x`), update firmware to >= 1.6.07 for CCP support, and verify single-IP binding.
- **Image dimension configuration limits**:
  - Setting camera `Width` or `Height` to `0` locks out subsequent configuration updates in backend drivers. Always configure non-zero dimensions that are exact multiples of 8.
- **`GigE Vision` `MTU` negotiation and multi-camera network saturation**:
  - Streaming uncompressed `RGB8` across multiple high-resolution cameras on a 1 GbE switch uplink causes packet drops and `DEADLINE_EXCEEDED` timeouts. Certain PoE cameras fail automatic MTU negotiation and fall back to 1400-byte packets.
  - Decision tree:
    - *Precondition*: High-resolution camera streams experience packet drops or `DEADLINE_EXCEEDED` timeouts over a 1 GbE uplink.
    - *Diagnostic check*: Inspect camera packet size (`GevSCPSPacketSize`), switch port MTU, and pixel format.
    - *Targeted action*: (1) Verify end-to-end Jumbo Frames (`9000` MTU / `GevSCPSPacketSize`) across camera configuration, switch ports, and host network interface; (2) Switch `PixelFormat` from `RGB8` to `BayerRG8` (reducing bandwidth by 3x); (3) Ensure `Width` and `Height` are exact multiples of 8; (4) Lower `AcquisitionFrameRate` or adjust exposure time.
- **Distortion coefficient ordering in Python utilities**:
  - In `intrinsic.perception.client.v1.python.camera._camera_utils.extract_distortion_params`, thin-prism distortion coefficients are returned in reverse order (`[k1, k2, p1, p2, k3, k4, k5, k6, s4, s3, s2, s1]`) relative to standard OpenCV convention (`s1, s2, s3, s4`).
  - Guardrail: Do not pass the raw thin-prism slice directly to OpenCV-based undistortion functions; reverse the thin-prism slice (`s4, s3, s2, s1` to `s1, s2, s3, s4`) before constructing OpenCV distortion vectors.

## 2. Perception pipelines, `6DoF` pose estimation, and in-hand refinement

- **Model residency and `GPU` reload latency spikes**:
  - Heavyweight ML perception models require 60+ seconds to load weights onto the GPU. When short-lived clients construct `ModelClientInterface` with `cleanup_model_on_destruction = true` (default), exiting the client scope unloads the model and causes a 25x–30x latency spike on subsequent inference calls.
  - Decision tree:
    - *Precondition*: First-run pose estimation or interleaved inference requests incur a 65s–80s latency spike.
    - *Diagnostic check*: Check perception and model server logs for model unload and reload transitions.
    - *Targeted action*: Initialize `ModelClientInterface` with `cleanup_model_on_destruction = false` on short-lived evaluation clients.
- **Model artifact download timeouts and cooperative cancellation**:
  - When model artifact retrieval exceeds 4 minutes on slow networks, `RunPoseEstimation` aborts with `StatusCode.DEADLINE_EXCEEDED` under its 5-minute timeout. Ensure `RunContext` cooperative cancellation is propagated through template matching and refinement routines so aborted operations terminate promptly rather than running as orphaned GPU compute jobs.
- **Dual-model instance prevention in detection and refinement**:
  - Referencing the same model asset across separate detection and refinement skills (`estimate_pose_multi_view` and `refine_pose`) causes the model server to instantiate two separate GPU model instances, doubling GPU memory usage and risking out-of-memory aborts.
  - Decision tree:
    - *Precondition*: Model server memory doubles or encounters GPU out-of-memory when chaining detection and refinement.
    - *Diagnostic check*: Check whether separate skill instances load redundant copies of the same model asset.
    - *Targeted action*: Use a single multi-view pose estimation skill configured with `refinement_only: true` (such as `estimate_sheet_parts` with `refinement_only: true`) to execute refinement on the already-loaded model instance.
- **Multi-view camera perspective conditioning and ray bounds**:
  - Multi-view 6DoF pose estimation (`estimate_pose_multi_view`) triangulates candidate poses across camera rays. When candidate views are nearly collinear or parallel, the ray midpoint system matrix becomes singular. Passing more than 4 camera views simultaneously causes combinatorial tensor broadcasting (O(B^K)) and memory exhaustion.
  - Guardrail: Do not pass more than 4 camera views simultaneously or nearly collinear views to `estimate_pose_multi_view`; bound candidate inputs to 2–4 views with distinct angular baselines to prevent singular ray midpoints and tensor memory exhaustion.
- **In-hand object pose refinement coordinate frame assumptions**:
  - Standard multi-view merging (`MergeResults` in `refine_pose`) computes relative extrinsics as `cam_0_t_cam_i = cam_0_t_world * world_ts_camera[i]`, assuming a moving camera in a static world. When a robot arm presents an in-hand workpiece to a stationary external camera across multiple poses, `world_ts_camera[i]` is constant across captures, collapsing relative camera extrinsics to identity.
  - Decision tree:
    - *Precondition*: Multi-view in-hand pose refinement yields inaccurate or collapsed object poses across robot presentations.
    - *Diagnostic check*: Inspect relative camera extrinsics across captures in `refine_pose` logs.
    - *Targeted action*: Compute relative object transforms using the moving robot TCP frame at each capture timestamp (`root_t_object`), verify frame alignment between camera object frame (`world_t_camera`) and sensor frame, and reparent the refined object to the robot TCP frame in `ObjectWorld`.

## 3. `Zenoh` `KV` store memory management, `REST` routing, and heap allocators

- **Preventing `zenoh-router` memory exhaustion (`capture_results/`)**:
  - High-resolution camera captures passed by reference store multi-megabyte image payloads in an in-memory `Zenoh` `KV` store under `capture_results/<UUID>`. If downstream perception skills fail to delete consumed keys, the `Zenoh` router accumulates payloads until it exhausts pod memory limits and restarts, dropping workcell pub/sub streams and `/tf` transforms.
  - Decision tree:
    - *Precondition*: `zenoh-router` pod memory climbs continuously or enters an out-of-memory crashloop.
    - *Diagnostic check*: Query `/kv_list_all_keys?prefix=capture_results&keyexpr=**` to inspect accumulated uncollected keys.
    - *Targeted action*: Wrap every consumer of `capture_results/<UUID>` in a `try...finally` block that explicitly deletes keys upon consumption.
- **Mandatory `/api/http-gateway/` prefix for `KV` store `REST` access**:
  - When querying the `KV` store HTTP `REST` gateway from a pod via `istio-ingressgateway.app-ingress.svc.cluster.local:80`, prefix routes with `/api/http-gateway/` (`/api/http-gateway/api/kvstore/stores/<store>/keys/<key>`).
  - Decision tree:
    - *Precondition*: HTTP `REST` requests to the `KV` store gateway return HTTP 404 or connection refused from inside a pod.
    - *Diagnostic check*: Inspect request URL path and target service address.
    - *Targeted action*: Add the `/api/http-gateway/` prefix when querying through Envoy; direct connections to `http-gateway.app-intrinsic-base.svc.cluster.local:8080` are blocked by network policy.
- **Heap allocator arena management in point cloud skills**:
  - Long-running Python and C++ point cloud skills (`estimate_point_cloud`) repeatedly allocating and freeing buffers larger than 128 KB cause default `glibc` `malloc` to hoard arena memory. Link point cloud binaries against TCMalloc and close transient gRPC channels in `try...finally` blocks to prevent unreturned heap fragmentation.

## 4. Camera-to-robot and camera-to-camera calibration workflows

- **ChArUco target precondition and dataset frame limits**:
  - `initialize_calibration` inspects `target.marker().has_charuco_pattern()` on the referenced `PoseEstimationConfig` Data Asset. If a non-ChArUco marker configuration is supplied, the skill fails immediately with `FAILED_PRECONDITION: PoseEstimationConfig (v1) target is not a Charuco pattern`.
  - `ai.intrinsic.calibration_service` enforces a hard ceiling of 100 captured frames per dataset (`No more than 100 capture data points can be stored`).
  - Decision tree:
    - *Precondition*: `CaptureData` fails with `CaptureData failed: Error: No more than 100 capture data points can be stored`.
    - *Diagnostic check*: Check service state via `inctl service state list --address=localhost:17080` (`calibration_service: State: Enabled`). Note that `inctl resource` is an invalid subcommand.
    - *Targeted action*: Clear or reset accumulated capture data points before re-running a multi-pose calibration sequence.
- **Motion planning transition filtering and automatic look-at tracking**:
  - Even when `sample_calibration_poses` generates collision-free static poses, `collect_calibration_data` silently filters out any pose for which the motion planner cannot find a feasible collision-free transition trajectory from the preceding pose.
  - Setting `random rotation around x/y = 0 degrees` in calibration parameters does not lock robot arm orientation; `collect_calibration_data` always applies automatic look-at tracking so the target board faces the camera across translational waypoints.
- **Runtime belief world state sync (`"world"` vs. `"init_world"`)**:
  - Camera calibration updates the calibrated camera pose inside the active runtime belief world (`world_id="world"`) even if reprojection error exceeds tolerance, allowing operator visual inspection.
  - Guardrail: Do not sync `"world"` back to `"init_world"` when calibration reprojection error exceeds tolerance; inspect the reprojection error report and re-sample poses until reprojection error falls within acceptable threshold before calling `SyncObject` or `Save`.
  - Guardrail: Do not pass non-existent world identifiers such as `"exec_world"` or `"belief"`; pass `"world"` for runtime belief state and `"init_world"` for initial workcell configuration (`ListWorlds` returns `["sim_world", "world", "init_world", "save_monitor"]`).
- **Service startup configuration payload and log inspection**:
  - Services such as `calibration_service` unpack startup configuration from `/etc/intrinsic/runtime_config.pb`. Supplying an empty `google.protobuf.Any` config payload triggers an immediate startup crash-loop (`INVALID_ARGUMENT: Cannot unpack empty Any to intrinsic_proto.perception.v1.CalibrationServiceConfig`). Populate valid configuration messages in solution asset definitions prior to deployment.
  - Note that standard perception log retrieval utilities omit calibration pods (`calibration-service`, `calibrate-camera-to-robot`, `collect-calibration-data`); inspect calibration pods directly via cluster tools (`kubectl` / `k9s` logs) or filter cluster logs by `resource.labels.pod_name=~"calibration"`.

## 5. Hardware `PCIe` bus contention and real-time control isolation

- **`GPU` `DMA` transfers and real-time control disconnects (`ICON`)**:
  - On industrial PCs sharing a `PCIe` root complex between discrete GPUs and real-time network interfaces (`realtime_nic*`), heavy GPU DMA transfers during vision inference can trigger `PCIe` link degradation (Gen 4 downgrading to Gen 1/3) and replay storms, stalling CPU interrupt handling for real-time robot control (`ICON`) by > 20 ms.
  - Decision tree:
    - *Precondition*: Vision inference coincides with real-time robot control (`ICON`) drops or `ICON task blocked >20 ms`.
    - *Diagnostic check*: Inspect `PCIe` link generation via `nvidia-smi -a | grep -A5 PCIe` (check if `Current` generation degraded to `1` or `3`); inspect error counters via `nvidia-smi pci -gErrCnt` (`REPLAY_COUNTER`, `BAD_TLP`); verify bus separation via `lshw -class network -businfo`.
    - *Targeted action*: Move the discrete GPU or real-time NIC to separate `PCIe` root complexes or replace degraded riser hardware to maintain Gen 4 signal integrity.

## 6. `System 2` reflection checkpoints and anti-thrashing circuit breakers

- **`System 2` reflection checkpoints**:
  - *Pre-calibration verification*: Verify ChArUco pattern dimensions (`square_length`, `marker_length` in meters) against physical hardware; confirm that prior dataset points are cleared on `calibration_service`; ensure runtime `"world"` is inspected prior to syncing extrinsics to `"init_world"`.
  - *Pre-streaming and inference verification*: Verify that camera resolution dimensions are non-zero multiples of 8 and network MTU is set to 9000 for multi-camera streams; confirm `cleanup_model_on_destruction = false` on ephemeral evaluation clients; ensure all `capture_results/<UUID>` keys are wrapped in `try...finally` deletion handlers.
- **Anti-thrashing circuit breakers**:
  - *Camera streaming circuit breaker*: If `CaptureStream` fails with write-protect or peer renegotiation timeout after 2 attempts, halt retries, verify that background preview streams are terminated, and inspect network MTU rather than looping.
  - *Pose estimation circuit breaker*: If multi-view pose estimation fails with singular matrix errors or returns no detections after 2 consecutive captures, halt inference, verify camera object vs. sensor frame alignment, and bound views to <= 4 with distinct angular baselines.
  - *Calibration service circuit breaker*: If `CaptureData` fails with `FAILED_PRECONDITION` (frame ceiling reached or target pattern error), halt the calibration sequence immediately, clear accumulated captures via `inctl`, and inspect target asset configuration before attempting another run.
