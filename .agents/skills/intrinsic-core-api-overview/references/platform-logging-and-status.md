# Platform services, logging, storage, simulation, and status reference

This document provides technical reference documentation, multi-RPC sequences, and API usage hints for structured logging, bag recording, performance telemetry, key-value storage, PubSub streaming, cluster administration, workcell operational modes, simulation control, and `ExtendedStatus` structured error reporting.

## Structured logging, bag recordings, and performance telemetry

The logging and telemetry pipeline spans on-premise structured event ingestion (`intrinsic_proto.data_logger.DataLogger`), bag packaging (`intrinsic_proto.data_logger.BagPackager`), Zenoh topic listeners (`intrinsic_proto.data_logger.PubSubListener`), container text log packaging (`intrinsic_proto.textlogs.v1.TextLogFetcher`), and performance metrics (`intrinsic_proto.performance.analysis.proto.PerformanceMetrics`).

### Core `gRPC` services and when to use them

| Service | Primary RPCs | When and why to use |
| :--- | :--- | :--- |
| `intrinsic_proto.data_logger.DataLogger` | `Log`, `ListLogSources`, `GetLogItems`, `GetMostRecentItem`, `SetLogOptions`, `GetLogOptions`, `SyncAndRotateLogs`, `CreateLocalRecording`, `ListLocalRecordings`, `GetLocalRecording`, `GetLoggerContext` | Emit typed `LogItem` records and binary `Blob` payloads (< 2 GB), query historical telemetry over time ranges, flush memory buffers to persistent storage, and package on-premise recordings. |
| `intrinsic_proto.data_logger.BagPackager` | `GenerateBag`, `ListBags`, `GetBag` | Convert uploaded recording streams into `.mcap` bag files and retrieve download URLs (`with_signed_url = true`). |
| `intrinsic_proto.data_logger.PubSubListener` | `SetTopicSubscriptions`, `GetTopicSubscriptions` | Subscribe to Zenoh PubSub topics (`Subscription.topic_expr`) and automatically forward published packets as `LogItem` records to `intrinsic_proto.data_logger.DataLogger/Log`. |
| `intrinsic_proto.textlogs.v1.TextLogFetcher` | `CreateAssetLogsPackage`, `GetOperation`, `WaitOperation`, `CancelOperation`, `DeleteOperation` | Query and package raw container stdout/stderr text logs for skills and resource instances into downloadable `.zip` archives over time windows up to 7.5 days. |

### Decision tree: diagnostic and telemetry collection

- **High-priority text logs and checkpoints during critical execution**: Emit `intrinsic_proto.data_logger.CriticalEventLog` (`LogItem.payload.event_log`) on `/text-log-out` or `/asset-text-log-out`.
- **Structured execution telemetry, perception frames, or sensor streams (< 2 GB blobs)**: Verify cluster deployment level (`platform_level:enterprise`) and ingress reachability. Dial `DataLogger` via `istio-ingressgateway.app-ingress.svc.cluster.local:80`; emit `LogItem` with `Blob`.
- **Container stdout/stderr text logs for installed skills over a time window (<= 7.5 days)**: Check if query duration exceeds 180 hours (7.5 days). Call `TextLogFetcher/CreateAssetLogsPackage` with `organization-id` metadata header; poll `GetOperation`.
- **Historical diagnostic analysis of a multi-minute or multi-hour run**: Call `DataLogger/SyncAndRotateLogs(wait_for_flush=True)` to flush in-memory ring buffers, then query `GetLogItems`.

### `API` usage hints for structured logging and bags

- **Enterprise vs. core deployment gating (`platform_level:enterprise` vs. `platform_level:core`)**:
  - `DataLogger`, `PubSubListener`, `WorkcellMode`, and `FrameCalibrationService` are gated behind `platform_level:enterprise` deployments and are **not deployed on `platform_level:core` workcells**. On core clusters, calling these services returns `StatusCode.UNIMPLEMENTED`.
- **Routing to `DataLogger` from skills and cluster services**:
  - Both skills and cluster services dial `DataLogger` through the cluster ingress gateway at `istio-ingressgateway.app-ingress.svc.cluster.local:80` inside Kubernetes pods (or `localhost:17080` outside the cluster, or via a leased `ResourceHandle`).
  - Configure gRPC channels with unlimited message size (`grpc.max_receive_message_length = -1`) when reading or writing `Blob` payloads.
- **`SetLogOptions` administrative map keys vs. RE2 `event_source` regex matching**:
  - The map key in `SetLogOptionsRequest.log_options` is an administrative identifier used only by `GetLogOptionsRequest.key`.
  - Stream matching uses `RE2::FullMatch` against `LogOptions.event_source`. When multiple rules match a stream, the rule with the highest `log_options_precedence_value` wins; assign higher precedence values to specific regexes and lower values to catch-all patterns (default fallback key: `"default_event_source"`).
  - `retain_on_disk_retention_duration` is evaluated at second precision and only takes effect when `retain_on_disk = true`.
- **Memory buffer vs. persistent disk routing in `GetLogItems`**:
  - `GetLogItems` does not automatically flush memory buffers before reading disk storage. If a query spans outside the in-memory ring buffer and returns a truncation cause, call `intrinsic_proto.data_logger.DataLogger/SyncAndRotateLogs` (`wait_for_flush = true`) first to flush pending memory buffers to disk before querying.
  - Raw `GetLogItems` calls with an unset `start_time` default to only the last 5 minutes (`now - 5min`). Always explicitly populate timezone-aware UTC timestamps (`datetime.datetime.now(datetime.timezone.utc)`).
- **Python `StructuredLogs` single-page truncation vs. `EventSourceReader` pagination**:
  - `structured_logs.query(...)` and `structured_logs.query_for_time_range(...)` issue only a single `GetLogItems` call and ignore `next_page_cursor`. To paginate across multiple pages or convert items to Pandas DataFrames, use `EventSourceReader` (`structured_logs.get_event_source(name).read(...)`).
- **`Context.labels` server-side filtering vs. `only_metadata` and execution context propagation**:
  - `GetLogItemsRequest.Query.filter_labels` matches exclusively against `LogItem.context.labels` (`map<string, string>`). Typed fields such as `skill_id`, `executive_plan_id`, and `icon_session_id` are ignored by `filter_labels`.
  - To filter by `skill_id` or `executive_plan_id` server-side, duplicate those values into `Context.labels` before logging. To filter client-side without downloading heavy payloads, set `only_metadata = true` in `GetLogItemsRequest`.
  - Distinguish `SkillLoggingContext.skill_id` (static string asset ID such as `"ai.intrinsic.move_robot"`) from `SkillLoggingContext.data_logger_context.skill_id` (`uint64` dynamic per-execution invocation ID). Pass `context = skill_logging_context.data_logger_context` when calling `intrinsic_proto.icon.v1.IconApi/OpenSession`, `intrinsic_proto.motion_planning.v1.MotionPlannerService/PlanTrajectory`, or building `ExtendedStatus` (`Relations.log_context`).
  - Because `0` is a valid ICON session or action ID, check presence (`HasField("icon_session_id")` / `HasField("icon_action_id")`) rather than testing for non-zero values.
- **High-priority text checkpoints (`CriticalEventLog`)**:
  - Emit `intrinsic_proto.data_logger.CriticalEventLog` (`LogItem.payload.event_log` on `/text-log-out` or `/asset-text-log-out`) to preserve critical operational text logs during high-bandwidth events.
- **Conjunctive (`AND`) downsampling semantics (`DownsamplerOptions`)**:
  - When both `sampling_interval_time` and `sampling_interval_count` are set in `DownsamplerOptions`, a candidate item is kept only if **both** thresholds have elapsed since the previous kept item for that `event_source`.
- **60-character Zenoh topic limit and metadata constraints in `PubSubListener`**:
  - `LogItem.metadata.event_source` is limited to **at most 60 characters** (due to database identifier limits). Because `PubSubListener` sets `metadata.event_source` to the Zenoh topic name by default, any Zenoh topic longer than 60 characters fails ingestion with `INVALID_ARGUMENT` unless the payload is a `LogItem` with an explicit `metadata.event_source` of 60 characters or fewer.
  - Publishers sending `LogItem` messages over Zenoh must leave `acquisition_time`, `workcell_name`, and `name` unset (the server auto-generates them and rejects pre-populated values).
  - Use specific key expressions in `SetTopicSubscriptions` (`allow = true`) to prevent duplicate logging, pass exact `topic_expr` strings when unsubscribing (`allow = false`), and use `GetTopicSubscriptions` (`CheckTopicSubscription` is unimplemented).
- **Recording categories, duration clamping (`CreateLocalRecording`), and `BagPackager` nuances**:
  - Lightweight-only recordings (`text_logs` with no custom regexes or forced scene context) support up to 24 hours (`1440` minutes). Including any non-lightweight category (`scene_data`, `robot_data`, `perception_data`, `debug_data`, `skill_data`), custom regex selector, or forced scene context clamps the maximum allowed duration to 10 minutes.
  - `CreateLocalRecording` enforces a server-side cooldown between calls (`RESOURCE_EXHAUSTED`); retry with exponential backoff.
  - Always populate `query.bag_id`. Recordings in `BagStatus.UNCOMPLETABLE` state (due to dropped messages) remain eligible for `GenerateBag`, which packages available data best-effort into `UNCOMPLETABLE_COMPLETED`.
  - When `GenerateBag` exceeds reverse-proxy HTTP timeouts (HTTP 504 / `DEADLINE_EXCEEDED`), transition immediately to polling `intrinsic_proto.data_logger.BagPackager/GetBag` (`with_signed_url = false`) every 5–30 seconds until `BagRecord.bag_file` is populated (`BagStatus.COMPLETED` or `BagStatus.UNCOMPLETABLE_COMPLETED`). Always populate `ListBagsRequest.list_query` rather than deprecated top-level fields.
- **`TextLogFetcher` header, query window, and LRO lifecycle constraints**:
  - All RPCs on `intrinsic_proto.textlogs.v1.TextLogFetcher` require the `organization-id` gRPC metadata header (discoverable via `intrinsic_proto.data_logger.DataLogger/GetLoggerContext`, stripped of any `@domain` suffix).
  - `AssetLogsQueryRequest` requires both `start_time` and `end_time` (<= 7.5 days / 180 hours), and deduplicate `asset_references` to guarantee isolated, collision-free writes to `.log` files inside the output `.zip` archive.
  - `WaitOperation` ignores `WaitOperationRequest.timeout` and blocks until completion or gRPC context deadline (`DEADLINE_EXCEEDED`); unknown or expired operations (evicted after 24 hours) return `codes.Internal`, and `ListOperations` is unimplemented.
- **Performance metrics (`PerformanceMetrics`) and inter-skill event pubsub (`ai.intrinsic.struct_formatter` / `ai.intrinsic.event_pubsub`)**:
  - Use `intrinsic_proto.performance.analysis.proto.PerformanceMetrics` to record real-time control loop histograms (`"apply_command_duration"`, `"read_status_duration"`, `"duration_between_read_status_calls"`, `"process_duration"`, `"execution_duration"`) and EtherCAT diagnostics (`"ethercat_performance_metrics"`).
  - In public SDK environments where `LogItem.Payload.performance_metrics` is unavailable, pack `PerformanceMetrics` into `LogItem.Payload.any` (`google.protobuf.Any`, field `1`), store `"cycle_number"` inside `PerformanceMetrics.metrics.metrics` (`google.protobuf.Struct`), and always populate `metadata.acquisition_time`.
  - When deserializing nested `intrinsic_proto.performance.skills.Event` payloads with `ai.intrinsic.struct_formatter`, omit `deserialize_to_type` (setting `deserialize_to_type.event` raises `TypeError`), and ensure at most one entry per protobuf type in `SerializeToEvent.data_to_serialize`.
  - Subscribers on `ai.intrinsic.event_pubsub` topics must handle synthetic `"SKILL_CANCELLED"` and `"TIMEOUT"` events emitted when the publisher skill is canceled or times out.

## Platform storage, `KVStore`, file upload, `PubSub`, and relay routing

### Key-value store (`intrinsic_proto.kvstore.KVStore`) and `StorageLocation`

- **gRPC vs. Zenoh PubSub/Queryables store scopes and consistency**:
  - The gRPC service `intrinsic_proto.kvstore.KVStore` (`Get`, `Set`, `Delete`, `List`) operates exclusively on the local store (`"kv_store"`) with `high_consistency = True` (polling read-back confirmation).
  - Note that neither `/api/kvstore/stores/<store>/keys/<key>` nor the `istio-ingressgateway` path `/api/http-gateway/api/kvstore/stores/<store>/keys/<key>` exists as an HTTP REST route (`KVStore` has no HTTP/REST annotations and returns 404). Multi-store access (`"kv_store"`, `"kv_store_repl"`, or custom prefixes) is performed over Zenoh PubSub/Queryables (`pubsub.KVStore()`, `pubsub.KVStoreReplicated()`, `pubsub.KVStoreWithPrefix(prefix)`).
- **Stripping the `"kv_store/"` key prefix**:
  - Calling `intrinsic_proto.kvstore.KVStore/List` returns keys prefixed with `"kv_store/"` (for example, `"kv_store/my_key"`), whereas `Get`, `Set`, and `Delete` expect unprefixed keys (`"my_key"`). Strip the prefix via `raw_key.removeprefix("kv_store/")`.
- **Workcell identity (`intrinsic.platform.proto.WorkcellInfo`)**:
  - Stored in the local `"kv_store"` under key `"workcell_info"` (`workcell_name`, `project`). In pure on-premise Intrinsic Core deployments without cloud connectivity, `"workcell_info"` may return `NOT_FOUND`; fall back to a configured default cluster name.
- **Passing large payloads by reference (`intrinsic_proto.kvstore.StorageLocation`)**:
  - Pass multi-megabyte point clouds or camera images by reference rather than through Behavior Tree return protos. Store the payload in `"kv_store"` (warning threshold: 25 MiB) and pass `intrinsic_proto.kvstore.StorageLocation(store="kv_store", key=...)`.
  - When capturing images across multiple robot waypoints (`CaptureImages`), allocate unique keys (such as `f"waypoint_{i}_capture"`) to preserve each capture under a distinct camera identifier key. Always explicitly set `store = "kv_store"` on `StorageLocation` messages consumed by C++ services.
- **Unsetting `KUBERNETES_SERVICE_HOST` for in-pod peer subscriptions**:
  - When `KUBERNETES_SERVICE_HOST` is present in the environment, Zenoh peer configuration disables local TCP listen endpoints. Python services inside Kubernetes pods that create local peer KVStore/PubSub subscriptions temporarily unset `KUBERNETES_SERVICE_HOST` during `pubsub.PubSub()` initialization.
- **Recursive `google.protobuf.Any` unpacking**:
  - Because intermediate replication paths can wrap an `Any` inside another `Any`, unpack nested `Any` layers in a loop (`while msg.Is(any_pb2.Any.DESCRIPTOR): ...`) before checking the concrete payload descriptor.

### Decision tree: platform storage and streaming selection

- **Local single-workcell strongly consistent state (read-after-write confirmation)**: Use gRPC `intrinsic_proto.kvstore.KVStore` (operates on `"kv_store"` with `high_consistency=True`).
- **Replicated cross-workcell or custom-prefixed key-value state**: Use Zenoh queryables via `pubsub.KVStoreReplicated()` or `pubsub.KVStoreWithPrefix(prefix)`.
- **Large payloads (> 25 MiB point clouds, camera images) across Behavior Tree nodes**: Store in `"kv_store"` and pass `intrinsic_proto.kvstore.StorageLocation(store="kv_store", key=...)`.
- **Spoke-to-hub star topology between edge workcells without cloud routing**: Route via `ai.intrinsic.line_orchestration_relay` on Hub (ports 7447 internal, 17447 external).

### `ICON` file upload service (`intrinsic_proto.icon.FileUploadService`)

- **`intrinsic_proto.icon.FileUploadService/UploadFile`, `ListFiles`, and `RemoveFiles`**:
  - Manages binary shared libraries (`"my_custom_actions.so"`) and configuration files on the ICON server.
  - `UploadFileRequest.filename` must be a pure basename without `"/"` or `".."`.
  - `RemoveFiles` is non-atomic across multiple filenames; pass a single-element list `["<filename>"]` per call for reliable error tracking.
  - Uploaded or removed files take effect only after an ICON server restart.

### `PubSub` bridge and line orchestration relay

- **Topic vs. KV store subscription wire formats and queryables**:
  - PubSub topic subscriptions receive `intrinsic_proto.pubsub.PubSubPacket` envelopes (`payload` as `google.protobuf.Any` and `publish_time`), whereas KV store subscriptions receive raw `google.protobuf.Any` payloads (with zero-length byte payloads indicating key deletions).
  - Filter out internal introspection topics starting with `in/_introspection/` before deserializing `PubSubPacket`, as they carry raw JSON.
  - When querying a Zenoh queryable via `GetOne`, zero responses always return `DEADLINE_EXCEEDED`, and multiple responses return `FAILED_PRECONDITION` (use `Get` for multi-responder wildcards).
- **Edge-to-edge line orchestration (`ai.intrinsic.line_orchestration_relay` and `ai.intrinsic.line_orchestration_forwarder`)**:
  - Connects spoke workcells to a centralized Hub workcell (`RelayRouterServiceConfig`) in a star topology without cloud routing. The Hub Relay Router dials the Hub's internal router on port `7447` and spoke workcell routers on external LoadBalancer port `17447`.
  - Forwarding a topic to `interipc_ps/<cluster_id>/<topic>` creates a new outer `PubSubPacket.publish_time`; read original timestamps from the inner payload.
  - Inside Zenoh KV subscription callbacks, forwarders call `SetAny(..., highConsistency=False)` to maintain non-blocking callback thread execution.

---

## Operation mode and simulation control

### Solution service (`intrinsic_proto.solution.v1.SolutionService`) and operation mode (`intrinsic_proto.config.OperationMode`)

- **Three-way asymmetry on `OPERATION_MODE_UNSPECIFIED` (`0`)**:
  - Always pass an explicit `OperationMode` (`SIMULATION` [`1`] or `REAL_HARDWARE` [`2`]): `intrinsic_proto.assets.v1.SolutionDeploymentService` rejects `0` with an error, `intrinsic_proto.conductor.ConductorService/StartSolution` defaults `0` to `SIMULATION`, and `intrinsic_proto.simulation.v1.SimulationService/StartSolution` treats `0` as `REAL_HARDWARE`.
  - In `intrinsic_proto.services.ServiceManifest.service_def`, `sim_spec` is mandatory for validation. Populate both `real_spec` and `sim_spec` because `REAL_HARDWARE` mode does not fall back to `sim_spec`.
- **Distinguishing simulation vs. hardware inside skills (`ai.intrinsic.is_hardware_check`)**:
  - `ai.intrinsic.is_hardware_check` returns `false` during `Preview` and `true` during `Execute` (including when running in physics simulation). To check whether `Execute` is running in `SIMULATION` vs. `REAL_HARDWARE`, query `intrinsic_proto.conductor.ConductorService/GetExecutionContext` (`ExecutionContext.mode`) or `intrinsic_proto.assets.v1.SolutionDeploymentService/GetSolutionDeployment` (`SolutionDeployment.operation_mode`). In Python client scripts, check `if solution.is_simulated:` before accessing `solution.simulator` (which is `None` in `REAL_HARDWARE`).
- **`intrinsic_proto.solution.v1.SolutionService/GetStatus` readiness rules and behavior tree persistence**:
  - Executing processes via `intrinsic_proto.executive.ExecutiveService` is only safe when `GetStatus` returns `state == READY` (which verifies that all installed skill assets are registered in `SkillRegistry`).
  - Unauthenticated calls to `CreateBehaviorTree` or `UpdateBehaviorTree` update `HotSharedState` in memory but are lost on solution restart unless called with authenticated credentials. When loading a process by ID, check `intrinsic_proto.assets.v1.InstalledAssets/GetInstalledAsset` (`process` variant) first and fall back to `intrinsic_proto.solution.v1.SolutionService/GetBehaviorTree`.

### Simulation control (`SimulationService`, `SimulatorWorldSync`, `GazeboService`, and `SetSimulatedInputsService`)

- **`intrinsic_proto.simulation.v1.SimulationService/ResetSimulation`**:
  - Rewinds simulation time to `0`, resets digital inputs to `false`, clones the belief world into `sim_world`, reloads Gazebo, and restarts ICON servers.
  - Modifying the live belief world (`intrinsic_proto.world.ObjectWorldService/UpdateWorld`) does not automatically update Gazebo's `sim_world`. To sync passive joints without a full `ResetSimulation`, apply `UpdateWorld` to both `"world"` and `"sim_world"` (valid world IDs: `["sim_world", "world", "init_world", "save_monitor"]`), or spawn objects with `create_in_world = TargetWorld.BELIEF_AND_SIM`.
  - Prefer passing `start_from_world_state` to `intrinsic_proto.executive.ExecutiveService/Run` over calling `ResetSimulation` manually before running a process.
- **`intrinsic_proto.simulation.v1.SimulatorWorldSync/SyncWorld` and `GazeboService`**:
  - The first message on `SyncWorld` must populate `session_init`, and only one active stream is permitted per cluster (`RESOURCE_EXHAUSTED`). High-frequency pose/joint updates flow over PubSub (`state_updates_pubsub_topic_name`), not over `SyncWorld`.
  - `intrinsic_proto.simulation.v1.GazeboService/ListTopics` returns only model plugin, world System plugin, and sensor topics; publish exclusively to documented plugin and sensor topics.
- **`intrinsic_proto.simulation.v1.SetSimulatedInputsService/ListSimulatedInputs` and `SetSimulatedInputs`**:
  - In `SetDigitalInputBit`, `block_name` must match the **Hardware Module (HWM) block name** returned by `ListSimulatedInputs` (e.g., `"standard_digital_input_status"`), not the ICON input interface alias (e.g., `"standard_in"`).
  - All input bits in a single `SetSimulatedInputsRequest` are applied atomically in the exact same Gazebo simulation step.
- **Simulated Hardware Module prefix rules (`intrinsic_proto.sim.SimHardwareModuleConfig`)**:
  - In `InferInterfacesFromInstances`, setting `omit_instance_name_prefix: false` activates the `prefix_override` oneof and **still omits the `<instance_name>_` prefix**. Leave `prefix_override` completely unset to preserve default instance prefixes, and declare required companion interfaces (`JointPositionStateInterface`, `JointVelocityStateInterface`, `JointAccelerationStateInterface`, `JointLimitsCommandInterface`) alongside `JointPositionCommandInterface`.

### Decision tree: simulation state and operational modes

- **Full clean-slate simulation reset**: Diagnostic check: check if ongoing physical operations or uncommitted Behavior Tree edits will be wiped. Targeted action: call `SimulationService/ResetSimulation` and wait for ICON state to settle.
- **Testing a behavior tree from a starting snapshot**: Diagnostic check: verify whether initial state requires Gazebo physics reload or only world object placement. Targeted action: supply `start_from_world_state` in `ExecutiveService/Run`.
- **Synchronizing passive joints into Gazebo physics**: Diagnostic check: check whether update target is strictly `"sim_world"` or both `"world"` and `"sim_world"`. Targeted action: call `ObjectWorldService/UpdateWorld` on both `"world"` and `"sim_world"` (or use `TargetWorld.BELIEF_AND_SIM`).
- **Toggling digital I/O lines in simulation**: Diagnostic check: confirm `block_name` matches Hardware Module block name (from `ListSimulatedInputs`), not alias. Targeted action: call `SetSimulatedInputsService/SetSimulatedInputs` using `block_name` from `ListSimulatedInputs`.
- **Checking execution environment inside a custom skill**: Diagnostic check: verify whether runtime is Preview vs Execute and whether Execute is SIMULATION vs REAL_HARDWARE. Targeted action: query `ConductorService/GetExecutionContext` or check `solution.is_simulated` in Python.

---

## Kubernetes accounts, access control, and workcell mode

### Accounts and access control (`intrinsic_proto.accounts.accesscontrol.v1.AccessControlService`, `intrinsic_proto.accounts.invitations.v1.InvitationsService`, `intrinsic_proto.accounts.resourcemanager.v1.ResourceManagerService`, `intrinsic_proto.accounts.tokens.v2.AccountsTokensService`)

- **Resource naming rules and base64 role binding names**:
  - Format resource names passed to `AccessControlService` or `ResourceManagerService` without leading slashes (use `"organizations/{org}"`, `"users/{email}"`, `"roles/{role}"`, `"operations/{id}"`).
  - `Organization.name` returned by `ResourceManagerService/GetOrganization` and `ListOrganizations` is the bare organization ID (e.g., `"exampleorg"` without `"organizations/"`); normalize it before passing to RPCs requiring `"organizations/{org}"`.
  - `ListOrganizationRoleBindings` returns URL-safe base64 role binding names (`+` -> `-`, `/` -> `_`); pass these URL-safe names when calling `DeleteRoleBinding` over HTTP/REST.
  - `intrinsic_proto.accounts.invitations.v1.InvitationsService/ApplyInvitation` returns an `Operation` without a `GetOperation` RPC; poll by calling `ApplyInvitation` repeatedly with the same token until `done == true`.
  - In `ResourceManagerService/ListOrganizations`, only `filter: ""` and `filter: "is_member()"` are supported, and soft-deleted organizations are omitted (`GetOrganization` inspects soft-deleted records).

### Cluster administration and workcell operational mode

- **Cluster reset and software updates (`intrinsic_proto.cluster.v1.ClusterService` & `intrinsic_proto.inversion.v1.IpcUpdater`)**:
  - Use `intrinsic_proto.cluster.v1.ClusterService/Reset` with `RESET_TYPE_SOLUTION` (`2`) to clear a wedged solution while preserving IPC identity and certificates (`RESET_TYPE_FACTORY` [`1`] wipes IPC identity).
  - In `+accept` update mode, call `intrinsic_proto.inversion.v1.IpcUpdater/ReportUpdateInfo`, verify `state == STATE_UPDATE_AVAILABLE`, and pass the exact `available.version_id` to `intrinsic_proto.inversion.v1.IpcUpdater/ApproveUpdate`.
- **Workcell operational mode (`intrinsic_proto.workcellmode.v1.WorkcellMode`)**:
  - `intrinsic_proto.workcellmode.v1.WorkcellMode/SetWorkcellMode` accepts only settled target modes (`MODE_DEVELOPMENT` [`5`], `MODE_MAINTENANCE` [`6`], `MODE_PRODUCTION` [`7`]). Poll `intrinsic_proto.workcellmode.v1.WorkcellMode/GetWorkcellMode` until any transitional state resolves to the target mode.

---

## Structured error reporting (`intrinsic_proto.status.ExtendedStatus`)

`intrinsic_proto.status.ExtendedStatus` provides a hierarchical causal tree of error reports across skills, services, and Behavior Tree nodes.

### Domain code ranges and propagation rules

- **Domain code ranges**:
  - `0`: Unspecified/unknown status code.
  - `1–9999`: System and platform errors (`ai.intrinsic.executive` uses `11302` for parameterization failure, `18002` for timeout, `18201` for footprint conflict, and `31100`/`31300`/`31800` for `Fallback`/`Retry`/`Sequence` structural failure wrappers; `ai.intrinsic.errors:604` warns when a status code is undeclared in `StatusSpecs`).
  - `10000+`: Component, skill, service, and solution errors (partitioned into functional 100-code blocks such as `10100–10199` for grasping and `10200–10299` for motion/IK).
- **Automatic skill component wrapping**:
  - The Skills SDK enforces that the top-level `ExtendedStatus` returned by a skill has `status_code.component == skill_id`. If a skill raises or returns an upstream service `ExtendedStatus` directly, the SDK wraps it in a new top-level `ExtendedStatus` with `component = skill_id` and `code = 0`. Always explicitly wrap upstream errors inside a skill-specific code (`>= 10000`) via `ExtendedStatusError(skill_id, code, ...).add_context(service_es)` in Python or `.AttachExtendedStatus(skill_id, code, ...)` in C++.
- **Binary proto requirement for `StatusSpecs` and SDK language nuances**:
  - `status_specs` initialization functions (`status_specs.init_once(component, filename=...)` in Python, `InitExtendedStatusSpecs` in C++, `statusspecs.InitFromFile` in Go) require a binary-serialized `intrinsic_proto.assets.StatusSpecs` file (`.binarypb`), not a `.pbtxt` textproto.
  - In Go `statusspecs.Create`, always pass `statusspecs.WithTimestamp(time.Now())` to prevent a nil-pointer dereference, and use `extstatus.FromGRPCError(err)` when extracting downstream gRPC errors.
  - In C++ `StatusBuilder`, `.generic_code` is ignored on `.AttachExtendedStatus(...)` and `.WrapExtendedStatus(...)`; chain `.SetCode(absl::StatusCode::...)` explicitly when changing categorical status codes.
  - On a `bt.Fail` node, set `failure_message=""` when configuring custom status fields via `fail_node.on_failure.emit_extended_status(...)`.
- **Dual transport on `google.longrunning.Operation` and payload size limits**:
  - Fatal operation errors are packed in `operation.error.details` (`google.rpc.Status.details`), whereas non-fatal validation warnings are unpacked from `operation.metadata` (`RunMetadata.diagnostics`). Keep `debug_report` messages under 10 kB to stay within gRPC trailing metadata limits.
- **Shallow `bt.ExtendedStatusMatch` vs. deep recursive CEL matching**:
  - `bt.ExtendedStatusMatch` inspects **only** the top-level `status_code`, stopping at the outer envelope without traversing `ExtendedStatus.context`. When a failing skill is wrapped by a `Fallback` (`31100`), `Retry` (`31300`), or `Sequence` (`31800`) node, the skill's root-cause code is pushed into `.context`.
  - Use `bt.Blackboard(cel_expression=...)` with recursive `.exists()` checks over `.context` to match error codes at any depth in the causal tree:

```python
def build_deep_error_match_cel(blackboard_key: str, target_codes: list[int]) -> str:
  """Constructs a CEL expression matching target error codes up to depth 2 in .context."""
  codes_str = ", ".join(str(c) for c in target_codes)
  return (
      f"(has({blackboard_key}.status_code) &&"
      f" {blackboard_key}.status_code.code in [{codes_str}]) ||"
      f" (has({blackboard_key}.context) &&"
      f" {blackboard_key}.context.exists(c0, (has(c0.status_code) &&"
      f" c0.status_code.code in [{codes_str}]) || (has(c0.context) &&"
      " c0.context.exists(c1, has(c1.status_code) && c1.status_code.code in"
      f" [{codes_str}]))))"
  )
```

### Decision tree: error handling and status inspection

- **Direct unwrapped task node failure**: Diagnostic check: check if failure originates directly on the task node without parent composite nodes. Targeted action: match with `bt.ExtendedStatusMatch(blackboard_key, matcher=MatchStatusCode(...))`.
- **Composite node failure (`Fallback` 31100, `Retry` 31300, `Sequence` 31800) or SDK `code=0` wrap**: Diagnostic check: check if root-cause status code is pushed into child `ExtendedStatus.context` elements. Targeted action: use deep recursive CEL expression with `.exists()` over `.context` to extract root cause.
- **Upstream service gRPC failure in custom skill**: Diagnostic check: check if service error is an upstream gRPC code that would be auto-wrapped to `code=0`. Targeted action: wrap inside a skill-level code (>= 10000) via `ExtendedStatusError.add_context(service_es)`.
- **Asynchronous `Operation` failure from Conductor or `TextLogFetcher`**: Diagnostic check: check whether failure is fatal (`operation.error`) or non-fatal diagnostic warnings. Targeted action: inspect `operation.error.details` for fatal errors; inspect `operation.metadata` for warnings.

---

## System 2 reflection and anti-thrashing circuit breakers

### System 2 reflection checklist

Execute this brief self-critique before initiating high-impact diagnostic operations:

1. **Deployment tier verification**: Is the cluster running `platform_level:enterprise`? If running on `platform_level:core`, confirm that the task does not depend on `DataLogger`, `PubSubListener`, `WorkcellMode`, or `FrameCalibrationService`.
2. **World identifier validation**: Does the target world ID match one of `["sim_world", "world", "init_world", "save_monitor"]`? Confirm that live belief modifications target `"world"`, not `"belief"` or `"exec_world"`.
3. **Channel message capacity**: Are gRPC receive limits explicitly configured to unlimited (`-1`) or at least `100 * 1024 * 1024` bytes before reading large binary blobs?
4. **State mutation blast radius**: Will invoking `ResetSimulation` or `ClusterService/Reset` disrupt ongoing physical operations or uncommitted in-memory Behavior Trees?

### Anti-thrashing circuit breakers

To prevent action thrashing and retry loops during execution:

1. **Bag packaging timeouts**: When `GenerateBag` returns HTTP 504 (`DEADLINE_EXCEEDED`), **halt all further calls to `GenerateBag`**. Switch to polling `BagPackager/GetBag` (`with_signed_url = false`) every 10 seconds (up to 15 minutes) until `BagRecord.bag_file` is populated.
2. **Asynchronous operation waiting**: When calling `TextLogFetcher/WaitOperation`, enforce an explicit client-side gRPC timeout. If the deadline expires without completion, issue `GetOperation` to inspect `AssetLogsPackageMetadata.stage`; if stuck in `STAGE_QUERYING` across 3 checks, cancel the operation and verify the `asset_references` list.

---

## Contrastive bifurcation analysis (tau- / tau+)

The following contrastive pairs illustrate the exact bifurcation points where failing operational trajectories diverged from succeeding ones:

| Operational domain | Failing trajectory (tau-) | Succeeding trajectory (tau+) | Bifurcation point invariant |
| :--- | :--- | :--- | :--- |
| **Service routing** | Dialing internal namespace FQDNs directly from a skill pod fails due to Kubernetes network policies. | Dialing `istio-ingressgateway.app-ingress.svc.cluster.local:80` routes through the cluster ingress gateway. | All cluster and skill traffic routes through the Istio ingress gateway. |
| **Multi-store key-value** | Requesting `/api/http-gateway/api/kvstore/stores/kv_store_repl/keys/foo` returns HTTP 404 `Not Found`. | Accessing replicated state via `pubsub.KVStoreReplicated().get("foo")` succeeds over Zenoh. | KVStore has no REST route; multi-store access requires Zenoh queryables. |
| **Simulation sync** | Modifying `"world"` leaves Gazebo's `"sim_world"` unchanged, leading to physics divergence. | Updating both `"world"` and `"sim_world"` or using `TargetWorld.BELIEF_AND_SIM` synchronizes passive joints. | Gazebo simulates `"sim_world"`, not the live belief `"world"`. |
| **Error recovery** | Using `bt.ExtendedStatusMatch` on a composite node fails to match child skill error codes. | Using recursive CEL `.exists()` over `.context` matches root-cause error codes at any depth. | Composite nodes wrap child failures and push root causes into `.context`. |

---

## Enforced paired negative guardrails (<= 5 total)

To satisfy Tenet #14 and eliminate the pink elephant problem in model attention, the following paired negative constraints represent the only negative rules in this reference:

1. **Rule 1 (Ingress routing)**: Do not dial internal cluster FQDNs directly from skills or external clients; route all calls through the cluster ingress gateway at `istio-ingressgateway.app-ingress.svc.cluster.local:80` (or `localhost:17080` / `ResourceHandle`).
2. **Rule 2 (KV store REST gateway)**: Do not query `/api/kvstore/...` or `/api/http-gateway/api/kvstore/...` via HTTP REST (which returns 404); access multi-store key-values through Zenoh PubSub/Queryables (`pubsub.KVStore()`, `pubsub.KVStoreReplicated()`, `pubsub.KVStoreWithPrefix(prefix)`) or local gRPC `intrinsic_proto.kvstore.KVStore`.
3. **Rule 3 (World identifiers)**: Do not pass invalid world IDs like `"belief"` or `"exec_world"` to `ObjectWorldService`; use strictly valid world IDs `["sim_world", "world", "init_world", "save_monitor"]` (with `"world"` representing the live belief world).
4. **Rule 5 (Workcell operational mode)**: Do not pass `MODE_TRANSITION_TO_*` enums to `SetWorkcellMode`; supply only settled target modes (`MODE_DEVELOPMENT`, `MODE_MAINTENANCE`, `MODE_PRODUCTION`) and poll `GetWorkcellMode` until transitions resolve.

All other operational rules in this guide are formulated exclusively as affirmative target actions.
