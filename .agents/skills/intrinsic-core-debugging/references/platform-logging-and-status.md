# Debugging platform ingress routing, `gRPC` status codes, and `KVStore` / `PubSub`

## 1. `Envoy` ingress routing and `x-resource-instance-name` header rules

All external requests targeting `localhost:17080` and in-cluster requests targeting `istio-ingressgateway.app-ingress.svc.cluster.local:80` traverse the Istio/Envoy ingress gateway (`app-ingress/istio-ingressgateway`). Envoy routes requests based on the `x-resource-instance-name` gRPC metadata header:

### 1.1 Multi-instance service assets vs. core platform singletons

- **Deployed service asset instances (`rs-<name>`)**:
  - Multiple instances of a service asset share the same fully qualified gRPC service definition. Clients must attach `x-resource-instance-name: <instance_name>` on every RPC (for example, `x-resource-instance-name: icon` or `inctl icon status --instance_name=icon --address=localhost:17080`). Omitting the header returns `StatusCode.UNIMPLEMENTED` with an empty description.
  - In standalone Python stubs outside container runtimes, pass `metadata=[('x-resource-instance-name', '<instance_name>')]` directly to the RPC invocation.
- **Core platform runtime services (`intrinsic_runtime`)**:
  - Core singleton services (`google.longrunning.Operations`, `intrinsic_proto.geometry.GeometryService`, `intrinsic_proto.world.ObjectWorldService`, `intrinsic_proto.assets.v1.InstalledAssets`, `intrinsic_proto.executive.ExecutiveService`, `intrinsic_proto.scene_object.v1.SceneObjectImport`) are grouped under the synthetic instance `intrinsic_runtime`.
  - Envoy routes to core runtime services **only if** the `x-resource-instance-name` header is omitted (`withoutHeaders: { x-resource-instance-name: {} }`) or set to `exact: intrinsic_runtime`.
  - **Negative rule 1 (paired)**: Do not pass service-specific `x-resource-instance-name` headers to core runtime singletons; omit the header or set it to `exact: intrinsic_runtime`. Passing an asset instance header to a core service causes Envoy to reject the call with `StatusCode.UNIMPLEMENTED`. This frequently occurs when polling `google.longrunning.Operations/GetOperation` or `WaitOperation` using metadata retained from an earlier service invocation.

### 1.2 In-cluster network policy isolation and `HTTP` `REST` gateway path rewriting

- **Network policy isolation**: Custom skills and pods in isolated namespaces cannot dial internal base-namespace services (such as `http-gateway.app-intrinsic-base.svc.cluster.local:8080`) directly (`[Errno 111] Connection refused`).
- **Negative rule 2 (paired)**: Do not dial internal base-namespace service DNS directly from isolated pods; route all HTTP and gRPC traffic through `istio-ingressgateway.app-ingress.svc.cluster.local:80`.
- **Mandatory `/api/http-gateway/` prefix**: The Istio `VirtualService` for the HTTP gateway mounts under `/api/http-gateway/` and rewrites the request path to `/`. Requesting the raw path `/api/kvstore/stores/<store>/keys/<key>` returns HTTP 404 (empty body) from Envoy. Always prefix HTTP REST requests with `/api/http-gateway/` (for example, `http://istio-ingressgateway.app-ingress.svc.cluster.local:80/api/http-gateway/api/kvstore/stores/kv_store/keys/<key>`).
- **Expected `NOT_FOUND` on `"workcell_info"`**: Querying `/api/http-gateway/api/kvstore/stores/kv_store/keys/workcell_info` on a local workcell returns HTTP 404 (`{"code":5, "message":"Not Found"}`). This indicates a successfully routed request where the cloud-specific `workcell_info` key is absent on offline workcells, not an ingress failure.

## 2. Meta-level taxonomy of `gRPC` status codes on local workcells

### 2.1 Status code 12 (`UNIMPLEMENTED`) and route or method resolution

| Observed error signature | Responsible layer | Root-cause mechanism | Diagnostic verification step |
| :--- | :--- | :--- | :--- |
| `code = Unimplemented desc = ` (empty description) | Envoy `VirtualService` route matching | Ingress gateway failed to match route rules due to missing or mismatched `x-resource-instance-name` header. | Verify `--instance_name=<name>` is passed for service assets, or verify the header is omitted for core platform singletons. |
| `code = Unimplemented desc = unexpected HTTP status code ... 404` | HTTP/2 transport mapping | Client dialed a removed proto method or directed a gRPC stub to an HTTP REST route. | Verify the target service proto definition and ensure the stub address does not target an `/api/http-gateway/` path. |
| `Skill <id> has not implemented Execute` | Skill runner error-rewriting (`skill_service_impl.cc`) | Downstream gRPC call or equipment method inside `Execute` returned `kUnimplemented`, which `CreateSkillError` unconditionally wrapped as missing implementation. | Inspect container stderr immediately preceding the `CreateSkillError` log line in `skill_service_impl.cc` to identify the failing downstream RPC. |

### 2.2 Status code 14 (`UNAVAILABLE`) and endpoint discovery

- **Single-service `no healthy upstream`**:
  - Envoy Endpoint Discovery Service (EDS) has zero ready backend endpoints. The target container crashed or actively rejected the connection.
  - Run `kubectl describe pod <pod-name> -n app-intrinsic-app-chart` or `-n app-resources` to inspect container termination reasons (`SIGSEGV`, unhandled exceptions). Check all sidecar containers in multi-container pods; inference sidecars often exit with code 128 (`StartError`) while the main container logs benign wait messages.
- **Multi-service `UNAVAILABLE` and CPU quota throttling**:
  - On hybrid-architecture Industrial PCs, running multiple compute-heavy pods can exhaust performance cores and trigger Linux CFS quota throttling, starving control-plane xDS streams (`istiod`, `kube-apiserver`). Check CPU Pressure Stall Information (PSI) and runnable thread queues (`vmstat r`).
- **Downstream "Ports not open" startup gate timeouts**:
  - Startup failures reporting `Ports not open: executive.app-intrinsic-app-chart:8080` or `logger.app-intrinsic-base:8080` are downstream health timeouts. The named service delayed binding its port while waiting for upstream dependencies during synchronous initialization.
  - When the executive port is not open, check `simulation-service:8088` and `gzserver:50053` logs to verify whether scene mesh loading stalled upstream channel readiness.

### 2.3 Status code 8 (`RESOURCE_EXHAUSTED`) on channel message limits

- Default gRPC channels enforce a 4 MB (`4,194,304` bytes) receive limit. Querying `intrinsic_proto.world.ObjectWorldService` or `intrinsic_proto.geometry.GeometryService` for complex meshes returns `grpc: received message larger than max`.
- Configure explicit 64 MB channel options on client stubs:
  - **Python**: Pass `options=[('grpc.max_receive_message_length', 64 * 1024 * 1024), ('grpc.max_send_message_length', 64 * 1024 * 1024)]` to `grpc.insecure_channel`.
  - **Go**: Pass `grpc.WithDefaultCallOptions(grpc.MaxCallRecvMsgSize(64 * 1024 * 1024))` to `grpc.Dial`.
  - **C++**: Configure `grpc::ChannelArguments args; args.SetMaxReceiveMessageSize(64 * 1024 * 1024);`.

## 3. Platform `KVStore`, `Zenoh` `PubSub`, and parameter default merging

### 3.1 Stripping `"kv_store/"` key prefixes and store routing

- **Prefix stripping invariant**: When exchanging keys between external storage manifests and `intrinsic_proto.kvstore.KVStore` RPCs (`Get`, `Set`, `Delete`), strip leading `"kv_store/"` prefixes (`raw_key.removeprefix("kv_store/")`) to prevent double-prefixed lookups (`"kv_store/kv_store/..."`).
- **Store scope selection**: The local gRPC `KVStore` service targets the `"kv_store"` namespace with high-consistency polling. Accessing `"kv_store_repl"` or custom prefixed namespaces requires Zenoh PubSub/Queryables (`pubsub.KVStore()`, `pubsub.KVStoreReplicated()`, `pubsub.KVStoreWithPrefix(prefix)`).

### 3.2 Protobuf parameter default merging with `UnpackAnyAndMerge`

When custom skills execute, the skill server merges parameter defaults declared in the asset manifest into the incoming `google.protobuf.Any` parameters container using `UnpackAnyAndMerge`. Address two critical `proto3` presence traps:

1. **Scalar presence trap (overwriting explicit zero/false values)**:
   - In `proto3`, scalar fields (`bool`, `int32`, `float`, `string`) lack field presence unless declared with `optional`. If a caller passes `false` or `0`, standard `proto3` serialization omits the default value from the wire payload.
   - When `UnpackAnyAndMerge` merges manifest defaults, any non-zero or `true` manifest default overwrites the caller's explicit `0` or `false`.
   - **Negative rule 3 (paired)**: Do not omit the `optional` keyword on `proto3` scalar fields that have non-zero manifest defaults; declare them as `optional` so explicit zero or false values are preserved during `UnpackAnyAndMerge`.
2. **Ghost submessage trap (suppressing manifest default submessages)**:
   - Protobuf message fields possess explicit presence. When authoring tools serialize unpopulated submessages as empty objects (`{}`), the serialized `Any` payload contains an empty message (`has_<submessage>() == true`).
   - `UnpackAnyAndMerge` treats the empty submessage as explicitly populated and skips merging default submessage values, leading to missing required fields downstream. Strip empty top-level submessages before serializing to `Any`.
3. **Executive default merge diagnostic signature**:
   - When parameter default merging alters incoming skill parameters at runtime, the executive dispatcher explicitly logs: `Parameters for skill '<id>' were modified by merging defaults. Differences: ...`
   - Inspect executive logs for this signature to verify whether unexpected parameter values stem from manifest default merging or client-side payload serialization.

### 3.3 High-frequency `/tf` logging budget rate limiting and `TCP` probes

- **Transform logging budget and database disconnection**: High-frequency transform publishing exceeding the 26 MB flush threshold (`Current batch byte size exceeds threshold 26214400`) or database migration disconnects (`timescaledb-0`) causes the logging service to drop messages (`log exceeded logging budget and was rate limited for event source: /tf`). For external low-latency consumers, subscribe via direct local endpoints (`localhost:17080`) or direct SSH tunnels (`inctl ssh`) rather than high-overhead API relay tunnels (`inctl cluster`).
- **Zenoh router probes**: Periodic `early eof` TCP read errors logged every 1–1.5 seconds on `zenoh-router` (`7447`) originate from `SolutionService/GetStatus` liveness probes. For services sensitive to raw TCP polling, configure the `intrinsic.ai/opt-out-error-checking` annotation.

### 3.4 Deployment tier distinctions (`platform_level:core` vs. `enterprise`)

- Local workcell environments run `platform_level:core` configurations (`ObjectWorldService` worlds `["sim_world", "world", "init_world", "save_monitor"]`, Zenoh KVStore).
- **Negative rule 4 (paired)**: Do not attempt to dial cloud-only `platform_level:enterprise` endpoints on local workcell virtual machines; inspect local container logs via `inctl logs pull` or `kubectl logs`. Services such as `DataLogger`, `PubSubListener`, and `timescaledb` are not deployed on core workcells, and requests return `StatusCode.UNIMPLEMENTED`.

## 4. Summary diagnostic decision tree for ingress and platform status

Follow this input-aware decision tree (`precondition -> diagnostic check -> targeted action`) when diagnosing communication or status failures:

```
Communication or status failure observed
 ├── Precondition: gRPC status UNIMPLEMENTED (code 12)
 │    ├── Diagnostic check: Is desc empty ("")?
 │    │    └── Targeted action: Verify x-resource-instance-name header. Set for service assets; omit for core singletons.
 │    ├── Diagnostic check: Is desc "unexpected HTTP status code ... 404"?
 │    │    └── Targeted action: Verify proto method exists and client stub does not target an /api/http-gateway/ path.
 │    └── Diagnostic check: Is error "Skill <id> has not implemented Execute"?
 │         └── Targeted action: Inspect container stderr right before CreateSkillError in skill_service_impl.cc to find failing downstream RPC.
 ├── Precondition: gRPC status UNAVAILABLE (code 14)
 │    ├── Diagnostic check: Single service reports "no healthy upstream"?
 │    │    └── Targeted action: Run kubectl describe pod in target namespace; inspect container exit codes (SIGSEGV, code 128).
 │    ├── Diagnostic check: Multiple services report timeouts or UNAVAILABLE?
 │    │    └── Targeted action: Inspect node CPU PSI and vmstat r for CFS quota throttling on hybrid CPU cores.
 │    └── Diagnostic check: Solution startup reports "Ports not open"?
 │         └── Targeted action: Check upstream dependencies (e.g. simulation-service:8088 and gzserver:50053 mesh loading).
 ├── Precondition: gRPC status RESOURCE_EXHAUSTED (code 8)
 │    └── Targeted action: Configure client channel options with 64 MB limits (grpc.max_receive_message_length).
 ├── Precondition: KVStore or REST path returns HTTP 404
 │    ├── Diagnostic check: Request path is raw /api/kvstore/stores/...?
 │    │    └── Targeted action: Prepend /api/http-gateway/ to request path for Istio VirtualService URL rewriting.
 │    └── Diagnostic check: Request targets .../keys/workcell_info?
 │         └── Targeted action: Treat as expected offline state; workcell_info is cloud-only.
 └── Precondition: Skill executes with unexpected parameter values
      └── Diagnostic check: Executive logs "Parameters for skill ... were modified by merging defaults"?
           └── Targeted action: Declare scalar fields with optional to protect explicit false/0 values during UnpackAnyAndMerge.
```

## 5. `System 2` reflection and circuit breaker checkpoints

### 5.1 Pre-execution reflection checklist
Before executing state-mutating commands (such as restarting pods, modifying Kubernetes manifests, or resetting world states):
1. **Header audit**: Does the client request attach `x-resource-instance-name` only for service assets, and omit it for core singletons (`Operations`, `GeometryService`)?
2. **Schema audit**: Are all `proto3` scalar fields with non-zero manifest defaults declared with `optional` to protect `UnpackAnyAndMerge` execution?
3. **Endpoint audit**: Does the destination route through `localhost:17080` or `istio-ingressgateway` with appropriate path prefixes rather than internal DNS names?

### 5.2 Anti-thrashing circuit breaker
- **Negative rule 5 (paired)**: Do not repeat identical failing requests beyond 3 attempts when receiving `UNIMPLEMENTED` or `UNAVAILABLE`; halt further calls and branch to container status inspection.
- When an operation fails repeatedly:
  1. Inspect pod statuses and container exit codes (`kubectl get pods -A`, `kubectl describe pod <pod>`).
  2. Inspect container stderr and application startup logs (`kubectl logs <pod> -c <container>`).
  3. Verify ingress route registration (`kubectl get virtualservice -A`).
