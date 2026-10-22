---
name: intrinsic-core-api-overview
description: >-
  Intrinsic Core gRPC service selection, Envoy x-resource-instance-name routing, and progressive disclosure hub across ObjectWorldService, MotionPlannerService, ICON, cameras, KVStore, and platform APIs.
  Triggers: select Intrinsic Core API, gRPC routing, Envoy ingress, x-resource-instance-name, asset instance vs platform service, coordinate frame mutation, multi-world lifecycle, trajectory planning, real-time control, camera capture, KVStore.
  Subsystems: INGRESS, ENVOY, WORLD, MOTION_PLANNING, ICON, PERCEPTION, EXECUTIVE, KVSTORE.
  Dependencies: localhost:17080, grpc, intrinsic.util.grpc, ObjectWorldClient, MotionPlannerClient, icon_api.
  Anti-keywords: inctl service create, inctl skill create, inctl solution list, kubectl scale, legacy WORKSPACE.
---

# Intrinsic Core API overview and progressive disclosure hub

## Progressive disclosure reference hub

Read the domain reference under `references/` matching your task before writing gRPC or Python SDK code:

| Reference guide | Domain and gRPC / SDK surface | When to read it |
| :--- | :--- | :--- |
| [references/assets-and-solutions.md](references/assets-and-solutions.md) | `InstalledAssets`, `AssetDeploymentService`, `intrinsic_proto.storage.StorageService`, `ArtifactCatalogService`, `intrinsic_proto.solution.v1.SolutionService` | Communicate with an Intrinsic asset vs. an Intrinsic Core platform service, install assets, or manage solution lifecycles. |
| [references/world-and-kinematics.md](references/world-and-kinematics.md) | `ObjectWorldService` (`intrinsic_proto.world.ObjectWorldService/CloneWorld`), `intrinsic_proto.kinematics.KinematicsService`, `FrameCalibrationService`, `Skeleton` | Read or mutate digital twin frames/transforms (`node_to_update`, `ReparentObject`). Manage initial vs. belief vs. simulation worlds. |
| [references/geometry-and-math.md](references/geometry-and-math.md) | `intrinsic_proto.geometry.GeometryService`, `Pose3`, `Rotation3`, `Octree`, `OrientedBoundingBox`, `Renderable` | Constructing spatial poses (`Pose3(rotation=..., translation=...)`), converting quaternions, registering meshes, and collision footprints. |
| [references/motion-planning-and-icon.md](references/motion-planning-and-icon.md) | `MotionPlannerService` (`MotionPlannerClient`), `IconApi` (`icon_api.Client`) | Move robot in real time or execute trajectory. Plan collision-free path or IK, settling (`intrinsic.is_settled`). |
| [references/perception-and-hardware.md](references/perception-and-hardware.md) | `CameraService`, `intrinsic_proto.perception.v1.PoseEstimationService`, `CalibrationService`, `intrinsic_proto.gripper.GenericGripper`, `GPIOService` | Capture camera images or estimate poses (`grpc.max_receive_message_length`), optical frames (`camera_t_target`), calibration, and gripper/GPIO. |
| [references/longrunning-operations.md](references/longrunning-operations.md) | `google.longrunning.Operations`, central LRO proxy, `intrinsic_proto.executive.ExecutiveService`, `intrinsic_proto.conductor.ConductorService` | Polling long-running operations (`WaitOperation` vs. `GetOperation`), handling LRO proxy routing, and inspecting executive operations. |
| [references/platform-logging-and-status.md](references/platform-logging-and-status.md) | `intrinsic_proto.logging.DataLogger`, `BagPackager`, `KVStore`, `SimulationService`, `ExtendedStatus` | Share state via KV store or executive blackboard, emitting structured logs, local telemetry, and simulation resets. |
| [../intrinsic-core-bazel/SKILL.md](../intrinsic-core-bazel/SKILL.md) | Bzlmod module configuration, canonical SDK dependencies (`@ai_intrinsic_sdks`), protobuf targets, and hermetic build rules. | Declaring dependencies for any Intrinsic gRPC service or SDK module in BUILD files. |

## How to communicate with an Intrinsic asset vs. an Intrinsic Core platform service

External and inter-service gRPC traffic routes through the Envoy ingress gateway (`localhost:17080` externally or `istio-ingressgateway.app-ingress.svc.cluster.local:80` inside Kubernetes pods). Envoy uses two distinct routing mechanisms depending on target multiplicity:

| Dimension | Intrinsic asset (service instance) | Intrinsic Core platform service (workcell singleton) |
| :--- | :--- | :--- |
| **Examples** | Custom microservices, camera drivers (`basler_camera`), grippers, hardware modules (`ur_module`, `icon`). | `ObjectWorldService`, `MotionPlannerService`, `ExecutiveService`, `Operations`, `SystemServiceState`. |
| **Envoy routing key** | Requires **`x-resource-instance-name: <instance_name>`** metadata header on every RPC. | Routed by **gRPC URI path prefix** (`/intrinsic_proto.<pkg>.<Service>/<Method>`). Omit instance header. |
| **Connection mode** | Use `connection.ConnectionParams(address=..., instance_name="<name>", header="x-resource-instance-name")`. | Plain `grpc.insecure_channel(address)` passed directly to the generated gRPC service stub. |
| **Inside `Skill.execute()`** | Read `context.resource_handles["<slot>"]` and extract `handle.connection_info.grpc`. | Access `context.object_world` and `context.motion_planner` directly (`context.grpc_channel` does not exist). |
| **Inside `Skill.preview()`** | Hardware handles not leased; use `context.get_object_for_equipment("<slot>")`. | `context.object_world` is read-only; record speculative changes via `context.record_world_update(...)`. |

```python
import os
from typing import Any
import grpc
from intrinsic.util.grpc import connection, interceptor
from intrinsic.world.proto import object_world_service_pb2_grpc
from intrinsic.world.python import object_world_client

_CHANNEL_OPTIONS = [("grpc.max_receive_message_length", -1)]


def resolve_workcell_ingress_address(default_address: str = "localhost:17080") -> str:
  """Resolves in-cluster Envoy ingress DNS when executing inside a Kubernetes pod."""
  if "KUBERNETES_SERVICE_HOST" in os.environ:
    return "istio-ingressgateway.app-ingress.svc.cluster.local:80"
  return default_address


def connect_to_asset_instance(
    instance_name: str, address: str | None = None
) -> grpc.Channel:
  """Creates a gRPC channel routed to a specific named Intrinsic asset instance."""
  params = connection.ConnectionParams(
      address=address or resolve_workcell_ingress_address(),
      instance_name=instance_name,
      header="x-resource-instance-name",
  )
  base = grpc.insecure_channel(params.address, options=_CHANNEL_OPTIONS)
  return grpc.intercept_channel(
      base, interceptor.HeaderAdderInterceptor(params.headers)
  )


def create_channel_from_resource_handle(handle: Any) -> grpc.Channel:
  """Extracts connection_info.grpc from a leased ResourceHandle message."""
  info = handle.connection_info.grpc
  return connect_to_asset_instance(
      instance_name=info.server_instance,
      address=info.address or resolve_workcell_ingress_address(),
  )


def connect_to_platform_world_service(
    address: str | None = None, world_id: str = "world"
) -> object_world_client.ObjectWorldClient:
  """Connects directly to the singleton ObjectWorldService without instance headers."""
  channel = grpc.insecure_channel(
      address or resolve_workcell_ingress_address(), options=_CHANNEL_OPTIONS
  )
  stub = object_world_service_pb2_grpc.ObjectWorldServiceStub(channel)
  return object_world_client.ObjectWorldClient(world_id=world_id, stub=stub)
```

## Paired safety guardrails

1. **Envoy ingress routing headers**: Attach `x-resource-instance-name: <instance_name>` metadata header when calling deployed asset instances (e.g. `icon`, `basler_camera`, custom services); do not pass `x-resource-instance-name` to core platform runtime services (`ObjectWorldService`, `GeometryService`, `ExecutiveService`, `Operations`), as mismatched instance headers cause Istio to reject requests with `UNIMPLEMENTED`.
2. **Context handle access across execution phases**: Read leased equipment connections via `context.resource_handles["<slot>"]` strictly within `Skill.execute()`, and query proxy objects via `context.get_object_for_equipment("<slot>")` in `Skill.preview()` (do not attempt to access `context.resource_handles` during `preview()`, nor access non-existent `context.grpc_channel`).
3. **Spatial pose construction parameter ordering**: Supply explicit keyword arguments when instantiating `Pose3(rotation=..., translation=...)` (do not pass positional arguments, which silently map translation vectors to rotation quaternions and cause `ValueError: Quaternion is not normalized`).
4. **Multi-world transformation scoping and cleanup**: Specify `node_to_update` when invoking `update_transform` across non-neighboring nodes, and manage speculative worlds in a `try...finally` block with `DeleteWorld` (do not pass custom strings to `cloned_world_id` on `CloneWorld`; supply `cloned_world_hint="sandbox"` and let the server generate the ID).
5. **CLI addressing and subcommand selection**: Supply `--address=localhost:17080` for commands (run `inctl service state list --address=localhost:17080` and `inctl asset instance list --address=localhost:17080`; explore options via `inctl help` and `inctl --help`); do not run un-scoped service or solution commands without state flags.

## Diagnostic decision tree for Intrinsic Core API workflows

```
[gRPC / CLI error during Intrinsic Core API workflow]
  │
  ├─► [Symptom: rpc error: code = Unimplemented desc = (empty description)]
  │     └─► Cause: Missing x-resource-instance-name header on asset instance or invalid header on platform service.
  │     └─► Action: Add x-resource-instance-name for asset instances, or omit header for core platform services.
  │
  ├─► [Symptom: rpc error: code = Unavailable desc = no healthy upstream]
  │     └─► Cause: Target container crashed (SIGSEGV), inference sidecar exited (128), or CFS CPU throttling.
  │     └─► Action: Check kubectl describe pod in app-intrinsic-app-chart; verify upstream service dependencies.
  │
  ├─► [Symptom: grpc: received message larger than max (X vs 4194304)]
  │     └─► Cause: Mesh or point cloud transfer exceeded default 4 MB gRPC receive message ceiling.
  │     └─► Action: Configure channel with [("grpc.max_receive_message_length", -1)] or 64 MB limit.
  │
  ├─► [Symptom: KeyError: 'resource_handles' or AttributeError during Skill.preview()]
  │     └─► Cause: Attempting to access live hardware resource handles during preview phase.
  │     └─► Action: Use context.get_object_for_equipment("<slot>") and record updates via context.record_world_update().
  │
  ├─► [Symptom: INVALID_ARGUMENT: node_to_update is required when updating non-neighboring nodes]
  │     └─► Cause: update_transform invoked across multi-hop transform path without disambiguating target node.
  │     └─► Action: Pass node_to_update=target_node explicitly to update_transform().
  │
  ├─► [Symptom: HTTP 404 Not Found on REST gateway /api/kvstore/stores/...]
  │     └─► Cause: Envoy gateway missing mandatory /api/http-gateway/ path prefix rewrite.
  │     └─► Action: Prepend /api/http-gateway to the request path (/api/http-gateway/api/kvstore/...).
  │
  └─► [Symptom: failed to upload image: failed to dial "/run/containerd/containerd.sock": connect: connection refused]
        └─► Cause: Backend artifacts-deployment pod in k3s cannot reach host containerd socket.
        └─► Action: Cap at <= 2 attempts; do not inspect local /run or proxy socket; verify build/test hermetically with bazel.
```

## System 2 reflection and circuit breaker checkpoints

- **Pre-execution reflection**: Before calling APIs or authoring code, verify: (1) routing target (`x-resource-instance-name` header for assets vs. direct URI path for singletons); (2) execution phase (`resource_handles` in `execute()` vs. `get_object_for_equipment` in `preview()`); (3) message length (`options=[("grpc.max_receive_message_length", -1)]`); (4) explicit keyword arguments on `Pose3(rotation=..., translation=...)`.
- **Anti-thrashing circuit breaker**: Cap operation/status polling at <= 3 retries (backoff: 1s, 2s, 4s). If receiving repeated empty `UNIMPLEMENTED` or `UNAVAILABLE: no healthy upstream`, halt redialing; inspect Envoy routing headers and pod container exit codes (`kubectl describe pod`) instead.
- **Backend containerd socket and OCI upload circuit breaker**: Cap asset installation retries at <= 2 attempts. If `inctl asset install` or sideloading fails with containerd socket connection refused (`/run/containerd/containerd.sock` or `localhost:17127`), halt cluster upload attempts immediately. The failure is on the backend `k3s` daemon on the cluster host, NOT in the local client environment (e.g. Bubblewrap sandbox). Do NOT search for sockets with `find /run`, inspect local `/run` sockets, or attempt to proxy `/run/containerd/containerd.sock`. Verify hermetically with `bazel build //...` and `bazel test //...`.

## Verification criteria

- [ ] **Ingress routing verified**: Attached `x-resource-instance-name` header for asset instances; omitted header for platform services.
- [ ] **Clean operation polling**: Polled `google.longrunning.Operations` on a headerless channel without leaking instance names.
- [ ] **Phase-appropriate context**: Read `context.resource_handles` strictly in `execute()`; used `get_object_for_equipment` in `preview()`.
- [ ] **Payload options set**: Configured `grpc.max_receive_message_length = -1` for large perception or geometry transfers.
- [ ] **Multi-world safety**: Managed cloned worlds inside `try...finally` with `DeleteWorld`; passed `node_to_update` on non-neighbor transforms.
