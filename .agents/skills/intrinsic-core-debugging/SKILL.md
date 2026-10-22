---
name: intrinsic-core-debugging
description: >-
  Meta-level debugging workflows, architectural layer isolation, and progressive disclosure routing across Envoy ingress,
  Kubernetes pods, Behavior Trees, ObjectWorld synchronization, ICON real-time control, and hardware modules.
  Triggers: debug Intrinsic Core failures, gRPC UNIMPLEMENTED, UNAVAILABLE no healthy upstream, Ports not open, Behavior Tree stalls, ICON overruns, multi-world desynchronization, container crashes.
  Subsystems: INGRESS, ENVOY, KUBERNETES, EXECUTIVE, WORLD, MOTION_PLANNING, ICON, PERCEPTION.
  Dependencies: localhost:17080, inctl, kubectl, sudo coredumpctl, grpc.
  Anti-keywords: inctl service create, inctl skill create, inctl solution list, legacy WORKSPACE, cloud deployments.
---

# Intrinsic Core meta-debugging workflow and component routing

## Architectural triage hierarchy and layer isolation

When an anomaly surfaces in an Intrinsic workcell, isolate the originating architectural layer before inspecting application code:

1. **Ingress and transport layer (`localhost:17080` / Envoy `VirtualService`)**: External and inter-pod gRPC traffic routes through Envoy. Named asset instances (`icon`, `ur_module`, camera drivers) require metadata header `x-resource-instance-name: <name>`. Core platform singletons (`ObjectWorldService`, `ExecutiveService`, `GeometryService`, `Operations`) route by URI prefix and must omit the instance header (or set `exact: intrinsic_runtime`).
2. **Kubernetes control plane and container readiness layer (`app-intrinsic-base`, `app-resources`, `skills`)**: Services initialize upstream channels synchronously before opening serving ports (`Ports not open`). Multi-container pods (e.g. `skills`, `ml-models-service`) can have healthy primary containers alongside crashed inference sidecars (`Exit Code 128`). Inspect exit codes (137 OOMKilled, 139 SIGSEGV) with `kubectl describe pod` and crashes via `sudo coredumpctl`.
3. **Executive and Behavior Tree orchestration layer (`ExecutiveService`)**: Enforces two-phase operation lifecycle (`CreateOperation` to stage, `StartOperation` to execute). Manages CEL blackboard scoping, default parameter merging, and parallel footprint locks.
4. **Digital twin and multi-world synchronization layer (`ObjectWorldService`)**: Separates static scene (`"init_world"`), runtime belief state (`"world"`), and simulation (`"sim_world"`). Preview runs pause belief tracking; reset via `inctl world reset --address=localhost:17080`.
5. **Motion planning and real-time control layer (`MotionPlannerService` & `rs-icon`)**: Collision checking over BVH trees (`CoalCollisionChecker`). Hard real-time loop across four phases (`rs`, `proc`, `ac`, `exec`) over shared memory futexes.

## Progressive disclosure reference hub

Read the domain reference guide under `references/` matching the failing subsystem before applying remediation:

| Reference guide | Architectural domain and diagnostic focus | When to read it |
| :--- | :--- | :--- |
| [references/assets-and-solutions.md](references/assets-and-solutions.md) | `ServiceManifest`, `.binpb` configs, OCI registry uploads, `Ports not open`, GPU slicing. | Asset installation errors, wire-format parsing errors, or `Pending`/`CrashLoopBackOff` pods. |
| [references/executive-and-behavior-trees.md](references/executive-and-behavior-trees.md) | Two-phase lifecycle, CEL expressions, blackboard bindings, protobuf 100-recursion limit. | Behavior Tree stalls, CEL `params` variable errors, recovery subtree matching, or tree bloat. |
| [references/geometry-and-math.md](references/geometry-and-math.md) | `GeometryService`, `Pose3` keyword args, antipodal quaternions, unscaled millimeter CAD meshes. | Missing geometry refs, non-normalized quaternions, pose comparison errors, or slow planning. |
| [references/longrunning-operations.md](references/longrunning-operations.md) | `google.longrunning.Operations`, central `operations:8080` proxy, header forwarding, `WaitOperation`. | `NOT_FOUND` on LRO polling, operation cancellation stalls, or missing asset metadata. |
| [references/motion-planning-and-icon.md](references/motion-planning-and-icon.md) | `MotionPlannerClient`, IK diagnostics, 2π flips, ICON four-phase cycle (`rs`, `proc`, `ac`, `exec`). | `compute_ik` failures, ICON cycle overruns (`exec` > 95%), `AlreadyExistsError`, or hardware faults. |
| [references/perception-and-vision.md](references/perception-and-vision.md) | GenICam state machines, Jumbo Frames (9000 MTU), 6DoF pose estimation, Zenoh KV store buffers. | Camera register write locks, dropped video frames, GPU inference reload spikes, or calibration. |
| [references/platform-logging-and-status.md](references/platform-logging-and-status.md) | Envoy routing, `/api/http-gateway/` REST prefix, gRPC status taxonomy, 4 MB payload ceilings. | Empty `UNIMPLEMENTED`, HTTP 404 on KV store, 4 MB `RESOURCE_EXHAUSTED`, or logging sync. |
| [references/world-and-kinematics.md](references/world-and-kinematics.md) | `ObjectWorldService` multi-world instances (`"init_world"`, `"world"`, `"sim_world"`), Gazebo lockstep. | Stale digital twin poses, world updater paused, robot simulation oscillations, or SDF drift. |
| [references/bazel.md](references/bazel.md) (and [intrinsic-core-bazel](../intrinsic-core-bazel/SKILL.md)) | Bazel build/test errors, Bzlmod module resolution, pip lockfile updates, and 0-0-2 circuit breakers. | Module not found, missing `@ai_intrinsic_sdks`, requirements lockfile errors, or linker ABI crashes. |

## Core `CLI` and incident investigation tools

Execute CLI and Linux inspection commands when triaging a local workcell (`--address=localhost:17080`):

| Investigation task | Command syntax | Operational contract |
| :--- | :--- | :--- |
| **Discover subcommands** | `inctl help` | Run `inctl help` (not `inctl --help`, which only prints Go logging flags). |
| **Inspect service states** | `inctl service state list --address=localhost:17080 --output=json` | Output JSON reveals raw `STATE_CODE_ERROR` masked by tabular summary strings. |
| **Map running assets** | `inctl asset instance list --address=localhost:17080` | Maps active resource instance names (e.g. `icon`, `ur_module`) to asset IDs. |
| **Inspect `ICON` status** | `inctl icon status --instance_name=icon --address=localhost:17080` | `--instance_name=icon` is mandatory for Envoy `x-resource-instance-name` routing. |
| **Clear hardware faults** | `inctl icon clear-faults --instance_name=icon --address=localhost:17080` | Resets module faults safely without tearing down shared-memory futexes. |
| **Reset multi-world state** | `inctl world reset --address=localhost:17080` | Synchronizes belief/sim worlds to `"init_world"` and unpauses frozen world updater. |
| **Stage vs. run process** | `inctl process set --server=localhost:17080` | Note `--server` flag; only stages `CreateOperation`. Call `StartOperation` to run. |
| **Container status / crash** | `kubectl describe pod <pod> -n <namespace>` | Exposes exit codes (128 start error, 137 OOMKilled, 139 SIGSEGV) across sidecars. |
| **Targeted log filtering** | `kubectl logs -n <namespace> <pod> -c <container> --tail=200` | Filters logs by specific container. |
| **Core dump analysis** | `sudo coredumpctl list` and `sudo coredumpctl info <pid>` | Inspects process terminations and stack traces from crashed C++ binaries. |

## Paired safety guardrails

1. **Ingress metadata routing headers**: Attach `x-resource-instance-name: <name>` when invoking named asset instances (e.g., `--instance_name=icon` on `inctl icon`), and omit it when querying core platform singletons (`ObjectWorldService`, `MotionPlannerService`, `ExecutiveService`, `Operations`); do not supply asset instance headers to platform services, which causes Envoy to return an empty `UNIMPLEMENTED` status code.
2. **Command-line flag syntax and discovery**: Run `inctl help` or `inctl <subcommand> --help` to discover subcommands, supply `--address=localhost:17080` for asset/service/icon/world subcommands, and use `--server=localhost:17080` for process commands (do not run `inctl --help`, which intercepts Go logging flags without listing subcommands, nor pass `--address` to `inctl process`).
3. **Behavior Tree execution lifecycle**: Invoke `StartOperation` or call `executive.run()` to start staged operations (do not assume `inctl process set` executes the tree; it only stages the definition via `CreateOperation`).
4. **Hardware module fault remediation**: Clear controller and hardware faults via `inctl icon clear-faults --instance_name=icon --address=localhost:17080` after physical safety interlocks are cleared (do not restart hardware module pods directly, which destroys shared memory segments and futexes while controller processes remain attached).
5. **Service configuration wire format**: Serialize configuration protobufs inside `google.protobuf.Any` and supply binary `.binpb` files to `inctl service add --config=<path>` (do not pass `.textproto` files, which fail with invalid wire-format parsing errors).

## Diagnostic decision tree for Intrinsic Core failure modes

```
[Anomaly detected on workcell]
  │
  ├─► [Symptom: rpc error: code = Unimplemented desc = (empty description)]
  │     ├─► Precondition: Client issued gRPC request or inctl command across localhost:17080.
  │     ├─► Diagnostic check: Inspect x-resource-instance-name header against VirtualService routes.
  │     └─► Targeted action: Supply --instance_name=<name> for asset instances, or omit header for core platform services.
  │
  ├─► [Symptom: rpc error: code = Unavailable desc = no healthy upstream]
  │     ├─► Precondition: VirtualService route matched, but target pod or sidecar is not serving.
  │     ├─► Diagnostic check: Run kubectl describe pod <pod> -n <ns>; check container exit codes (128, 137, 139).
  │     └─► Targeted action: Inspect crashed container logs or run sudo coredumpctl info for SIGSEGV tracebacks.
  │
  ├─► [Symptom: Ports not open: <service>.<namespace>:8080]
  │     ├─► Precondition: Solution startup health gate timed out before target service bound port 8080.
  │     ├─► Diagnostic check: Trace synchronous upstream gRPC channels (e.g. executive -> simulation_service:8088 -> gzserver).
  │     └─► Targeted action: Resolve upstream dependency stall (e.g. Gazebo mesh loading) before checking target container.
  │
  ├─► [Symptom: Behavior Tree staged via inctl process set does not execute]
  │     ├─► Precondition: Tree staged in ExecutiveService via CreateOperation.
  │     ├─► Diagnostic check: Verify operation state via ExecutiveService/ListOperations.
  │     └─► Targeted action: Issue ExecutiveService/StartOperation or call executive.run() to trigger execution.
  │
  ├─► [Symptom: ICON control loop overrun: Long duration between read_status_calls]
  │     ├─► Precondition: Real-time loop (500 Hz / 1 kHz) missed deadline on isolated real-time core.
  │     ├─► Diagnostic check: Inspect rs-icon logs; compare futex sleep (exec) vs. computation (rs + proc + ac).
  │     └─► Targeted action: If exec > 95%, resolve host CPU preemption, E-core scheduling, or CFS quota throttling.
  │
  ├─► [Symptom: Gazebo simulation ignores world edits or live robot poses freeze]
  │     ├─► Precondition: Behavior Tree executed in simulation preview or multi-world desynchronized.
  │     ├─► Diagnostic check: Run inctl world reset --address=localhost:17080 or inspect world updater paused state.
  │     └─► Targeted action: Reset worlds via inctl world reset or supply start_from_world_state=worlds.EditWorldId.BELIEF.
  │
  ├─► [Symptom: failed to upload image: failed to dial "/run/containerd/containerd.sock": connect: connection refused]
  │     ├─► Precondition: inctl asset install or sideloading attempted across localhost:17127 / localhost:17080.
  │     ├─► Diagnostic check: Confirm failure is on backend k3s host daemon, NOT in local client environment.
  │     └─► Targeted action: Stop after at most 2 attempts. Never debug local /run or proxy sockets; verify hermetically via bazel build and bazel test.
  │
  └─► [Symptom: grpc: received message larger than max (X vs. 4194304)]
        ├─► Precondition: Mesh, point cloud, or complex BehaviorTree transferred across default channel.
        ├─► Diagnostic check: Check transfer payload size against default 4 MB ceiling.
        └─► Targeted action: Configure channel options with [("grpc.max_receive_message_length", -1)].
```

## `System 2` reflection and circuit breaker checkpoints

### Pre-execution reflection checkpoint
Before executing mutations or restart sequences on a failing workcell:
1. **Identify fault layer**: Determine whether failure originates in ingress, container readiness, Executive, `ObjectWorld`, or `ICON`.
2. **Verify routing headers**: Ensure `x-resource-instance-name` is attached only for asset instances and omitted for core singletons.
3. **Verify container exit states**: Run `kubectl describe pod` to distinguish OOM (`137`), crash (`139`), or sidecar failure (`128`).
4. **Verify world synchronization**: Confirm `"world"` and `"sim_world"` are aligned before diagnosing motion planning errors.

### Anti-thrashing circuit breaker
- **Retry budget**: Limit diagnostic queries and retries to <= 3 attempts with exponential backoff (1s, 2s, 4s).
- **Trip condition**: If three successive checks fail with identical status codes (`UNAVAILABLE` or `UNIMPLEMENTED`), halt command execution. Escalate from application logic to upstream dependency inspection (`Ports not open` dependency chains or host CPU CFS throttling).
- **Backend containerd socket and OCI upload circuit breaker**: When `inctl asset install` or sideloading fails with `failed to dial "/run/containerd/containerd.sock": connect: connection refused` or connection refused on `localhost:17127`, cap retries at <= 2 attempts. The containerd socket and mount reside in `k3s` on the cluster host, NOT in the local client environment (e.g. Bubblewrap sandbox). Do NOT run `find /run` or attempt to proxy `/run/containerd/containerd.sock`. Immediately halt cluster upload attempts and fall back to verifying hermetic Bazel build and test targets locally (`bazel build //...`, `bazel test //...`).

## Verification criteria

- [ ] **Layer isolated**: Identified originating fault layer using the five-tier architectural triage hierarchy.
- [ ] **Ingress headers verified**: Applied `x-resource-instance-name` for asset instances and omitted for platform singletons.
- [ ] **Container health checked**: Verified pod exit codes and core dumps with `kubectl describe pod` and `sudo coredumpctl`.
- [ ] **Deterministic CLI used**: Supplied `--address=localhost:17080` for asset/service/icon/world and `--server=localhost:17080` for process.
- [ ] **multi-world synchronized**: Verified `"world"` and `"sim_world"` alignment via `inctl world reset` before debugging motion.
- [ ] **Circuit breaker respected**: Halted repetitive action loops after 3 attempts and inspected upstream dependencies.
