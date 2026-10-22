---
name: intrinsic-core-solutions
description: >-
  Intrinsic Core solution lifecycle orchestration, two-step service sideloading (inctl asset install + inctl service add),
  service state inspection, and ICON controller management. Triggers on solution start/stop, service sideloading,
  State: Error recovery, or ICON faults. Target subsystems: SOLUTION, SERVICES, ICON. Dependencies: inctl, localhost:17080.
  Anti-keywords: bzl, BUILD, code-editing, direct docker commands.
---

# Managing Intrinsic Core solutions

## Architecture of an `Intrinsic` solution

An Intrinsic **Solution** is the top-level deployable robotic application running on the workcell PC. It orchestrates:
1. **Services**: Microservice runtime containers (hardware drivers, perception models, kinematic solvers).
2. **World model**: Geometric digital twin and scene graphs (`ObjectWorld` via `intrinsic_proto.world.ObjectWorldService`).
3. **Executive**: Behavior tree orchestrator dispatching skills to accomplish task goals (`intrinsic_proto.executive.ExecutiveService`).

## Two-step service sideloading workflow

Deploying custom microservices follows a strict two-step **class vs. instance** lifecycle model:

### Step 1: install the service asset class
Installing a `.bundle.tar` archive registers the reusable service **type/class** in the catalog. Bundles must include a compiled `FileDescriptorSet` (via `--file_descriptor_set` during bundling). No container runs at this stage:
```bash
inctl asset install /path/to/my_service_bundle.tar --address=localhost:17080
```

> [!IMPORTANT]
> **Backend containerd socket and OCI upload circuit breaker**:
> If `inctl asset install` fails with `failed to dial "/run/containerd/containerd.sock": connect: connection refused` or connection refused on `localhost:17127`, stop after at most 2 attempts. The containerd socket and its mount reside inside `k3s` on the cluster host, NOT in the local client environment (e.g. Bubblewrap sandbox). Do NOT search for sockets via `find /run` or attempt to proxy unix domain sockets. Proceed directly to verify the service implementation hermetically using local Bazel build and test commands (`bazel build //...`, `bazel test //...`; see [intrinsic-core-bazel](../intrinsic-core-bazel/SKILL.md)).

### Step 2: instantiate service runtime containers
Instantiate named runtime containers from the registered asset class. Serialize configuration protos into binary `.binpb` format using `google.protobuf.Any` before passing to `--config` (do not pass `.textproto` files; serialize configuration protos to binary `.binpb` wire format instead):
```bash
inctl service add "<package.service_name>" --name=<instance_name> --config=/path/to/my_config.binpb --address=localhost:17080
```
> [!NOTE]
> A single installed asset class (e.g. `perception.camera_driver`) can be instantiated multiple times (`--name=wrist_cam`, `--name=overhead_cam`), each with a distinct `.binpb` configuration specifying its hardware address.

## Solution lifecycle and state verification

All local control-plane operations target `--address=localhost:17080`. Run `inctl help` or `inctl <subcommand> --help` to inspect commands (do not run `inctl --help`, which is intercepted by Go logging flags; run `inctl help` instead):

### Inspecting active solution and service states
Inspect running services and catalog assets using dedicated inspection subcommands:
```bash
# List all instantiated solution assets (services, hardware, scene objects):
inctl asset instance list --address=localhost:17080

# Inspect runtime service states (prints human-readable State: Enabled):
inctl service state list --address=localhost:17080

# Inspect raw gRPC status enums (STATE_CODE_ENABLED, STATE_CODE_ERROR):
inctl service state list --address=localhost:17080 --output=json
```

### Starting and stopping solutions
Deploy or terminate versioned solutions through the lifecycle manager. Stop active solutions before switching execution modes (e.g. transitioning from simulation `sim` to physical hardware `real`):
```bash
# Stop the currently running solution:
inctl solution stop --address=localhost:17080

# Verify clean stopped state (service state list returns empty or no running services):
inctl service state list --address=localhost:17080

# Start a versioned solution:
inctl solution start <solution_id> --address=localhost:17080
```

### Managing the real-time controller and hardware locks
Core hardware modules (`ur_module`), simulator bridges (`gazebo_simulator`), and the real-time controller (`icon`) are managed by the workcell infrastructure (do not attempt `inctl service state disable icon`; manage real-time controllers via `inctl icon` instead). Always pass `--instance_name=icon` when calling `inctl icon` against `--address=localhost:17080` (omitting `--instance_name` returns `rpc error: code = Unimplemented`):
```bash
# Inspect ICON controller operational status:
inctl icon status --instance_name=icon --address=localhost:17080

# Enable or disable the real-time controller:
inctl icon enable --instance_name=icon --address=localhost:17080
inctl icon disable --instance_name=icon --address=localhost:17080

# Clear safety and hardware faults (remove stale lockfile first if container crashed):
rm -f /tmp/intrinsic_icon/ur_module.lock
inctl icon clear-faults --instance_name=icon --address=localhost:17080
```

## Diagnostic decision tree for deployment failures

```
[Issue detected during solution operation]
  │
  ├─► [Symptom: inctl icon returns "rpc error: code = Unimplemented"]
  │     └─► Cause: Ingress missing x-resource-instance-name header.
  │     └─► Action: Append --instance_name=icon to the inctl icon invocation.
  │
  ├─► [Symptom: inctl service add fails with "cannot parse invalid wire-format data"]
  │     └─► Cause: Configuration proto was supplied as human-readable textproto.
  │     └─► Action: Pack config into google.protobuf.Any and serialize to binary .binpb.
  │
  ├─► [Symptom: Startup fails with "Ports not open: executive...:8080"]
  │     └─► Cause: Executive is blocked waiting on an upstream dependency channel.
  │     └─► Action: Check simulation-service (port 8088) and gzserver (port 50053) mesh loading.
  │
  ├─► [Symptom: ur_module or icon remains in State: Faulted / Error after crash]
  │     └─► Cause: Stale shared memory lockfile on host filesystem.
  │     └─► Action: rm -f /tmp/intrinsic_icon/ur_module.lock && inctl icon clear-faults --instance_name=icon.
  │
  ├─► [Symptom: inctl asset install fails with failed to dial "/run/containerd/containerd.sock": connection refused]
  │     └─► Cause: Backend artifacts-deployment pod in k3s cannot reach host containerd socket.
  │     └─► Action: Cap at <= 2 attempts; do not inspect local /run; verify build/test hermetically with bazel.
  │
  ├─► [Symptom: Pods remain permanently Pending with Insufficient nvidia.com/gpu]
  │     └─► Cause: GPU time-slicing replica limit reached across vision/simulation containers.
  │     └─► Action: Reduce concurrent camera/inference pods or inspect inctl service logs.
  │
  └─► [Symptom: Solution redeploy hangs or fails with missing skills]
        └─► Cause: Asynchronous namespace termination race (0/0 pods ready evaluated as Ready).
        └─► Action: Execute inctl solution stop and wait for terminating pods to clear before starting.
```

## `System 2` reflection and circuit breaker checkpoints

### Pre-mutation reflection checkpoint
Before executing state-mutating commands (`inctl solution start`, `inctl solution stop`, `inctl service add`, or `inctl icon clear-faults`):
1. Verify the current active state via `inctl service state list --address=localhost:17080`.
2. Confirm the target configuration wire format (`.binpb`) and instance flags (`--instance_name=icon`).
3. Ensure no asynchronous namespace termination is pending via `inctl service state list --address=localhost:17080`.

### Anti-thrashing circuit breaker
- **Verification retry budget**: Cap service health polling at 3 attempts with a 5-second backoff.
- **Trip condition**: If a service remains in `STATE_CODE_ERROR` or a pod fails readiness after 3 checks, halt repeated restarts. Branch immediately to upstream dependency inspection or ICON fault diagnosis per the decision tree above.
- **Backend containerd socket and OCI upload circuit breaker**: Cap asset installation retries at <= 2 attempts. If `inctl asset install` fails with containerd socket connection refused (`/run/containerd/containerd.sock` or `localhost:17127`), halt cluster upload attempts immediately. Recognize this is a backend `k3s` daemon failure, not a local sandbox issue. Verify package hermetically with `bazel build //...` and `bazel test //...`.

## Verification criteria

- [ ] **Two-step sideloading verified**: Asset class is registered via `inctl asset install` before runtime containers are instantiated via `inctl service add --config=config.binpb`, or if cluster installation encounters backend containerd socket failures (`/run/containerd/containerd.sock` connection refused), hermetic local build and test via Bazel (`bazel build //...`, `bazel test //...`) succeed.
- [ ] **Service health verified**: Active services report `STATE_CODE_ENABLED` via `inctl service state list --address=localhost:17080 --output=json`.
- [ ] **Controller status verified**: Real-time controller reports `Operational Status: ENABLED` via `inctl icon status --instance_name=icon --address=localhost:17080`.
