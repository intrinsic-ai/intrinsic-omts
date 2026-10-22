---
name: intrinsic-core-concepts
description: >-
  Intrinsic Core zero-cloud architecture, core primitives (Assets, Services, Skills, Solutions, ICON), CLI inspection commands, and workspace search rules.
  Triggers: query workcell state, understand Intrinsic Core concepts, inspect running services or assets, search Bazel workspace.
  Subsystems: ICON, Executive, Asset/Solution catalog, Istio ingress gateway.
  Log/input dependencies: none (uses local port 17080).
  Anti-keywords: cloud deployment, GCP projects, remote orchestration, behavior tree CEL authoring, robot motion planning algorithms.
---

# Intrinsic Core concepts and architecture

## Overview

Intrinsic Core is an on-premise industrial robotics operating platform. Intrinsic Core executes completely on-premise on an industrial PC (IPC) without external cloud services, telemetry relays, or internet connectivity.

## Zero-cloud execution model

When developing or executing code on Intrinsic Core, adhere to the zero-cloud paradigm:
- **On-premise execution**: Authentication, configuration, and execution are completely self-contained within the local workcell.
- **Direct local CLI addressing**: Target the local cluster gateway directly via `--address=localhost:17080` (or `--server=localhost:17080` for `inctl process`).
- **Direct gRPC ingress**: All platform services are exposed through a local Istio/Envoy gateway listening on port `17080`.
- **Paired negative guardrails (sparse safety limits)**:
  1. Do not run `inctl solution list` or `inctl solution get` with `--address=localhost:17080` (cloud-only commands); run `inctl service state list --address=localhost:17080` and `inctl asset instance list --address=localhost:17080` to inspect local solutions instead.
  2. Do not handcraft skill boilerplate without CLI tooling; scaffold skills using `inctl skill create <skill_id> --proto_package=<pkg>` instead.
  3. Do not omit `--instance_name=icon` when running `inctl icon` subcommands over port 17080; always provide `--instance_name=icon --address=localhost:17080` to route through the Istio ingress gateway.
  4. Do not run unfiltered `find .` or `grep -rn .` in the workspace or from root `/` (hangs on Bazel symlink trees, device streams, and pseudo-filesystems); always exclude hidden directories, `.cache`, and `bazel-*`, and ALWAYS run `grep` or `find` commands with a maximum 60-second timeout (e.g. `timeout 60s grep ...`).
- **CLI help syntax**: Run `inctl help` or `inctl <subcommand> --help` to display Cobra commands. Running `inctl --help` invokes the Go flag parser and displays only logging flags.

## Core platform primitives

Intrinsic Core applications are composed of five core primitives:

### 1. Assets
An **Asset** is a versioned, immutable package distributed as a bundle tarball (`.bundle.tar`) containing container images, descriptors, and manifests. Examples include hardware module definitions, gripper interfaces, CAD scene meshes, and skill libraries. Install bundles locally via:
```bash
inctl asset install <bundle_path> --address=localhost:17080
```

> [!NOTE]
> If `inctl asset install` fails with `failed to dial "/run/containerd/containerd.sock": connect: connection refused` or connection refused on `localhost:17127`, stop after at most 2 attempts. The failure is on the backend server side (the containerd socket and its mount belong inside `k3s` on the cluster host, NOT in the local client environment, e.g. Bubblewrap sandbox). Do NOT attempt to debug local `/run` sockets, search for sockets with `find /run`, or attempt to proxy `/run/containerd/containerd.sock`. Fall back to local hermetic Bazel verification (`bazel build //...`, `bazel test //...`).

### 2. Services
A **Service** is a long-running microservice daemon providing capabilities across the cluster over gRPC or HTTP. Services represent hardware interfaces (`ur_module`, `gazebo_simulator`), motion planners, and vision pipelines. Services declare lifecycle hooks (`SIGTERM` handling) and expose health endpoints.

### 3. Skills
A **Skill** is a discrete robotic task or capability encapsulated as an action or condition node within a Behavior Tree (BT). Examples include Cartesian linear motion, gripper actuation, or scene graph frame spawning. Skills declare equipment leases (`required_equipment`) and execute within an `ExecuteContext`.

### 4. Solutions
A **Solution** is the top-level deployable robotic application binding Services, the World model (`ObjectWorld`), and an Executive Behavior Tree to accomplish a workcell objective. Start or stop local solutions via `inctl solution start` and `inctl solution stop` with `--address=localhost:17080`.

### 5. `ICON`
**ICON** (intrinsic control core) is the real-time execution engine running a high-frequency control loop (1 kHz). It interfaces directly with motor servo drives (via EtherCAT) or hardware modules (via shared memory/IPC) to compute inverse kinematics, enforce dynamic limits, and stream joint trajectory setpoints safely.

## Input-aware decision tree for platform inspection

Select diagnostic and inspection commands based on current runtime objective:

```
Goal: Inspect workcell state
├── Inspect active service states?
│   └── Run: inctl service state list --address=localhost:17080
│       └── Output: Active services (calibration_service, gazebo_simulator, icon, ur_module) and state (Enabled, Faulted)
├── Inspect configured asset instances?
│   └── Run: inctl asset instance list --address=localhost:17080
│       └── Output: Maps instance names to asset IDs (e.g. icon -> ai.intrinsic.generic_realtime_control_service)
├── Inspect installed catalog assets?
│   └── Run: inctl asset list --asset_types=service --address=localhost:17080
│       └── Output: Lists installed service packages, including uninstantiated assets
├── Inspect or control real-time ICON engine?
│   └── Run: inctl icon status --instance_name=icon --address=localhost:17080
│       └── Output: Real-time loop status, active parts, and controller state
└── Inspect or stage behavior tree process?
    └── Run: inctl process get --server=localhost:17080
        └── Output: Currently staged BehaviorTree proto definition (note --server flag)
```

## Mandatory workspace search rules

Always run search commands (`grep`, `find`) with an explicit maximum 60-second timeout (e.g. `timeout 60s ...`) to prevent unbounded hangs on cyclic symlinks, character devices, or unbuffered pipes.
- **Scope searches locally**: Never search the root filesystem `/` (`grep -r /` or `find /`); scanning outside the workspace traverses `/proc`, `/sys`, and container caches and will freeze the process.
- **Exclude build and cache trees**: Always exclude Bazel output trees and caches (`.cache`, `bazel-*`).

```bash
# Search workspace files safely with a 60-second timeout:
timeout 60s grep -rn --exclude-dir={.cache,'bazel-*'} "search_term" .
timeout 60s find . -not -path '*/.*' -not -path './bazel-*' \( -name "*.py" -o -name "*.bzl" \)

# Inspect Bazel external dependency sources safely:
EXT_DIR="$(bazel info output_base)/external"
timeout 60s find "$EXT_DIR" -maxdepth 1 -type d
timeout 60s find "$EXT_DIR/<specific_repo>" -name "*.py"
```

## Cross-skill progressive disclosure

For specialized domain capabilities, consult domain-specific skills:
- For gRPC and protobuf API surface, core services, and data models: [intrinsic-core-api-overview](../intrinsic-core-api-overview/SKILL.md)
- For Bazel hermetic rules, external repositories, and dependency management: [intrinsic-core-bazel](../intrinsic-core-bazel/SKILL.md)
- For cross-service debugging, gRPC error codes, and failure diagnosis: [intrinsic-core-debugging](../intrinsic-core-debugging/SKILL.md)
- For robot motion planning, safety envelopes, and joint jog commands: [intrinsic-core-robot-motion](../intrinsic-core-robot-motion/SKILL.md)
- For containerized service authoring and proto definitions: [intrinsic-core-service-authoring](../intrinsic-core-service-authoring/SKILL.md)
- For behavior tree leaf action nodes, skill lifecycle, and scaffolding: [intrinsic-core-skill-authoring](../intrinsic-core-skill-authoring/SKILL.md)
- For Solution Building Library (SBL), behavior tree composition, and CEL expressions: [intrinsic-core-solution-building](../intrinsic-core-solution-building/SKILL.md)
- For deploying, managing, and inspecting active solutions, services, and ICON controllers: [intrinsic-core-solutions](../intrinsic-core-solutions/SKILL.md)

## `System 2` reflection checkpoint and anti-thrashing circuit breakers

### Pre-execution reflection checkpoint
Before executing CLI mutations or cluster queries, verify:
1. Is the addressing flag correct (`--address=localhost:17080` for asset/service/solution/icon, `--server=localhost:17080` for process)?
2. Does the service asset command include its required instance header flag (for example, `--instance_name=icon`)?
3. Does the directory search command explicitly exclude `bazel-*` and `.cache`?

### Anti-thrashing circuit breaker
- **On `UNIMPLEMENTED` (gRPC status 12)**: If an `inctl` command fails with empty `UNIMPLEMENTED`, check whether `--instance_name=<name>` was omitted or if a cloud-only subcommand was invoked instead of blindly retrying.
- **On `UNAVAILABLE` (gRPC status 14)**: If a command fails with connection refused or no healthy upstream, run `inctl service state list --address=localhost:17080` once to verify gateway connectivity. If unreachable after 2 attempts, halt and inspect cluster pod health and container logs (via `kubectl get pods -A` and `kubectl logs`).
- **Retry ceiling**: Switch to an alternate diagnostic branch if the same CLI command fails twice (maximum 2 retry attempts).
- **On containerd socket dial failure (`/run/containerd/containerd.sock`) or port 17127 connection refused**: If `inctl asset install` fails with containerd socket connection refused, cap attempts at 2. Halt immediately. This is a backend `k3s` cluster host issue, not a local client sandbox issue. Do NOT attempt to debug local `/run` sockets, search with `find /run`, or attempt to proxy `/run/containerd/containerd.sock`. Fall back to local hermetic Bazel build and test verification (`bazel build //...`, `bazel test //...`).
