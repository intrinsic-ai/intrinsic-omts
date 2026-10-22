# Managing resources, equipment bindings, and execution in `SBL`

## 1. Safe iteration and lookup on `solution.resources`

In `intrinsic.solutions`, all deployed services, robots, grippers, cameras, and controllers are registered as `ResourceHandle` objects under `solution.resources` (`providers.ResourceProvider`). Note that `solution.services` does not exist on `deployments.Solution`.

### Iteration and reflection contract on `ResourceProvider`

The `providers.ResourceProvider` class implements dynamic attribute lookup (`__getattr__`), item indexing (`__getitem__`), and directory reflection (`__dir__`), but omits `__len__`, `__iter__`, and `__contains__`:
- Calling `len(solution.resources)` raises `TypeError: object of type 'Resources' has no len()`.
- Iterating with `for r in solution.resources:` triggers Python sequence index fallback (`__getitem__(0)`), raising `KeyError: 'Resource 0 not registered'`.
- Checking existence via `hasattr(solution.resources, "my_service")` or `getattr(solution.resources, "my_service", None)` invokes `__getattr__` and raises `KeyError` when the resource is not present.
- Checking membership via `"my_service" in solution.resources` triggers sequence fallback iteration starting at index 0, raising `KeyError`.

> [!IMPORTANT]
> **Paired safety guardrail 1**: Do not call `len(solution.resources)`, `for r in solution.resources:`, or `hasattr()` on `ResourceProvider`; check existence and iterate strictly via `dir(solution.resources)` and bracket notation `solution.resources[name]`.

### Verified discovery and iteration patterns

Use `dir(solution.resources)` to safely inspect, filter, and iterate over registered handles:

```python
# 1. Safely check whether a resource handle exists
if "my_service" in dir(solution.resources):
  handle = solution.resources.my_service

# 2. Safely iterate over all registered ResourceHandle instances
all_handles = [solution.resources[name] for name in dir(solution.resources)]

# 3. Filter resources by capability or type
robot_handles = [
    solution.resources[name]
    for name in dir(solution.resources)
    if "robot" in name
]
```

### Attribute name sanitization for non-identifier characters

Resource instances containing `"::"` (for example, `"robot::arm"`) are exposed via sanitized Python attributes where `"::"` is replaced by `"__"` (`solution.resources.robot__arm`). Alternatively, access the resource directly via raw bracket notation (`solution.resources["robot::arm"]`).

---

## 2. Equipment slot bindings, deconfliction, and resolved dependencies

Skills bind to `ResourceHandle` objects from `solution.resources` through two primary mechanisms: equipment slots and resolved service dependencies.

### Equipment slot bindings and deconfliction

1. **Inspecting compatible resources (`compatible_resources`)**:
   Query compatible handles for a skill equipment slot via `skills.ai.intrinsic.move_robot.compatible_resources.arm_part`. Unlike `solution.resources`, the returned `ResourceListImpl` implements full collection semantics (supporting `len()` and iteration `for r in ...`).
2. **Keyword argument deconfliction (`_resource` suffix)**:
   If a skill defines both a protobuf configuration parameter and an equipment slot sharing the exact same name, the Solution Building Library appends `_resource` to the equipment slot keyword argument to prevent parameter collisions (`deconflict_param_and_resources`).
3. **Automatic default slot binding**:
   When exactly one compatible resource handle exists in the active solution deployment, the Solution Building Library automatically defaults that slot parameter upon skill instantiation.

### Resolved dependency parameters (`intrinsic_proto.assets.v1.ResolvedDependency`)

When instantiating asset-based skills (such as `enable_gripper`), passing a `ResourceHandle` from `solution.resources` automatically populates `ResolvedDependency(name=handle.name)`:
- At container startup, the platform mounts `/etc/intrinsic/runtime_config.pb` and rewrites dependency annotations into `intrinsic_proto.assets.v1.ResolvedDependency`.
- The platform populates `ResolvedDependency.interfaces["grpc://<pkg>.<Service>"].grpc.connection` with the cluster ingress gateway address (`istio-ingressgateway.app-ingress.svc.cluster.local:80`) and sets the `x-resource-instance-name: <instance_name>` metadata header, clearing `ResolvedDependency.name`.

### Tree construction optimization (`_with_recommended_config`)

By default, skill wrapper constructors issue synchronous gRPC round-trips to `RecommendAssetConfiguration` during instantiation. When composing large Behavior Trees in a loop, pass `_with_recommended_config=False` to the skill constructor to bypass synchronous configuration lookups and avoid construction latency:

```python
# Instantiate skills rapidly in loops without synchronous gRPC lookups
move_nodes = [
    skills.ai.intrinsic.move_robot(
        arm_part=solution.resources.robot,
        target_joint_configuration=cfg,
        _with_recommended_config=False,
    )
    for cfg in joint_trajectory_configs
]
```

---

## 3. Bridging raw `gRPC` stubs via `solution.grpc_channel`

When calling platform `gRPC` services directly (such as `ObjectWorldServiceStub`, `InstalledAssetsStub`, or `MotionPlannerServiceStub`), reuse `solution.grpc_channel`.

### Buffer limits and channel configuration

The channel provided by `solution.grpc_channel` is pre-configured with unlimited receive and send message sizes:
- `grpc.max_receive_message_length = -1`
- `grpc.max_send_message_length = -1`

Calling platform services via unconfigured standalone gRPC channels retains default 4 MB (`4,194,304` bytes) limits, which fail with `RESOURCE_EXHAUSTED` when transmitting dense visual meshes, point clouds, or multi-megabyte world state snapshots:

> [!IMPORTANT]
> **Paired safety guardrail 2**: Do not instantiate unconfigured standalone gRPC channels with default 4 MB message limits when calling platform services; reuse `solution.grpc_channel` (configured with unlimited receive and send buffer sizes) to prevent `RESOURCE_EXHAUSTED` failures on dense geometry meshes.

```python
from intrinsic.world.proto import object_world_service_pb2_grpc

# Reuse the pre-configured channel to handle multi-megabyte world state payloads
world_stub = object_world_service_pb2_grpc.ObjectWorldServiceStub(
    solution.grpc_channel
)
```

---

## 4. Behavior tree execution, lifecycle states, and parameter scopes

### Blocking vs. non-blocking execution semantics

The executive separates operation staging, execution dispatch, and lifecycle monitoring:

> [!IMPORTANT]
> **Paired safety guardrail 3**: Do not invoke `executive.run(None)` expecting synchronous blocking execution or return timestamps; supply a behavior tree to `solution.executive.run(tree)` or call `solution.executive.start(blocking=True)` to block until operation completion.

- **Non-blocking caveat on `executive.run(None)`**:
  Passing `plan_or_action=None` to `solution.executive.run(None)` calls `_start_with_retry(parameters=None, resources=None)` and returns immediately without blocking, ignoring `simulation_mode` and `parameters`. To start an already loaded operation and block until execution completes, invoke `solution.executive.start(blocking=True)`.
- **Return value and execution timestamps**:
  `solution.executive.run()` returns `None` rather than execution timestamps. Retrieve start and end timestamps from `solution.executive.last_execution_time_window`.

### Operation lifecycle state machine (`RunMetadata.State`)

Every execution dispatches an `Operation` (`solution.executive.operation`) tracked by runtime states:
`ACCEPTED` -> `PREPARING` -> `RUNNING` -> `SUSPENDING` -> `SUSPENDED` -> `CANCELING` -> `SUCCEEDED` / `FAILED` / `CANCELED`.

Inspect live runtime blackboard variables and keys during execution or upon suspension:

```python
# Inspect blackboard state on the active operation
active_op = solution.executive.operation
if active_op.state == active_op.State.SUSPENDED:
  keys = active_op.blackboard.list_keys()
  value = active_op.blackboard.get_value("my_blackboard_key")
```

### Passing debug parameter values (`PARAMETER_DEBUG_VALUES`)

When executing a saved Behavior Tree (`solution.processes["my_tree"]`) that references `params.<field>` in CEL expressions, pass the embedded debug parameter `Any` payload to instantiate the `params` blackboard scope:

```python
saved_bt = solution.processes["my_tree"]
solution.executive.run(
    saved_bt,
    parameters=saved_bt.user_data_protos.get("PARAMETER_DEBUG_VALUES"),
    start_from_world_state=worlds.EditWorldId.BELIEF,
)
```

Executing saved trees without passing `parameters` fails with `ai.intrinsic.executive:15101: The expression 'params.<field>' refers to the following blackboard variables that do not exist: params`.

### Handling `ExecutionFailedError` and inspecting diagnostics

When `solution.executive.run()` raises `intrinsic.solutions.execution.ExecutionFailedError`, extract structured diagnostic trees rather than relying solely on the top-level exception string:

```python
from intrinsic.solutions import execution

try:
  solution.executive.run(tree, start_from_world_state=worlds.EditWorldId.BELIEF)
except execution.ExecutionFailedError as e:
  op = solution.executive.operation
  # Inspect structured failure status
  if op.proto.error.message:
    print(f"Operation error: {op.proto.error.message}")
  # Inspect structured ExtendedStatus diagnostics and wire-type mismatches
  diagnostics = op.metadata.diagnostics
  for diag in diagnostics:
    print(f"Diagnostic [{diag.source}]: {diag.message}")
```

Key execution error codes and mechanisms:
- `ai.intrinsic.executive:15101`: CEL parameter variable missing; pass `PARAMETER_DEBUG_VALUES`.
- `ai.intrinsic.executive:21101`: `DEADLINE_EXCEEDED: PrepareProcessStart-ResetBeliefWorldAndSim`. Conflicting initial joint states or frames between `icon`, `ur_module`, and `"init_world"`. Run `inctl world reset --address=localhost:17080`.
- Protobuf recursion limits (`DecodeError` / `max recursion depth 100`): Occurs when re-submitting a tree containing populated runtime state. Strip `OUTPUT_ONLY` fields (`RunMetadata`, node states) before re-submitting. For offline inspection, set `decoder.SetRecursionLimit(10000)` and `sys.setrecursionlimit(10000)`.

### Operation cancellation and unresponsive executive `RPC` calls

Invoking `CancelOperation` acquires the executive operation lock and calls `ClearSkillOperations` across all registered skill gRPC channels:
- If a skill container is unreachable, crashed, or in `GRPC_CHANNEL_TRANSIENT_FAILURE`, the executive thread blocks while attempting to establish a connection before releasing the lock, rejecting subsequent requests with `Incompatible state`.
- Before retrying cancellation or re-submitting processes, verify skill pod health via `kubectl get pods -A` and `inctl service state list --address=localhost:17080`.

---

## 5. Pod resource allocation, container lifecycle, and hardware failure recovery

### `Kubernetes` pod resource allocation and scheduling limits

1. **GPU time-slicing replica exhaustion (`nvidia.com/gpu: 1`)**:
   A single physical GPU provides a fixed number of time-sliced virtual replicas (typically 8 or 16). Workcells running multiple simulated cameras (`genicam`), `gzserver`, `ml-models-service`, and perception pods can exhaust virtual GPU replicas. When exhausted, subsequent pods remain permanently in `Pending` with event `Insufficient nvidia.com/gpu`. Run `kubectl describe pod <pod_name> -n app-intrinsic-app-chart` to verify GPU allocation.
2. **Avoiding Linux CFS quota CPU throttling on hybrid architectures**:
   On hybrid Intel processors (P-cores and E-cores), strict container CPU limits (`limits: {cpu: "2"}`) trigger Linux Completely Fair Scheduler (CFS) quota throttling, causing 10–50 ms thread freezes and pushing motion planning threads onto slower E-cores. Configure generous CPU limits or rely primarily on CPU requests for compute-intensive service containers.
3. **Multi-container pod log inspection**:
   In the `skills` namespace and core service deployments, multiple containers frequently share a single pod. Query logs with an explicit container name (`kubectl logs <pod_name> -c <container_name> -n skills`) so healthy logs from a sibling container do not mask a crashlooping container.

### Hardware safety and lockfile deadlock remediation

> [!IMPORTANT]
> **Paired safety guardrail 4**: Do not switch solution execution mode between simulation and real hardware while running; invoke `inctl solution stop --address=localhost:17080` before changing execution modes.

> [!IMPORTANT]
> **Paired safety guardrail 5**: Do not restart faulted controller pods repeatedly without removing `/tmp/intrinsic_icon/ur_module.lock`; delete the stale host lockfile and clear faults via `inctl icon clear-faults --instance_name=icon --address=localhost:17080`.

When an operational hardware module (`rs-ur-module`) or real-time controller (`rs-icon`) crashes or terminates abruptly, the shared memory lockfile `/tmp/intrinsic_icon/ur_module.lock` remains on the host filesystem volume:
- Subsequent container restarts fail to acquire the lock and enter `CrashLoopBackOff` with `Resource temporarily unavailable`.
- Clear the condition by deleting the host lockfile `/tmp/intrinsic_icon/ur_module.lock` and resetting faults:
  ```bash
  rm -f /tmp/intrinsic_icon/ur_module.lock
  inctl icon clear-faults --instance_name=icon --address=localhost:17080
  ```
- After invoking `clear-faults`, verify that `inctl icon status --instance_name=icon --address=localhost:17080` reports `Operational Status: ENABLED` before commanding motion, as hardware modules take several seconds to complete driver re-initialization.

---

## 6. Diagnostic decision trees, `System 2` reflection, and circuit breakers

### Input-aware decision tree 1: resource binding and channel selection

```
[Determine workcell capability or service binding requirement]
       │
       ├── Precondition: Skill requires equipment slot binding (e.g. arm_part, gripper)
       │      │
       │      ├── Diagnostic check: Query skills.<id>.compatible_resources.<slot>
       │      ├── Diagnostic check: Check if slot name collides with proto parameter name
       │      │      ├── Collision detected -> Targeted action: Pass <slot_name>_resource=handle
       │      │      └── No collision       -> Targeted action: Pass <slot_name>=handle
       │      └── Diagnostic check: Handle existence in solution.resources
       │             ├── "handle" in dir(solution.resources) -> Targeted action: Assign handle
       │             └── Handle missing -> Targeted action: Verify asset instance in inctl asset instance list
       │
       ├── Precondition: Asset-based skill consuming external service interface
       │      │
       │      ├── Diagnostic check: Inspect skill equipment descriptor
       │      └── Targeted action: Pass solution.resources[instance]; platform runtime mounts
       │          /etc/intrinsic/runtime_config.pb and populates ResolvedDependency gateway
       │
       └── Precondition: Calling platform gRPC service directly (world, planner, assets)
              │
              ├── Diagnostic check: Inspect payload size / mesh complexity
              └── Targeted action: Connect stub to solution.grpc_channel (reusing unlimited buffer sizes)
```

### Input-aware decision tree 2: execution failure and container crash diagnosis

```
[Diagnose execution failure or container crash]
       │
       ├── Precondition: ExecutionFailedError raised during executive.run()
       │      │
       │      ├── Diagnostic check: Check error code in operation.proto.error
       │      │      ├── Code 15101 (params.<field> missing)
       │      │      │      └── Targeted action: Supply parameters=saved_bt.user_data_protos.get("PARAMETER_DEBUG_VALUES")
       │      │      ├── Code 21101 (PrepareProcessStart-ResetBeliefWorldAndSim timeout)
       │      │      │      └── Targeted action: Run inctl world reset --address=localhost:17080 to resync init_world
       │      │      └── Recursion limit exceeded / UNIMPLEMENTED on unary call
       │      │             └── Targeted action: Strip OUTPUT_ONLY fields (RunMetadata) before resubmitting
       │      └── Diagnostic check: Inspect op.metadata.diagnostics for wire-type mismatches
       │             └── Schema mismatch -> Targeted action: Call solution.update_skills() to refresh descriptors
       │
       ├── Precondition: Controller pod in CrashLoopBackOff or State: Faulted
       │      │
       │      ├── Diagnostic check: Check host filesystem for stale lockfile
       │      │      ├── /tmp/intrinsic_icon/ur_module.lock exists
       │      │      │      └── Targeted action: rm -f /tmp/intrinsic_icon/ur_module.lock && inctl icon clear-faults
       │      │      └── Lockfile absent
       │      │             └── Targeted action: Check container logs (kubectl logs <pod> -c <container> -n <ns>)
       │      └── Diagnostic check: Operational status after reset
       │             └── Poll inctl icon status until Operational Status: ENABLED
       │
       ├── Precondition: Pod stuck in Pending state
       │      │
       │      ├── Diagnostic check: Run kubectl describe pod <pod> and check Events
       │      │      ├── "Insufficient nvidia.com/gpu"
       │      │      │      └── Targeted action: GPU time-slicing exhausted; reduce camera or sim replicas
       │      │      └── "PodFitsHostPorts"
       │      │             └── Targeted action: Port 17127 conflict; ensure deployment strategy is Recreate
       │      └── Diagnostic check: Multi-node real-time taint
       │             └── Real-time pod on non-rtpc node -> Targeted action: Delete pod to reschedule on rtpc node
       │
       └── Precondition: Startup readiness gate timeout ("Ports not open: executive...:8080")
              │
              ├── Diagnostic check: Trace upstream service startup dependency chain
              │      ├── simulation_service (:8088) / gzserver (:50053) blocked
              │      │      └── Targeted action: Verify CAS geometry mesh loading and org-id proxy credentials
              │      └── logger (:8080) blocked
              │             └── Targeted action: Inspect timescaledb-0 in app-intrinsic-base for startup errors
              └── Diagnostic check: Executive cancellation hangs
                     └── Skill in transient failure -> Targeted action: Verify skill container health via inctl service state list
```

### System 2 reflection checkpoints

Execute these structured self-critiques before taking state-mutating actions:

1. **Checkpoint A: pre-execution verification (before `executive.run` or `executive.start`)**:
   - Verify that all required equipment resource handles exist in `dir(solution.resources)`.
   - If `solution.world` was mutated, confirm `start_from_world_state=worlds.EditWorldId.BELIEF` is passed.
   - If running a saved Behavior Tree with CEL `params.<field>` references, confirm `PARAMETER_DEBUG_VALUES` is injected into `parameters`.
   - Confirm whether synchronous blocking (`executive.start(blocking=True)`) or asynchronous dispatch is required.
2. **Checkpoint B: pre-mutation deployment & hardware check (before restarting pods or changing configurations)**:
   - Check whether the solution is actively running before switching modes (`inctl solution stop`).
   - Check for stale host lockfiles (`/tmp/intrinsic_icon/ur_module.lock`) before restarting controller pods.
   - Verify node GPU allocation capacity via `kubectl describe nodes` before adding GPU-dependent pods.

### Anti-thrashing circuit breakers

To prevent infinite action-thrashing loops, apply these bounded retry limits:
- **Execution failure circuit breaker**: If an operation fails with `ExecutionFailedError`, cap execution retries at <= 2. On the second consecutive failure, halt execution attempts and branch to inspecting `operation.metadata.diagnostics`, reviewing CEL parameter bindings, and checking controller fault logs.
- **Pod crashloop circuit breaker**: If a container enters `CrashLoopBackOff`, cap pod deletion/restart attempts at <= 2. On the second failure, halt restarts and branch to checking `/tmp/intrinsic_icon/ur_module.lock`, container logs (`-c <container>`), and CFS CPU throttling metrics.
