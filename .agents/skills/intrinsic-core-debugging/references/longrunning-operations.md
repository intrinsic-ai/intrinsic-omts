# Debugging long-running operations, proxy routing, and cancellation

This reference covers diagnosing `google.longrunning.Operations` routing failures, Istio ingress header mismatches, `WaitOperation` vs. `GetOperation` semantics, cancellation deadlocks in the Executive, `System 2` reflection checkpoints, input-aware decision trees, and anti-thrashing circuit breakers.

## 1. Central `operations:8080` proxy vs. asset instance `LRO` routing

On a local workcell (`localhost:17080`), external requests and pod-to-pod calls targeting `google.longrunning.Operations` pass through the in-cluster Istio ingress gateway (`istio-ingressgateway.app-ingress.svc.cluster.local:80`). Envoy routes requests based on the `x-resource-instance-name` gRPC metadata header:

| Target operation category | Required `x-resource-instance-name` header | Backend router / pod target |
| :--- | :--- | :--- |
| **Core cluster platform runtime operations** (`geomservice`, `workcell-cluster-service`, `scene-object-import`, `asset-deployment`, `solution-deployment-v1`) | Omit header (`withoutHeaders: { x-resource-instance-name: {} }`) or set `exact: intrinsic_runtime` | Central `operations:8080` proxy in `app-intrinsic-base` namespace |
| **Asset-specific service operations** (deployed service instances, custom hardware or perception services) | Set `metadata=[('x-resource-instance-name', '<instance_name>')]` | Directly routed by Envoy `VirtualService` to the backing asset pod |

### Ingress header matching rules
- **Asset-specific operation polling**:
  - When querying or waiting on an operation hosted by a deployed service asset instance, attach `metadata=[('x-resource-instance-name', '<instance_name>')]` to the `GetOperation` or `WaitOperation` call so Envoy routes directly to the backing pod.
  - Calling `WaitOperation` through the central `operations:8080` proxy without the instance header returns `StatusCode.NOT_FOUND`.
- **Omitting instance headers on core platform operations**:
  - When polling a core platform operation (such as solution deployment, scene object import, or geometry processing), strip or omit any service-specific `x-resource-instance-name` header (or set `exact: intrinsic_runtime`).
  - Forwarding an unexpected header causes the Istio ingress gateway to reject the `Operations` call with status code 12 (`UNIMPLEMENTED`) and an empty description (`desc = ""`).

## 2. `WaitOperation` vs. `GetOperation` polling semantics

The Intrinsic platform supports both polling via `GetOperation` and blocking streams via `WaitOperation`, but their behavior varies across backend language runtimes:

| Runtime / backend | `GetOperation` support | `WaitOperation` support | `CancelOperation` terminal behavior | `DeleteOperation` precondition |
| :--- | :--- | :--- | :--- | :--- |
| **Go backend** (`intrinsic/longrunning/go`) | Supported | Supported (blocks until timeout or completion) | Returns `StatusCode.FAILED_PRECONDITION` if `done == true` | Supported (`StatusCode.OK`) |
| **Python backend** (`incode/longrunning/python`) | Supported | Supported (blocks until timeout or completion) | Returns `StatusCode.FAILED_PRECONDITION` if `done == true` | Supported (`StatusCode.OK`) |
| **`C++` backend** (`OperationScheduler`) | Supported | Unimplemented (`codes.Unimplemented`), synthesized as `StatusCode.NOT_FOUND` through `operations:8080` | Returns `StatusCode.OK` even if already terminal | Requires terminal state consumed via `GetOperation`; returns `StatusCode.INVALID_ARGUMENT` if active |

### `C++` backend operation limitations and synthetic `NOT_FOUND`
- **`C++` `ProcessGeometry` operations**:
  - `intrinsic_proto.geometry.GeometryService/ProcessGeometry` returns an operation named `"geometry/ProcessGeometry/<fingerprint>/<nanos>"`.
  - Because `C++` `OperationScheduler` unconditionally returns `codes.Unimplemented` for `WaitOperation`, the central `operations:8080` proxy (`ProxySet`) swallows the unimplemented response as a probe miss and returns `StatusCode.NOT_FOUND`.
  - Poll `ProcessGeometry` completion using `google.longrunning.Operations/GetOperation` in a loop with exponential backoff.
- **Preconditions for `DeleteOperation` on `C++` backends**:
  - On `C++` `OperationScheduler`, invoking `DeleteOperation` on an active operation returns `StatusCode.INVALID_ARGUMENT`.
  - Furthermore, `DeleteOperation` requires that the operation's result future has already been consumed via `GetOperation` (or cancelled via `CancelOperation`). Always query the terminal state via `GetOperation` before issuing `DeleteOperation`.

### Solution deployment `LRO` serialization
- Versioned solution lifecycle RPCs (`inctl solution start` and `inctl solution stop`) enqueue asynchronous operations on a runner queue to serialize Kubernetes `ChartAssignment` mutations.
- Await completion of running deployment operations before issuing subsequent deployment commands; concurrent chart mutations trigger `update chart assignment: already exists` errors.

## 3. Clean operation cancellation and executor deadlock prevention

Cancelling asynchronous workflows requires careful coordination between the Executive, skill containers, and internal worker threads:

### Executive `CancelOperation` and skill container health (`ClearSkillOperations`)
- Invoking `intrinsic_proto.executive.ExecutiveService/CancelOperation` acquires the executive operation lock and calls `ClearSkillOperations` across all registered skill gRPC channels.
- If any skill container is unreachable, in `CrashLoopBackOff`, or in `GRPC_CHANNEL_TRANSIENT_FAILURE`, the executive thread blocks while attempting to clear skill operations before releasing the executor lock.
- While the lock is held, the Executive becomes unresponsive and rejects subsequent requests with `Incompatible state`.
- When `CancelOperation` hangs, inspect skill container health using `inctl service state list --address=localhost:17080` and `kubectl get pods -n skills` to locate the crashed pod.

### Thread join deadlocks and uncancellable execution (`SkillOperation::WaitOperation`)
- When cancelling an active behavior tree or skill, `ForceAbandonOperation` can hang inside `SkillOperation::WaitOperation`.
- The deadlock occurs when `finished_notification_.WaitForNotification()` and `thread_.join()` block waiting for orphaned worker threads to exit while holding synchronization locks.
- Inspect skill logs (`kubectl logs -n skills <pod_name>`) to determine whether background threads are trapped in non-interruptible system calls.

### Mutex acquisition during synchronous polling loops
- In Conductor, holding an internal lock (`mu_`) across synchronous LRO polling loops deadlocked all other control RPCs (`StartSolution`, `StopSolution`, `SaveWorld`) when downstream asset deployment stalled.
- Always release all shared synchronization locks prior to initiating polling loops, and enforce explicit client-side deadlines.

### Callback re-entrancy in custom task executors
- In custom services implementing prioritized task executors, cancellation callbacks execute while holding the executor's internal queue lock.
- Defer queue inspection or new task enqueueing to decoupled worker threads rather than invoking them synchronously inside cancellation callbacks to prevent self-deadlock.

### Differentiating cluster `LRO` tasks from `Executive` operations
- Cluster `LRO` tasks (`google.longrunning.Operations` routed through `operations:8080`) represent general background jobs (mesh processing, scene import, solution deployment). Query them via `google.longrunning.Operations/GetOperation`.
- Executive operations (`intrinsic_proto.executive.ExecutiveService`) represent behavior tree execution instances (`CreateOperation` to stage, `StartOperation` to execute, `CancelOperation` to abort). Both exchange `google.longrunning.Operation` messages, but Executive operations populate metadata with `intrinsic_proto.executive.RunMetadata`.

## 4. `System 2` reflection checkpoint before long-running operations

Before executing, polling, or cancelling long-running operations, perform this structured `System 2` reflection check:

1. **Target service architecture**:
   - Is the target operation handled by a `C++` backend (`geomservice`, calibration), Go backend, or Python backend?
   - If `C++`, poll using `GetOperation` in a loop with exponential backoff.
2. **Ingress metadata routing**:
   - Is this a core platform runtime service (`operations:8080`) or a deployed service asset instance?
   - For core runtime operations, ensure `x-resource-instance-name` is omitted or set to `exact: intrinsic_runtime`.
   - For service asset instances, attach `metadata=[('x-resource-instance-name', instance_name)]`.
3. **Deadlines and synchronization**:
   - Are all application mutexes and locks released before initiating the polling loop?
   - Is an explicit overall deadline configured on the polling context?

## 5. Input-aware diagnostic decision trees

### Diagnostic decision tree: polling failures and status code errors

```
[Precondition: Polling an operation returns an unexpected gRPC status code]
  │
  ├──► [Diagnostic check: code = Unimplemented (status 12) with empty desc = ""]
  │      └──► [Targeted action: Inspect outgoing gRPC metadata. If targeting a core runtime service,
  │            strip x-resource-instance-name or set it to 'intrinsic_runtime'. If targeting an asset,
  │            ensure x-resource-instance-name matches the deployed instance name.]
  │
  ├──► [Diagnostic check: code = NotFound (status 5) returned by WaitOperation]
  │      └──► [Targeted action: The operation is likely hosted on a C++ backend (e.g. geomservice) or
  │            routed through operations:8080. Switch from WaitOperation to polling GetOperation in a loop.]
  │
  ├──► [Diagnostic check: code = InvalidArgument (status 3) on DeleteOperation]
  │      └──► [Targeted action: The operation is still running or its result future was not consumed.
  │            Call GetOperation to retrieve the final result, or cancel via CancelOperation, before deleting.]
  │
  └──► [Diagnostic check: code = FailedPrecondition (status 9) on CancelOperation]
         └──► [Targeted action: The operation has already finished (done == true) on a Go/Python backend.
               Query GetOperation to inspect the completed result.]
```

### Diagnostic decision tree: cancellation hangs and executor lockups

```
[Precondition: CancelOperation or Executive operation hangs indefinitely]
  │
  ├──► [Diagnostic check: inctl service state list reports one or more services in ERROR]
  │      └──► [Targeted action: Run 'kubectl get pods -n skills' to identify pods in CrashLoopBackOff.
  │            Delete or restart the failing pod to unblock ClearSkillOperations and release the executor lock.]
  │
  ├──► [Diagnostic check: Executive rejects subsequent RPCs with "Incompatible state"]
  │      └──► [Targeted action: An operation is still transitioning or holding the executor lock.
  │            Verify that all skill containers are responsive, then retry CancelOperation once.]
  │
  └──► [Diagnostic check: Solution start/stop reports "update chart assignment: already exists"]
         └──► [Targeted action: Concurrent solution deployment operations are racing. Poll active operations
               via GetOperation until done == true before submitting new chart modifications.]
```

## 6. Anti-thrashing circuit breakers

To prevent tight busy-loops, resource exhaustion, and action thrashing during operation management:

- **Polling backoff and retry cap**:
  - Limit polling frequency using exponential backoff: start at 0.5s, multiply by 1.5, and cap at 5.0s.
  - Enforce an overall timeout (default 120s for standard operations, 300s for solution deployments).
  - If `GetOperation` fails with transient transport errors (`UNAVAILABLE`), permit at most 3 consecutive retries before aborting to diagnostic inspection.
- **Cancellation retry cap**:
  - If `CancelOperation` does not return within 15s, stop repeated cancellation attempts.
  - Branch immediately to inspecting skill pod health via `kubectl get pods -n skills` rather than spamming cancellation RPCs.
- **Deletion circuit breaker**:
  - Attempt `DeleteOperation` at most once after an operation completes. If it returns `INVALID_ARGUMENT`, log a warning and proceed without retrying in a tight loop.

## 7. Operational safety guardrails

To maintain system stability and prevent deadlocks across the workcell, adhere to these five paired safety guardrails:

1. **Do not poll `WaitOperation` through the central `operations:8080` proxy or against `C++` backends**; poll `google.longrunning.Operations/GetOperation` in a loop with exponential backoff instead.
2. **Do not forward service-specific `x-resource-instance-name` headers when querying core platform runtime operations**; omit the header or set it to `exact: intrinsic_runtime`.
3. **Do not hold mutexes or synchronization locks across synchronous `LRO` polling loops**; release all locks prior to polling and configure explicit client-side timeouts.
4. **Do not mix synchronous deployment RPCs with asynchronous `LRO` solution lifecycle calls**; await active deployment operation completion before submitting new chart mutations.
5. **Do not invoke queue inspection or enqueueing methods synchronously inside cancellation callbacks**; decouple re-entrant operations to a separate worker thread.

## 8. Verified diagnostic commands and `Python` polling workflow

### Essential inspection commands on local workcells

```bash
# 1. Inspect active runtime services and error states
inctl service state list --address=localhost:17080 --output=json

# 2. Inspect skill pod health when cancellation hangs
kubectl get pods -n skills

# 3. Inspect pod logs for crashed skill containers
kubectl logs -n skills <pod_name> --tail=100

# 4. Check Istio ingress routing rules for Operations
kubectl get virtualservice operations -n app-intrinsic-base -o yaml

# 5. Inspect central operations proxy logs for probe misses or backend timeouts
kubectl logs -n app-intrinsic-base -l app=operations --tail=100
```

### Verified `Python` polling implementation

```python
import time
import grpc
from google.longrunning import operations_pb2, operations_pb2_grpc


def poll_operation(
    operation_name: str,
    target_address: str = "localhost:17080",
    instance_name: str | None = None,
    timeout_seconds: float = 120.0,
    raise_on_error: bool = False,
) -> operations_pb2.Operation:
  """Polls an LRO to completion using GetOperation with exponential backoff.

  Args:
    operation_name: Resource name of the operation.
    target_address: Ingress gateway address (defaults to localhost:17080).
    instance_name: Resource instance name header (for deployed service assets;
      must be None for core platform runtime operations).
    timeout_seconds: Overall polling timeout ceiling in seconds.
    raise_on_error: If True and operation.error is set, raises RuntimeError.

  Returns:
    Terminal google.longrunning.Operation protobuf message.
  """
  channel_options = [
      ("grpc.max_receive_message_length", 64 * 1024 * 1024),
      ("grpc.max_send_message_length", 64 * 1024 * 1024),
  ]
  channel = grpc.insecure_channel(target_address, options=channel_options)
  stub = operations_pb2_grpc.OperationsStub(channel)

  # Attach instance header only for deployed service assets
  metadata = []
  if instance_name:
    metadata.append(("x-resource-instance-name", instance_name))

  start_time = time.monotonic()
  poll_interval = 0.5
  transient_failures = 0

  while time.monotonic() - start_time < timeout_seconds:
    try:
      req = operations_pb2.GetOperationRequest(name=operation_name)
      op = stub.GetOperation(req, metadata=metadata, timeout=10.0)
      transient_failures = 0

      if op.done:
        if raise_on_error and op.HasField("error"):
          raise RuntimeError(
              f"Operation {operation_name} failed with status"
              f" code {op.error.code}: {op.error.message}"
          )
        return op
    except grpc.RpcError as err:
      if err.code() == grpc.StatusCode.UNAVAILABLE:
        transient_failures += 1
        if transient_failures >= 3:
          raise RuntimeError(
              f"Operation {operation_name} polling aborted after 3 transient"
              " failures"
          ) from err
      else:
        raise

    time.sleep(poll_interval)
    poll_interval = min(poll_interval * 1.5, 5.0)

  raise TimeoutError(f"Operation {operation_name} timed out after {timeout_seconds}s")
```
