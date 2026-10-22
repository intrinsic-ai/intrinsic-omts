# Debugging the executive, behavior trees, `CEL` expressions, and blackboard dataflow

## Two-phase executive operation lifecycle and `CLI` staging

- **Operation staging vs. execution**: The Executive (`intrinsic_proto.executive.v1.ExecutiveService`) strictly separates operation staging (`CreateOperation`) from live execution (`StartOperation`).
- **`CLI` behavior (`inctl process set` vs. `inctl process get`)**:
  - Running `inctl process set <file.pb> --server=localhost:17080` calls `ListOperations`, `DeleteOperation`, and `CreateOperation`, but never calls `StartOperation`. To start execution after staging via the `CLI`, invoke `intrinsic_proto.executive.v1.ExecutiveService/StartOperation` via gRPC or run `solution.executive.run()` in Python.
  - Running `inctl process get <process_name> --server=localhost:17080` returns only the static `BehaviorTree` definition protobuf (`metadata.GetBehaviorTree()`), not live node execution states. Always pass `<process_name>` without the `.bt.pb` suffix. To inspect live runtime status (`PREPARING`, `RUNNING`, `SUSPENDED`, `SUCCEEDED`, `FAILED`, `CANCELED`), query `ExecutiveService/GetOperation` or `ExecutiveService/ListOperations`.
- **Process start failure from simulation reset mismatch (`ai.intrinsic.executive:21101`)**:
  - When `StartOperation` fails immediately with `ai.intrinsic.executive:21101` (`DEADLINE_EXCEEDED: PrepareProcessStart-ResetBeliefWorldAndSim: ResetSimulation`), inspect initial joint state and frame alignment between `icon`, `ur_module`, and `"init_world"`. Conflicting initial joint configurations prevent simulation reset from completing within the RPC deadline.
- **Behavior tree cancellation deadlocks (`ClearSkillOperations`)**:
  - When `CancelOperation` hangs or rejects subsequent calls with `Incompatible state`, the Executive thread is blocked attempting to call `ClearSkillOperations` across all registered skill gRPC channels while holding the executor lock. Inspect skill container health via `inctl service state list --address=localhost:17080` and `kubectl get pods` to identify any skill container in `CrashLoopBackOff` or `GRPC_CHANNEL_TRANSIENT_FAILURE`.
- **World-updater pause persistence after simulation preview runs**:
  - Running an operation in `PREVIEW` or `FAST_PREVIEW` mode pauses the Executive's background world updater so preview trajectories do not overwrite the live belief world (`world_id="world"`). The world updater remains paused until a non-preview operation starts or `inctl world reset --address=localhost:17080` is executed.
- **Loop counter state during operation recovery (`ResetOperation`)**:
  - Recovering an operation without modifying the behavior tree invokes `ExecutiveService/ResetOperation` with `keep_blackboard=true`. While general blackboard keys are preserved, re-verify whether loop and retry node counters require explicit state re-initialization.
- **Branchless local workcell behavior tree persistence (`FailedPrecondition`)**:
  - When persisting a behavior tree in Python (`solution.behavior_trees[...] = bt`) fails with `FailedPrecondition: solution does not have an origin nor does it track a branch`, the local workcell lacks a cloud origin branch. Execute modified behavior trees directly via `solution.executive.load(bt)` or `solution.executive.run(bt)` rather than writing to `solution.behavior_trees`.

## Parallel branch execution locks, footprint deconfliction, and cold-start timeouts

- **Parallel branch serialization from default `GetFootprint()` (`lock_the_universe: true`)**:
  - The base class implementation of `SkillInterface.GetFootprint()` returns a `Footprint` message with `lock_the_universe: true`.
  - When a parallel composite node executes two branches concurrently, any skill that leaves `GetFootprint()` unimplemented requests an exclusive lock on the entire workcell, causing the Executive to serialize the parallel branches. Override `GetFootprint()` to set `lock_the_universe: false` and reserve only the specific equipment resources required by that skill.
- **First-run skill execution timeout (`DEADLINE_EXCEEDED`) that succeeds on retry**:
  - When a motion or perception skill fails with `DEADLINE_EXCEEDED` after 60 seconds on its very first run but succeeds immediately on retry, the skill container is healthy but the downstream `MotionPlannerService` is blocked deserializing complex workcell triangle meshes from `ObjectWorldService` on cold start. Compare timestamps between skill container logs and downstream service logs (`kubectl logs`).
- **Asynchronous footprint deconfliction false conflicts**:
  - Footprint conflict checks query world service gRPC endpoints asynchronously. Transient false conflicts can occur if a subsequent skill evaluates its footprint before a preceding skill's world state release has fully propagated.
- **Distinguishing compute stalls from memory paging jitter via `PagefaultInfo`**:
  - When robot motion exhibits timing jitter, execution latency spikes, or real-time controller clock stalls, inspect the periodic `PagefaultInfo [Abs Major | Abs Minor | Rel Major | Rel Minor ]` entries in the robot controller container log.
  - If `Rel Major == 0` and `Rel Minor == 0`, memory locking (`mlockall`) is active and operating system page faults are ruled out as the cause of the stall, confirming that delays originate in trajectory generation or downstream RPC latency.

## Behavior tree serialization limits, recursion limits, and asset loading

- **Deceptive `UNIMPLEMENTED` and recursion limit errors on `CreateOperation`**:
  - When importing or re-submitting a behavior tree fails with `UNIMPLEMENTED: Failed to fetch installed subprocess assets: No message returned for unary request` or `cannot decode message cel.expr.Expr.Select from binary: maximum recursion depth of 100 reached`, the protobuf parser aborted decoding a message exceeding 100 nested levels or the 4 MB gRPC channel limit.
  - Authored behavior trees are compact (~50 KB), but running trees causes the Executive to populate recursive runtime metadata (`RunMetadata`, `called_tree_state`, node execution states, script node outputs, and condition evaluation ASTs).
  - Strip all output-only runtime fields (`field_behavior = OUTPUT_ONLY`, including `RunMetadata`, `called_tree_state`, node `state`, `failure_reason`, `blackboard_scope`, `resources_current_map`, `num_tries`, `num_times`, and `stdout_list`) before passing a `BehaviorTree` protobuf back to `CreateOperation`.
  - When unpacking deeply nested `RunMetadata` offline in Python, elevate the parser limit:
    ```python
    import os, sys
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    sys.setrecursionlimit(50000)
    from google.protobuf.internal import decoder
    decoder._recursion_limit = 10000
    ```
- **Cumulative subprocess asset payload size during asset loading**:
  - When loading a behavior tree fails with `RESOURCE_EXHAUSTED: Received message larger than max`, check the total count and serialized size of installed subprocess assets in the solution catalog via `intrinsic_proto.assets.v1.InstalledAssets/ListInstalledAssets` with `view: ASSET_VIEW_TYPE_FULL`. Prune unused or oversized subprocess assets to reduce the cumulative catalog payload below channel limits.
- **Mapping `PythonScript` runtime traceback line numbers**:
  - Runtime tracebacks from `PythonScript` nodes (`Cell In[...], line N, in compute(params, context)`) originate from the backend execution template. Map line `N` back to user code using:
    - `user_line = N - 32` when both parameter and return value schemas are defined.
    - `user_line = N - 28` when only a return value schema is defined.
    - `user_line = N - 22` when neither parameters nor return values are defined.
- **Bypassing synchronous configuration recommendations during tree instantiation**:
  - Instantiating Python SDK skill wrapper classes triggers synchronous `RecommendAssetConfiguration` RPCs by default. Pass `_with_recommended_config=False` to skill wrapper constructors to eliminate multi-second instantiation delays.

## Executive parameter passing, `Proto3` default merging, and descriptor pools

- **Skill server default parameter merging (`UnpackAnyAndMerge`)**:
  - When a caller sets a scalar parameter to `false` or `0` in Proto3 without the `optional` keyword, wire serialization omits the field. When the skill server unpacks the `google.protobuf.Any` payload (`UnpackAnyAndMerge`), it treats omitted fields as unset and overwrites them with non-zero manifest defaults.
  - Declare scalar skill parameters as `optional` in the skill `.proto` to preserve explicit `false` or `0` overrides.
  - Submessage fields in Proto3 have explicit field presence. If the client serializes an empty submessage (`{}`), manifest default submessages are not merged.
- **Protobuf `Any` type URLs and script node descriptor pools**:
  - Top-level `BehaviorTree` metadata (`parameter_descriptor_fileset`) only contains descriptors for process-level inputs and outputs; it omits embedded script node descriptors (`Node.task.execute_code.file_descriptor_set`).
  - When resolving script node types programmatically, collect and merge `file_descriptor_set` entries across all script nodes in the tree.
- **Dynamic skill sideloading and descriptor pool invalidation**:
  - Deploy and sideload skills exclusively while the Executive is idle. Sideloading during active execution rebuilds the descriptor pool, invalidating raw descriptor pointers held by active blackboard messages and triggering undefined behavior or memory faults.

## `CEL` expression evaluation, blackboard dataflow, and error recovery

- **Initializing the `params` blackboard scope in Python**:
  - When running a saved behavior tree via Python (`executive.run(solution_bt)`) fails with `ai.intrinsic.executive:15101: The expression 'params.<field>' refers to ... blackboard variables that do not exist: params`, explicitly pass the embedded debug parameter protobuf:
    ```python
    executive.run(
        solution_bt,
        parameters=solution_bt.user_data_protos.get("PARAMETER_DEBUG_VALUES"),
    )
    ```
- **`CEL` wrapper unwrapping and case-sensitive wrapper types**:
  - `CEL` automatically unwraps `google.protobuf.*Value` wrapper types into primitive values (`int`, `uint`, `string`) during evaluation. When re-wrapping integer outputs, standard protobuf unsigned wrapper types require a capital `I` (`google.protobuf.UInt64Value` and `google.protobuf.UInt32Value`, not `Uint64Value`).
- **Reserved keywords and string map key quoting in `CEL`**:
  - Choose protobuf field names and blackboard identifiers distinct from reserved `CEL` keywords (`false`, `in`, `null`, `true`).
  - When indexing a `BlackboardValue` map with a string key in Python, quote the string inside brackets (`bv.items['"my_key"']`). Python's `__getitem__` formats `f"[{key}]"` directly, so passing an unquoted string emits `[my_key]` in `CEL` (parsed as a variable identifier, risking lookup failure or expression injection). Callers must supply `bv.items['"my_key"']` to generate a `CEL` string literal lookup.
- **Protobuf reflection errors on repeated field assignments**:
  - Assigning a singular `CEL` expression result to a repeated parameter path without an index causes `InternalSetFieldValue` to invoke `MutableMessage` on a repeated field, aborting the Executive. Ensure the target `parameter_path` matches the schema field cardinality.
- **Shallow `bt.ExtendedStatusMatch` vs. deep recursive `CEL` matching**:
  - `bt.ExtendedStatusMatch(blackboard_key="...", matcher=bt.ExtendedStatusMatch.MatchStatusCode(component="...", code=...))` inspects only the top-level `status_code` on the blackboard and does not traverse `ExtendedStatus.context`.
  - When a failing skill is wrapped inside a composite control node (`Fallback` `31100`, `Retry` `31300`, or `Sequence` `31800`), the composite code replaces the top-level status code and pushes the skill's root-cause code into `.context`. Use `bt.Blackboard(cel_expression=...)` with recursive `.exists()` checks over `.context` to match wrapped child error codes.
- **Suppressing spurious `GetStatus` `TCP` probe errors with `opt-out-error-checking`**:
  - Periodic `early eof` TCP read errors every 1–1.5 seconds on custom middleware, proxies, or non-HTTP services are caused by `SolutionService/GetStatus` liveness polling opening and closing raw TCP sockets.
  - Apply the label `intrinsic.ai/opt-out-error-checking="true"` to the pod or deployment template to instruct cluster services to bypass intrusive TCP health checking on sensitive socket endpoints.

## Input-aware decision tree for executive and behavior tree diagnostics

```
[Precondition: Executive operation or behavior tree exhibits unexpected failure]
  │
  ├─► Symptom: StartOperation fails immediately with ai.intrinsic.executive:21101
  │     └─► Diagnostic check: Inspect joint state alignment between icon, ur_module, and "init_world".
  │     └─► Targeted action: Reconcile kinematic frame offsets and reset belief world via inctl world reset.
  │
  ├─► Symptom: Process staged via inctl process set but robot remains idle
  │     └─► Diagnostic check: Query ExecutiveService/GetOperation or ListOperations.
  │     └─► Targeted action: Invoke ExecutiveService/StartOperation via gRPC or solution.executive.run().
  │
  ├─► Symptom: Re-submitting tree fails with UNIMPLEMENTED or recursion depth limit 100
  │     └─► Diagnostic check: Check serialized BehaviorTree proto size (> 3 MB indicates unstripped state).
  │     └─► Targeted action: Strip OUTPUT_ONLY fields (RunMetadata, node state, called_tree_state, stdout_list).
  │
  ├─► Symptom: Motion or perception skill fails with DEADLINE_EXCEEDED on run 1, succeeds on run 2
  │     └─► Diagnostic check: Compare skill timestamps against MotionPlannerService logs.
  │     └─► Targeted action: Downstream mesh caching stall; preserve warm pod state or increase initial deadline.
  │
  ├─► Symptom: Real-time robot motion exhibits timing jitter or clock stalls
  │     └─► Diagnostic check: Inspect PagefaultInfo [Abs Major | Abs Minor | Rel Major | Rel Minor] in controller log.
  │     └─► Targeted action: If Rel Major == 0 and Rel Minor == 0, mlockall is active; investigate trajectory planning.
  │
  ├─► Symptom: CancelOperation hangs or rejects subsequent requests with Incompatible state
  │     └─► Diagnostic check: Inspect inctl service state list and kubectl get pods for CrashLoopBackOff.
  │     └─► Targeted action: Restart dead skill pods to unblock ClearSkillOperations and release executor lock.
  │
  ├─► Symptom: Skill bool or scalar parameter unexpectedly reverts to manifest default
  │     └─► Diagnostic check: Search executive logs for "Parameters for skill ... were modified by merging defaults".
  │     └─► Targeted action: Add optional keyword to scalar parameter definition in skill .proto file.
  │
  ├─► Symptom: CEL expression returns 15101 (missing params) or 15150 (parse error)
  │     └─► Diagnostic check: Check if params passed to executive.run() and check for reserved tokens (in, null, true).
  │     └─► Targeted action: Supply PARAMETER_DEBUG_VALUES to run() and rename colliding protobuf field identifiers.
  │
  └─► Symptom: Custom service or middleware logs early eof TCP read errors every 1-1.5s
        └─► Diagnostic check: Confirm TCP probe requests originate from SolutionService/GetStatus liveness polling.
        └─► Targeted action: Annotate pod template with intrinsic.ai/opt-out-error-checking="true".
```

## Contrastive bifurcation analysis

| Diagnostic scenario | Failing trajectory (tau-) | Succeeding trajectory (tau+) | Bifurcation point invariant |
| :--- | :--- | :--- | :--- |
| **`CLI` staging vs. run** | Running `inctl process set` and assuming the process has started execution. | Invoking `ExecutiveService/StartOperation` or `solution.executive.run()` after staging. | `inctl process set` stages `CreateOperation` only; live execution requires `StartOperation`. |
| **Tree re-submission** | Re-submitting an executed tree directly to `CreateOperation`, triggering 100-level recursion abort. | Stripping `RunMetadata` and `OUTPUT_ONLY` fields before serializing the tree message. | Output-only execution state must be stripped before tree re-submission to stay within proto limits. |
| **`Proto3` parameter overrides** | Omitting `optional` on scalar fields, allowing `false` or `0` to be overwritten by manifest defaults. | Declaring scalar fields as `optional` in Proto3 schema to preserve wire presence. | Explicit field presence (`optional`) is required to distinguish `false`/`0` from unset fields. |
| **Composite error recovery** | Using shallow `bt.ExtendedStatusMatch` on a composite node, failing to match child errors. | Querying `.context` recursively via `CEL` `.exists()` expressions to locate root-cause codes. | Composite control nodes wrap child errors and push original status codes into `.context`. |
| **String map indexing** | Indexing `bv.items[key]`, generating unquoted identifier `items[my_key]` and lookup failure. | Indexing `bv.items['"my_key"']`, generating quoted literal `items["my_key"]`. | Python string values must contain quotes to generate valid `CEL` string literal lookups. |

## `System 2` reflection checkpoint and anti-thrashing circuit breaker

### Pre-execution reflection checklist
Before executing high-cost mutations, modifying behavior tree definitions, or resetting cluster services, verify:
1. **Staging verification**: Has `StartOperation` been invoked following `CreateOperation`, or is the process merely staged?
2. **Protobuf hygiene**: Have all `OUTPUT_ONLY` runtime fields been stripped before submitting the behavior tree proto?
3. **Parameter presence**: Are all scalar parameters intended to hold `false` or `0` marked as `optional` in their `.proto` schema?
4. **Error hierarchy**: Does the recovery logic traverse `.context` via recursive `CEL` expressions if matching errors beneath composite nodes?
5. **Real-time health**: Has `PagefaultInfo` confirmed zero relative page faults before attributing timing jitter to real-time control?

### Anti-thrashing circuit breaker
To prevent repetitive trial-and-error loops during debugging:
- If a behavior tree fails validation or execution >= 3 consecutive times with identical error codes (`15101`, `15150`, or `21101`), halt parameter and string modifications.
- Immediately export the live operation protobuf via `ExecutiveService/GetOperation`, inspect the raw serialized message size and blackboard items, and verify skill pod container readiness before attempting another execution.

## Paired negative guardrails

1. **Do not** pass behavior trees containing populated `OUTPUT_ONLY` runtime fields to `CreateOperation`; strip execution metadata (`RunMetadata`, `called_tree_state`, node `state`, `stdout_list`) before serialization to stay below the 100-level recursion ceiling and 4 MB channel limit.
2. **Do not** leave `GetFootprint()` unimplemented in custom skills intended for parallel execution; implement `GetFootprint()` with `lock_the_universe: false` and reserve only required equipment slots to prevent the Executive from serializing parallel branches.
3. **Do not** index `BlackboardValue` with unquoted string identifiers in Python (`bv.items[key]`); pass quoted strings (`bv.items['"key"']`) to produce a `CEL` string literal lookup and prevent identifier resolution errors or expression injection.
4. **Do not** declare scalar skill parameters without the `optional` keyword in Proto3 schemas; mark scalar fields as `optional` so explicit `false` or `0` values are serialized and not overwritten by manifest defaults in `UnpackAnyAndMerge`.
5. **Do not** use shallow `bt.ExtendedStatusMatch` when catching errors wrapped by composite control nodes (`Fallback`, `Retry`, `Sequence`); query `ExtendedStatus.context` recursively using `CEL` `.exists()` expressions to inspect nested child root-cause codes.
