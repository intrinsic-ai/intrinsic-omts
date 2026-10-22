# Composing behavior trees, `CEL` expressions, and `PBT` signatures in `SBL`

## Input-aware decision tree for control-flow and node selection

When designing or modifying behavior trees in the Solution Building Library (SBL), use the following input-aware decision tree to select the correct control node, decorator, or action leaf:

```
                                  What is the architectural requirement?
                                                    │
       ┌──────────────────────────────┬─────────────┴────────────────┬──────────────────────────────┐
       ▼                              ▼                              ▼                              ▼
Sequential steps            Conditional branching          Fallback / recovery            Concurrent / iterative
       │                              │                              │                              │
Abort on 1st fail?          Binary or multi-branch?        Try until 1st success?         All run / repeat?
       │                              │                              │                              │
 [ bt.Sequence ]               ┌──────┴──────┐                [ bt.Fallback ]                ┌──────┴──────┐
(Implicit wrap)                ▼             ▼             (Explicit Task wrap)              ▼             ▼
                         Binary path?   Priority select?             │                 All succeed?   While / count?
                               │             │               ┌───────┴───────┐               │             │
                          [ bt.Branch ] [ bt.Selector ]      ▼               ▼         [ bt.Parallel ] [ bt.Loop ]
                         (Implicit wrap)(Explicit wrap) Bounded retry? Catch failure?  (Fail aborts)  (loop_counter)
                                                             │               │
                                                       [ bt.Retry ] [ bt.SubTreeCondition ]
                                                     (0-indexed cnt) (Converts fail to false)
```

| Composition requirement | Node type | Canonical SBL class | Execution semantics and wrapping contract |
| :--- | :--- | :--- | :--- |
| **Sequential steps** | Composite control | `bt.Sequence(children=[...])` | Executes children in order; succeeds when all succeed; aborts on first failure (`31800`). Supports implicit `bt.Task` wrapping. Clear children with `set_children([])`. |
| **Binary conditional branch** | Composite control | `bt.Branch(if_condition=..., then_child=..., else_child=...)` | Evaluates single condition; routes to `then_child` if true, `else_child` if false. Supports implicit `bt.Task` wrapping on both child branches. |
| **Multi-branch conditional selection** | Composite control | `bt.Selector(branches=[...])` | Evaluates branch conditions in order; executes first true branch (`31400`). Branches require explicit `bt.Selector.Branch(condition=..., node=bt.Task(...))`. |
| **Prioritized recovery** | Composite control | `bt.Fallback(tries=[...])`<br>`bt.Fallback(children=[...])` | Executes branches in order; succeeds on first success; fails when all fail (`31100`). `bt.Fallback.Try` requires explicit `bt.Task(action)`. `children` and `tries` are mutually exclusive. |
| **Concurrent branches** | Composite control | `bt.Parallel(children=[...])` | Executes children concurrently; succeeds when all succeed; fails if any child fails (`31200`). Recovery nodes across parallel branches must reside in distinct subtrees. |
| **Bounded retry** | Composite / decorator | `bt.Retry(max_tries=N, child=..., recovery=...)` | Retries `child` up to `max_tries` times. If `recovery` fails, retry aborts immediately (`31300`). Provides 0-indexed `retry_node.retry_counter`. |
| **Iterative execution** | Composite control | `bt.Loop(max_times=N, while_condition=..., do_child=...)` | Evaluates condition on blackboard before each iteration; aborts if condition generator fails (`32001`). Index repeated fields via `loop_node.loop_counter`. |
| **Skill or script leaf** | Action leaf | `bt.Task(action=skill)`<br>`bt.PythonScript(...)` | Executes skill invocation or embedded Python script (`31500`). Provides fluent `.on_failure` builder to emit or forward `ExtendedStatus`. |
| **Blackboard mutation** | Action leaf | `bt.Data(blackboard_key=..., operation=..., ...)` | Creates, updates, or removes keys on the blackboard via CEL expression, protobuf message, or world query (`32100`). |
| **Subtree container** | Structural composite | `bt.SubTree(behavior_tree=...)` | Encapsulates a reusable behavior tree or root node (`31900`). |
| **Failure condition check** | Condition decorator | `bt.SubTreeCondition(node)` | Converts child execution failure into boolean `False` without propagating a failure state to parent nodes. |
| **Explicit branch failure** | Action leaf | `bt.Fail(name=..., failure_message=...)` | Explicitly fails branch with `31600`. Mutual exclusion applies between `failure_message` and custom title decorators. |

---

## Control-flow execution semantics and node wrapping rules

The `intrinsic.solutions.behavior_tree` module (`from intrinsic.solutions import behavior_tree as bt`) provides control-flow composition nodes and execution decorators.

### Implicit vs. explicit `bt.Task` wrapping rules

1. **Where implicit task wrapping works**:
   - Passing a skill invocation (`ActionBase` from `solution.skills`) or `bt.PythonScript` directly to `bt.BehaviorTree(root=...)`, `bt.Sequence(children=[...])`, `bt.Parallel(children=[...])`, `bt.Loop(do_child=...)`, `bt.Branch(then_child=..., else_child=...)`, `bt.Retry(child=..., recovery=...)`, or `bt.Fallback(children=[...])` automatically wraps the action in `bt.Task(action)` via `_transform_to_node()`.
2. **Paired negative guardrail 1**:
   - Do not pass unwrapped skill invocations or code nodes directly to `bt.Fallback.Try` or `bt.Selector.Branch`; wrap actions explicitly in `bt.Task(action)` before assigning to `node=`.
   - *Rationale*: `Fallback.Try` and `Selector.Branch` are typed dataclasses without dynamic transform hooks. Passing unwrapped skills fails at protobuf serialization with `TypeError: expected BehaviorTree.Node got BehaviorCall`.

```python
from intrinsic.solutions import behavior_tree as bt

# Correct explicit bt.Task wrapping inside bt.Fallback.Try
fallback_node = bt.Fallback(
    name="Grasp with fallback",
    tries=[
        bt.Fallback.Try(
            condition=bt.Blackboard(f"{check_skill.result.is_ready}"),
            node=bt.Task(action=primary_grasp_skill, name="Primary grasp"),
        ),
        bt.Fallback.Try(
            condition=None,
            node=bt.Task(action=backup_grasp_skill, name="Backup grasp"),
        ),
    ],
)
```

### Child node manipulation and clearing syntax

- **Paired negative guardrail 2**:
  - Do not call `seq.set_children()` with zero arguments; pass an empty list `seq.set_children([])` or assign `seq.children = []`.
  - *Rationale*: `NodeWithChildren.set_children()` inspects `children[0]`. Calling it without arguments raises `IndexError: tuple index out of range`.

### Explicit failure nodes and decorator mutual exclusion

- When using `bt.Fail(name="...", failure_message="...")`, passing a non-empty `failure_message` populates `decorators.on_failure.emit_extended_status.extended_status.title`.
- Setting both `failure_message="..."` on `bt.Fail` and configuring `fail_node.on_failure.emit_extended_status(title=...)` raises a `ValueError` during proto serialization. When emitting structured `ExtendedStatus` messages, instantiate `bt.Fail(name="...")` with `failure_message=""` and configure error details exclusively via `.on_failure.emit_extended_status(...)`.

---

## Blackboard dataflow, `CEL` expressions, and type resolution

### Blackboard value references and key determinism

- **Deterministic vs. UUID return value keys**:
  - Instantiating a skill generates a default unique blackboard key (`f"{skill_name}_{uuid.uuid4()}"`).
  - Pass `return_value_key="detected_pose"` during skill instantiation to assign a deterministic blackboard key for multi-skill data pipelines.
  - Access typed return values via `skill.result` (`BlackboardValue`). Subfield accesses (`skill.result.estimated_pose.position.x`) emit dot-separated Common Expression Language (CEL) paths (`"detected_pose.estimated_pose.position.x"`).
- **Special CEL conversion for duration and timestamp types**:
  - CEL treats `google.protobuf.Duration` and `Timestamp` as built-in types rather than standard messages, forbidding direct `.seconds` and `.nanos` field accesses.
  - `BlackboardValue.__getattribute__` transparently converts `.seconds` to `f"{path}.getSeconds()"` and `.nanos` to `f"{path}.getMilliseconds() * 1000000"`.

### Repeated field indexing and quoted string keys

- **Quoted string keys for CEL literal indexing and injection prevention**:
  - Repeated fields on `BlackboardValue` support indexing by integer literal (`skill.result.items[0]`), another `BlackboardValue` (`skill.result.items[loop_node.loop_counter]`), or string index key.
  - In `blackboard_value.py`, `BlackboardValue.__getitem__(key)` constructs the CEL access path via `f"[{key}]"`. When `key` is a Python `str`, `f"[{key}]"` omits quote characters, producing `items[my_key]` (which CEL parses as a variable identifier rather than a string literal).
  - To index by a string literal in CEL, pass the quotes inside the Python string content: `skill.result.items['"part_name"']` or `skill.result.items["\"part_name\""]`. This ensures CEL evaluates `items["part_name"]` as a string map lookup and eliminates CEL expression injection risks (`CWE-94`).

### Parameter passing and assignment limitations

- **Assigning BlackboardValue and CelExpression into skills**:
  - Direct assignment:
    ```python
    move_skill = skills.move_to_pose(target_pose=detect_skill.result.pose)
    ```
  - Arithmetic or string CEL expressions (`from intrinsic.solutions import cel`):
    ```python
    offset_x = cel.CelExpression(f"{detect_skill.result.pose.position.x} + 0.05")
    ```
- **Preserving CelExpression identity**:
  - Preserve `cel.CelExpression` objects directly in parameter dictionaries and constructors to ensure parameter builders recognize and serialize them as dynamic CEL expressions rather than literal strings.
- **Protobuf reflection cardinality on repeated fields**:
  - When assigning a singular CEL expression to a repeated parameter path, specify an element index (`items[0]`). Assigning a singular CEL expression to an unindexed repeated field causes runtime reflection errors.
- **Paired negative guardrail 3**:
  - Do not assign `BlackboardValue` or `CelExpression` to protobuf `map<K, V>` fields; decompose maps into indexed repeated messages or set static configurations in python.
  - *Rationale*: SBL parameter serialization raises `TypeError: Cannot set field ... from blackboard value, not supported for maps` when targeting map fields.

### `CEL` type unwrapping and casing rules

- **Wrapper unwrapping and re-wrapping**:
  - Per the CEL specification, CEL automatically unwraps `google.protobuf.*Value` wrapper types into primitive CEL values (`int`, `uint`, `string`) during expression evaluation.
  - When converting CEL unsigned integers back into protobuf wrapper messages, use capital `I`: `google.protobuf.UInt64Value` and `google.protobuf.UInt32Value` (not `Uint64Value`).
- **Reserved CEL keywords**:
  - Use identifier names distinct from reserved CEL tokens (`false`, `in`, `null`, `true`) for protobuf fields and blackboard variables to prevent CEL parser errors (`ai.intrinsic.executive:15150`).
- **Process parameter scope initialization**:
  - When executing behavior trees that reference `params.<field>`, supply parameter payloads via `executive.run(solution_bt, parameters=solution_bt.user_data_protos.get("PARAMETER_DEBUG_VALUES"))` to instantiate the `params` blackboard scope.

---

## Error handling, recovery subtrees, and deep status matching

Every `bt.Node` exposes a fluent `.on_failure` builder property to emit custom `ExtendedStatus` messages or forward native skill errors to the blackboard.

### Emitting and forwarding extended status

1. **Emitting custom status on failure**:
   ```python
   grasp_task = bt.Task(action=grasp_skill, name="Grasp part")
   grasp_task.on_failure.emit_extended_status(
       component="ai.intrinsic.solutions.tending",
       code=10042,
       title="Failed to grasp workpiece",
       user_message="Verify gripper jaws are unobstructed.",
       debug_message="Force threshold not reached within 2.5s",
       to_blackboard_key="last_grasp_error",
   )
   ```
2. **Forwarding native skill status directly**:
   ```python
   insert_task = bt.Task(action=insert_skill, name="Insert part")
   insert_task.on_failure.emit_extended_status_to("insert_error")
   ```

### Catalog of built-in executive failure codes

When control-flow or decorator nodes fail, the executive emits structured status messages with component `"ai.intrinsic.executive"`:

| Code | Title | Failure cause |
| :--- | :--- | :--- |
| `31000` | `"Disabled node failed"` | Node was skipped via `node.disable_execution(result_state=bt.DisabledResultState.FAILED)`. |
| `31001` | `"Node decorator condition unsatisfied"` | Decorator condition evaluated to `false`. |
| `31010` | `"Unreported node failure"` | Child node terminated with failure without emitting a status code. |
| `31100` | `"Fallback node failed"` | All branches of `bt.Fallback` failed. Child errors are attached to `.context`. |
| `31200` | `"Parallel node failed"` | One or more children in `bt.Parallel` failed. |
| `31300` | `"Retry node failed"` | All retry attempts failed (or `recovery` subtree failed). History is attached to `.context`. |
| `31400` | `"Selector node failed"` | No branch condition in `bt.Selector` evaluated to `true`, or selected branch failed. |
| `31500` | `"Task node failed"` | An underlying skill or task node failed. |
| `31600` | `"Fail node failed"` | Explicit `bt.Fail` node executed. |
| `31700` | `"Debug node failed"` | A `bt.Debug(fail_on_resume=True)` node was resumed. |
| `31800` | `"Sequence node failed"` | A child node of `bt.Sequence` failed. |
| `31900` | `"Subtree node failed"` | An invoked `bt.SubTree` node failed. |
| `32001` | `"Loop node failed to generate loop protos"` | A `bt.Loop` node failed while evaluating its loop generator expression. |
| `32100` | `"Data node failed"` | Blackboard mutation via `bt.Data` failed. |

### Error matching syntax and the shallow matching limitation

- **Paired negative guardrail 4**:
  - Do not pass `status_code=` as a keyword argument to `bt.ExtendedStatusMatch`; pass `matcher=bt.ExtendedStatusMatch.MatchStatusCode(...)`.
  - *Rationale*: The second constructor parameter is named `matcher`. Passing `status_code=` raises `TypeError: ExtendedStatusMatch.__init__() got an unexpected keyword argument 'status_code'`.

```python
recovery_branch = bt.Sequence(children=[retract_task, retry_task]).set_decorators(
    bt.Decorators(
        condition=bt.ExtendedStatusMatch(
            blackboard_key="insert_error",
            matcher=bt.ExtendedStatusMatch.MatchStatusCode(
                component="ai.intrinsic.insertion_skill",
                code=1001,
            ),
        )
    )
)
```

- **Shallow matching limitation and Skills SDK wrapping**:
  - `bt.ExtendedStatusMatch` checks **only** the top-level `status_code` on the blackboard.
  - When a failing task is enclosed inside a composite node (`Fallback` `31100`, `Retry` `31300`, or `Sequence` `31800`), the composite node's wrapper status replaces the top-level status and pushes the root-cause status into `.context`.
  - Similarly, when a skill returns an `ExtendedStatus` from a downstream service whose component differs from the skill ID, the Skills SDK runtime wraps it in a top-level `ExtendedStatus` with `code=0`.
  - For nested trees and downstream service errors, use deep recursive CEL matching with `.exists()` over `.context`:
    ```python
    from intrinsic.solutions import cel

    deep_match_expr = (
        "(has(last_error.status_code) && last_error.status_code.code in [1001, 1002]) || "
        "(has(last_error.context) && last_error.context.exists(c0, "
        "(has(c0.status_code) && c0.status_code.code in [1001, 1002]) || "
        "(has(c0.context) && c0.context.exists(c1, "
        "(has(c1.status_code) && c1.status_code.code in [1001, 1002])))))"
    )
    deep_match_condition = bt.Blackboard(cel_expression=cel.CelExpression(deep_match_expr))
    ```

### Idiomatic recovery and assertion patterns

1. **Precondition assertion pattern**:
   ```python
   check_condition = cel.CelExpression(f"({sensor_val} > 0.0) && ({sensor_val} <= 0.5)")
   precondition_guard = bt.Branch(
       if_condition=bt.Blackboard(check_condition),
       else_child=bt.Fail(name="Precondition violated", failure_message="Sensor value out of bounds"),
   )
   safe_sequence = bt.Sequence(children=[sensor_read_task, precondition_guard, motion_task])
   ```
2. **Cleanup-and-fail pattern**:
   ```python
   # Execute motion; if it fails, clean up hardware and explicitly fail so parent sees error
   cleanup_sequence = bt.Sequence(children=[rehome_task, release_gripper_task, bt.Fail(name="Motion failed")])
   safe_execution = bt.Fallback(children=[execute_motion_sequence, cleanup_sequence])
   ```
3. **Tiered recovery with retry counter**:
   - `retry_node.retry_counter` is 0-indexed (`0` on initial run, `1` on first retry).
   - Reference `retry_node.retry_counter` in branch conditions to escalate recovery actions across attempts.

---

## Process recovery, recursion limits, and parameterizable behavior trees

### Resuming execution from specific nodes

When a long-running behavior tree fails, resume execution from specific nodes while preserving blackboard state using `solution.executive.run(...)`:

```python
target_id = my_tree.get_node_identifier(place_part_sequence, autogenerate_missing_ids=True)
solution.executive.run(
    my_tree,
    recover_from_nodes=[target_id],
    keep_blackboard=True,
)
```

**Topological invariants and recovery status codes**:
- Select unique node identifiers for all recovery targets (`70101` Duplicate recovery nodes found).
- Select recovery nodes that occupy distinct, non-overlapping subtree branches (`70102` Recovery node not found).
- Target operational action or control nodes rather than condition decorators (`70102` Recovery node not found).
- Ensure multiple recovery nodes reside in separate branches of a `bt.Parallel` node (`70103` Ambiguous recovery).
- Supply required parameter values before resuming reusable processes (`70150` Recovery failed to parameterize reusable process).

### Protobuf recursion limits and runtime state stripping

- **Paired negative guardrail 5**:
  - Do not resubmit behavior trees containing populated `OUTPUT_ONLY` runtime fields to `CreateOperation`; strip execution metadata (`called_tree_state`, `node_state`, `stdout_list`) before serialization to stay below the 100-level recursion ceiling.
  - *Rationale*: Authored behavior trees are compact (~50KB), but executing them embeds deep runtime execution metadata. Resubmitting an unstripped tree exceeds protobuf's default 100-level recursion ceiling and 4MB message limit, triggering deceptive `UNIMPLEMENTED: Failed to fetch installed subprocess assets` or `cannot decode message cel.expr.Expr.Select` errors.

### Parameterizable behavior trees and script nodes

1. **Asset metadata declaration ordering**:
   - Call `tree.set_asset_metadata(...)` before calling `tree.set_signature(...)`. Calling `set_signature` first raises `TypeError: set_asset_metadata() must be called before set_signature()`.
   - Access process inputs inside the tree via `tree.params.<field_name>`.
2. **Dynamic protobuf schemas for bt.PythonScript**:
   - Construct custom input/output schemas using `solution.proto_builder.create_signature_with_args(...)`.
   - Script node outputs use process-scoped type URLs (`type.intrinsic.ai/assets/<process_id>/<MessageFullName>`). Register custom script node descriptor sets when inspecting script node outputs.

---

## Contrastive bifurcation analysis

When authoring or troubleshooting SBL behavior trees, ground modifications at the exact bifurcation point where failing trajectories diverge from succeeding ones:

| Domain | Failing trajectory (tau-) | Succeeding trajectory (tau+) | Bifurcation point invariant |
| :--- | :--- | :--- | :--- |
| **Task wrapping** | Passing raw skill `bt.Fallback.Try(node=grasp_skill)` causes `TypeError: expected BehaviorTree.Node got BehaviorCall` during serialization. | Wrapping action explicitly `bt.Fallback.Try(node=bt.Task(grasp_skill))` serializes cleanly. | Composite dataclasses require explicit `bt.Node` instances. |
| **CEL map indexing** | Writing `bv.items["part_name"]` produces `items[part_name]`, causing CEL variable lookup failure. | Writing `bv.items['"part_name"']` produces `items["part_name"]`, resolving as a string literal. | Python string content must include quotes for CEL literal indexing. |
| **Tree resubmission** | Resubmitting an executed tree with populated `RunMetadata` triggers `100` recursion ceiling abort. | Clearing `OUTPUT_ONLY` runtime fields before resubmission serializes within 50KB limits. | Runtime execution state must be stripped before tree resubmission. |
| **Error recovery** | Using `bt.ExtendedStatusMatch` on a composite node's output fails to match wrapped child errors. | Using deep recursive CEL with `.exists()` traverses `.context` to detect root-cause status codes. | Composite nodes bury child errors inside `.context`. |
| **Node clearing** | Calling `seq.set_children()` raises `IndexError: tuple index out of range`. | Calling `seq.set_children([])` or `seq.children = []` clears child nodes safely. | `set_children` requires an iterable list argument. |

---

## System 2 reflection checkpoints and anti-thrashing circuit breakers

### Pre-execution `System 2` reflection checklist

Before compiling, serializing, or executing an SBL behavior tree, verify these four platform constraints:

1. **Task wrapping verification**: Are all skill invocations inside `bt.Fallback.Try` and `bt.Selector.Branch` explicitly wrapped in `bt.Task(action)`?
2. **Blackboard scope check**: If the tree references `params.<field>`, are parameter payloads passed to `executive.run(parameters=...)`?
3. **Cardinality alignment**: Do singular CEL assignments target singular protobuf fields, or specify an index for repeated fields?
4. **State cleanliness**: If re-submitting an existing tree, have all output-only runtime fields been cleared to avoid protobuf recursion aborts?

### Anti-thrashing circuit breaker for tree validation

When encountering behavior tree validation or CEL parsing errors, enforce a bounded retry budget:

```
                  Validation or CEL parsing failure
                                  │
                       Attempt 1: Fix syntax / types
                                  │
                         Still failing?
                                  │
                       Attempt 2: Check descriptor schemas
                                  │
                         Still failing?
                                  │
                 ┌────────────────┴────────────────┐
                 ▼                                 ▼
         [ Circuit breaker: HALT edits ]   [ Branch to diagnostic inspection ]
         + Halt blind code modifications   + Inspect serialized proto size
         + Prevent file-edit churn loops   + Check for unstripped runtime state
                                           + Verify protobuf recursion depth
```

If tree validation or CEL parsing fails after <= 3 verification cycles, halt file edits immediately and branch to diagnostic analysis:
1. Inspect the raw serialized protobuf size using `inctl process get <name> --server=localhost:17080`. If size exceeds 1MB, verify output-only runtime fields are stripped.
2. Check for CEL keyword collisions (`in`, `null`, `true`, `false`) across field names and identifiers.
3. Validate that string map keys are explicitly quoted inside the Python string content (`['"key"']`) to prevent variable lookup errors or expression injection.
