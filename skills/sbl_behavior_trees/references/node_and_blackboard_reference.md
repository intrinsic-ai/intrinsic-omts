# SBL Behavior Tree Node, Blackboard, CEL, and `bt.PythonScript` Reference

Exact constructor signatures, properties, execution semantics, and code patterns
for the Intrinsic Solution Building Library (SBL) Behavior Tree API
(`from intrinsic.solutions import behavior_tree as bt`).

## 1. Open-Source SDK Layout & Imports

In `intrinsic-omts`, SBL sources are fetched via Bazel Bzlmod into
`EXT="$(bazel info output_base)/external"`:

| Bzlmod Repo | Local Path under `$EXT` | Key Modules |
| :--- | :--- | :--- |
| `@ai_intrinsic_sdks` | `$EXT/ai_intrinsic_sdks+/intrinsic/solutions/` | `behavior_tree.py`, `cel.py`, `blackboard_value.py`, `proto_building.py`, `execution.py`, `deployments.py`, `worlds.py`, `provided.py`, `pbt_registration.py` |
| `@ai_intrinsic_sdks` | `$EXT/ai_intrinsic_sdks+/intrinsic/world/python/` | `object_world_client.py` (`ObjectWorldClient`), `object_world_resources.py` (`WorldObject`, `KinematicObject`, `Frame`) |
| `@ai_intrinsic_sdks` | `$EXT/ai_intrinsic_sdks+/intrinsic/skills/python/` | `basic_compute_context.py` (`BasicComputeContext`) |
| `@intrinsic-core` | `$EXT/intrinsic-core+/intrinsic/solutions/internal/` | `code_execution.py` (`PythonScript`, `get_function_body_as_str`), `actions.py` (`ActionBase`), `skill_generation.py`, `blackboard.py`, `resources.py` |

```python
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import cel, deployments, execution
from intrinsic.solutions import proto_building as pb
```

## 2. Base `bt.Node`, Identifiers, Decorators, and Naming Rules

### 2.1 Identifiers, Enums, `bt.Decorators`, and `bt.Node`

```python
@dataclasses.dataclass(frozen=True, kw_only=True)
class NodeIdentifier:
  tree_id: str  # Non-empty string matching ^[a-zA-Z0-9][a-zA-Z0-9_-]*$
  node_id: int  # Non-zero uint32 in [1, 0xFFFFFFFF]


class NodeName(str):
  pass


class NodeId(int):
  pass


NodeInTreeType = Union[NodeIdentifier, NodeName, NodeId]


class Decorators:
  def __init__(
    self,
    condition: bt.Condition | None = None,
    breakpoint_type: bt.BreakpointType | None = None,
    execution_mode: bt.NodeExecutionMode | None = None,
    disabled_result_state: bt.DisabledResultState | None = None,
  ) -> None: ...


class Node(abc.ABC):
  def __init__(
    self, name: str | None = None, node_id: int | None = None
  ) -> None: ...
```

- **Enums on `bt`**:
  - `bt.NodeState`: `UNSPECIFIED`, `ACCEPTED`, `SELECTED`, `RUNNING`,
    `SUCCEEDED`, `FAILED`, `CANCELED`, `SUSPENDED`, `SUSPENDING`, `CANCELING`.
  - `bt.BreakpointType`: `LOG` (`1`), `PAUSE` (`2`).
  - `bt.NodeExecutionMode`: `NORMAL`, `DISABLED`.
  - `bt.DisabledResultState`: `SUCCEEDED`, `FAILED`.
- **Methods & Properties on every `bt.Node`**:
  - `.name -> str | None`, `.node_id -> int | None`, `.state -> NodeState | None`
  - `.decorators -> Decorators | None`,
    `.set_decorators(decorators: Decorators | None) -> Node`
  - `.breakpoint -> BreakpointType`,
    `.set_breakpoint(breakpoint_type: BreakpointType | None) -> Node`
  - `.execution_mode -> NodeExecutionMode`,
    `.disable_execution(result_state: DisabledResultState | None = None) -> Node`
    (skips execution at runtime and finishes in `result_state`, defaulting to
    `SUCCEEDED`), `.enable_execution() -> Node`
  - `.generate_and_set_unique_id() -> int`
  - `.set_user_data_proto(key: str, proto: message.Message) -> Node`,
    `.delete_user_data_proto(key: str) -> None`, `.user_data_protos`
  - `.set_user_data_bytes(key: str, value: bytes) -> Node`,
    `.delete_user_data_bytes(key: str) -> None`, `.user_data_bytes`
  - `.on_failure -> Node.FailureSettings`:
    - `.emit_extended_status(component: str, code: int, *, title: str = "", user_message: str = "", debug_message: str = "", to_blackboard_key: str = "") -> Node`
    - `.emit_extended_status_proto(extended_status: extended_status_pb2.ExtendedStatus, to_blackboard_key: str = "") -> Node`
    - `.emit_extended_status_to(blackboard_key: str) -> Node`: Writes the node's
      failure `ExtendedStatus` proto to `blackboard_key` for matching via
      `bt.ExtendedStatusMatch`.
  - `.show() -> None`, `.dot_graph() -> pgv.AGraph`

### 2.2 Auto-Wrapping, ID Uniqueness, and Node Naming Rules

1. **Auto-Wrapping (`_transform_to_node`)**: Composite node parameters typed as
   `Node | ActionBase | CodeExecution` automatically wrap `ActionBase` (skill
   call) or `CodeExecution` (`bt.PythonScript`) into `bt.Task(node)`. Wrap
   explicitly in `bt.Task(action=..., name="...")` when setting `.name`,
   `.node_id`, `.decorators`, or `.on_failure`.
2. **ID Uniqueness**: Accessing `tree.proto` calls
   `tree.ensure_all_unique_ids()`. `tree.validate_id_uniqueness()` verifies that
   all `tree_id`s across the hierarchy are unique and all `node_id`s within each
   `BehaviorTree` are unique.
3. **Node Naming Rules**:
   - Passing a `bt.Node` to `bt.SubTree(behavior_tree=node, name="...")`
     requires `name` to be non-`None` (`ValueError` otherwise).
   - Calling `tree.set_asset_metadata(...)` requires `tree.name` to be non-empty.
   - `bt.NodeName("...")` or `tree.find_tree_and_node_id("...")` requires the
     name to be unique across the tree (`InvalidArgumentError` if 0 or >1 match).
   - **OMTS Convention**: `tests/unit/test_behaviors.py` enforces that all
     direct children in each phase subtree `bt.Sequence` have **unique `.name`
     strings** and do **not** use `"Step "` or `"Prep:"` prefixes.

## 3. Composite, Structural, and Leaf Nodes

### 3.1 Constructors and Signatures

```python
class BehaviorTree:
  def __init__(
    self,
    name: str | None = None,
    root: bt.Node | bt.ActionBase | bt.CodeExecution | None = None,
    bt: bt.BehaviorTree | behavior_tree_pb2.BehaviorTree | None = None,
    *,
    tree_id: str | None = None,
  ) -> None: ...


class Sequence(NodeWithChildren):
  def __init__(
    self,
    children: Sequence[bt.Node | bt.ActionBase | bt.CodeExecution]
    | None = None,
    name: str | None = None,
    *,
    node_id: int | None = None,
  ) -> None: ...


class Parallel(NodeWithChildren):
  def __init__(
    self,
    children: Sequence[bt.Node | bt.ActionBase | bt.CodeExecution]
    | None = None,
    name: str | None = None,
    *,
    node_id: int | None = None,
  ) -> None: ...


class Fallback(bt.Node):
  @dataclasses.dataclass
  class Try:
    condition: bt.Condition | None
    node: bt.Node

  def __init__(
    self,
    children: list[bt.Node | bt.ActionBase | bt.CodeExecution] | None = None,
    name: str | None = None,
    *,
    tries: list[bt.Node | bt.ActionBase | bt.CodeExecution | bt.Fallback.Try]
    | None = None,
    node_id: int | None = None,
  ) -> None: ...


class Selector(NodeWithChildren):
  @dataclasses.dataclass
  class Branch:
    condition: bt.Condition | None
    node: bt.Node

  def __init__(
    self,
    branches: Sequence[
      bt.Node | bt.ActionBase | bt.CodeExecution | bt.Selector.Branch
    ]
    | None = None,
    name: str | None = None,
    *,
    children: Sequence[bt.Node | bt.ActionBase | bt.CodeExecution]
    | None = None,
    node_id: int | None = None,
  ) -> None: ...


class SubTree(bt.Node):
  def __init__(
    self,
    behavior_tree: bt.Node | bt.BehaviorTree | None = None,
    name: str | None = None,
    node_id: int | None = None,
  ) -> None: ...


class Task(bt.Node):
  def __init__(
    self,
    action: (
      bt.ActionBase
      | bt.CodeExecution
      | behavior_call_pb2.BehaviorCall
      | code_execution_pb2.CodeExecution
    ),
    name: str | None = None,
    node_id: int | None = None,
    task_state: behavior_tree_pb2.BehaviorTree.TaskNode.state | None = None,
  ) -> None: ...


class Fail(bt.Node):
  def __init__(
    self,
    failure_message: str = "",
    name: str | None = None,
    node_id: int | None = None,
  ) -> None: ...


class Debug(bt.Node):
  def __init__(
    self,
    fail_on_resume: bool | None = False,
    name: str | None = None,
    node_id: int | None = None,
  ) -> None: ...
```

### 3.2 Key Methods and Execution Semantics

- **`bt.BehaviorTree`**: `.set_root(root)`, `.root`, `.name`, `.tree_id`,
  `.proto`, `.ensure_all_unique_ids()`, `.validate_id_uniqueness()`,
  `.visit(callback)`, `.find_nodes_by_name(name)`,
  `.find_trees_and_nodes_by_name(name)`, `.find_tree_and_node_id(name)`,
  `.find_node_by_id(node_id)`, `.get_node_identifier(node_in_tree)`,
  `.remove_node(node_id)`, `BehaviorTree.create_from_proto(proto)`.
- **`bt.Sequence`**: Executes `children` in order (`.set_children(*children)`,
  mutable `.children` list). Succeeds when all children succeed; fails
  immediately if any child fails.
- **`bt.Parallel`**: Starts all `children` concurrently. Succeeds when all
  children succeed; cancels running children and fails if any child fails.
  - **Universe Lock Rule (`StatusCode: 18201`)**: `ai.intrinsic.update_world`
    declares `lock_the_universe: true`. Never place `update_world` (or
    `DioCncMachine` door/vise tasks) inside `bt.Parallel` alongside
    `move_robot` or `move_to_contact` (causes Executive `StatusCode: 18201`).
- **`bt.Fallback` (Try-Until-Success)**: Cannot pass both `children` and
  `tries` (`ValueError`). Evaluates `tries` in order, skipping any `Fallback.Try`
  whose `condition` is `False`. Succeeds as soon as one `Try.node` succeeds;
  moves to the next `Try` if `Try.node` fails. Fails only if all `tries` fail.
- **`bt.Selector` (Switch-Case — Distinct from `bt.Fallback`)**: Passing bare
  `children` is deprecated; use `branches=[bt.Selector.Branch(condition=..., node=...)]`.
  Evaluates branch conditions in order until the **first** branch with a `True`
  (or `None`) condition is selected, executes that branch's `node`, and
  **returns its outcome (`SUCCEEDED` or `FAILED`) directly** without falling
  back to subsequent branches if `branch.node` fails.
- **`bt.SubTree`**: Embeds a `BehaviorTree` (or wraps a `Node` when `name` is
  provided; `.set_behavior_tree(behavior_tree, name=None)`). Shares the parent
  tree's blackboard scope (`PROCESS_TREE`).
- **`bt.Task`**: Wraps an `ActionBase` or `CodeExecution`. `task.result`
  delegates to `action.result`.
- **`bt.Fail`**: Immediately fails and sets `ExtendedStatus.title` to
  `failure_message` (component `"ai.intrinsic.executive"`, code `11206`). Cannot
  combine `failure_message` with `.on_failure.emit_extended_status(title=...)`.
- **`bt.Debug`**: Suspends tree execution (`SUSPENDED`). On resume, succeeds if
  `fail_on_resume=False` or fails if `fail_on_resume=True`.

## 4. Branching and Conditions

### 4.1 `bt.Branch` and `bt.Condition` Subclasses

```python
class Branch(bt.Node):
  def __init__(
    self,
    if_condition: bt.Condition | None = None,
    then_child: bt.Node | bt.ActionBase | bt.CodeExecution | None = None,
    else_child: bt.Node | bt.ActionBase | bt.CodeExecution | None = None,
    name: str | None = None,
    *,
    node_id: int | None = None,
  ) -> None: ...


class Blackboard(bt.Condition):
  def __init__(
    self, cel_expression: str | cel.CelExpression | syntax_pb2.Expr
  ) -> None: ...


class SubTreeCondition(bt.Condition):
  def __init__(
    self, tree: bt.BehaviorTree | bt.Node | bt.ActionBase | bt.CodeExecution
  ) -> None: ...


class AllOf(bt.CompoundCondition):  # Logical AND (True if empty)
  def __init__(self, conditions: list[bt.Condition] | None = None) -> None: ...


class AnyOf(bt.CompoundCondition):  # Logical OR (False if empty)
  def __init__(self, conditions: list[bt.Condition] | None = None) -> None: ...


class Not(bt.Condition):  # Logical NOT
  def __init__(self, condition: bt.Condition) -> None: ...


class ExtendedStatusMatch(bt.Condition):
  class MatchStatusCode:
    def __init__(self, component: str, code: int) -> None: ...

  def __init__(
    self,
    blackboard_key: str,
    matcher: bt.ExtendedStatusMatch.MatchStatusCode,
  ) -> None: ...
```

- `bt.Branch` builder methods: `.set_if_condition(cond)`,
  `.set_then_child(child)`, `.set_else_child(child)`. Requires `if_condition`
  and at least one of `then_child` or `else_child`. If the active branch child
  is `None`, `Branch` immediately returns `SUCCEEDED`.

### 4.2 Branching and Condition Code Examples

```python
estimate_task = bt.Task(
  action=solution.skills.ai.intrinsic.estimate_pose_multi_view(
    camera_1=solution.resources.orbbec_camera,
    perception=solution.resources.pose_estimator_service,
    pose_estimator=pose_estimator_proto,
    capture_data=[capture_task.result.capture_data],
  ),
  name="Estimate Workpiece Pose",
)

# 1. If/Else Branch using compound Blackboard CEL conditions:
verify_and_pick = bt.Branch(
  name="Verify Pose Score Before Pick",
  if_condition=bt.AllOf(
    [
      bt.Blackboard(f"size({estimate_task.result.estimates}) > 0"),
      bt.Blackboard(f"{estimate_task.result.estimates[0].score} >= 0.75"),
    ]
  ),
  then_child=pick_sequence,
  else_child=bt.Fail(
    failure_message="Workpiece pose score below 0.75 threshold",
    name="Fail Low Confidence",
  ),
)

# 2. Matching a failure status code with ExtendedStatusMatch:
grasp_attempt = bt.Task(action=move_to_contact_action, name="Touchdown")
grasp_attempt.on_failure.emit_extended_status_to("touchdown_err")

recover_on_timeout = bt.Fallback(
  name="Touchdown With Timeout Recovery",
  tries=[
    bt.Fallback.Try(condition=None, node=grasp_attempt),
    bt.Fallback.Try(
      condition=bt.ExtendedStatusMatch(
        blackboard_key="touchdown_err",
        matcher=bt.ExtendedStatusMatch.MatchStatusCode(
          component="ai.intrinsic.move_to_contact",
          code=10301,
        ),
      ),
      node=retract_and_realign_sequence,
    ),
  ],
)
```

## 5. Loops and Retries

> **Note**: `bt.Repeat` does **not** exist in `intrinsic.solutions.behavior_tree`.
> Always use `bt.Loop` for both fixed-count and conditional loops.

### 5.1 `bt.Loop` and `bt.Retry` Signatures

```python
class Loop(bt.Node):
  def __init__(
    self,
    max_times: int = 0,
    do_child: bt.Node | bt.ActionBase | bt.CodeExecution | None = None,
    while_condition: bt.Condition | None = None,
    name: str | None = None,
    loop_counter_key: str | None = None,
    *,
    # DEPRECATED for-each arguments (emit DeprecationWarning; do not use):
    for_each_value_key: str | None = None,
    for_each_protos: list[Any] | None = None,
    for_each_generator_cel_expression: str | None = None,
    node_id: int | None = None,
  ) -> None: ...


class Retry(bt.Node):
  def __init__(
    self,
    max_tries: int = 0,
    child: bt.Node | bt.ActionBase | bt.CodeExecution | None = None,
    recovery: bt.Node | bt.ActionBase | bt.CodeExecution | None = None,
    name: str | None = None,
    retry_counter_key: str | None = None,
    *,
    node_id: int | None = None,
  ) -> None: ...
```

### 5.2 Semantics and Code Examples

- **`bt.Loop`** (`.set_do_child(child)`, `.set_while_condition(cond)`,
  `.loop_counter`):
  - `max_times == 0` (default): No iteration limit. Loops indefinitely if
    `while_condition` is `None`, or while `while_condition` is `True`.
  - `max_times > 0`: Runs `do_child` at most `max_times` times and returns
    `SUCCEEDED` (or stops earlier with `SUCCEEDED` if `while_condition` becomes
    `False`).
  - If `do_child` fails on any iteration, `bt.Loop` immediately returns
    `FAILED`.
  - `loop.loop_counter.value` (`google.protobuf.Int64Value`, 0-indexed) is valid
    inside `do_child` and `while_condition`.
- **`bt.Retry`** (`.set_child(child)`, `.set_recovery(recovery)`,
  `.retry_counter`):
  - Runs `child`. On failure, if tries remain (`max_tries == 0` is infinite;
    `max_tries > 0` is total attempts), executes `recovery` (if set) before
    retrying `child`. If `recovery` fails, `Retry` fails immediately.
  - When retrying `ai.intrinsic.move_to_contact` (which begins with
    `ActionId.TARE (0)`), always set `recovery` to a linear `move_robot` task
    back to the unloaded standoff pose so the F/T sensor is tared in free space
    before the next attempt, and outer-wrap the `bt.Retry` in a `bt.Fallback`
    with a linear seat task for Gazebo stabilization timeouts (`code=10301`).

```python
# 1. Finite cycle loop (max_times > 0) or infinite loop (max_times == 0):
cycle_loop = bt.Loop(
  name="Machine Tending Loop",
  max_times=num_cycles,  # 0 = infinite loop; N > 0 = exactly N cycles
  do_child=single_cycle_sequence,
)

# 2. Iterating over repeated skill outputs with loop_counter.value:
sp = move_robot.intrinsic_proto.skills
mp = move_robot.intrinsic_proto.motion_planning.v1
iter_loop = bt.Loop(name="Inspect Detections")
iter_loop.set_while_condition(
  bt.Blackboard(
    f"{iter_loop.loop_counter.value} < size({estimate_task.result.estimates})"
  )
)
iter_loop.set_do_child(
  bt.Task(
    action=move_robot(
      arm_part=solution.world.ur_module,
      motion_segments=[
        sp.MotionSegment(
          motion_type=sp.MotionSegment.MotionType.ANY,
          cartesian_pose=mp.PoseEquality(
            moving_frame=solution.world.gripper.tool_frame,
            target_frame=solution.world.root,
            target_frame_offset=estimate_task.result.estimates[
              iter_loop.loop_counter.value
            ].root_t_target,
          ),
        )
      ],
    ),
    name="Move to Detection Index",
  )
)
```

## 6. Blackboard Variables, Skill Outputs, CEL, and `bt.Data`

### 6.1 Skill Outputs (`return_value_key`, `.result`, and `BlackboardValue`)

- Every generated skill call accepts `return_value_key: str | None = None`
  (defaults to `f"{skill_name}_{uuid4_hex}"`). Multiple nodes can pass the same
  `return_value_key` to overwrite a shared blackboard variable.
- `task.result_key -> str` returns the blackboard key name.
- `task.result -> blackboard_value.BlackboardValue` returns a descriptor-backed
  proxy:
  - **Field validation**: `task.result.field` checks the skill's
    `return_value_descriptor` at tree-construction time (`AttributeError` if
    invalid).
  - **List indexing**: `task.result.estimates[0]` or
    `task.result.estimates[loop.loop_counter.value]` validates `LABEL_REPEATED`.
  - **`Duration` / `Timestamp`**: `.seconds` maps to `.getSeconds()` in CEL;
    `.nanos` maps to `.getMilliseconds() * 1000000`.
  - **Post-run inspection**: Call `solution.executive.get_value(task.result)`
    after `solution.executive.run(tree)` to read the protobuf message in Python.

### 6.2 `cel.CelExpression` and `bt.Data`

```python
class CelExpression:
  def __init__(
    self, expression: str | blackboard_value.BlackboardValue
  ) -> None: ...


class Data(bt.Node):
  class OperationType(enum.Enum):
    CREATE_OR_UPDATE = 1
    REMOVE = 2

  def __init__(
    self,
    *,
    blackboard_key: str = "",
    operation: bt.Data.OperationType = OperationType.CREATE_OR_UPDATE,
    proto: message.Message | skill_utils.MessageWrapper | None = None,
    name: str | None = None,
    node_id: int | None = None,
    # DEPRECATED arguments (do not use):
    cel_expression: str | None = None,
    world_query: bt.WorldQuery | None = None,
    protos: list[Any] | None = None,
  ) -> None: ...
```

- **CEL Patterns**:
  - Field presence: `has(grasp_result.target_pose)` (takes `msg.field`, not a
    bare variable name).
  - List length & quantifiers: `size(pose_est.estimates) > 0`,
    `pose_est.estimates.exists(e, e.score > 0.85)`.
  - Ternary & proto literals: `use_fast ? 0.25 : 0.05`,
    `intrinsic_proto.Vector3{x: 0.0, y: 0.0, z: offset_val.value}`.
- **`bt.Data` Deprecations**: `cel_expression`, `world_query`, and `protos` on
  `bt.Data` are **deprecated** (`data_node.result` returns `None` when
  `cel_expression` is used). Only use `bt.Data` with `proto=...` (static proto
  initialization) or `operation=bt.Data.OperationType.REMOVE`:

```python
from google.protobuf import wrappers_pb2

init_flag = bt.Data(
  name="Initialize Retry Flag",
  blackboard_key="needs_recalibration",
  proto=wrappers_pb2.BoolValue(value=False),
)
# Read via init_flag.result.value or CEL "needs_recalibration.value"

clear_flag = bt.Data(
  name="Clear Retry Flag",
  blackboard_key="needs_recalibration",
  operation=bt.Data.OperationType.REMOVE,
)
```

## 7. `bt.PythonScript` (`CodeExecution`)

### 7.1 Signatures and `proto_building` (`pb`) Specs

```python
class PythonScript(bt.CodeExecution):
  def __init__(
    self,
    signature_with_args: pb.SignatureWithArgs | None = None,
    *,
    function_body: str,
    return_value_key: str | None = None,
  ) -> None: ...


def get_function_body_as_str(func: Callable[..., Any]) -> str: ...


@dataclasses.dataclass
class FieldSpec:
  name: str
  number: int
  # "float", "double", "int64", "int32", "bool", "string", "bytes", or _pb2 cls:
  type: pb.TypeSpec
  repeated: bool = False
  optional: bool = False
  unit: str | None = None
  doc: str | None = None
  default_value: Any | None = None
  arg: Any | None = None  # Static value, BlackboardValue, or CelExpression


@dataclasses.dataclass
class MapFieldSpec:
  name: str
  number: int
  key_type: pb.MapKeyTypeSpec  # bool | int | str | "string" | "int32" | "int64"
  value_type: pb.TypeSpec
  doc: str | None = None
  default_value: Mapping[Any, Any] | None = None
  arg: Any | None = None


@dataclasses.dataclass
class DependencySpec:
  type: pb.DependencyTypeSpec  # Proto message/enum class or module


@dataclasses.dataclass
class MessageSpec:
  name: str | None = None
  package: str | None = None
  doc: str | None = None
  fields: list[pb.FieldSpec | pb.MapFieldSpec] = dataclasses.field(
    default_factory=list
  )
  dependencies: list[pb.DependencySpec] = dataclasses.field(
    default_factory=list
  )
```

- **Unwrapping `_MESSAGE_CLASS`**: If passing a dynamic skill `MessageWrapper`
  to `FieldSpec(type=...)`, unwrap it with
  `getattr(wrapper_cls, "_MESSAGE_CLASS", wrapper_cls)` or pass a compiled
  `_pb2` class (e.g. `pose_pb2.Pose`).
- **Offline vs Connected `ProtoBuilder`**: Use
  `solution.proto_builder.create_signature_with_args(...)` when connected, or
  `pb.Signature(parameters=..., return_value=..., well_known_types={}).with_args(...)`
  in offline/hermetic builds without a stub.

```python
from intrinsic.math.proto import pose_pb2

sig = pb.Signature(
  parameters=pb.MessageSpec(
    fields=[
      pb.FieldSpec(name="detected_pose", number=1, type=pose_pb2.Pose),
      pb.FieldSpec(name="z_offset_m", number=2, type=float),
    ],
  ),
  return_value=pb.MessageSpec(
    fields=[pb.FieldSpec(name="grasp_pose", number=1, type=pose_pb2.Pose)],
  ),
  well_known_types={},
)


def compute_grasp_pose(params, context):
  out = node_pb2.ReturnValue()
  out.grasp_pose.CopyFrom(params.detected_pose)
  out.grasp_pose.position.z += params.z_offset_m
  return out


script = bt.PythonScript(
  sig.with_args(
    detected_pose=estimate_task.result.estimates[0].root_t_target,
    z_offset_m=0.08,
  ),
  function_body=bt.get_function_body_as_str(compute_grasp_pose),
  return_value_key="computed_grasp",
)
script_task = bt.Task(action=script, name="Compute Grasp Pose")
```

### 7.2 Runtime Environment, `context.object_world`, and OMTS Helpers

At runtime, `CodeExecutionService` wraps `function_body` inside
`def compute(params: node_pb2.Params | None, context: BasicComputeContext) -> node_pb2.ReturnValue | None:`
with `np` (`numpy`) and `node_pb2` pre-imported, a **65-second hard timeout**,
and requires platform flag `"enable_python_task_node" true`.

| `context.object_world` Method | Signature & Behavior |
| :--- | :--- |
| `<name>` (attribute access) | `context.object_world.<name>` resolves root child objects, global-alias objects, and root frames (`WorldObject`, `KinematicObject`, or `Frame`). |
| `get_object` / `get_kinematic_object` | `get_object(name: str) -> WorldObject`, `get_kinematic_object(name: str) -> KinematicObject` |
| `get_frame` | `get_frame(frame_name: str, object_name: str | None = None) -> Frame` |
| `get_transform` | `get_transform(node_a, node_b) -> data_types.Pose3` (returns `a_t_b`) |
| `update_transform` | `update_transform(node_a, node_b, a_t_b: data_types.Pose3, node_to_update=None) -> None` |
| `create_frame` / `delete_frame` | `create_frame(frame_name: str, parent=None, parent_t_frame: Pose3 = Pose3()) -> Frame`, `delete_frame(frame, *, force: bool = False) -> None` |
| `reparent_object` / `reparent_object_to_final_entity` | `reparent_object(child, new_parent) -> None`, `reparent_object_to_final_entity(child, kinematic_parent) -> None` |
| `update_joint_positions` | `update_joint_positions(kinematic_object: KinematicObject, joint_positions: list[float], joint_names: list[str] | None = None) -> None` |
| `disable_collisions` / `enable_collisions` | `disable_collisions(first_object, second_object) -> None`, `enable_collisions(first_object, second_object) -> None` |

- **OMTS Script Helpers (`src/utils/script_utils.py` & `src/hardware/vision.py`)**:
  - `load_python_script(source, function_name=None, call_args="context, params") -> str`:
    Loads a standalone Python module/function (e.g.,
    `calculate_and_update_dynamic_frames` or `randomize_placement_frame`) and
    appends `<function_name>(context, params)`.
  - `create_dwell_task(dwell_time_sec: float, solution=None, task_name=None) -> bt.Task`:
    Creates a `bt.PythonScript` task running `import time; time.sleep(...)`
    (used instead of `ai.intrinsic.sleep_for`, which is not in `@intrinsic-core`).
  - `OrbbecVision._create_frame_calc_script_task` (`src/hardware/vision.py`):
    Binds the 7 scalar pose fields (`pos_x`, `pos_y`, `pos_z`, `ori_x`, `ori_y`,
    `ori_z`, `ori_w`) from `estimate_action.result.estimates[0].root_t_target`
    via `solution.proto_builder.create_signature_with_args(...)` into
    `load_python_script(calculate_and_update_dynamic_frames)`, updating
    `root/grasp` and `root/pre_grasp` in `context.object_world`.
- **Behavior in `REALITY` vs `PREVIEW` / `DRAFT` (`FAST_PREVIEW`)**:
  `bt.PythonScript` (`execute_code`) executes live on `CodeExecutionService` in
  **all** simulation modes. In `PREVIEW` and `DRAFT`, the Executive clones the
  belief world into a temporary preview world and passes that preview `world_id`
  to `context.object_world`, so frame/transform updates take effect in preview
  without mutating the `REALITY` belief world.

## 8. Parameterized Behavior Trees (pBTs) vs `bt.SubTree`

```python
tree = bt.BehaviorTree(name="Pick Subprocess", root=pick_sequence)

# 1. Set asset metadata first (tree.name must already be non-empty):
tree.set_asset_metadata(
  id="ai.intrinsic.pick_subprocess",
  vendor="ai.intrinsic",
  doc="Parameterized pick subprocess.",
  subprocess=True,
)

# 2A. Define signature via proto_building.Signature:
sig = solution.proto_builder.create_signature(
  parameters=pb.MessageSpec(
    name="PickParams",
    package="my_pkg",
    fields=[
      pb.FieldSpec(name="approach_offset_z", number=1, type=float),
      pb.FieldSpec(name="max_tries", number=2, type=int, default_value=3),
    ],
  ),
  return_value=pb.MessageSpec(
    name="PickResult",
    package="my_pkg",
    fields=[pb.FieldSpec(name="succeeded", number=1, type=bool)],
  ),
)
tree.set_signature(sig)
tree.return_value_expression = sig.create_return_value_expression(
  succeeded=cel.CelExpression("true")
)

# 2B. Or initialize with dynamic/compiled protos:
# param_msg = solution.proto_builder.create_message(
#   "my_pkg", "PickParams", {"approach_offset_z": float, "max_tries": int}
# )
# tree.initialize_pbt_with_protos(
#   parameter_proto=param_msg, return_value_proto=return_msg
# )
```

- **Accessing pBT Parameters**: Use `tree.params.<field_name>` (returns a
  `BlackboardValue` at `"params.<field_name>"`) or CEL `"params.<field_name>"`.

| Property | `bt.SubTree(behavior_tree=...)` | Saved Process / pBT (`subprocess=True`) |
| :--- | :--- | :--- |
| **Invocation** | Inlined directly as a node in the parent tree | Saved via `solution.processes.save(tree)` and called as a skill |
| **Blackboard Scope** | **Shared** (`PROCESS_TREE`) with parent tree | **Isolated** per subprocess call |
| **Parent Blackboard Access** | Can directly read/write any key in parent tree | Cannot access parent blackboard; pass inputs via `params` and outputs via `return_value_expression` |
