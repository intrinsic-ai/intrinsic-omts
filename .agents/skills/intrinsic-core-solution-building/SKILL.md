---
name: intrinsic-core-solution-building
description: >-
  Intrinsic Solution Building Library (SBL) Python SDK guide for connecting to workcells, composing Behavior Trees,
  querying/mutating ObjectWorld frames, binding equipment resources, and orchestrating execution.
  Triggers: connect to solutions, compose behavior trees, mutate world frames, bind equipment resources, execute trees.
  Subsystems: SBL, Executive, ObjectWorld, Resources, BehaviorTree.
  Dependencies: localhost:17080, intrinsic.solutions.
  Anti-keywords: inctl service add, raw C++ authoring, BUILD rules.
---

# Solution building library (`SBL`) `Python` `SDK` guide

## Architecture of `intrinsic.solutions`

The Solution Building Library (`from intrinsic.solutions import deployments, behavior_tree as bt, cel, worlds`) provides the primary Python SDK for inspecting, mutating, and orchestrating deployed Intrinsic solutions over gRPC (`localhost:17080` or remote endpoints):

| Component / attribute | SDK surface and type | Responsibility and contract | Target usage guidance |
| :--- | :--- | :--- | :--- |
| **`deployments.connect(...)`** | `deployments.Solution` | Establishes gRPC channels with unlimited message length (`-1`). | Connect directly via `address="localhost:17080"`; instantiate one `Solution` per thread. |
| **`solution.skills`** | `providers.SkillProvider` | Dynamic namespace providing auto-generated Python classes for installed skills. | Pass `_with_recommended_config=False` in loops to bypass synchronous gRPC config lookups. |
| **`solution.world`** | `worlds.ObjectWorld` | Connected to runtime belief world (`worlds.EditWorldId.BELIEF` / `"world"`). | Pass `start_from_world_state=worlds.EditWorldId.BELIEF` to `executive.run()` to sync edits to simulation. |
| **`solution.resources`** | `providers.ResourceProvider` | Registry of hardware and service handles (`ResourceHandle`). | Inspect handles via `solution.resources.<name>` and iterate using `dir(solution.resources)`. |
| **`solution.executive`** | `execution.Executive` | Loads, executes, and cancels Behavior Trees. | Run trees via `executive.run(tree)` or block on staged operations via `executive.start(blocking=True)`. |

## Progressive disclosure reference hub

Read the domain reference under `references/` matching your task before authoring SBL code:

| Reference guide | Domain and Python SDK surface | When to read it |
| :--- | :--- | :--- |
| [references/behavior-trees-and-cel.md](references/behavior-trees-and-cel.md) | `bt.Sequence`, `bt.Fallback.Try`, `bt.Task`, `BlackboardValue`, `CelExpression`, `bt.ExtendedStatusMatch`, PBTs. | Composing control-flow trees, passing data via CEL expressions, handling errors, or authoring PBTs. |
| [references/world-and-motion.md](references/world-and-motion.md) | `solution.world` (`ObjectWorld`), `Pose3`, frame transforms, `reparent_object`, `register_geometry_v1`, `move_robot`. | Mutating coordinate frames, reparenting objects, syncing belief world to simulation, or moving robots. |
| [references/resources-and-execution.md](references/resources-and-execution.md) | `solution.resources`, `dir()` iteration rule, `_resource` suffix, `solution.grpc_channel`, `executive.run`. | Looking up equipment handles, resolving dependencies, bridging raw gRPC stubs, or handling execution errors. |
| [../intrinsic-core-bazel/SKILL.md](../intrinsic-core-bazel/SKILL.md) | Bzlmod dependencies, rules_python toolchain, and hermetic `py_binary` targets. | Declaring dependencies for SBL scripts, third-party libraries, and Bazel execution. |

## End-to-end connection and execution template

```python
from intrinsic.math.python import data_types
from intrinsic.solutions import behavior_tree as bt, deployments, worlds


def connect_and_run(address: str = "localhost:17080") -> None:
  # 1. Connect directly to active solution over local Envoy ingress
  solution = deployments.connect(address=address)
  solution.update_skills()

  # 2. Mutate belief world using explicit keyword arguments on Pose3
  world = solution.world
  waypoint = data_types.Pose3(
      rotation=data_types.Rotation3.identity(),
      translation=[0.45, 0.0, 0.30],
  )
  world.update_transform(
      node_a=world.root,
      node_b=world.waypoint_frame,
      a_t_b=waypoint,
      node_to_update=world.waypoint_frame,
  )

  # 3. Construct Behavior Tree with explicit bt.Task wrapping in conditional nodes
  move_home = solution.skills.ai.intrinsic.move_robot(
      arm_part=solution.resources.robot,
      target_joint_configuration=world.robot.joint_configurations.home,
  )
  tree = bt.BehaviorTree(
      name="Home workflow",
      root=bt.Sequence(children=[bt.Task(action=move_home, name="Move home")]),
  )

  # 4. Execute tree while syncing simulation ("sim_world") from belief ("world")
  solution.executive.run(
      tree,
      start_from_world_state=worlds.EditWorldId.BELIEF,
  )
```

## Paired safety guardrails

1. **Direct SDK imports and connection parameter exclusivity**: Import SBL libraries directly in Python (`from intrinsic.solutions import deployments, behavior_tree as bt, worlds; from intrinsic.math.python import data_types`); do not run recursive searches (`grep`, `find`) across root `/`. Connect directly via `deployments.connect(address="localhost:17080")` (do not supply `org` when `address` or `grpc_channel` is set; `org` is restricted strictly to cloud discovery).
2. **Resource iteration and membership**: Inspect handles via `solution.resources.<name>` or bracket indexing `solution.resources[name]`, and iterate using `dir(solution.resources)` (do not call `len(solution.resources)`, `for r in solution.resources:`, or `hasattr(...)`, which fail with `TypeError` or `KeyError`).
3. **Explicit `bt.Task` wrapping in conditional branches**: Wrap action calls explicitly in `bt.Task(action=skill)` when supplying child nodes to `bt.Fallback.Try(node=...)` or `bt.Selector.Branch(node=...)` (do not pass raw skill invocations directly to `Fallback.Try` or `Selector.Branch`, which lack implicit task conversion and fail protobuf serialization).
4. **Belief and simulation world synchronization**: Pass `start_from_world_state=worlds.EditWorldId.BELIEF` to `executive.run()` after modifying `solution.world`, or execute `inctl world reset --address=localhost:17080` (do not call `solution.simulator.reset()` directly; use `start_from_world_state` or `inctl world reset` instead).
5. **Execution blocking and return value handling**: Pass the behavior tree to `solution.executive.run(tree)` or call `solution.executive.start(blocking=True)` to block until operation completion, and read timestamps from `executive.last_execution_time_window` (do not rely on `solution.executive.run(None)` to block or return timestamps; calling `run(None)` returns immediately with `None`).

## Diagnostic decision tree for `SBL` workflows

```
[Issue detected during SBL Python workflow]
  │
  ├─► [Symptom: ValueError: Org is not supported when connecting via grpc_channel or address]
  │     └─► Cause: Conflicting connection parameters supplied to deployments.connect().
  │     └─► Action: Omit org when connecting via address="localhost:17080" or grpc_channel.
  │
  ├─► [Symptom: KeyError: 'Resource 0 not registered' or TypeError: object has no len()]
  │     └─► Cause: Direct iteration or len() invoked on solution.resources.
  │     └─► Action: Iterate via [solution.resources[k] for k in dir(solution.resources)].
  │
  ├─► [Symptom: FailedPrecondition: solution does not have an origin nor does it track a branch]
  │     └─► Cause: Attempting solution.behavior_trees[...] dictionary assignment on branchless workcell.
  │     └─► Action: Pass BehaviorTree directly to solution.executive.run(tree) or executive.load(tree).
  │
  ├─► [Symptom: TypeError: expected BehaviorTree.Node got BehaviorCall]
  │     └─► Cause: Raw skill passed to bt.Fallback.Try or bt.Selector.Branch without task wrapping.
  │     └─► Action: Wrap action explicitly via bt.Fallback.Try(node=bt.Task(action=skill)).
  │
  ├─► [Symptom: Gazebo simulation ignores solution.world edits or reports missing obstacles]
  │     └─► Cause: Simulation world ("sim_world") not synchronized from belief world ("world").
  │     └─► Action: Pass start_from_world_state=worlds.EditWorldId.BELIEF to executive.run().
  │
  ├─► [Symptom: CEL evaluation error: params.<field> refers to variables that do not exist]
  │     └─► Cause: Saved BehaviorTree executed without parameter debug payload.
  │     └─► Action: Pass parameters=saved_bt.user_data_protos.get("PARAMETER_DEBUG_VALUES").
  │
  └─► [Symptom: UNIMPLEMENTED / maximum recursion depth of 100 reached on CreateOperation]
        └─► Cause: Resubmitting BehaviorTree containing output-only runtime execution metadata.
        └─► Action: Strip RunMetadata, called_tree_state, and node state fields before resubmitting.
```

## System 2 reflection and circuit breaker checkpoints

### Pre-execution reflection checkpoint
Before executing SBL scripts or mutating workcell state:
1. Verify connection parameters (`address="localhost:17080"`, no `org`) and thread isolation (one `Solution` per thread).
2. Confirm target resource handles exist via `"handle_name" in dir(solution.resources)`.
3. Verify explicit keyword arguments on `data_types.Pose3(rotation=..., translation=...)`.
4. Ensure `start_from_world_state=worlds.EditWorldId.BELIEF` is set whenever `solution.world` edits must be reflected in simulation.

### Anti-thrashing circuit breaker
- **Execution polling budget**: Cap operation completion checks at 3 queries with a 5-second backoff.
- **Trip condition**: If `executive.run()` raises `ExecutionFailedError` or child nodes fail with composite error codes (`31100`, `31800`), halt repeated executions. Inspect `solution.executive.operation.metadata.diagnostics` or strip output-only execution state rather than churning tree construction.

## Verification criteria

- [ ] **Thread-safe connection**: Created isolated `Solution` instance via `deployments.connect(address="localhost:17080")`.
- [ ] **Safe resource inspection**: Queried and iterated resources using `dir(solution.resources)`.
- [ ] **Direct tree execution**: Executed trees via `executive.run(tree)` without branch dictionary assignment.
- [ ] **Explicit task wrapping**: Wrapped all actions inside `bt.Fallback.Try(node=bt.Task(...))` and `bt.Selector.Branch(node=bt.Task(...))`.
- [ ] **World synchronization**: Supplied `start_from_world_state=worlds.EditWorldId.BELIEF` to `executive.run()`.
- [ ] **Execution verification**: Checked operation status via `executive.operation.proto.state` and handled `ExecutionFailedError`.
