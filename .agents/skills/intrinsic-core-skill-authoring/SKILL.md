---
name: intrinsic-core-skill-authoring
description: >-
  Authoring Intrinsic Core robot skills and stateless behavior tree leaf action nodes.
  Triggers: custom skill authoring, implementing skill lifecycle (execute, preview, get_footprint), equipment leasing, cooperative cancellation, scene graph manipulation.
  Subsystems: EXECUTIVE, SKILLS, WORLD, ICON.
  Dependencies: skill_interface.Skill, ExecuteContext, PreviewContext, ObjectWorldClient.
  Anti-keywords: custom services, ServiceManifest, executive engine internals.
---

# Authoring Intrinsic Core skills

Prerequisite: read the [intrinsic-core-bazel skill](../intrinsic-core-bazel/SKILL.md).

## Behavior tree leaf node architecture and lifecycle

In the Intrinsic platform, a skill operates as a modular robot action leaf node within an executive behavior tree (BT). Action leaf nodes are ticked by the executive and must remain purely stateless across ticks.

### Core lifecycle contracts

Skills inherit from `skill_interface.Skill` (`from intrinsic.skills.python import skill_interface`) and implement four lifecycle methods:
- **`execute(self, request, context)`**: Runtime entrypoint executed on node tick. Read parameters from `request.params`, access leased equipment handles via `context.resource_handles["<slot>"]`, and interact with `context.object_world` or `context.motion_planner`.
- **`preview(self, request, context)`**: Speculative dry-run entrypoint. `PreviewContext` lacks physical hardware handles (`resource_handles`). Use `context.get_object_for_equipment("<slot>")` and record speculative scene updates via `context.record_world_update(update, elapsed, duration)`.
- **`get_footprint(self, request, context)`**: Resource lock declaration. Return `footprint_pb2.Footprint` (`from intrinsic.skills.proto import footprint_pb2`). Evaluate whether `lock_the_universe=True` is needed for motion safety and collision-free guarantees, or `lock_the_universe=False` with explicit equipment locks for parallel branch execution.
- **`@classmethod def required_equipment(cls) -> dict[str, str]`**: Declares the mapping of logical equipment slot names to selector requirements needed by the skill.

### Stateless leaf node design and cooperative cancellation

Action nodes are pure functional transformations of tick inputs (`request.params`) and blackboard variables:
- Delegate persistent world state, multi-cycle tracking buffers, or hardware connections to external services or blackboard keys rather than storing state across ticks in `self`.
- Support cooperative cancellation (halting in BT semantics): declare `supports_cancellation: true` in the skill manifest, invoke `context.canceller.ready()`, monitor `context.canceller.cancelled`, and terminate halted executions by raising `skill_interface.SkillCancelledError()`.
- Compose multi-skill workflows as behavior trees (Process Assets) rather than invoking skills directly from inside another skill ("subskills").

### Scaffolding skills via the `intrinsic` `CLI`

The blessed way to create a skill is through the CLI scaffolding tool:
```bash
inctl skill create com.my_org.sample_operation --proto_package=com.my_org
```
This generates the authentic package structure (`BUILD`, `.manifest.textproto`, `.proto`, and skill implementation) and aligns dependencies. Author hermetic unit tests (`py_test`) for skill logic.

> [!IMPORTANT]
> **Hermetic Bazel dependencies and testing**: All skill dependencies, targets, and unit tests must be managed hermetically under Bazel using `@ai_intrinsic_sdks` and `@rules_python`. See [intrinsic-core-bazel](../intrinsic-core-bazel/SKILL.md) for canonical dependency declarations, `MODULE.bazel` configuration, and `BUILD` target mappings. Run tests with standard `bazel test //...`.

### Instantiating and verifying skills in the workcell cluster

Deploy custom skills into the active solution on the workcell cluster via a single-step installation:
```bash
# Instantiate skill bundle into the running solution on the local cell
inctl skill install <path/to/skill_bundle.tar> --address=localhost:17080
```
Verify that the skill is loaded and active in the cluster:
```bash
# List all skills loaded into the current solution
inctl skill list --address=localhost:17080
```
On zero-cloud workcells, never pass `--org`, `--project`, or `--cluster` flags; always target the local cell endpoint with `--address=localhost:17080`.

> [!IMPORTANT]
> **Backend containerd socket circuit breaker**: If `inctl skill install` fails with containerd socket connection refused (`/run/containerd/containerd.sock` or `localhost:17127`), stop after <= 2 attempts. The socket is inside backend `k3s`, not in the local client sandbox. Verify the skill hermetically via Bazel (`bazel test //...`, `bazel build //...`). See circuit breaker below.

### Anti-patterns and strict prohibitions

- **No external network fetching**: Live network fetching during evaluation or skill execution is strictly prohibited. All dependencies must be resolved offline and hermetically via Bazel (see [intrinsic-core-bazel](../intrinsic-core-bazel/SKILL.md)).
- **Code authoring and local test primacy**: Implement the Python skill class (`skill_interface.Skill`), declare `@classmethod def required_equipment(cls)`, implement required operations, and verify locally with standard `bazel test //...` before cluster packaging.
- **Anti-disassembly circuit breaker**: Never run `objdump`, `strings`, or `nm` on `/usr/local/bin/inctl` to forge binary `.tar` bundles or manifests. On local workcells without `inbuild`, authoring the Python implementation and verifying `inctl asset list --address=localhost:17080` satisfies cluster checks.
- **Never scan the root filesystem (`/`)**: Import SDK libraries directly; do not run recursive searches (`find /`, `grep -r ... /`). Scanning root traverses virtual filesystems (`/proc`, `/sys`) and hangs execution.
- **Never store mutable state across ticks**: Do not assign persistent state to `self.*` attributes in `execute()`. Leaf nodes must remain strictly stateless.
- **Behavioral evaluation contracts**: Evaluators verify skills behaviorally via hermetic Bazel compilation, local unit tests (`py_test`), cluster bundle installation (`inctl skill install`), and runtime behavior tree execution. Strongly-typed class signatures like `class MySkill(skill_interface.Skill[Params, Result]):` are fully supported by behavioral evaluators. Verify your skills by executing local tests and cluster runners rather than stopping at static syntax.
- **Never debug or proxy containerd sockets**: If installation fails with containerd socket connection refused, halt after <= 2 attempts. The socket is in backend `k3s`; verify hermetically via Bazel rather than debugging local sockets.

## Standard skill template

Directly inherit from `skill_interface.Skill` without import fallbacks:

```python
"""Stateless Intrinsic skill template implementing skill_interface.Skill."""

from typing import Any
from intrinsic.skills.proto import footprint_pb2
from intrinsic.skills.python import skill_interface


class SampleOperationSkill(skill_interface.Skill):
  """Stateless robot action leaf node."""

  @classmethod
  def required_equipment(cls) -> dict[str, str]:
    return {"robot": "robot_arm"}

  def get_footprint(self, request: Any, context: Any) -> Any:
    return footprint_pb2.Footprint(lock_the_universe=True)

  def preview(self, request: Any, context: Any) -> Any:
    return None

  def execute(self, request: Any, context: Any) -> Any:
    if hasattr(context, "canceller"):
      context.canceller.ready()
    robot = context.resource_handles.get("robot")
    if robot is None:
      raise skill_interface.SkillError(13, "Required leased equipment 'robot' not found.")
    if hasattr(context, "canceller") and context.canceller.cancelled:
      raise skill_interface.SkillCancelledError("Skill execution cancelled by executive.")
    initial_pose = getattr(robot, "get_pose", lambda: None)()
    try:
      return self._perform_operation(robot, getattr(request, "params", request), context)
    except Exception as e:
      if initial_pose is not None and hasattr(robot, "move_to_pose"):
        robot.move_to_pose(initial_pose)
      raise e

  def _perform_operation(self, robot: Any, params: Any, context: Any) -> Any:
    raise NotImplementedError()
```

## Scene graph manipulation with `ObjectWorldClient`

Skills inspect and mutate the belief state scene graph via `ObjectWorldClient` (`context.object_world`):
- **Resolve objects**: `node = client.get_object("node_name")` resolves handles in the world tree.
- **Create coordinate frames**: `client.create_frame(frame_name="tip", parent=parent_node, parent_t_frame=pose)`.
- **Construct `Pose3` transforms**: Always pass keyword arguments `Pose3(rotation=Rotation3.identity(), translation=[x, y, z])` (`from intrinsic.math.python.pose3 import Pose3`, `from intrinsic.math.python.rotation3 import Rotation3`). Native Python float sequences for `translation` (`list[float]` or `tuple[float, float, float]`) rather than raw NumPy arrays.

## How to communicate with an Intrinsic asset vs. an Intrinsic Core platform service

Envoy routes calls based on target entity (see [intrinsic-core-api-overview](../intrinsic-core-api-overview/SKILL.md)):
- **Leased asset instances**: Attach `x-resource-instance-name: <instance_name>` from `context.resource_handles["<slot>"].connection_info.grpc`.
- **Intrinsic Core platform services**: Access singleton services (`ObjectWorldService` via `context.object_world`) directly without instance headers.

## Input-aware decision tree: lifecycle and diagnostic dispatch

- **Concurrency & universe locking**: Retain `lock_the_universe=True` for motion planning and collision guarantees. Set `lock_the_universe=False` only when actions are purely independent with explicit equipment locks.
- **Executive reports `Skill has not implemented Execute`**: Inspect container stderr logs preceding the error for custom log lines to locate failing downstream gRPC status codes.
- **Scalar parameters (`false`, `0`) overwritten by defaults**: Mark scalar fields as `optional` in `.proto` schema to ensure explicit field presence.
- **Speculative dry run during preview**: Use `context.record_world_update(update, elapsed, duration)` instead of mutating `context.object_world` or querying physical handles.

## Verification hooks, reflection checkpoints, and circuit breakers

- **Pre-execution system 2 reflection**: Verify hardware dependencies match manifest, `get_footprint()` evaluates locking, cancellation checks `context.canceller`, and scalar parameter fields use `optional`.
- **Anti-thrashing circuit breaker**: If container fails with `Skill has not implemented Execute`, inspect stderr for downstream gRPC errors; halt after 2 attempts. If containerd socket connection refused occurs (`localhost:17127`), halt after <= 2 attempts and verify hermetically via Bazel.

## Completion criteria

- [ ] **Stateless leaf interface**: Inherits directly from `skill_interface.Skill` without fallbacks; delegates state across ticks to blackboard.
- [ ] **Cooperative cancellation**: Declares `supports_cancellation: true` in manifest, calls `context.canceller.ready()`, and raises `SkillCancelledError`.
- [ ] **Explicit footprint**: Evaluates locking in `get_footprint()` (`lock_the_universe=True` for motion safety, or explicit leases for concurrent branches).
- [ ] **Equipment leasing**: Declares required slots via `required_equipment(cls)` and resolves from `context.resource_handles`.
- [ ] **Scene graph transforms**: Constructs `Pose3` with keyword arguments and native Python float sequences.
- [ ] **Cluster instantiation**: Installed via `inctl skill install` and verified via `inctl skill list`, or verified hermetically via Bazel on backend socket failures.
