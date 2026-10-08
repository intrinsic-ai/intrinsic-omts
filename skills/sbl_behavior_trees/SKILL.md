---
name: sbl-behavior-trees
description: >-
  Builds, extends, and debugs Solution Building Library (SBL) Behavior Trees in
  Python using Intrinsic Core and the Open Machine Tending Solution (OMTS). Use
  when writing or modifying SBL Behavior Trees, phase subtrees, branching
  (bt.Branch, bt.Selector, bt.Fallback), loops and retries (bt.Loop, bt.Retry),
  blackboard variables and CEL expressions (bt.Data, bt.Blackboard,
  cel.CelExpression), skill input/output chaining (.result, return_value_key),
  in-tree Python script nodes (bt.PythonScript, pb.Signature), or discovering
  skill parameters and nested protobuf messages (such as move_robot,
  MotionSegment, CollisionRule, or PoseEquality). Don't use for converting
  legacy textproto behavior trees to Python (use convert-textproto-to-sbl) or
  for Kubernetes cluster administration.
---

# SBL Behavior Trees in OMTS (`intrinsic-omts`)

Guides the authoring, extension, and verification of Python Behavior Trees built
with the Intrinsic Solution Building Library (`intrinsic.solutions.behavior_tree`
as `bt`) in `intrinsic-omts`.

---

## 1. Core SBL & OMTS Architecture

### Single-Tree Executive Model
OMTS constructs the entire multi-phase machine tending pipeline as a **single**
`bt.BehaviorTree` (`build_machine_tending_behavior_tree` in
`src/behaviors/machine_tending_bt.py`) and submits it once to the Flowstate
Executive in `src/main.py`:

```python
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import execution

tree = bt.BehaviorTree(name="Machine Tending", root=root_sequence)
sim_mode = (
  execution.SimulationMode.PREVIEW
  if is_preview
  else execution.SimulationMode.REALITY
)
solution.executive.run(tree, simulation_mode=sim_mode)
```

Submitting a single tree allows `SimulationMode.PREVIEW` to simulate the full
multi-cycle trajectory on a cloned preview world without mutating the `REALITY`
belief world between phases.

### 3-Tier OMTS Code Structure
Separate hardware skill construction, reusable motion/script factories, and
high-level tree orchestration into three layers:

1. **Tier 1 — Hardware & Service Adapters (`src/hardware/*.py`)**:
   - `Robot` (`src/hardware/robot.py`), `RobotiqGripper` / `DioGripper`
     (`src/hardware/gripper.py`), `DioCncMachine` (`src/hardware/machine.py`),
     and `OrbbecVision` (`src/hardware/vision.py`).
   - Every adapter method returns a `bt.Node` (`bt.Task` or `bt.Sequence`) and
     never executes imperatives directly against the cluster during tree build.
2. **Tier 2 — Reusable Motion & Script Factories (`src/behaviors/motions.py`, `src/utils/script_utils.py`)**:
   - `create_move_to_frame_task`, `create_move_through_frames_task`,
     `create_compliant_touchdown_task`, `create_relative_retract_task`,
     `create_seated_approach_tasks`, `load_python_script`, and
     `create_dwell_task`.
   - **Clean `approach_frames` & `approach_motion_types` Usage**:
     `create_seated_approach_tasks` accepts `approach_frames: Sequence[str] = ()`
     and `approach_motion_types: str | Sequence[str] = "ANY"`. A single motion
     type string (such as `"ANY"` or `"LINEAR"`) is automatically expanded across
     all `approach_frames`, and `"LINEAR"` is always appended for the final
     segment into the `frame_name` standoff pose. Build `entry_frames`
     explicitly and prepend optional transit frames with `if transit_frame_name:
     entry_frames.insert(0, transit_frame_name)` rather than using list
     comprehensions or manual `["ANY"] * ...` list arithmetic:

```python
entry_frames = [
  machine_approach_frame_name,
  preplace_vise_frame_name,
]
if transit_frame_name:
  entry_frames.insert(0, transit_frame_name)

tasks.extend(
  create_seated_approach_tasks(
    robot=robot,
    frame_name=place_vise_frame_name,
    parent_object=parent_object,
    touchdown=config.load_touchdown,
    config=config,
    label="Load Vise",
    approach_frames=entry_frames,
    approach_motion_types="ANY",
    excluded_collision_pairs=vise_collision_pairs,
  )
)
```

3. **Tier 3 — Phase Subtree Builders (`src/behaviors/*.py`)**:
   - `build_pick_from_infeed_subtree` (`pick.py`), `build_load_machine_subtree`
     (`load_machine.py`), `build_machining_handshake_subtree` (`machining.py`),
     `build_unload_machine_subtree` (`unload_machine.py`),
     `build_return_to_infeed_subtree` (`return_infeed.py`), and
     `build_machine_tending_behavior_tree` (`machine_tending_bt.py`, which
     assembles the five phase `bt.Sequence` subtrees into a cycle sequence and
     repeats cycles with `bt.Loop`).

### Strict Configuration Rule (No Hidden Default Fallbacks)
All motion speeds, offsets, force thresholds, timeouts, and frame names come
from `configs/<cell>/app_config.yaml` via `AppConfig` (`src/core/config.py`).
Never use `.get("key", default)` or `getattr(cfg, "key", default)` in behavior
or hardware code; let missing keys raise `KeyError` or `AttributeError`
immediately.

---

## 2. How to Discover Skill Parameters & Protobuf Messages (Open-Source Workflow)

In `intrinsic-omts`, Intrinsic Core dependencies are fetched via Bazel Bzlmod
into `$(bazel info output_base)/external`:
- `@intrinsic-core` (`$EXT/intrinsic-core+`): SBL SDK, skill implementations,
  and `*manifest.textproto` files.
- `@intrinsic_apis` (`$EXT/intrinsic_apis+`): Public `.proto` definitions and
  `py_proto_library` (`_py_pb2`) targets.
- `@ai_intrinsic_sdks` (`$EXT/ai_intrinsic_sdks+`): Core SBL Python modules
  (`behavior_tree.py`, `proto_building.py`, `cel.py`, `blackboard_value.py`).

### Method 1: Search `.proto` and `*manifest.textproto` in Bazel's `output_base`

If external repositories have not been fetched yet, fetch the lightweight
manifest target first (avoids downloading multi-GB container tarballs):

```bash
bazel fetch //:omts_solution_manifest
EXT="$(bazel info output_base)/external"

# 1. Find a skill's manifest and parameter proto file:
find "$EXT/intrinsic-core+" "$EXT/intrinsic_apis+" \
  -name "*move_robot*manifest.textproto" -o -name "move_robot.proto"

# 2. Search for a nested proto message (e.g. CollisionRule or CollisionAction):
grep -rn "message CollisionRule\|message CollisionAction" \
  "$EXT/intrinsic_apis+" "$EXT/intrinsic-core+"

# 3. Find the Bazel py_proto_library target for a .proto file:
bazel query "kind(py_proto_library, @intrinsic_apis//intrinsic/world/proto:*)"
# -> @intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2
```

For prebuilt `.bundle.tar` skills (`imported_asset_bundle` in `BUILD`, such as
`//:hand_e_gripper_cmd_skill`), inspect `descriptors-transitive-descriptor-set.proto.bin`
using `google.protobuf.descriptor_pb2.FileDescriptorSet` (see
[references/skills_and_protos_reference.md](references/skills_and_protos_reference.md)).

### Method 2: Programmatic Python Introspection

Given a skill class `skill_cls = solution.skills.ai.intrinsic.move_robot`
(against a live cluster or `FakeSolution` in `tests/unit/test_behaviors.py`):

```python
import inspect

skill_cls = solution.skills.ai.intrinsic.move_robot

# 1. Inspect keyword arguments, types, and defaulted equipment slots:
print(inspect.signature(skill_cls))

# 2. Print auto-generated docstring (lists all nested wrapper paths & enums):
print(skill_cls.__init__.__doc__)

# 3. Inspect protobuf descriptors and required equipment capabilities:
param_desc = skill_cls.info.parameter_descriptor()
return_desc = skill_cls.info.return_value_descriptor()
equipment_slots = skill_cls.info.resource_selectors
```

### Method 3: The 4-Way Protobuf Translation Pattern

Every nested protobuf message reachable by a skill can be constructed in two
ways that SBL automatically bridges across descriptor pools:

1. **Dynamic SBL Wrapper on `<skill_cls>` (follows the `.proto` `package` declaration)**:
   - `intrinsic/world/proto/collision_settings.proto` declares
     `package intrinsic_proto.world;`.
   - Access via `move_robot.intrinsic_proto.world.CollisionSettings.CollisionRule(...)`.
   - Zero Python proto imports or extra Bazel `deps` required.
2. **Static Python `_pb2` Import (follows the `.proto` file path)**:
   - File `intrinsic/world/proto/collision_settings.proto` maps to:
     `from intrinsic.world.proto import collision_settings_pb2`
     -> `collision_settings_pb2.CollisionSettings.CollisionRule(...)`.
   - Add Bazel dep `@intrinsic_apis//intrinsic/world/proto:collision_settings_py_pb2`
     (or `@intrinsic-core//...:<stem>_py_pb2`).
3. **Automatic Pythonic Conversions**:
   - SBL skill and wrapper kwargs automatically convert `data_types.Pose3` ->
     `intrinsic_proto.Pose`, `float` / `int` (seconds) or `timedelta` ->
     `google.protobuf.Duration`, `WorldObject` -> `ObjectReference`, `Frame` ->
     `FrameReference` / `TransformNodeReference`, and `JointConfiguration` ->
     `JointVec`.

### Worked Example: Discovering & Defining `move_robot` and `CollisionRule`

1. **Inspect the manifest (`move_robot_manifest.textproto`)**:
   - `parameter.message_full_name: "intrinsic_proto.skills.MoveRobotParams"`
   - `dependencies.required_equipment` keys `"motion_planner_service"` and
     `"world_service"` (auto-defaulted when the cell has a single instance).
2. **Inspect `move_robot.proto` and `collision_settings.proto`**:
   - Every top-level field of `MoveRobotParams` (`motion_segments`, `arm_part`,
     `curve_parameters`, etc.) becomes a keyword argument of `move_robot(...)`.
   - `motion_segments` is `repeated MotionSegment`
     (`package intrinsic_proto.skills;`). Inside `MotionSegment`, choose a
     `waypoint` oneof (`joint_position`, `cartesian_pose`,
     `constraint_intersection`, or `relative_cartesian_pose`), `motion_type`
     (`ANY`, `JOINT`, `LINEAR`), and optional `collision_settings`
     (`intrinsic_proto.world.CollisionSettings`).
   - Inside `collision_settings.proto` (`package intrinsic_proto.world;`),
     `CollisionSettings` contains `bool disable_collision_checking` and
     `repeated CollisionRule collision_rules`, where each `CollisionRule` takes
     `left` (`repeated ObjectOrEntityReference`), `right`
     (`repeated ObjectOrEntityReference`), and `collision_action`
     (`CollisionAction(is_excluded=True)` or `CollisionAction(margin=...)`).
3. **Define the `move_robot` node in Python** (using static `_pb2` imports or
   dynamic wrappers on `move_robot.intrinsic_proto`):

```python
from intrinsic.world.proto import (
  collision_action_pb2,
  collision_settings_pb2,
)

move_robot = solution.skills.ai.intrinsic.move_robot
seg_cls = move_robot.intrinsic_proto.skills.MotionSegment

allow_part_contact = collision_settings_pb2.CollisionSettings(
  disable_collision_checking=False,
  collision_rules=[
    collision_settings_pb2.CollisionSettings.CollisionRule(
      left=[gripper_ref],
      right=[part_ref],
      collision_action=collision_action_pb2.CollisionAction(is_excluded=True),
    )
  ],
)

step = move_robot(
  arm_part=solution.world.ur_module,
  motion_segments=[
    seg_cls(
      cartesian_pose=(
        move_robot.intrinsic_proto.motion_planning.v1.PoseEquality(
          moving_frame=tool_frame_ref,
          target_frame=target_frame_ref,
          target_frame_offset=standoff_pose_proto,
        )
      ),
      motion_type=seg_cls.MotionType.LINEAR,
      collision_settings=allow_part_contact,
    )
  ],
)
```

For the complete 29-row protobuf translation table and full parameter schemas
for `move_robot`, `move_to_contact`, `update_world`, `attach_object_to_robot`,
`capture_images`, `estimate_pose_multi_view`, and `gripper_cmd_skill`, see
[references/skills_and_protos_reference.md](references/skills_and_protos_reference.md).

---

## 3. Behavior Tree Nodes: Composition, Branching, Loops & Retries

### Composition & Node Naming Rules
- **`bt.Sequence(children=[...], name="...")`**: Executes children in order;
  fails immediately if any child fails.
- **`bt.Parallel(children=[...], name="...")`**: Executes children concurrently;
  succeeds when all succeed, or cancels running children if any child fails.
  - **Universe-Lock Rule (`StatusCode: 18201`)**: Never place a skill with
    `lock_the_universe: true` (such as `ai.intrinsic.update_world` or
    `DioCncMachine` door/vise tasks that call `update_world`) inside a
    `bt.Parallel` alongside robot motion (`move_robot`, `move_to_contact`).
    Run `update_world` sequentially before or after motion nodes.
- **`bt.SubTree(behavior_tree=node_or_tree, name="...")`**: Embeds a phase tree
  inline (`PROCESS_TREE` scope) and **shares the parent blackboard**. When
  `behavior_tree` is a `bt.Node` (such as `bt.Sequence`), `name` is mandatory.
- **`bt.Task(action=..., name="...")`**: Wraps a skill call or `bt.PythonScript`
  with an explicit display `.name`, `.decorators`, or `.on_failure` handler.
- **OMTS Node Naming Invariant**: Direct children of every phase `bt.Sequence`
  must have **unique, descriptive `.name` strings** without legacy `"Step XX:"`
  or `"Prep:"` prefixes (enforced by `tests/unit/test_behaviors.py`).

### Branching & Conditions (`bt.Branch`, `bt.Selector`, `bt.Fallback`)
- **`bt.Branch(if_condition=..., then_child=..., else_child=..., name="...")`**:
  Evaluates a `bt.Condition` (`bt.Blackboard`, `bt.SubTreeCondition`,
  `bt.AllOf`, `bt.AnyOf`, `bt.Not`, or `bt.ExtendedStatusMatch`). Missing
  `then_child` or `else_child` defaults to returning `SUCCEEDED`.
- **`bt.Selector(branches=[bt.Selector.Branch(condition=..., node=...)])`
  (Switch-Case)**: Selects the **first** branch whose `condition` evaluates to
  `True` (or `None`), executes `branch.node`, and returns its result directly
  **without** falling back to subsequent branches if `branch.node` fails.
- **`bt.Fallback(tries=[bt.Fallback.Try(condition=..., node=...)])`
  (Try-Until-Success)**: Tries children in order; if a child's condition is
  `False` or its execution returns `FAILED`, advances to the next `Try` until
  one returns `SUCCEEDED`.
- **`bt.Fail(failure_message="...", name="...")`**: Immediately fails the node
  with an `ExtendedStatus` title.

```python
pose_est = solution.skills.ai.intrinsic.estimate_pose_multi_view(
  capture_data=[capture_task.result.capture_data],
  pose_estimator=pose_estimator_proto,
)
check_detection = bt.Branch(
  name="Verify Workpiece Detection Confidence",
  if_condition=bt.AllOf(
    [
      bt.Blackboard(f"size({pose_est.result.estimates}) > 0"),
      bt.Blackboard(f"{pose_est.result.estimates[0].score} >= 0.8"),
    ]
  ),
  then_child=bt.Sequence(children=[spawn_task, pick_task], name="Pick Part"),
  else_child=bt.Fail(
    name="Abort On Low Confidence",
    failure_message="Pose estimate confidence below threshold 0.8",
  ),
)
```

### Loops & Retries (`bt.Loop` and `bt.Retry`)
> [!IMPORTANT]
> `bt.Repeat` does **not** exist in `intrinsic.solutions.behavior_tree`. Always
> use `bt.Loop` for finite, conditional, and infinite loops.

- **`bt.Loop(max_times=0, do_child=..., while_condition=None, name="...")`**:
  - `max_times=0` (default): No iteration cap. Repeats indefinitely if
    `while_condition=None`, or while `while_condition` evaluates to `True`.
  - `max_times > 0`: Executes `do_child` at most `max_times` times (or stops
    early with `SUCCEEDED` if `while_condition` evaluates to `False`).
  - If `do_child` returns `FAILED`, `bt.Loop` immediately terminates with
    `FAILED`.
  - Access the `0`-indexed `Int64Value` iteration counter inside the loop via
    `loop_node.loop_counter.value`.
  - Do not use `for_each_*` arguments on `bt.Loop`; they are deprecated.
- **`bt.Retry(max_tries=..., child=..., recovery=None, name="...")`**:
  - Retries `child` up to `max_tries` total attempts (`0` = infinite). If
    `recovery` is provided, runs `recovery` after each failed `child` attempt
    before retrying (fails immediately if `recovery` fails).
  - **`bt.Fallback` + `bt.Retry` Pattern for `move_to_contact`**: In
    `create_compliant_touchdown_task` / `create_seated_approach_tasks`
    (`src/behaviors/motions.py`), `move_to_contact` is wrapped in a `bt.Retry`
    (`max_tries=2`) whose `recovery` task moves linearly back to the unloaded
    standoff pose (`-touchdown.standoff_m`) so `ActionId.TARE (0)` zeroes the
    F/T sensor in free space before retrying, and outer-wrapped in a
    `bt.Fallback` whose second `Try` executes a linear Cartesian move to the
    seated contact pose (`Fallback Linear Seat`) if Gazebo simulation still
    times out on contact force stabilization (`move_to_contact:10301`).

```python
# 1. Finite cycle loop or infinite loop (max_times=0) with while_condition:
cycle_loop = bt.Loop(
  name="Machine Tending Cycles",
  max_times=num_cycles,  # >0 for N cycles, 0 for infinite
  while_condition=bt.Blackboard("cell_ready"),
  do_child=single_cycle_sequence,
)

# 2. Compliant touchdown with bt.Retry (re-approaching standoff) + bt.Fallback:
retry_touchdown = bt.Retry(
  name="Load Vise: Compliant Touchdown (+Z Tool) (Retry)",
  max_tries=2,
  child=touchdown_task,
  recovery=reapproach_standoff_task,
)
resilient_touchdown = bt.Fallback(
  name="Load Vise: Compliant Touchdown (+Z Tool)",
  tries=[
    bt.Fallback.Try(condition=None, node=retry_touchdown),
    bt.Fallback.Try(condition=None, node=fallback_seat_task),
  ],
)
```

---

## 4. Blackboard Variables, Inputs/Outputs, CEL & `bt.PythonScript`

### Skill Output Chaining (`task.result` and `return_value_key`)
- Every skill or `bt.PythonScript` with a return proto exposes `.result` (a
  typed `BlackboardValue` proxy that validates field names against the protobuf
  `Descriptor` at tree-construction time) and accepts `return_value_key="..."`.
- Pass `task.result.<field>` or `task.result.<repeated_field>[<idx>]` directly
  to downstream skill kwargs or `pb.Signature.with_args(...)`:

```python
spawn_task = bt.Task(
  action=solution.skills.ai.intrinsic.create_object(
    object_to_create=workpiece_asset_id,
    object_names=["workpiece"],
    poses=[pose_est.result.estimates[0].root_t_target],
  ),
  name="Spawn 'workpiece' at Detected Pose",
)
# After solution.executive.run(tree), read values back on the host via:
# val = solution.executive.get_value(pose_est.result)
```

### `bt.Data` and `cel.CelExpression`
- **`bt.Data`**: Use `bt.Data(name="...", blackboard_key="...", proto=...)` with
  `google.protobuf.wrappers_pb2` (`Int64Value`, `BoolValue`, `DoubleValue`,
  `StringValue`) or compiled protos to initialize/update a blackboard key, or
  `operation=bt.Data.OperationType.REMOVE` to delete a key.
  - **Never use `cel_expression`, `world_query`, or `protos` on `bt.Data`** —
    all three are deprecated in SBL. Inline `cel.CelExpression("...")` directly
    at the consumer parameter or `bt.Blackboard("...")` condition instead.
- **CEL Expressions (`from intrinsic.solutions import cel`)**:
  - Check optional/message field presence with `has(msg.field)` (requires a
    field selection `e.f`, not a bare variable name).
  - Check repeated field length and predicates with `size(est.estimates) > 0`
    and `est.estimates.exists(e, e.score > 0.9)`.

### In-Tree `bt.PythonScript` Nodes
- Construct typed inputs/outputs with `intrinsic.solutions.proto_building as pb`
  (`pb.Signature`, `pb.MessageSpec`, `pb.FieldSpec`). Inside the script body,
  `params` (`node_pb2.Params`), `context` (`BasicComputeContext` with
  `context.object_world`), `node_pb2` (`node_pb2.ReturnValue`), and `np` are
  pre-imported (65-second execution timeout).
- **OMTS Script Utilities (`src/utils/script_utils.py`, `src/hardware/vision.py`)**:
  - Use `load_python_script("src/utils/<name>.py", "fn_name", ...)` for
    in-tree world-mutation scripts (such as `randomize_placement_frame` in
    `src/utils/random_placement.py`) and `create_dwell_task(duration_s, name)`
    for deterministic pauses.
  - Use `OrbbecVision._create_frame_calc_script_task` (`src/hardware/vision.py`),
    which embeds `calculate_and_update_dynamic_frames`
    (`src/utils/dynamic_frame_calculator.py`), when wiring
    `estimates[0].root_t_target` into a `bt.PythonScript` calculator.
- **`REALITY` vs `PREVIEW` / `FAST_PREVIEW`**: The Executive always executes
  `bt.PythonScript` in `PREVIEW` and `FAST_PREVIEW` against a temporary cloned
  preview `_WORLD_ID`. Calls to `context.object_world.create_frame(...)` or
  `update_transform(...)` immediately update the preview world for downstream
  motion planners without mutating the `REALITY` belief world.

### ObjectWorld Frame Hand-Off vs Blackboard Scope
- **Dynamic Frame Hand-Off**: In OMTS, `calculate_and_update_dynamic_frames`
  (`src/utils/dynamic_frame_calculator.py`) computes grasp poses from
  `root_t_target` and creates/updates frames `root/grasp` and `root/pre_grasp`
  via `context.object_world`. Downstream `Pick` and `Load` subtrees plan
  directly to `TransformNodeReference` `root/grasp` and `root/pre_grasp`.
- **`bt.SubTree` vs `subprocess=True` pBTs**: `bt.SubTree` shares the parent
  tree's blackboard (`PROCESS_TREE` scope). A Parameterized Behavior Tree saved
  with `tree.set_asset_metadata(..., subprocess=True)` runs in an **isolated**
  blackboard scope and can only exchange data via `tree.params.<field>` and
  `tree.return_value_expression`.

---

## 5. Critical Gotchas, Runtime Diagnostics & Safety Invariants

- **Never Set `disable_collision_checking=True`**:
  `test_behaviors.py` (`test_ur_robot_has_no_disable_collision_checking`)
  forbids `disable_collision_checking=True`. Always keep
  `disable_collision_checking=False` and pass segment-scoped
  `CollisionSettings.CollisionRule` entries with
  `CollisionAction(is_excluded=True)` via
  `Robot._build_collision_settings(excluded_collision_pairs)` (filtering out
  objects not yet in the world via `object_exists_in_world` so `move_robot`
  does not fail with `Object not found`).
- **Post-`move_to_contact` & Post-Attach/Detach Collisions (`StatusCode: ai.intrinsic.move_robot:10201`)**:
  - When `move_robot` fails with `10201` (*"Invalid initial joint configuration.
    Collision reported: Left entity: `<A>`, Right entities: `<B>`"* or *"IK
    could not find a collision free configuration for the end of segment ..."*),
    inspect which entity pair is in contact at the segment start or goal.
  - After a compliant touchdown (`move_to_contact`) seats the workpiece or
    gripper fingers onto a support surface (such as `enclosure` or a CNC vise
    `schunk_egp_64nnb`), **both** `(support_surface, workpiece_object_name)`
    **and** `(tool_object_name, support_surface)` — in addition to
    `(tool_object_name, workpiece_object_name)` — can remain in geometric
    contact at the start of the subsequent retract/exit `move_robot` task (or at
    the end of an approach segment).
  - Always set `robot.enclosure_object_name` in `configs/<cell>/app_config.yaml`
    when the cell table/enclosure is a contact surface, and include all three
    pairs (`(tool, workpiece)`, `(support_surface, workpiece)`, and
    `(tool, support_surface)`) in `excluded_collision_pairs` for approach and
    post-touchdown retract motions (`pick.py`, `load_machine.py`,
    `unload_machine.py`, `return_infeed.py`).
- **`move_to_contact` Stabilization Timeouts in Simulation (`StatusCode: ai.intrinsic.move_to_contact:10301`)**:
  - `ai.intrinsic.move_to_contact` executes a 4-stage ICON state machine:
    `ActionId.TARE (0)` -> `ActionId.APPROACH (1)` (`MakeContact` until
    `ELAPSED_TIME_SECONDS > 0.05s` and `SENSED_FORCE > stop_switching_force`) ->
    `ActionId.STABILIZE (2)` (`MakeContact` with
    `disable_reference_lowpass_filter=True` waiting for
    `SENSED_FORCE > stop_switching_force` and `IS_SETTLED == True` within
    `SETTLING_TIMEOUT_S = 2.0s`) -> `ActionId.JOINT_STOP (4)`.
  - When the `2.0s` `STABILIZE` window expires with
    `sensed_force_magnitude <= min_contact_force` (`1.0 N`), `move_to_contact`
    fails with `10301` (*"Stabilize action timed out without making contact.
    Force is ... N."*). In Gazebo, this occurs either when a descent force
    transient (`> 0.05s`) triggers `STABILIZE` early and `JOINT_STOP` halts the
    arm partway down from standoff, or when the tool/workpiece reaches the
    surface and Gazebo's discrete rigid-body contact solver fails to sustain a
    static reaction force `> 1.0 N` at zero velocity.
  - Always wrap `move_to_contact` via `create_seated_approach_tasks` /
    `create_compliant_touchdown_task` (`src/behaviors/motions.py`) in a
    `bt.Fallback` + `bt.Retry` (`max_tries=2`): the `bt.Retry` `recovery` node
    must move linearly back to the unloaded standoff pose
    (`-touchdown.standoff_m`) so `ActionId.TARE (0)` zeroes the F/T sensor in
    free space before retrying, and the outer `bt.Fallback` executes a linear
    Cartesian move to the seated contact pose (`Fallback Linear Seat`) if
    retries still time out in simulation.
- **Multi-Cell Optional Hardware (`if machine is not None:`) & Support-Surface Fallback**:
  - Cells without a CNC enclosure or vise (such as `lab_bb_01`) omit `machine`
    (or set `machine: null`) in `configs/<cell>/app_config.yaml`. Phase builders
    must guard all CNC door, vise, and cycle handshake nodes with
    `if machine is not None:`.
  - When `machine` is `None` (`vise_object_name` is `None`), `place_vise` sits
    directly on the cell's support surface (`config.robot.enclosure_object_name`).
    Both `load_machine.py` and `unload_machine.py` must fall back to excluding
    `[(config.robot.enclosure_object_name, workpiece_object_name),
    (config.robot.tool_object_name, config.robot.enclosure_object_name)]` when
    `not vise_object_name and config.robot.enclosure_object_name`.
- **Out-of-Envelope IK Failures Across Cells (`StatusCode: ai.intrinsic.move_robot:10301`)**:
  - `move_robot:10301` (*"IK solver couldn't find any solutions for the given
    constraints. Target frame: ..."*) indicates that a target frame (or a
    dynamically shifted frame such as `cycle.return_shift` in
    `randomize_placement_frame`) is outside the arm's kinematic reach or
    violates orientation constraints.
  - When configuring or debugging a cell with a shorter-reach arm or offset base
    mounting (e.g., UR3e in `lab_bb_01` vs UR5e in `omts`), compare root-frame
    coordinates in `configs/<cell>/app_config.yaml` (such as
    `cycle.return_shift.{center_x, center_y, bounds_x, bounds_y}`) against the
    arm base transform in `configs/<cell>/align_robot.updates.pbtxt`, the
    workpiece/frame poses in `configs/<cell>/scene.updates.pbtxt`, or the live
    belief world (`bazel run //tools/world:inspect_world -- --address=localhost:17080`).
- **Never Call `ai.intrinsic.sleep_for`**:
  `sleep_for` is not bundled in open-source `@intrinsic-core` or
  `//:omts_solution`. Implement deterministic delays via `create_dwell_task`
  (`src/utils/script_utils.py`) or `bt.PythonScript`:

```python
delay_node = bt.Task(
  action=bt.PythonScript(
    function_body=f"import time\ntime.sleep({float(delay_s)!r})"
  ),
  name=f"Wait Actuation Delay ({delay_s}s)",
)
```

- **`solution.resources` Raises `KeyError` on Missing Keys**:
  `solution.resources.__getattr__` raises `KeyError` (not `AttributeError`),
  which breaks `hasattr(solution.resources, name)`. Use `try ... except
  (KeyError, AttributeError):` or `resolve_adio_resource(solution, adio_device)`
  from `src/utils/math_utils.py`.

---

## 6. Verification, Hermetic Testing & Simulation Workflow

Every Behavior Tree change must pass the hermetic contract suite in
`tests/unit/test_behaviors.py` (`HermeticSolutionAndBehaviorTreeContractTest`),
which loads the real `.asset_info.binpb` protobuf descriptors from
`//:omts_solution_manifest` into a `FakeSolution` without a live cluster:

```bash
# 1. Run hermetic Behavior Tree & protobuf contract tests (synchronous):
bazel test --test_output=errors //tests/unit:test_behaviors

# 2. Run the full test suite:
bazel test --test_output=errors //...

# 3. Run Ruff formatting & linting (2-space indent, <=80 chars, double quotes):
ruff format --check .
ruff check .

# 4. Live simulation verification (reset world before each run):
inctl world reset --address=localhost:17080
bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --config="configs/lab_bb_01/app_config.yaml"
```

When adding a new skill to a Behavior Tree, register its Bazel target in
`_OMTS_SKILL_ASSETS` in root `BUILD` (consumed by `solution_manifest_only` in
`bazel/solution_manifest.bzl`) so `//:omts_solution` and
`//:omts_solution_manifest` remain in lockstep.

---

## 7. Progressive Reference Guides

Load these reference files on demand when you need complete node signatures,
blackboard/CEL specifications, or full skill/protobuf schemas:

- **[references/node_and_blackboard_reference.md](references/node_and_blackboard_reference.md)**:
  Complete constructor signatures and semantics for all `bt.*` nodes,
  `bt.Branch` / `bt.Selector` / `bt.Fallback`, all 6 `bt.Condition` classes,
  `bt.Loop` (`loop_counter.value`) and `bt.Retry`, `BlackboardValue` indexing,
  `cel.CelExpression`, `bt.Data`, `bt.PythonScript` (`BasicComputeContext` and
  `ObjectWorldClient` methods), and Parameterized Behavior Trees (pBTs).
- **[references/skills_and_protos_reference.md](references/skills_and_protos_reference.md)**:
  Step-by-step `@intrinsic-core` / `@intrinsic_apis` proto discovery commands,
  `.bundle.tar` descriptor inspection, the Master 4-Way Protobuf Translation
  Table, automatic Pythonic type conversions, and complete field-by-field
  schemas and code examples for `move_robot`, `move_to_contact`, `update_world`,
  `attach_object_to_robot`, `detach_object`, `capture_images`,
  `estimate_pose_multi_view`, `create_object`, `gripper_cmd_skill`, and DIO
  skills.
