# MoveIt Grasp Planning (third-party integration)

Model-based grasp planning for OMTS, backed by
[`intrinsic-ai/intrinsic-moveit`](https://github.com/intrinsic-ai/intrinsic-moveit)
and MoveIt Task Constructor. Given a part in the Object World, the planning
service enumerates grasp candidates over the part's surfaces, ranks them, and
writes the winner's grasp and pre-grasp poses back into the world. Two CLIs
drive it for scene bring-up and rehearsal.

> [!IMPORTANT]
> This is **not** part of first-party OMTS and OMTS never deploys it for you.
> Every command below is inert until you have completed the intrinsic-moveit
> integration. If `//:omts_solution` is all you have deployed, start with
> [Prerequisites](#prerequisites).

Once a first-party grasp planner ships it will become the default and will live
under `src/` and `tools/grasping/` using the unqualified names. Everything here
is named `moveit_*` so the two never have to be told apart by context.

---

## Prerequisites

Work through the intrinsic-moveit repository first — it is checked out beside
`omts/` on a machine with IOC set up, so
[`../../../intrinsic-moveit/README.md`](../../../intrinsic-moveit/README.md).

1. **Build and install the bundles.** Follow *Build & Deploy to Flowstate
   Cluster (Sideloading)* to produce and install
   `ai.intrinsic.moveit_planning_service` and
   `ai.intrinsic.moveit_plan_grasp_skill`. On IOC, install with
   `inctl` directly rather than the repository's `make install_*` targets, which
   assume `--org`/`--cluster`:

   ```bash
   ./bin/inctl asset install --address localhost:17080 \
       ./images/moveit_plan_grasp_skill.bundle.tar
   ```

2. **Configure the ROS bridge, and bring it up first.** The bridge is the
   *upstream* `flowstate_ros_bridge` from `sdk-ros` — intrinsic-moveit does not
   ship one, it only supplies a configuration and a launch wrapper for it. Its
   stock configuration carries placeholders (`robot/robot/base_link`, an empty
   joint name list) that leave MoveIt with no usable joint states, so a
   cluster-deployed bridge instance must first be reconfigured from
   `configs/flowstate_ros_bridge_config.pbtxt` — chiefly
   `robot_base_frame_id: "ur_module/base_link"`, `robot_controller_instance:
   "icon"`, and the six UR joint names. See intrinsic-moveit's
   `docs/flowstate_ros_bridge_configuration.md`.

   Order matters: `moveit_planning_service` verifies its planning scene at
   startup and needs the bridge's static transforms and `/joint_states` already
   flowing. A locally launched bridge
   (`ros2 launch moveit_planning_service flowstate_ros_bridge.launch.py`) applies
   the correct values as launch defaults and needs no extra configuration.

3. **Create the output frames.** The grasp skill only *updates* frames; it never
   creates them. `root/grasp` and `root/pre_grasp` are declared in
   [`configs/scene.updates.pbtxt`](../../configs/scene.updates.pbtxt):

   ```bash
   bazel run //tools/world:apply_scene_updates -- --address=localhost:17080
   ```

4. **Smoke-test the service** with the raw `ros2 service call` in
   intrinsic-moveit's *Testing Integration with ROS 2 Service Calls* before
   involving OMTS. If that call fails, nothing here will work.

---

## Layout

| Path | Role |
| :--- | :--- |
| [`moveit_grasp_planner.py`](./moveit_grasp_planner.py) | `MoveItGraspPlannerInterface` and its `MoveItGraspPlanner` / `MockMoveItGraspPlanner` implementations; surface constants. |
| [`moveit_grasp_planning.py`](./moveit_grasp_planning.py) | `build_moveit_grasp_planning_subtree` — plan a grasp, then approach the resulting pre-grasp. |
| [`moveit_grasp_tour.py`](./moveit_grasp_tour.py) | `build_moveit_grasp_tour_subtree` — the same block repeated over several parts. |
| [`tools/`](./tools/) | The two CLIs. |
| [`tests/`](./tests/) | Offline tests for both subtrees and both CLIs' argument parsing. |

The integration borrows exactly two things from first-party OMTS:
`//src/hardware:robot` (for `RobotInterface` and `UrRobot`) and
`//src/behaviors:motions` (for the approach move). Nothing first-party depends
on anything here.

---

## `tools:moveit_plan_and_move`

Plans a grasp and moves the arm to the resulting pre-grasp frame. Because the
skill writes its result into the world, the approach motion just targets
`root/pre_grasp` by name — there is no pose hand-off between the two steps.

```bash
# Dry run: plan a grasp on raw_stock_50x50x75_1 without moving the arm
bazel run //third_party/intrinsic_moveit/tools:moveit_plan_and_move -- \
    --address=localhost:17080 --plan_only --surfaces=0,1,4,5

# Plan and approach the pre-grasp of a specific part
bazel run //third_party/intrinsic_moveit/tools:moveit_plan_and_move -- \
    --address=localhost:17080 --surfaces=0,1,4,5 \
    --target_object=raw_stock_50x50x75_2

# Rank grasps across all three parts and approach the best one.
# --target_object is repeatable and each value accepts a comma-separated list.
bazel run //third_party/intrinsic_moveit/tools:moveit_plan_and_move -- \
    --address=localhost:17080 --surfaces=0,1,4,5 \
    --target_object=raw_stock_50x50x75_1,raw_stock_50x50x75_2,raw_stock_50x50x75_3

# Narrow further to force a strictly vertical approach
bazel run //third_party/intrinsic_moveit/tools:moveit_plan_and_move -- \
    --address=localhost:17080 --surfaces=4 --num_rotations=8
```

---

## `tools:moveit_grasp_tour`

Visits several parts in sequence. For each object it plans a grasp and
approaches the resulting pre-grasp frame, then moves on. The arm stops at the
pre-grasp and never descends to the grasp pose, so nothing is touched and the
gripper is never commanded. This is the rehearsal loop for validating that every
part in a scene is plannable and reachable before wiring a real pick.

```bash
# Plan a grasp for every part without moving the arm
bazel run //third_party/intrinsic_moveit/tools:moveit_grasp_tour -- \
    --address=localhost:17080 --plan_only --surfaces=0,1,4,5

# Full tour over the default objects (_1 .. _3)
bazel run //third_party/intrinsic_moveit/tools:moveit_grasp_tour -- \
    --address=localhost:17080 --surfaces=0,1,4,5

# Objects are visited in the order given; --object is repeatable
bazel run //third_party/intrinsic_moveit/tools:moveit_grasp_tour -- \
    --address=localhost:17080 --surfaces=0,1,4,5 \
    --object=raw_stock_50x50x75_2 \
    --object=raw_stock_50x50x75_1

# Validate spec syntax offline, without connecting to a solution
bazel run //third_party/intrinsic_moveit/tools:moveit_grasp_tour -- \
    --dry_run --surfaces=0,1,4,5
```

Apart from `--surfaces`, the default scene needs no tuning: every part,
including the one held in the vice, plans and approaches with the stock rotation
count and pre-grasp gap. Per-object overrides are appended to the object name as
`name[:key=value]...` and win over the corresponding global flag:

| Key | Meaning | Global fallback |
| :--- | :--- | :--- |
| `surfaces` | Surface indices to sample, `0:+X 1:-X 2:+Y 3:-Y 4:+Z 5:-Z`, or `all` | `--surfaces` (default `all`) |
| `rotations` | Grasp candidates per surface | `--num_rotations` (default `4`) |
| `retract` | Gap in meters between the grasp and pre-grasp frames | `--retract_dist_m` (default: the planner's own 0.05 m) |

Narrowing the surface set does not improve the odds of finding *a* grasp — it
removes candidates. Use it to choose which grasps are acceptable (`0,1,4,5` to
bar end-cap picks, `4` to force a strictly vertical approach), and reach for
`rotations` or `retract` only when a part actually fails.

Per object the tool emits a single motion: `move_cartesian` to `root/pre_grasp`
with motion type `ANY`, issued after that object's plan lands.

> [!IMPORTANT]
> One plan is issued **per object**, never pooled. `root/grasp` and
> `root/pre_grasp` are singleton frames that the skill overwrites in place, so a
> pooled request would rank all candidates jointly and approach only the winner.
> Sequencing the plans is what makes a tour possible without per-object frames
> or a blackboard.

> [!CAUTION]
> The tour deliberately does **not** descend from the pre-grasp to the grasp
> pose. An earlier version ran a compliant touchdown at each stop and crashed on
> the way down. Approaching a planned pre-grasp is a much weaker claim than
> executing the descent into it: the pre-grasp is collision-checked as a goal
> pose, whereas the descent traverses the volume immediately around the part,
> where the planned grasp axis and the real clearances have to agree. Keep
> descent work in [`src/behaviors/pick.py`](../../src/behaviors/pick.py), where
> the gripper is actually in the loop.

`retract_dist_m` therefore does double duty: it is both the gap between the
grasp and pre-grasp frames *and* how far above the part the tour comes to rest.
Shrinking it makes the rehearsal a closer approximation of a real pick, and a
more demanding one.

Three defaults differ from what you might expect, and all three are
flag-controlled:

| Default | Value | Why |
| :--- | :--- | :--- |
| Failure handling | fail fast | The tour aborts on the first unplannable or unreachable object, so the error surfaces immediately during bring-up. `--continue_on_failure` skips the object instead. |
| `--transit_frame` | off | Objects are swept pre-grasp to pre-grasp. Pass `--transit_frame=view` to route via `root/view` if the direct transit paths are awkward. |
| Collision checking | on | Unlike `UrRobot`'s own default, which disables it. The tour reaches into the vice, and it runs unattended across several parts, so the check is worth its cost here. |

---

<a id="why-surfaces-0145"></a>
## Why `--surfaces=0,1,4,5`

The stock's collision box is `0.05 0.075 0.05` m
([`raw_stock_50x50x75.sdf:L32`](../../models/raw_stock_50x50x75/raw_stock_50x50x75.sdf#L32)),
so its **long axis is Y**. That splits the six faces into two groups:

| Surfaces | Face | Size |
| :--- | :--- | :--- |
| `0`, `1` (±X) and `4`, `5` (±Z) | the four **long faces** | 50 × 75 mm |
| `2`, `3` (±Y) | the two **end caps** | 50 × 50 mm |

`0,1,4,5` is exactly "any long face, never an end cap". Approaching a long face
puts the jaws across the block's middle; approaching an end cap grips it by the
tip, leaving 75 mm of part cantilevered out of the gripper. Both are
geometrically reachable — the jaws span 50 mm either way — so the planner will
happily pick an end-cap grasp if it is allowed to, which is why the set has to
be stated rather than left to collision checking to sort out.

This is a property of *this* part, not a general rule: the tools still default
to `all`, which is the right default for stock of unknown shape and orientation.

---

## Tests

These are **not** in `//tests/unit:all`, which stays first-party:

```bash
bazel test //third_party/intrinsic_moveit/tests:all
```

They run fully offline against `MockMoveItGraspPlanner` and `MockRobot`; no
cluster, robot or planning service is required.

---

## Troubleshooting

| Error Code / Message | Root Cause | Resolution |
| :--- | :--- | :--- |
| `moveit_plan_grasp_skill: Robot model does not define the required end-effector or IK frame` | `--tool_frame_name` or `--end_effector_group` names something absent from the SRDF. Note `hande_tcp`, `hande_tool_frame` and `tool_frame` are all valid coincident aliases, so this is usually a typo or a hardware description that lacks the alias links. | Check the `hand` group in `robot_hardware.srdf` and pass a link listed there. |
| `moveit_plan_grasp_skill: Output pregrasp frame could not be resolved in World Service` | The skill only updates pre-existing frames; it never creates them. | Run `bazel run //tools/world:apply_scene_updates` so `root/grasp` and `root/pre_grasp` exist before planning. |
| `moveit_plan_grasp_skill: Target object was not found in the planning scene` | The part was moved moments before planning and the MoveIt scene has not caught up, or the object name does not resolve. | Raise `--settle_sec`, and confirm the name with `bazel run //tools/world:inspect_world`. |
| `moveit_plan_grasp_skill: No reachable or collision-free grasp candidates found` | The surface set is too narrow for how the part is currently lying, the part is out of reach or occluded, or the candidate set is too coarse. | Widen `--surfaces` — `all` is the widest, and `0,1,4,5` is the demo's set. Then raise `--num_rotations`. Confirm reachability with `bazel run //tools/world:inspect_world`. |
| A part that used to work now finds no candidates, or its pre-grasp is unreachable | All three default parts, including the one in the vice, are known to plan and approach with `--surfaces=0,1,4,5` and stock rotations and retract. A new failure usually means the part moved, the scene drifted, or the arm is starting from an awkward pose. | Reset the world with `inctl world reset --address localhost:17080`, then re-check the pose with `bazel run //tools/world:inspect_world`. Only then reach for `--object=<name>:surfaces=...` or `:retract=...`. |
| The tour aborts partway and you want the remaining objects checked anyway | The tour fails fast by design, so the first bad object stops the run. | Re-run with `--continue_on_failure` to skip unplannable objects and collect every failure in one pass. |
