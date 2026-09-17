# MoveIt Grasp Planning (third-party integration)

Model-based grasp planning for OMTS, backed by
[`intrinsic-ai/intrinsic-moveit`](https://github.com/intrinsic-ai/intrinsic-moveit)
and MoveIt Task Constructor. Given a part in the Object World, the planning
service enumerates grasp candidates over the part's surfaces, ranks them, and
writes the winner's grasp and pre-grasp poses back into the world. A dedicated
CLI tool drives it for scene bring-up and verification.

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
| [`tools/moveit_plan_grasp_and_move.py`](./tools/moveit_plan_grasp_and_move.py) | CLI tool to plan a grasp on target workpiece(s) and move the arm to pre-grasp. |
| [`tests/`](./tests/) | Offline unit tests for grasp planning subtree and CLI argument parsing. |

The integration borrows exactly two things from first-party OMTS:
`//src/hardware:robot` (for `RobotInterface` and `UrRobot`) and
`//src/behaviors:motions` (for the approach move). Nothing first-party depends
on anything here.

---

## `tools:moveit_plan_grasp_and_move`

Plans a grasp and moves the arm to the resulting pre-grasp frame. Because the
skill writes its result into the world, the approach motion just targets
`root/pre_grasp` by name — there is no pose hand-off between the two steps.

The tool stops at the pre-grasp and does not descend to the grasp pose, so the
workpiece is untouched and the gripper is never commanded. This makes it safe
to run repeatedly while validating that the part is plannable and reachable
before executing real picks in [`src/behaviors/pick.py`](../../src/behaviors/pick.py).

### Tutorial Workflow

#### 1. Plan and Approach on the Tabletop Surface
When the solution starts up, `raw_stock_50x50x75` is positioned on the table
surface by default via [`configs/raw_stock_on_surface.updates.pbtxt`](../../configs/raw_stock_on_surface.updates.pbtxt).

```bash
# Dry run: plan a grasp on raw_stock_50x50x75 without moving the arm
bazel run //third_party/intrinsic_moveit/tools:moveit_plan_grasp_and_move -- \
    --address=localhost:17080 --plan_only --surfaces=0,1,4,5

# Plan and approach the pre-grasp on the table surface
bazel run //third_party/intrinsic_moveit/tools:moveit_plan_grasp_and_move -- \
    --address=localhost:17080 --surfaces=0,1,4,5
```

#### 2. Relocate Workpiece to the CNC Vice & Plan Again
Next, apply the vice scene update live to the running solution to place the block
inside the CNC machine vice:

```bash
bazel run //tools/world:apply_scene_updates -- \
    --address=localhost:17080 \
    --files configs/raw_stock_in_vice.updates.pbtxt
```

Now execute `moveit_plan_grasp_and_move` again. The planner will generate reachable
grasp candidates inside the vice and navigate the arm to the pre-grasp pose:

```bash
bazel run //third_party/intrinsic_moveit/tools:moveit_plan_grasp_and_move -- \
    --address=localhost:17080 --surfaces=0,1,4,5
```

#### 3. Resetting the Scene
To return the workpiece to its initial pose on the tabletop surface, either apply
the surface update config or reset the world:

```bash
bazel run //tools/world:apply_scene_updates -- \
    --address=localhost:17080 \
    --files configs/raw_stock_on_surface.updates.pbtxt

# Or reset the entire world:
inctl world reset --address localhost:17080
```

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
| `moveit_plan_grasp_skill: No reachable or collision-free grasp candidates found` | The surface set is too narrow for how the part is currently lying, the part is out of reach or occluded, or the candidate set is too coarse. | Widen `--surfaces` — `all` is the widest, and `0,1,4,5` is the demo's set. Confirm reachability with `bazel run //tools/world:inspect_world`. |
| A part that used to work now finds no candidates, or its pre-grasp is unreachable | The part, including when placed in the vice, plans and approaches with `--surfaces=0,1,4,5` and default retraction. A new failure usually means the part moved, the scene drifted, or the arm is starting from an awkward pose. | Reset the world with `inctl world reset --address localhost:17080`, re-apply the position updates, and re-check the pose with `bazel run //tools/world:inspect_world`. |
