# Mesh grasp planning (`src/grasping/`)

The `mesh` grasp planner picks a Robotiq Hand-E grasp from the workpiece's
triangle mesh instead of its bounding box, so curved and non-cuboid parts can
be picked. It is **simulation only**: the gripper model comes from the Hand-E
SDF colliders and is not calibrated against hardware.

## How it works

After perception updates the workpiece pose, the pick opens the gripper and
runs the [`mesh_plan_grasp`](../skills/mesh_plan_grasp/) skill, which:

1. Generates top-down candidates ([`candidates.py`](candidates.py)): it
   samples points on the near-vertical faces of the mesh, casts a ray from each
   point through the part to the opposite contact, and keeps pairs that fit the
   10 to 60 mm jaw opening, whose normals oppose each other within 15 degrees,
   and whose closing fingers do not cross another part of the mesh. Candidates
   closest to the part's centroid come first.
2. Checks candidates in that order with Core's motion planner: collision-checked
   inverse kinematics at the pregrasp (jaws open), then a collision-checked
   Cartesian-linear plan to the grasp. This uses
   Core's own collision models of the gripper, the part, the table and the
   enclosure.
3. Writes the first candidate that passes to the `grasp` and `pre_grasp`
   frames, creating missing frames on `root`. If none passes, the skill fails.

The pick then moves linearly to the grasp pose instead of touching down with
force control first, closes the gripper and lifts. The unload does the same: it
plans a new grasp on the part where the robot placed it, using separate
`grasp_unload` and `pre_grasp_unload` frames so the infeed return targets stay
unchanged (the suffix follows the configured grasp frame names).

## Configuration

```yaml
grasp:
  planner: "mesh"
  mesh_simulation_only: true
  mesh_pregrasp_distance_m: 0.1   # default
  mesh_max_approach_tilt_deg: 10.0 # default
```

Or select it for one run with `--grasp_planner=mesh --mesh_simulation_only`.

The skill plans on the workpiece's visual mesh, the `.glb` its SDF names, which
it reads from the Object World. That mesh must be closed and outward facing,
with at most 10,000 triangles. Validation rejects open edges, inconsistent
winding, nonmanifold edges and degenerate triangles; duplicated seam vertices
are matched by their coordinates.

## Demo workpieces (Lab BB-01)

The lab cell runs `raw_stock_2x3x5` (the default), `banana` or `bracket`. Name
the workpiece when deploying, and again when running the app:

```bash
bazel run //:omts_solution -c opt --//:setup=lab_bb_01 --//:workpiece=banana -- \
  --address=localhost:17080 \
  --operation_mode=sim

bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --config=configs/lab_bb_01/app_config.yaml \
  --workpiece=banana \
  --grasp_planner=mesh \
  --mesh_simulation_only
```

`--workpiece` sets the workpiece's scene object, `ai.intrinsic.<workpiece>`,
and its pose estimator, `ai.intrinsic.<workpiece>_estimator_text`. Register that
estimator as for the raw stock in the top-level README. The demo parts are
placed by `configs/lab_bb_01_objects/`.

## Limits

- Only the Hand-E is modeled (`candidates.HANDE`).
- Only top-down grasps are planned.
- Mass, friction and holding force are not modeled. The simulated gripper
  welds the part to a finger when it closes while touching it, and holds
  nothing otherwise. A simulated lift shows that the fingers reached the
  part, not that the grasp would hold.
- In the Lab BB-01 simulation the table's collision surface is 22 mm below the
  visible table, so parts rest slightly sunk into it; and Gazebo pads mesh
  colliders when it decomposes them into convex pieces, so the fingers stop
  short of curved parts (about 2.6 mm per side on the banana).
