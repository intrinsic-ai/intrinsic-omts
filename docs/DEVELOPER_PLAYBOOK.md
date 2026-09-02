# OMTS Developer Playbook & Operations Guide

This document covers day-to-day development, execution, testing, debugging tools, and troubleshooting workflows for OMTS.

---

## 1. Bazel Build & Run Reference

### Application Execution
To run the full machine tending pipeline against a live or simulated solution deployment:

```bash
# Run with vision-guided perception infeed
bazel run //src:omts_app -- --address=localhost:17080 --infeed_mode=perception

# Run with deterministic grid pallet infeed
bazel run //src:omts_app -- --address=localhost:17080 --infeed_mode=grid
```

### Build Configurations
OMTS supports multiple hardware cells via Bazel build flags:

```bash
# Default workcell (UR5e arm + OMTS CNC enclosure)
bazel build //:omts_solution

# Lab BB-01 workcell (UR3e arm + CAW enclosure + Orbbec Gemini camera)
bazel build //:omts_solution --//:lab=lab_bb_01
```

### Running Unit Tests
All unit tests run offline using mocks and mock SBL constructs:

```bash
bazel test //tests/...
```

---

## 2. Developer & Diagnostic Tools

OMTS provides diagnostic and interactive jogging utilities under [`tools/`](../tools/):

### A. Scene Updates Tool (`tools/world:apply_scene_updates`)
Applies `.pbtxt` world update files live to the running solution without restarting the cluster:

```bash
bazel run //tools/world:apply_scene_updates
```

* **Files applied by default:**
  * [`configs/ur_module.attachments.updates.pbtxt`](../configs/ur_module.attachments.updates.pbtxt) (robot mounting, gripper, and tool frame attachments).
  * [`configs/scene.updates.pbtxt`](../configs/scene.updates.pbtxt) (pre-defined scene frames: `view`, `pre_grasp`, `grasp`, `machine_approach`, `place_vise`).
  * [`configs/align_robot.updates.pbtxt`](../configs/align_robot.updates.pbtxt) (base mounting pose).
  * [`configs/lab_bb_01_orbbec_gemini.updates.pbtxt`](../configs/lab_bb_01_orbbec_gemini.updates.pbtxt) (camera wrist mounting).
* **Automated Cleanup:** Checks for and deletes stale `detected_workpiece` objects to prevent SBL name collisions.

### B. World Inspector (`tools/world:inspect_world`)
Inspects active objects, kinematic links, frames, joint configurations, and live transforms:

```bash
bazel run //tools/world:inspect_world -- --address=localhost:17080
```

* Outputs:
  * Tree of all objects and child frames in the active belief world.
  * Joint configurations defined on robots.
  * Live Cartesian poses for `flange`, `tool_frame`, `view`, `pre_grasp`, and `grasp`.

### C. Interactive Robot Jogger (`tools/jogging:move_to_frame`)
Discovers all frames in the scene and prompts interactively to move the robot to any target frame:

```bash
# Interactive frame selection
bazel run //tools/jogging:move_to_frame -- --address=localhost:17080

# Move directly to a specific frame
bazel run //tools/jogging:move_to_frame -- --address=localhost:17080 --frame=view

# Move linearly to pre_grasp
bazel run //tools/jogging:move_to_frame -- --address=localhost:17080 --frame=pre_grasp --motion_type=LINEAR
```

---

## 3. Common Troubleshooting & Gotchas

| Error Code / Message | Root Cause | Resolution |
| :--- | :--- | :--- |
| `ai.intrinsic.move_robot:10301: IK solver couldn't find any solutions` | 1. Tool orientation misaligned (e.g. tool $+Z$ pointing horizontal or upward).<br>2. Target frame position outside robot reach. | 1. Ensure tool orientation aligns with downward approach via $\mathbf{q}_{\text{tool}} = \mathbf{q}_{\text{part}} \cdot [0.5, 0.5, 0.5, 0.5]$.<br>2. Run `bazel run //tools/world:inspect_world` to verify target $(x, y, z)$. |
| `ai.intrinsic.move_robot:10601: Frame with name ... does not exist on object` | Attempted to target an object reference instead of a valid frame in `PoseEquality`. | Ensure `target_frame` references a valid frame (`FrameReferenceByName(object_name="root", frame_name="pre_grasp")`). Do not target raw objects. |
| `ai.intrinsic.create_object:3: New object name cannot be used here` | Object with the specified name already exists in the belief world. | Issue a `DeleteObjectRequest` via `world.batch_update(...)` before executing the tree. |
| `ai.intrinsic.executive:13001: Execution failed (PROTECTIVE_STOP)` | UR arm or ICON entered protective stop due to contact or torque limits. | 1. Clear faults on the UR teach pendant.<br>2. Run `clear-faults` and world reset on cluster.<br>3. Verify motion constraints before re-running. |
| `ModuleNotFoundError: No module named 'intrinsic'` | Python command run directly via system Python instead of Bazel runfiles. | Always execute scripts via `bazel run //path:target` to resolve dependencies within the hermetic sandbox. |
