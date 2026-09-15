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
bazel build //:omts_solution --config=lab_bb_01
```

### Running Unit Tests
All unit tests run offline using hermetic mocks and mock SBL constructs:

```bash
# Run all unit tests
bazel test //tests/unit:all

# Run a specific unit test target
bazel test //tests/unit:test_behaviors
bazel test //tests/unit:test_hardware_adapters
bazel test //tests/unit:test_script_utils
```

There are currently 9 offline unit test suites: `test_infeed`, `test_tray`, `test_workpiece`, `test_behaviors`, `test_hardware_adapters`, `test_move_to_frame`, `test_store_frame`, `test_script_utils`, and `test_control_gripper`.

---

## 2. Developer & Diagnostic Tools

OMTS provides diagnostic and interactive jogging utilities under [`tools/`](../tools/):

### A. Scene Updates Tool (`tools/world:apply_scene_updates`)
Applies `.pbtxt` world update files live to the running solution without restarting the cluster:

```bash
bazel run //tools/world:apply_scene_updates -- --address=localhost:17080
```

* **Files applied by default:**
  * [`configs/ur_module.attachments.updates.pbtxt`](../configs/ur_module.attachments.updates.pbtxt) (robot mounting, gripper, and tool frame attachments).
  * [`configs/lab_bb_01_orbbec_gemini.updates.pbtxt`](../configs/lab_bb_01_orbbec_gemini.updates.pbtxt) (camera wrist mounting and flange parentage).
  * [`configs/scene.updates.pbtxt`](../configs/scene.updates.pbtxt) (pre-defined scene frames: `view`, `pre_grasp`, `grasp`, `machine_approach`, `pre_place_vise`, `place_vise`).
  * [`configs/align_robot.updates.pbtxt`](../configs/align_robot.updates.pbtxt) (base mounting pose).
* **Automated Cleanup:** Checks for and deletes stale `detected_workpiece` objects to prevent SBL name collisions.

> [!TIP]
> `apply_scene_updates` does **not** restore the raw stock poses — [`configs/relocate_raw_stock.updates.pbtxt`](../configs/relocate_raw_stock.updates.pbtxt) is not in its default file set. To put the whole scene back between demo runs, reset the world instead:
>
> ```bash
> inctl world reset --address localhost:17080
> ```
>
> This restores object poses only. It does not command the arm, so the robot stays wherever the last run left it; use [`tools/jogging:move_to_frame`](../tools/jogging/move_to_frame.py) if you need it back at a known pose.

> [!CRITICAL]
> **Camera Reparenting on Solution Startup:**
> When the Intrinsic solution container or simulation restarts, `orbbec_camera` defaults to a child of `root` at the origin `Pose3(identity, [0, 0, 0])`.
> You **must** run `bazel run //tools/world:apply_scene_updates` after starting the solution. Without applying `lab_bb_01_orbbec_gemini.updates.pbtxt`, `world.get_transform(root, camera.sensor)` evaluates to identity, causing dynamic grasp calculations to place `pre_grasp` at raw optical coordinates behind the robot base column (~`[-0.058, 0.009, 0.392]`), failing IK with `move_robot:10301`.

### B. World Inspector (`tools/world:inspect_world`)
Inspects active objects, kinematic links, frames, joint configurations, and live transforms:

```bash
bazel run //tools/world:inspect_world -- --address=localhost:17080
```

* Outputs:
  * Tree of all objects and child frames in the active belief world.
  * Joint configurations defined on robots.
  * Live Cartesian poses for `flange`, `tool_frame`, `view`, `pre_grasp`, `grasp`, `camera.sensor`, and `raw_stock_2x3x5`.

### C. Interactive Robot Jogger (`tools/jogging:move_to_frame`)
Discovers all frames in the scene and prompts interactively to move the robot to any target frame:

```bash
# Interactive frame selection
bazel run //tools/jogging:move_to_frame -- --address=localhost:17080

# Move directly to view frame
bazel run //tools/jogging:move_to_frame -- --address=localhost:17080 --frame=view

# Move linearly to pre_grasp
bazel run //tools/jogging:move_to_frame -- --address=localhost:17080 --frame=pre_grasp --motion_type=LINEAR
```

### D. Gripper Controller (`tools/gripper:control_gripper`)
Commands robotic end-effectors (Robotiq Hand-e, DIO, sideloaded, or mock) via interactive CLI menu or direct command-line arguments:

```bash
# Interactive terminal menu (open, close, quit)
bazel run //tools/gripper:control_gripper -- --address=localhost:17080

# Command open directly on live Robotiq gripper
bazel run //tools/gripper:control_gripper -- --address=localhost:17080 --action=open

# Command close directly on live Robotiq gripper
bazel run //tools/gripper:control_gripper -- --address=localhost:17080 --action=close

# Command pneumatic / digital I/O gripper
bazel run //tools/gripper:control_gripper -- --address=localhost:17080 --gripper_type=dio --open_pin=0 --close_pin=1 --action=open

# Offline mock mode for testing without a live solution
bazel run //tools/gripper:control_gripper -- --mock --action=open
```

### E. MoveIt Grasp Planning (third-party)

Model-based grasp planning and the `moveit_grasp_tour` rehearsal loop are a
third-party integration, not part of OMTS. They are documented with the code
they belong to, in
[`third_party/intrinsic_moveit/README.md`](../third_party/intrinsic_moveit/README.md).

> [!NOTE]
> Nothing in `//:omts_solution` deploys them. The tools do nothing until the
> [intrinsic-moveit](https://github.com/intrinsic-ai/intrinsic-moveit)
> integration has been completed against your solution.

---

## 3. Common Troubleshooting & Gotchas

| Error Code / Message | Root Cause | Resolution |
| :--- | :--- | :--- |
| `ai.intrinsic.move_robot:10301: IK solver couldn't find any solutions` | 1. **Camera unattached:** `orbbec_camera` defaulted to root `[0,0,0]`, causing raw optical detections to place `pre_grasp` behind the robot base (~`[-0.058, 0.009, 0.392]`).<br>2. Tool orientation misaligned or target outside reachable envelope. | 1. Run `bazel run //tools/world:apply_scene_updates` to reattach `orbbec_camera` to `ur_module/flange`.<br>2. Run `bazel run //tools/world:inspect_world` to verify target $(x, y, z)$. |
| `ai.intrinsic.move_to_contact:10301: Stabilize action timed out without making contact` | 1. Contact force threshold set too high for light contact.<br>2. Tool motion direction pointing away from surface. | 1. Use `contact_force_newtons=10.0` or `15.0`.<br>2. Ensure `FixedVector.direction = (0.0, 0.0, 1.0)` in moving tool frame (+Z tool points toward table/workpiece). |
| `ai.intrinsic.executive:13001: Execution failed (PROTECTIVE_STOP)` | UR arm or ICON entered protective stop due to torque/wrench limits or abrupt contact. | 1. Clear faults on the UR teach pendant.<br>2. Use compliant `move_to_contact` instead of rigid Cartesian moves when touching surfaces.<br>3. Verify workpiece grasp orientation uses symmetric candidate selection to prevent wrist joint 6 wrap. |
| `ai.intrinsic.move_robot:10601: Frame with name ... does not exist on object` | Attempted to target an object reference instead of a valid frame in `PoseEquality`. | Ensure `target_frame` references a valid frame (`FrameReferenceByName(object_name="root", frame_name="pre_grasp")`). Do not target raw objects. |
| `ai.intrinsic.create_object:3: New object name cannot be used here` | Object with the specified name already exists in the belief world. | Issue a `DeleteObjectRequest` via `world.batch_update(...)` or use `apply_scene_updates`. |
| `ModuleNotFoundError: No module named 'intrinsic'` | Python command run directly via system Python instead of Bazel runfiles. | Always execute scripts via `bazel run //path:target` to resolve dependencies within the hermetic sandbox. |

