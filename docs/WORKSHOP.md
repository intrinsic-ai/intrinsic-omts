# Intrinsic Core & OMTS Hands-On Workshop (90 Minutes)

This workbook guides you through running, inspecting, and modifying the **Open Machine Tending Solution (OMTS)** built on **Intrinsic Core** in simulation.

No AI coding assistants are used in this workshop. Every step is designed to show *why* each robotics primitive exists and *how* the platform components fit together.

## What You Will Learn

1. **Platform runtime**: How a solution deploys onto local Kubernetes (K3s) as containerized **Services** and **Skills**.
2. **Digital Twin (`ObjectWorld`)**: How coordinate frames, kinematic trees, and collision models connect simulation (Gazebo) and visualization (RViz).
3. **Motion planning & control**: Joint jogging via ICON, Cartesian moves (`ANY` vs. `LINEAR`), and automatic collision checking.
4. **Application customization**: Teaching frames interactively in RViz, tuning YAML cell parameters, and modifying an SBL **Behavior Tree** in Python.

## Ground Rules

- Open all commands in `~/intrinsic-omts` as `intrinsic_core_user`.
- Always pass `-c opt` to `bazel run` and `bazel test` so Bazel reuses the prebuilt cache on your VM.
- Keep **two terminal windows** open:
  - **Terminal 1**: Dedicated to RViz (and later the interactive frame editor).
  - **Terminal 2**: Running CLI tools and `omts_app`.
- Read every **Predict -> Run -> Observe** prompt before pressing Enter.

---

## Lab 0 (0:00–0:20): Deploy, Launch RViz & Set Up Perception

### 0.1. Deploy the OMTS Workcell in Simulation

In **Terminal 2**, deploy `//:omts_solution` in simulation mode (`--operation_mode=sim`):

```bash
cd ~/intrinsic-omts
bazel run -c opt //:omts_solution -- \
  --address=localhost:17080 \
  --operation_mode=sim
```

> **What is happening under the hood?**
> Intrinsic Core runs on a local K3s Kubernetes cluster. Deploying a solution installs its **Assets** (robot UR5e, Robotiq Hand-E gripper, CNC enclosure, Schunk vise, Orbbec camera, workpiece, skills, and services) and starts the long-running **Services** (Gazebo simulator, ICON real-time controller, ROS bridge, and NVIDIA pose estimator) in the `app-resources` namespace.

### 0.2. Launch RViz (The Digital Twin Viewer)

In **Terminal 1**, start RViz with the preconfigured OMTS display profile:

```bash
choom -n 1000 ros2 run rviz2 rviz2 -- -d ~/intrinsic-omts/configs/omts/omts.rviz
```

You should see the UR5e robot arm, the infeed table with the raw stock block (`raw_stock_2x3x5`), and the CNC enclosure with the Schunk vise inside.

- **Left-drag**: Orbit camera
- **Middle-drag** (or Shift + Left-drag): Pan
- **Scroll**: Zoom

### 0.3. Install Perception Model `DataAssets` & Recreate Pose Estimator Service

Once `//:omts_solution` is running, install the three prebuilt perception model
`DataAssets` onto the deployed solution and recreate
`nvidia_pose_estimator_service` so it binds them.

> **Important:** This step must run **after** deploying `//:omts_solution` (it
> fails before a solution is deployed) and must be repeated any time you stop
> and redeploy the solution (`inctl solution stop`).

```bash
cd ~/intrinsic-omts
inctl asset install --address=localhost:17080 \
  bazel-bin/external/intrinsic-core+/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/foundationpose_model_data_asset.bundle.tar
inctl asset install --address=localhost:17080 \
  bazel-bin/external/intrinsic-core+/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/foundationstereo_model_data_asset.bundle.tar
inctl asset install --address=localhost:17080 \
  bazel-bin/external/intrinsic-core+/intrinsic_perception/intrinsic/perception/service/nvidia_pose_estimator/sam3_model_data_asset.bundle.tar

inctl service delete --address=localhost:17080 nvidia_pose_estimator_service || true
inctl service add --address=localhost:17080 ai.intrinsic.nvidia_pose_estimator_service
```

### 0.4. Inspect the Running K3s Cluster & Assets

In **Terminal 2**, verify the pods and installed assets:

```bash
kubectl get pods -n app-resources
inctl asset list --address=localhost:17080 | head -n 20
```

Notice the distinction:
- **Services** (in `kubectl get pods -n app-resources`) are long-running processes: `rs-gazebo-simulator-0`, `rs-flowstate-ros-bridge-*`, `rs-nvidia-pose-estimator-service-*`, and ICON.
- **Skills** (e.g., `ai.intrinsic.move_robot`, `ai.intrinsic.move_to_contact`, `ai.intrinsic.estimate_pose_multi_view`) are stateless, composable actions invoked by the **Executive** when running a **Behavior Tree**.

### 0.5. Register the Workpiece Pose Estimator

Register the text-prompted 6-DoF pose estimator for the raw stock block (if you later need to change thresholds or re-register under the same name, uninstall the existing asset first — see Troubleshooting below):

```bash
bazel run -c opt @intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_train_service/tools:register_using_ioc_train_service -- \
  --address="localhost:17080" \
  --scene_object_id="ai.intrinsic.raw_stock_2x3x5" \
  --pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator_text" \
  --refinement_iters=5 \
  --confidence_threshold=0.05 \
  --visibility_threshold=0.05 \
  --text_prompt="cuboid"
```

> **Note**: Wait **1–2 minutes** after recreating `nvidia_pose_estimator_service` and registering the pose estimator for the model weights to finish loading on the GPU. Running `run_pose_estimation` before initialization completes will return a service-unavailable error.

*(Optional)* Verify pose detection directly against the live simulated camera feed and watch the **`/detections`** image panel in RViz:

```bash
bazel run -c opt @intrinsic-core//intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:run_pose_estimation -- \
  --address="localhost:17080" \
  --service_name="nvidia_pose_estimator_service" \
  --camera_name="orbbec_camera" \
  --pose_estimator_id="ai.intrinsic.raw_stock_2x3x5_estimator_text"
```

We will run the full `omts_app` cycle right after Lab 2, once we have explored the Digital Twin, jogging, and collision-free motion planning.

---

## Lab 1 (0:15–0:35): Digital Twin, Frames & Manual Control

### Concept: Belief World vs. Simulation World

- **Belief World (`world`, shown in RViz)**: The software's internal kinematic tree and geometry model (`ObjectWorld`). Motion planning, collision checking, and coordinate frame math all operate on the Belief World.
- **Simulation World (`sim_world`, Gazebo)**: The physics engine simulating gravity, contact forces, and camera renders. On a real workcell, the physical hardware replaces Gazebo, while the Belief World stays identical.

### 1.1. Inspect the Live Digital Twin

Run `inspect_world` to list all scene objects, joint angles, resources, and named coordinate frames:

```bash
bazel run -c opt //tools/world:inspect_world -- --address=localhost:17080
```

Find these frames under `root` in the output:
- `view`: Camera observation pose above the infeed table.
- `transit`: Safe mid-cell clearance waypoint.
- `machine_approach`: Safe standby pose just outside the CNC door.
- `vise_pre_place` / `vise_place`: Approach and seating poses inside the CNC vise.
- `pre_grasp` / `grasp`: Updated *dynamically* at runtime by perception.

### 1.2. Jog the Robot in Joint Space

Launch the interactive keyboard jogger (talks directly to the ICON real-time controller):

```bash
bazel run -c opt //tools/jogging:jog_interactive -- \
  --instance=icon \
  --host=localhost \
  --port=17080
```

Follow the on-screen keys to move individual joints (e.g., base joint 1 and shoulder/elbow joints) and watch RViz update live. Press `q` to quit when done.

> **Question to consider**: When jogging joint 1, the robot tool moves in a curved arc rather than a straight line. Why? Because you are commanding **joint space** (motor angles), not **Cartesian space** (`x, y, z` tool coordinates).

### 1.3. Actuate the Gripper and CNC Machine Independently

Try commanding individual hardware adapters directly:

```bash
# Open and close the Robotiq Hand-E gripper:
bazel run -c opt //tools/gripper:control_gripper -- --address=localhost:17080 --action=close
bazel run -c opt //tools/gripper:control_gripper -- --address=localhost:17080 --action=open

# Actuate the CNC door and Schunk vise (watch them move in RViz!):
bazel run -c opt //tools/machine:control_machine -- --address=localhost:17080 --action=close_door
bazel run -c opt //tools/machine:control_machine -- --address=localhost:17080 --action=open_door
```

---

## Lab 2 (0:35–0:50): Motion Planning & Collision Avoidance

Instead of manually computing joint angles, Intrinsic Core's `move_robot` skill takes a target **coordinate frame** and solves inverse kinematics + collision-free path planning against the Digital Twin.

### 2.1. Move Between Named Frames

Command the robot TCP (`gripper/tool_frame`) to `view` and then `machine_approach`:

```bash
bazel run -c opt //tools/jogging:move_to_frame -- \
  --address=localhost:17080 \
  --frame=view \
  --motion_type=ANY

bazel run -c opt //tools/jogging:move_to_frame -- \
  --address=localhost:17080 \
  --frame=machine_approach \
  --motion_type=ANY
```

### 2.2. Predict -> Run -> Observe: Collision Checking Against a Closed Door

1. First, close the CNC enclosure door:
   ```bash
   bazel run -c opt //tools/machine:control_machine -- \
     --address=localhost:17080 \
     --action=close_door
   ```
2. **Predict**: You are about to command the robot to move to `vise_pre_place` (which is *inside* the CNC enclosure) while the door is closed. Will the robot crash into the door, or will the motion planner refuse to execute?
3. **Run**:
   ```bash
   bazel run -c opt //tools/jogging:move_to_frame -- \
     --address=localhost:17080 \
     --frame=vise_pre_place \
     --motion_type=ANY
   ```
4. **Observe**: Read the terminal error. The motion planner rejects the request *before the robot moves* because `cnc_enclosure`'s door joint is closed in the Belief World.
5. **Fix**: Open the door and repeat the exact same command:
   ```bash
   bazel run -c opt //tools/machine:control_machine -- \
     --address=localhost:17080 \
     --action=open_door

   bazel run -c opt //tools/jogging:move_to_frame -- \
     --address=localhost:17080 \
     --frame=vise_pre_place \
     --motion_type=ANY
   ```

### 2.3. Compare `LINEAR` vs. `ANY` Motion Types

Exit the CNC machine in a straight line (`LINEAR`), then transit back across the cell with free joint-space planning (`ANY`):

```bash
bazel run -c opt //tools/jogging:move_to_frame -- \
  --address=localhost:17080 \
  --frame=machine_approach \
  --motion_type=LINEAR

bazel run -c opt //tools/jogging:move_to_frame -- \
  --address=localhost:17080 \
  --frame=view \
  --motion_type=ANY
```

> **Why both exist**: Use `LINEAR` when inserting/extracting parts in tight clearances (like the CNC vise) so the tool sweeps a predictable straight line. Use `ANY` for long cross-cell transits where the planner should pick the fastest collision-free joint trajectory.

---

## OMTS in Action: Run Your First Full Cycle

Now that you have seen K3s assets, 6-DoF pose estimation, Behavior Trees, the Digital Twin, jogging, and collision-free motion planning, run the full `omts_app` Behavior Tree for one cycle and watch RViz (including the `/detections` image panel):

```bash
bazel run -c opt //src:omts_app -- \
  --address=localhost:17080 \
  --config="configs/omts/app_config.yaml" \
  --num_cycles=1
```

Watch the 5 phases execute in sequence:
1. **Pick**: Open CNC door/vise -> move to `view` -> 6-DoF pose estimation -> approach & compliant touchdown -> grasp & attach block.
2. **Load**: Move through `transit` -> `machine_approach` -> `vise_pre_place` -> seat into vise -> clamp vise & release.
3. **Machining**: Retract to `machine_approach` -> close CNC door -> wait for simulated machining cycle.
4. **Unload**: Open CNC door/vise -> grasp machined block -> lift & exit CNC enclosure.
5. **Return**: Place block back on infeed table at a randomized `(x, y, rz)` offset -> release & return to `view`.

---

## Lab 3 (0:50–1:20): Customize OMTS (Frames -> Config -> Behavior Tree)

You will now modify the OMTS application across three layers without touching any low-level controller or driver code:

1. **Scene layer** (`configs/omts/scene.updates.pbtxt`): Move & teach coordinate frames.
2. **Config layer** (`configs/omts/app_config.yaml`): Tune cell cycle parameters.
3. **Orchestration layer** (`src/behaviors/machine_tending_bt.py`): Add a new Behavior Tree step.

### 3.1. Interactive Frame Editing in RViz & Teaching a New Frame (10 min)

#### Part A: Drag an Existing Frame in RViz

Start the 6-DoF interactive frame editor for `view` and `transit`:

```bash
bazel run -c opt //tools/world:edit_frames_rviz -- \
  --address=localhost:17080 \
  --zenoh_endpoint=tcp/127.0.0.1:17447 \
  --frame view transit
```

In RViz:
1. Click **Interact** in the top toolbar (next to *Move Camera*).
2. Drag the red/green/blue arrows on the `transit` marker by ~5–10 cm (keep it clear of the CNC enclosure and table).
3. Release the mouse button — `edit_frames_rviz` updates the live Belief World *and* writes the new coordinates to `configs/omts/scene.updates.pbtxt`.
4. Press `Ctrl+C` in Terminal 2 to stop the frame editor, then inspect your change:
   ```bash
   git diff configs/omts/scene.updates.pbtxt
   ```
5. Switch RViz back to **Move Camera** and verify the robot can reach your updated `transit` frame:
   ```bash
   bazel run -c opt //tools/jogging:move_to_frame -- \
     --address=localhost:17080 \
     --frame=transit \
     --motion_type=ANY
   ```

#### Part B: Teach a New `inspect` Frame

You will teach a new frame named `inspect` where the robot will hold the finished part for a visual check after unloading the CNC machine.

1. Move to `work_view` (or jog to your own safe open-air pose above the cell):
   ```bash
   bazel run -c opt //tools/jogging:move_to_frame -- \
     --address=localhost:17080 \
     --frame=work_view \
     --motion_type=ANY
   ```
   *(Optional)* Run `jog_interactive` for 10 seconds to tilt the wrist so the held part faces you in RViz.
2. Save the current robot TCP pose as a new named frame `inspect`:
   ```bash
   bazel run -c opt //tools/jogging:store_frame -- inspect --address=localhost:17080
   ```
3. Push the updated `.pbtxt` frames into the live Belief World (no Gazebo reset needed for frames):
   ```bash
   bazel run -c opt //tools/world:apply_scene_updates -- \
     --address=localhost:17080 \
     --no-reset_sim
   ```
4. Verify `inspect` now exists in the Digital Twin:
   ```bash
   bazel run -c opt //tools/world:inspect_world -- --address=localhost:17080 | grep inspect
   ```

### 3.2. Tune Cell Behavior via `app_config.yaml` (8 min)

Open `configs/omts/app_config.yaml` in your editor (e.g., `mousepad configs/omts/app_config.yaml &` or `nano configs/omts/app_config.yaml`).

Locate the `cycle:` section near the bottom (lines 67–84) and modify **two parameters** of your choice within the safe bounds below:

| Parameter | Default | Safe Workshop Range | Effect |
| :--- | :--- | :--- | :--- |
| `machining_timeout_seconds` | `2.0` | `1.0` – `6.0` | How long the CNC stays closed during simulated machining |
| `return_shift.center_x` | `0.45` | `0.38` – `0.48` | Table X center (meters) where the finished part is placed |
| `return_shift.center_y` | `0.0` | `-0.10` – `0.10` | Table Y center (meters) where the finished part is placed |
| `return_shift.bounds_rz_degrees` | `40.0` | `0.0` – `60.0` | Random yaw rotation range on return (`0.0` = no rotation) |

**Predict -> Run -> Observe**:
- Before running, predict where on the infeed table the part will be placed and how long the CNC door will remain closed.
- Run one cycle to verify your YAML change took effect without rebuilding C++ or redeploying the solution:
  ```bash
  bazel run -c opt //src:omts_app -- \
    --address=localhost:17080 \
    --config="configs/omts/app_config.yaml" \
    --num_cycles=1
  ```

### 3.3. Modify the Behavior Tree in Python (12 min)

Now wire your new `inspect` frame into the OMTS master Behavior Tree so the robot presents the finished workpiece for inspection right after unloading the CNC machine and before placing it back on the table.

Open `src/behaviors/machine_tending_bt.py` (`mousepad src/behaviors/machine_tending_bt.py &` or `nano src/behaviors/machine_tending_bt.py`).

1. **Add the import** near the top (around line 20):
   ```python
   from src.behaviors.motions import create_move_to_frame_task
   ```
2. **Create the inspection task** right after `unload_subtree = ...` (around line 108). Fill in the `TODO` placeholders with your chosen frame name (`"inspect"`), parent object (`config.frames.parent_object`), motion type (`"ANY"`), and a descriptive task name:
   ```python
   inspect_task = create_move_to_frame_task(
     robot=robot,
     frame_name=___,  # TODO: your newly taught frame name
     parent_object=___,  # TODO: parent object from config.frames
     motion_type=___,  # TODO: "ANY" or "LINEAR" — which is better here?
     task_name=___,  # TODO: descriptive node name for logs/traces
   )
   ```
3. **Insert `inspect_task` into `cycle_sequence`** (lines 115–124):
   - **Predict**: Between which two subtrees should `inspect_task` be placed so the robot is *holding the finished part* outside the CNC machine when it moves to `inspect`?
   - Insert `inspect_task` at that exact position in `children=[...]`.
4. **Run the updated application** and watch RViz:
   ```bash
   bazel run -c opt //src:omts_app -- \
     --address=localhost:17080 \
     --config="configs/omts/app_config.yaml" \
     --num_cycles=1
   ```
   Notice how fast the iteration loop is: editing Python Behavior Trees requires **zero** container rebuilds or cluster redeployment.

---

## Optional Stretch Challenges (For Faster Finishers)

Pick any of these if you finish Lab 3 early:

1. **Run the hermetic unit tests**:
   ```bash
   bazel test -c opt //tests/...
   ```
   Notice that `//tests/unit:test_behaviors` validates your Behavior Tree contract (and will flag if your new task changes expected structure or has invalid parameters) in under 10 seconds with no running cluster. Inspect `tests/unit/test_behaviors.py` to see how `solution_manifest_only` enables offline testing.
2. **Compare `reality` vs. `fast_preview` execution modes**:
   Run `omts_app` with `--simulation_mode=fast_preview` and observe what happens in RViz vs. Gazebo. Why is `fast_preview` useful when prototyping trajectories?
   *(Note: perception requires camera frames from Gazebo in `reality` mode; observe how `fast_preview` behaves with planning vs. sensor skills.)*
3. **Add a second inspection waypoint or dwell**:
   Wrap your `inspect_task` and a second frame move (e.g., `transit`) inside a dedicated `bt.Sequence(name="4b. Quality Inspection Subtree", children=[...])` in `src/behaviors/machine_tending_bt.py`.

---

## Troubleshooting & Reset Commands

If your robot or scene gets into an unexpected state, run:

```bash
# If the Digital Twin (ObjectWorld) is out of sync or broken, reset the world:
inctl world reset --address=localhost:17080

# Re-apply scene updates and reset Gazebo simulation back to clean initial state:
bazel run -c opt //tools/world:apply_scene_updates -- \
  --address=localhost:17080 \
  --reset_sim

# If RViz shows "No tf data" or missing meshes, restart the ROS bridge pod:
kubectl delete pod -n app-resources -l app=flowstate-ros-bridge || \
  kubectl get pods -n app-resources | awk '/ros-bridge/{print $1}' | xargs -r kubectl delete pod -n app-resources

# If the pose estimator was installed incorrectly or you want to change its thresholds
# (must uninstall before re-registering under the same asset name):
inctl asset uninstall ai.intrinsic.raw_stock_2x3x5_estimator_text --address=localhost:17080

# If everything is broken, stop the solution and start fresh from Lab 0.1 & 0.3:
inctl solution stop --address=localhost:17080

# Revert your local code/config edits back to clean git state:
git checkout -- configs/omts/app_config.yaml configs/omts/scene.updates.pbtxt src/behaviors/machine_tending_bt.py
```

## Continue Exploring Open Source

- **OMTS Repository**: <https://github.com/intrinsic-ai/intrinsic-omts>
- **Intrinsic Core Repository**: <https://github.com/intrinsic-ai/intrinsic-core>
- **Intrinsic Core Tutorials**: <https://developer.intrinsic.ai/docs/learn/tutorials/>
- **Architecture Deep Dive**: [`docs/ARCHITECTURE.md`](ARCHITECTURE.md)
- **Developer CLI Reference**: [`tools/README.md`](../tools/README.md)
- **Developer Community**: <https://developer.intrinsic.ai/>
