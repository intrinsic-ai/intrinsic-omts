# Camera & Robot Calibration Tools (`tools/calibration/`)

CLI utilities for ChArUco hand-eye camera calibration pose sampling, camera-to-robot
extrinsic calibration, and synchronizing physical robot kinematics with `ObjectWorld`.

## CLI Targets

| Target | Source File | Description |
| :--- | :--- | :--- |
| `//tools/calibration:sample_calibration_poses` | [`sample_calibration_poses.py`](sample_calibration_poses.py) | Samples collision-free robot poses viewing the ChArUco board and exports waypoints to `.pbtxt`. |
| `//tools/calibration:calibrate_camera_to_robot` | [`calibrate_camera_to_robot.py`](calibrate_camera_to_robot.py) | Executes hand-eye calibration across sampled waypoints, validates RMS error thresholds, updates `ObjectWorld`, and writes `orbbec_gemini.updates.pbtxt`. |
| `//tools/calibration:update_robot_kinematics` | [`update_robot_kinematics.py`](update_robot_kinematics.py) | Calls `RobotUpdateService` to check or synchronize physical UR factory kinematics into `solution.world`. |

## Usage Examples

```bash
# Sample hand-eye calibration poses around the ChArUco target:
bazel run //tools/calibration:sample_calibration_poses -- \
  --address=localhost:17080 \
  --camera=orbbec_camera

# Run hand-eye camera calibration and export updated extrinsics:
bazel run //tools/calibration:calibrate_camera_to_robot -- \
  --address=localhost:17080 \
  --camera=orbbec_camera \
  --output_updates_file=configs/omts/orbbec_gemini.updates.pbtxt

# Synchronize physical robot kinematic calibration with ObjectWorld:
bazel run //tools/calibration:update_robot_kinematics -- \
  --address=localhost:17080 \
  --resource_id=ur_module
```
