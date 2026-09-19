# Test Suite (`tests/`)

Hermetic unit test suites verifying OMTS domain logic, YAML configuration
validation, SBL Behavior Tree structure, hardware adapters, and CLI tools.

## Running Tests

```bash
# Run all 18 unit test targets offline (no live cluster or hardware required)
bazel test //tests/...

# Run a specific test suite with detailed error output
bazel test --test_output=errors //tests/unit:test_behaviors
```

## Unit Test Coverage (`tests/unit/`)

| Test Target | Module Under Test | Key Verifications |
| :--- | :--- | :--- |
| `test_config` | `src/core/config.py` | Strict YAML loading (`omts`, `lab_bb_01`), missing key rejection, optional `machine` section handling. |
| `test_behaviors` | `src/behaviors/*.py` | Master BT assembly (`bt.Loop` single/finite/continuous), optional `machine=None` node omission, segment-scoped collision rules, compliant touchdowns, and linear retracts across all 5 subtrees. |
| `test_hardware_adapters` | `src/hardware/*.py` | `UrRobot`, `RobotiqGripper`, `DioGripper`, `DioCncMachine`, and `OrbbecVision` skill generation, ADIO capability resolution, and belief-world joint synchronization. |
| `test_infeed` | `src/core/infeed.py` | `PerceptionInfeedStrategy` and `GridInfeedStrategy` part acquisition lifecycle. |
| `test_tray` | `src/core/tray.py` | Row-major slot iteration, pitch offset math, and slot occupancy transitions. |
| `test_workpiece` | `src/core/workpiece.py` | Workpiece state machine transitions (`RAW` through `INSPECTED_OK`/`REJECTED`). |
| `test_script_utils` | `src/utils/script_utils.py`, `dynamic_frame_calculator.py` | Source code extraction for `bt.PythonScript`, short-side grasp alignment, geodesic quaternion selection, and `min_safe_z` enforcement. |
| `test_execution_utils` | `src/utils/execution_utils.py` | Mapping between `SimulationMode` and `Executive.SimulationMode`. |
| `test_world_configs` | `configs/**/*.pbtxt` | Protobuf syntax and schema validation for all cell `.pbtxt` world update files. |
| `test_apply_scene_updates` | `tools/world/apply_scene_updates.py` | Live `create_frame` $\to$ `update_transform` adaptation and simulator reset handling. |
| `test_control_gripper` | `tools/gripper/control_gripper.py` | CLI argument parsing and gripper open/close task dispatch. |
| `test_control_machine` | `tools/machine/control_machine.py` | CLI argument parsing and CNC door/vise/cycle action execution. |
| `test_move_to_frame` | `tools/jogging/move_to_frame.py` | Frame discovery and Cartesian move execution. |
| `test_move_to_joint` | `tools/jogging/move_to_joint.py` | Joint configuration selection and joint move execution. |
| `test_store_frame` | `tools/jogging/store_frame.py` | Capturing live tool transforms and updating `.pbtxt` frame definitions. |
| `test_calibrate_camera` | `tools/calibration/*.py` | Camera calibration skill parameter wiring. |
| `test_update_robot_kinematics` | `tools/calibration/update_robot_kinematics.py` | Robot kinematic calibration update utilities. |
| `test_register_using_train_service` | `tools/pose_estimation/register_using_train_service.py` | FoundationPose estimator registration parameters. |
