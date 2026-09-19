# Utilities & Runtime Helpers (`src/utils/`)

Shared helper modules for dynamic in-tree script injection, 6D grasp pose
synthesis, kinematic/world queries, and executive mode mapping.

## Modules

| Module | Key Functions | Description |
| :--- | :--- | :--- |
| [`dynamic_frame_calculator.py`](dynamic_frame_calculator.py) | `calculate_and_update_dynamic_frames` | Executed inside `bt.PythonScript` during perception. Transforms camera-frame detections into `root`, enforces `min_safe_z`, aligns grasp yaw with the workpiece short side, selects the closest symmetric quaternion in SO(3) to prevent wrist wrapping, and updates `pre_grasp` and `grasp` frames in `ObjectWorld`. |
| [`script_utils.py`](script_utils.py) | `load_python_script`, `create_dwell_task` | Extracts source code from typed, linted Python modules/functions (resolving direct paths and Bazel runfiles) for injection into `bt.PythonScript(function_body=...)`, plus timed dwell task generation. |
| [`math_utils.py`](math_utils.py) | `normalize_angle`, `normalize_joint_angles`, `normalize_motion_types`, `describe_motion_types`, `object_exists_in_world`, `resolve_adio_resource`, `create_transform_node_ref` | Angle normalization, trajectory segment expansion, safe world object existence checks, capability-filtered ADIO resource lookup (`Icon2AdioPart`), and `TransformNodeReference` proto construction. |
| [`execution_utils.py`](execution_utils.py) | `to_executive_simulation_mode` | Maps `src.core.types.SimulationMode` (`REALITY`, `PREVIEW`, `FAST_PREVIEW`) to `intrinsic.solutions.execution.Executive.SimulationMode`. |
