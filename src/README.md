# OMTS Application Package (`src/`)

Main Python package for the Open Machine Tending Solution (`//src:omts_app`).
Connects to a deployed Intrinsic Core solution over gRPC, loads a
typed cell YAML configuration, assembles a single Behavior Tree, and executes
the machine tending cycle via `solution.executive.run(tree)`.

## Package Structure

| Directory / File | Description |
| :--- | :--- |
| [`main.py`](main.py) | CLI entrypoint (`//src:omts_app`) and pipeline runner (`run_machine_tending_pipeline`). |
| [`core/`](core/) | Domain models (`Workpiece`, `Tray`, `WorkcellState`), enums (`SimulationMode`, `InfeedMode`), infeed strategies, and strict YAML configuration schema (`AppConfig`). |
| [`behaviors/`](behaviors/) | Composable Behavior Tree subtrees (`pick`, `load_machine`, `machining`, `unload_machine`, `return_infeed`), reusable motion primitives, and master cycle builder (`machine_tending_bt.py`). |
| [`hardware/`](hardware/) | Stateless hardware adapters wrapping SBL skills (`UrRobot`, `RobotiqGripper`, `DioGripper`, `DioCncMachine`, `OrbbecVision`), plus the `GraspPlannerInterface` extension point and its `create_grasp_planner` factory. |
| [`utils/`](utils/) | Runtime helpers: AST script loader (`load_python_script`), in-tree dynamic grasp frame calculator (`dynamic_frame_calculator.py`), kinematics/world utilities, and execution mode mapping. |
| [`foundationpose/`](foundationpose/) | Triton Inference Server model configuration (`config.pbtxt`) and `intrinsic_mlmodel` Bazel packaging (`//src/foundationpose:foundationpose_mlmodel`). |

## Running the Application

```bash
# Run with default OMTS cell configuration (UR5e + CNC enclosure + vise)
bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --config=configs/omts/app_config.yaml

# Run with Lab BB-01 cell configuration (UR3e, no CNC machine)
bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --config=configs/lab_bb_01/app_config.yaml

# Override cycle count or executive simulation mode
bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --config=configs/omts/app_config.yaml \
  --num_cycles=3 \
  --simulation_mode=fast_preview

# Override the grasp planner backend from the 'grasp' config section
bazel run //src:omts_app -- \
  --address=localhost:17080 \
  --config=configs/omts/app_config.yaml \
  --grasp_planner=cuboid_center
```
