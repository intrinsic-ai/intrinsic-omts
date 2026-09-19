# Core Domain Models & Configuration (`src/core/`)

SDK-independent domain representations, infeed acquisition strategies, and
strict YAML configuration dataclasses for OMTS workcells.

## Modules

| Module | Key Symbols | Description |
| :--- | :--- | :--- |
| [`config.py`](config.py) | `AppConfig`, `RobotConfig`, `GripperConfig`, `MachineConfig`, `VisionConfig`, `FramesConfig`, `CycleConfig`, `load_app_config` | Frozen configuration dataclasses and fail-loud YAML loader. Validates all required cell parameters before connecting to hardware. |
| [`infeed.py`](infeed.py) | `InfeedStrategy`, `PerceptionInfeedStrategy`, `GridInfeedStrategy` | Strategy pattern encapsulating part localization via 3D vision pose estimation (`PERCEPTION`) or deterministic row/column indexing (`GRID`). |
| [`tray.py`](tray.py) | `Tray`, `TraySlot` | $N \times M$ pallet grid model with row-major slot lookup and metric pitch coordinate calculation (`get_slot_relative_pose`). |
| [`workpiece.py`](workpiece.py) | `Workpiece` | Lifecycle tracking (`RAW` $\to$ `DETECTED` $\to$ `IN_TRANSIT` $\to$ `IN_MACHINE` $\to$ `MACHINED` $\to$ `INSPECTED_OK` / `REJECTED`) and grasp metadata for individual parts. |
| [`workcell.py`](workcell.py) | `WorkcellState` | Cell-level runtime state tracker (active workpiece, fixture/door state, completed/failed cycle counters, and cycle duration timing). |
| [`types.py`](types.py) | `PartState`, `SlotState`, `InfeedMode`, `SimulationMode`, `FixtureState`, `MachineDoorState`, `Pose3D`, `JointPosition` | Core enumerations and immutable geometric primitives (`Pose3D`, `JointPosition`). |

## Design Invariants

* **Zero Intrinsic SDK Dependencies**: `src/core/` does not import
  `intrinsic.*` protos or gRPC stubs, keeping domain logic fast and testable
  in isolation.
* **Strict Configuration Validation**: `load_app_config()` raises `KeyError`
  immediately if any required field is omitted from `app_config.yaml`. Optional
  subsystems (such as `machine: MachineConfig | None`) are explicitly typed.
