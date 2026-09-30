# Hardware Abstraction Layer (`src/hardware/`)

Stateless hardware adapters that translate high-level robotic manipulation,
gripping, machine tool I/O, and 3D perception actions into Intrinsic SBL skill
nodes (`bt.Task` / `bt.Sequence`).

## Modules & Adapters

| Module | Abstract Interface | Concrete Implementation(s) | SBL Skills Wrapped |
| :--- | :--- | :--- | :--- |
| [`robot.py`](robot.py) | — | `Robot` | `move_robot` (`JOINT`, `LINEAR`, `ANY`, `RelativePoseEquality`, `RotationCone`), `move_to_contact`, `attach_object_to_robot`, `detach_object` |
| [`gripper.py`](gripper.py) | `GripperInterface` | `RobotiqGripper`, `DioGripper` | `gripper_cmd_skill` (metric finger joint position), `dio_set_output` (solenoid/relay pins) |
| [`machine.py`](machine.py) | `CncMachineInterface` | `DioCncMachine` | `dio_set_output`, `dio_wait_for_input` / `dio_read_input`, `update_world` (belief-world door/vise joint synchronization) |
| [`vision.py`](vision.py) | `VisionInterface` | `OrbbecVision` | `capture_images`, `estimate_pose_multi_view` (FoundationPose), `bt.PythonScript` dynamic frame calculation wrapped in `bt.Retry` |
| [`grasping.py`](grasping.py) | `GraspPlannerInterface` | `MoveItGraspPlanner` ([`third_party/intrinsic_moveit/`](../../third_party/intrinsic_moveit/README.md)) | Contract only; an implementation contributes the task that writes planned `grasp` / `pre_grasp` frames. |
| [`grasp_planners.py`](grasp_planners.py) | — | `create_grasp_planner` factory | Maps a `GraspPlannerType` to a planner instance: `None` for `cuboid_center` (the perception pipeline emits that grasp itself), `MoveItGraspPlanner` for `moveit`. |

## Adapter Design Rules

* **No Robot Interface**: `move_robot`, `move_to_contact`,
  `attach_object_to_robot`, and `detach_object` are vendor-neutral and bind to
  any ICON-controlled arm, so a single `Robot` class serves UR, KUKA, FANUC,
  etc. Robot-specific details come from the world (kinematics, named joint
  configurations) and `RobotConfig` (arm part and tool frame names).
* **Scoped Configuration Dataclasses**: Every adapter constructor accepts the
  connected `solution` handle and its scoped configuration dataclass
  (`RobotConfig`, `GripperConfig`, `MachineConfig`, `VisionConfig`).
* **Capability-Filtered ADIO Binding**: `DioGripper` and `DioCncMachine` resolve
  optional ADIO device handles via `resolve_adio_resource()` so skills only bind
  explicit resources advertising `Icon2AdioPart`.
* **Belief-World Joint Synchronization**: When `DioCncMachine` commands door or
  vise digital outputs, it appends an `update_world` task updating the
  corresponding scene object joint positions (`cnc_enclosure`,
  `schunk_egp_64nnb`) so collision checking reflects the physical state.
* **One-Way Grasp Planner Dependencies**: `grasping.py` holds only the
  `GraspPlannerInterface` contract and never imports a planner, so an
  out-of-tree integration can implement it without OMTS depending back on that
  integration. `grasp_planners.py` is the only first-party module that depends
  on `third_party/intrinsic_moveit` (enforced by that package's Bazel
  visibility), which is why `src/hardware/__init__.py` re-exports the interface
  but not the factory: importing `src.hardware` never pulls in MoveIt.
* **Single Owner of the Grasp Frames**: `OrbbecVision.build_perception_and_spawn_task`
  takes `publish_grasp_frames`. When a grasp planner is selected it is `False`,
  so the perception script still updates the workpiece pose and enforces
  `min_safe_z` but leaves `grasp` / `pre_grasp` to the planner.
