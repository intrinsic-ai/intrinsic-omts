# Workcell Configurations (`configs/`)

Per-cell application YAML configurations, SBL `ObjectWorldUpdates` (`.pbtxt`),
and hardware/service configuration manifests (`.textproto`).

## Directory Structure

| Directory | Cell / Scope | Description |
| :--- | :--- | :--- |
| [`common/`](common/) | Shared across cells | Asset manifests and service configs shared by all cells (`flowstate_ros_bridge`, `hande_gripper_service`, `orbbec_gemini_driver`, `pose_estimator_config`). |
| [`omts/`](omts/) | Production OMTS Cell | UR5e arm, Robotiq Hand-E gripper, Orbbec Gemini 335Le camera, CNC enclosure, and Schunk EGP 64 pneumatic vise. Selected by default (`--//:setup=omts`). |
| [`lab_bb_01/`](lab_bb_01/) | Lab BB-01 Cell | UR3e arm, Robotiq Hand-E gripper, Orbbec Gemini 335Le camera, CAW enclosure, without CNC machine/vise hardware. Selected via `--//:setup=lab_bb_01`. |
| [`kr_10/`](kr_10/) | KUKA KR10 (Stub) | Placeholder ICON config for KUKA KR10; not wired into `//:omts_solution`. |

## Configuration File Types

* **`app_config.yaml`**: Runtime application configuration parsed by
  [`src/core/config.py:load_app_config`](../src/core/config.py). Defines robot
  part/frame names, gripper parameters, optional CNC machine DIO pin/joint
  mappings, vision estimator settings, scene frame names, and cycle forces/timeouts.
  The optional `grasp:` section names the grasp planning backend; it currently
  accepts only `cuboid_center`, which is also the default when omitted.
* **`*.updates.pbtxt`**: `ObjectWorldUpdates` protos defining kinematic tree
  attachments (e.g. mounting `gripper` and `orbbec_camera` to `ur_module/flange`),
  robot base alignment, fixture poses, named scene frames (`view`,
  `pre_grasp`, `grasp`, `machine_approach`, `vise_pre_place`, `vise_place`),
  and cell-specific joint application limits
  (`omts/ur_module.limits.updates.pbtxt`).
  Can be applied live via `bazel run //tools/world:apply_scene_updates`.
* **`*.textproto`**: Service configurations for ICON realtime control
  (`icon_config.textproto`), UR hardware module (`ur_module_config.textproto`),
  and Orbbec camera driver (`gemini_device_config.textproto`).
