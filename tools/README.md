# Developer & Operational CLI Tools (`tools/`)

Standalone Bazel CLI utilities for cell commissioning, live world inspection,
teleoperation, pose teaching, hardware I/O testing, and code formatting/linting.

## Tool Subpackages

| Directory / Script | Bazel Target / Command | Purpose |
| :--- | :--- | :--- |
| [`world/`](world/README.md) | `//tools/world:apply_scene_updates`<br>`//tools/world:inspect_world` | Apply `.pbtxt` world updates live (with optional Gazebo `--reset_sim`) and inspect kinematic trees/transforms. |
| [`jogging/`](jogging/README.md) | `//tools/jogging:jog_interactive`<br>`//tools/jogging:move_to_frame`<br>`//tools/jogging:move_to_joint`<br>`//tools/jogging:store_frame`<br>`//tools/jogging:store_joint_config` | Interactive robot teleoperation, Cartesian/joint moves to named targets, and teaching/persisting frames into `scene.updates.pbtxt`. |
| [`gripper/`](gripper/README.md) | `//tools/gripper:control_gripper` | Open/close Robotiq or DIO grippers interactively or via CLI flags (`--action=open\|close`). |
| [`machine/`](machine/README.md) | `//tools/machine:control_machine` | Actuate CNC enclosure doors, pneumatic vises, and cycle handshake pins (`open_door`, `close_door`, `open_vise`, `close_vise`, `trigger_cycle`, `wait_cycle`). |
| [`calibration/`](calibration/README.md) | `//tools/calibration:sample_calibration_poses`<br>`//tools/calibration:calibrate_camera_to_robot`<br>`//tools/calibration:update_robot_kinematics` | Hand-eye ChArUco camera calibration pose sampling, calibration execution, and UR kinematics update helpers. |
| [`pose_estimation/`](pose_estimation/README.md) | `//tools/pose_estimation:register_using_train_service`<br>`//tools/pose_estimation:run_pose_estimation` | Register CAD scene objects with FoundationPose (`train_service`) and run standalone multi-view 6D pose estimation. |
| [`format.sh`](format.sh) | `./tools/format.sh` | Formats all Bazel (`buildifier`) and Python/Markdown (`ruff`) files in the repository. |
| [`lint.sh`](lint.sh) | `./tools/lint.sh` | Runs the exact CI lint and format checks (`buildifier --mode=check`, `ruff check`, `ruff format --check`). |
