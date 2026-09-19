# Robot Jogging & Pose Teaching Tools (`tools/jogging/`)

Interactive CLI utilities for keyboard joint teleoperation, Cartesian and joint
motion execution, and persisting taught frames/joint configurations to `.pbtxt`.

## CLI Targets

| Target | Source File | Description |
| :--- | :--- | :--- |
| `//tools/jogging:jog_interactive` | [`jog_interactive.py`](jog_interactive.py) | Real-time keyboard joint jogging via ICON (`joint_jogging` action with deadman timeout). |
| `//tools/jogging:move_to_frame` | [`move_to_frame.py`](move_to_frame.py) | Moves robot tool TCP (`gripper/tool_frame`) to a named scene frame using `ANY`, `LINEAR`, or `JOINT` interpolation. |
| `//tools/jogging:move_to_joint` | [`move_to_joint.py`](move_to_joint.py) | Moves robot arm to a stored joint configuration or explicit joint vector. |
| `//tools/jogging:store_frame` | [`store_frame.py`](store_frame.py) | Reads the live transform of `gripper/tool_frame` in `root` and writes/updates a named frame in `scene.updates.pbtxt`. |
| `//tools/jogging:store_joint_config` | [`store_joint_config.py`](store_joint_config.py) | Reads current joint angles from the robot arm and persists a named `JointConfiguration` to `.pbtxt`. |

## Usage Examples

```bash
# Keyboard joint teleoperation via ICON:
bazel run //tools/jogging:jog_interactive -- \
  --instance=icon \
  --host=localhost \
  --port=17080

# Move robot TCP to a named frame in root:
bazel run //tools/jogging:move_to_frame -- \
  --address=localhost:17080 \
  --frame=view \
  --motion_type=ANY

# Record current tool pose as 'view' in scene.updates.pbtxt:
bazel run //tools/jogging:store_frame -- view --address=localhost:17080
```
