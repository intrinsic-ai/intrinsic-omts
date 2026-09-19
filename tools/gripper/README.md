# Gripper Control CLI (`tools/gripper/`)

Command-line and interactive utility for testing and actuating Robotiq or DIO
end-effector grippers via SBL skills.

## CLI Target

| Target | Source File | Description |
| :--- | :--- | :--- |
| `//tools/gripper:control_gripper` | [`control_gripper.py`](control_gripper.py) | Instantiates `RobotiqGripper` or `DioGripper` and executes open/close tasks on `solution.executive`. |

## Usage Examples

```bash
# Open or close Robotiq gripper via CLI flags:
bazel run //tools/gripper:control_gripper -- \
  --address=localhost:17080 \
  --gripper_type=robotiq \
  --action=open

# Actuate DIO gripper pins:
bazel run //tools/gripper:control_gripper -- \
  --address=localhost:17080 \
  --gripper_type=dio \
  --action=close

# Launch interactive menu (omit --action):
bazel run //tools/gripper:control_gripper -- --address=localhost:17080
```
