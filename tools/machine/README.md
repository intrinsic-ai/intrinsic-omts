# CNC Machine & Vise Control CLI (`tools/machine/`)

Command-line and interactive utility for actuating CNC enclosure doors,
pneumatic vises, and machining cycle handshake signals via `DioCncMachine`.

## CLI Target

| Target | Source File | Description |
| :--- | :--- | :--- |
| `//tools/machine:control_machine` | [`control_machine.py`](control_machine.py) | Executes `open_door`, `close_door`, `open_vise`, `close_vise`, `trigger_cycle`, or `wait_cycle` tasks and synchronizes belief-world joint states. |

## Usage Examples

```bash
# Command CNC enclosure door open:
bazel run //tools/machine:control_machine -- \
  --address=localhost:17080 \
  --action=open_door

# Clamp pneumatic vise:
bazel run //tools/machine:control_machine -- \
  --address=localhost:17080 \
  --action=close_vise

# Launch interactive menu (omit --action):
bazel run //tools/machine:control_machine -- --address=localhost:17080
```
