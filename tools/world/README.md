# World Management & Inspection Tools (`tools/world/`)

CLI utilities for applying live `ObjectWorldUpdates` and inspecting kinematic
trees and transforms.

## CLI Targets

| Target | Source File | Description |
| :--- | :--- | :--- |
| `//tools/world:apply_scene_updates` | [`apply_scene_updates.py`](apply_scene_updates.py) | Parses `.pbtxt` `ObjectWorldUpdates` files, adapts `create_frame` to `update_transform` when frames already exist, applies updates to `solution.world`, and optionally resets Gazebo (`--reset_sim` / `--no-reset_sim`). |
| `//tools/world:inspect_world` | [`inspect_world.py`](inspect_world.py) | Connects to `solution.world` and prints all active scene objects, joint configurations, installed skills/resources, and key frame transforms (`flange`, `tool_frame`, `view`, `pre_grasp`, `grasp`). |

## Usage Examples

```bash
# Apply default OMTS .pbtxt scene updates and synchronize Gazebo sim_world:
bazel run //tools/world:apply_scene_updates -- \
  --address=localhost:17080 \
  --reset_sim

# Inspect live objects, resources, and frame transforms:
bazel run //tools/world:inspect_world -- --address=localhost:17080
```
