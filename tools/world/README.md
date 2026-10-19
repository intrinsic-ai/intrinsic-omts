# World Management & Inspection Tools (`tools/world/`)

CLI utilities for applying live `ObjectWorldUpdates`, interactively editing
frames and transforms in RViz, and inspecting kinematic trees and transforms.

## CLI Targets & Modules

| Target / Module | Source File | Description |
| :--- | :--- | :--- |
| `//tools/world:apply_scene_updates` | [`apply_scene_updates.py`](apply_scene_updates.py) | Parses `.pbtxt` `ObjectWorldUpdates` files, adapts `create_frame` to `update_transform` when frames already exist, applies updates to `solution.world` (and optionally `init_world` via `--update_init_world`), and optionally resets Gazebo (`--reset_sim` / `--no-reset_sim`). |
| `//tools/world:edit_frames_rviz` | [`edit_frames_rviz.py`](edit_frames_rviz.py) | Publishes 6-DoF ROS 2 `InteractiveMarker` controls (`/omts_frame_editor/update`) directly into RViz for every frame and transform in `.updates.pbtxt`, streaming live `ObjectWorldUpdates` (`/tf`) while dragging and auto-saving to `.updates.pbtxt` on mouse release. |
| `//tools/world:inspect_world` | [`inspect_world.py`](inspect_world.py) | Connects to `solution.world` and prints all active scene objects, joint configurations, installed skills/resources, and key frame transforms (`flange`, `tool_frame`, `view`, `pre_grasp`, `grasp`). |
| `//tools/world:pbtxt_pose_store_lib` | [`pbtxt_pose_store.py`](pbtxt_pose_store.py) | `.updates.pbtxt` parsing, persistence, CWE-22 path safety validation, and live `ObjectWorld` synchronization (`PoseEntry`, `update_pose_in_pbtxt`, `snap_entry_to_tool_pose`). `update_pose_in_pbtxt` updates pose entries in-place while preserving header and inline `#` comments in `.updates.pbtxt` files. |
| `//tools/world:rviz_marker_controller_lib` | [`rviz_marker_controller.py`](rviz_marker_controller.py) | 6-DoF `InteractiveMarker` feedback state machine (`RvizFrameEditorController`) and descendant marker pose synchronization. |
| `//tools/world:rviz_marker_node_lib` | [`rviz_marker_node.py`](rviz_marker_node.py) | Standalone ROS 2 `rclpy` `InteractiveMarkerServer` node and 6-DoF `InteractiveMarker` builder (`build_6dof_interactive_marker`) bridged over JSON-lines stdio or run in-process. |

## Usage Examples

```bash
# Apply default OMTS .pbtxt scene updates and synchronize Gazebo sim_world:
bazel run //tools/world:apply_scene_updates -- \
  --address=localhost:17080 \
  --reset_sim

# Inspect live objects, resources, and frame transforms:
bazel run //tools/world:inspect_world -- --address=localhost:17080
```

## RViz 6-DoF Interactive Frame Editor (`//tools/world:edit_frames_rviz`)

### Prerequisites

ROS 2 (providing `rclpy`, `interactive_markers`, `visualization_msgs`, and
`rmw_zenoh_cpp`) must be installed and sourced in your active shell environment
(`python3` on `PATH`, or specified via `--ros_python`), as it is required by the
`InteractiveMarkerServer` node (`rviz_marker_node.py`).

### Running the Interactive Frame Editor

With your ROS 2 environment sourced in the shell:

```bash
bazel run //tools/world:edit_frames_rviz -- \
  --address=localhost:17080 \
  --zenoh_endpoint=tcp/127.0.0.1:7447
```

### Connecting RViz (Local or Remote over SSH)

1. Ensure the cluster's Zenoh router port (`7447`) and solution gRPC port (`17080`) are reachable (forward `-L 7447:127.0.0.1:7447 -L 17080:127.0.0.1:17080` if connecting from a remote workstation over SSH), and launch RViz with `rmw_zenoh_cpp`:
   ```bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
   rviz2
   ```
2. In RViz:
   - Set **Global Options -> Fixed Frame** to `root`.
   - Click **Add** -> **By display type** -> **rviz_default_plugins/InteractiveMarkers** (or **By topic** -> `/omts_frame_editor/update` -> **InteractiveMarkers**).
   - Set **Update Topic** to `/omts_frame_editor/update`.
   - Select **Interact** in the top RViz toolbar.
3. **Interactive Controls**:
   - **Drag 6-DoF arrows/rings**: Translates or rotates the frame/object along X, Y, Z axes, streaming `/tf` and `solution.world` updates live during drag and **auto-saving to `.updates.pbtxt` on mouse release**.
   - **Right-click a marker**: Opens the context menu to **Save Pose to .pbtxt Now**, **Snap to Gripper TCP (`tool_frame`)**, **Toggle Control Axes (Local `INHERIT` / Parent `FIXED`)**, **Revert Pose from `.pbtxt`**, or **Reset Gazebo Simulation**.
