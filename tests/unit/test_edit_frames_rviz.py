# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the RViz 6-DoF interactive frame editor (edit_frames_rviz)."""

import io
import math
import os
import tempfile
from unittest import mock

from absl.testing import absltest
from google.protobuf import text_format
from intrinsic.math.python import data_types, proto_conversion
from intrinsic.world.proto import object_world_updates_pb2

from src.utils.world_utils import (
  connect_initial_world as shared_connect_initial_world,
)
from src.utils.world_utils import (
  find_file as shared_find_file,
)
from tools.world.edit_frames_rviz import (
  _configure_zenoh_env,
  _resolve_ros2_python_command,
  connect_initial_world,
  find_file,
  parse_args,
)
from tools.world.pbtxt_pose_store import (
  PoseEntry,
  add_frame_to_pbtxt,
  apply_pose_entry_to_world,
  load_pose_entries,
  load_pose_entries_from_file,
  resolve_safe_pbtxt_path,
  snap_entry_to_tool_pose,
  update_pose_in_pbtxt,
)
from tools.world.rviz_marker_controller import (
  AXIS_CONTROL_SPECS,
  ControlInteractionMode,
  ControlOrientationMode,
  FeedbackEventType,
  MenuEntryId,
  RvizFrameEditorController,
)
from tools.world.rviz_marker_node import (
  build_6dof_interactive_marker,
  build_marker_from_dict,
  run_subprocess_ros2_node,
)

SAMPLE_PBTXT = """# proto-file: intrinsic/world/public/proto/object_world_updates.proto
# proto-message: intrinsic_proto.world.ObjectWorldUpdates

updates {
  create_frame {
    new_frame_name: "view"
    parent_t_new_frame {
      position {
        x: 0.1753
        y: -0.1008
        z: 1.4011
      }
      orientation {
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
      }
    }
    parent_object_with_filter {
      reference {
        by_name {
          object_name: "root"
        }
      }
    }
  }
}
updates {
  update_transform {
    node_a {
      by_name {
        object {
          object_name: "enclosure"
        }
      }
    }
    node_b {
      by_name {
        object {
          object_name: "cnc_enclosure"
        }
      }
    }
    node_to_update {
      by_name {
        object {
          object_name: "cnc_enclosure"
        }
      }
    }
    a_t_b {
      position {
        x: -0.57
        y: -0.10
        z: 1.02
      }
      orientation {
        x: 0.0
        y: 0.0
        z: 0.7071
        w: 0.7071
      }
    }
  }
}
updates {
  update_object_joints {
    object {
      by_name {
        object_name: "cnc_enclosure"
      }
    }
    joint_positions: 0.4
  }
}
"""


class EditFramesRvizPbtxtAndWorldTest(absltest.TestCase):
  """Tests for .pbtxt path safety, persistence, and live ObjectWorld sync."""

  def setUp(self) -> None:
    super().setUp()
    self.temp_dir = tempfile.TemporaryDirectory()
    self.pbtxt_path = os.path.join(self.temp_dir.name, "scene.updates.pbtxt")
    with open(self.pbtxt_path, "w", encoding="utf-8") as f:
      f.write(SAMPLE_PBTXT)

  def tearDown(self) -> None:
    self.temp_dir.cleanup()
    super().tearDown()

  def test_resolve_safe_pbtxt_path_rejects_traversal_and_non_pbtxt(
    self,
  ) -> None:
    """Tests CWE-22 path traversal rejection and .pbtxt extension validation."""
    resolved = resolve_safe_pbtxt_path(
      "scene.updates.pbtxt", allowed_root=self.temp_dir.name
    )
    self.assertEqual(resolved, os.path.realpath(self.pbtxt_path))

    with self.assertRaises(ValueError):
      resolve_safe_pbtxt_path(
        "../../etc/passwd.pbtxt", allowed_root=self.temp_dir.name
      )

    with self.assertRaises(ValueError):
      resolve_safe_pbtxt_path("notes.txt", allowed_root=self.temp_dir.name)

    self.assertIs(find_file, shared_find_file)
    self.assertIs(connect_initial_world, shared_connect_initial_world)
    with mock.patch.dict(
      os.environ, {"BUILD_WORKSPACE_DIRECTORY": self.temp_dir.name}, clear=True
    ):
      self.assertEqual(
        resolve_safe_pbtxt_path("scene.updates.pbtxt"),
        os.path.realpath(self.pbtxt_path),
      )

  def test_load_pose_entries_and_update_pose_in_pbtxt(self) -> None:
    """Tests loading and updating create_frame and update_transform entries."""
    entries = load_pose_entries_from_file(
      self.pbtxt_path, allowed_root=self.temp_dir.name
    )
    self.assertLen(entries, 2)

    self.assertEqual(entries[0].entry_type, "create_frame")
    self.assertEqual(entries[0].parent_name, "root")
    self.assertEqual(entries[0].target_name, "view")
    self.assertEqual(entries[0].marker_name, "root/view")
    self.assertEqual(entries[0].update_index, 0)
    self.assertAlmostEqual(entries[0].position[0], 0.1753)

    self.assertEqual(entries[1].entry_type, "update_transform")
    self.assertEqual(entries[1].parent_name, "enclosure")
    self.assertEqual(entries[1].target_name, "cnc_enclosure")
    self.assertEqual(entries[1].marker_name, "enclosure/cnc_enclosure")
    self.assertEqual(entries[1].update_index, 1)
    self.assertAlmostEqual(entries[1].position[0], -0.57)

    # Update create_frame (index 0)
    updated_0 = update_pose_in_pbtxt(
      filepath=self.pbtxt_path,
      update_index=0,
      position=(0.25, -0.15, 1.35),
      orientation=(1.0, 0.0, 0.0, 0.0),
      allowed_root=self.temp_dir.name,
    )
    self.assertEqual(updated_0.target_name, "view")
    self.assertAlmostEqual(updated_0.position[0], 0.25)

    # Update update_transform (index 1)
    updated_1 = update_pose_in_pbtxt(
      filepath=self.pbtxt_path,
      update_index=1,
      position=(-0.60, -0.12, 1.05),
      orientation=(0.0, 0.0, 0.0, 1.0),
      allowed_root=self.temp_dir.name,
    )
    self.assertEqual(updated_1.target_name, "cnc_enclosure")
    self.assertAlmostEqual(updated_1.position[0], -0.60)

    # Verify non-pose updates (update_object_joints at index 2) and header are
    # preserved.
    with open(self.pbtxt_path, encoding="utf-8") as f:
      raw_text = f.read()
    self.assertIn("# proto-file:", raw_text)
    parsed = object_world_updates_pb2.ObjectWorldUpdates()
    text_format.Parse(raw_text, parsed)
    self.assertLen(parsed.updates, 3)
    self.assertTrue(parsed.updates[2].HasField("update_object_joints"))
    self.assertAlmostEqual(
      parsed.updates[2].update_object_joints.joint_positions[0], 0.4
    )

    with self.assertRaises((IndexError, ValueError)):
      update_pose_in_pbtxt(
        filepath=self.pbtxt_path,
        update_index=2,
        position=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        allowed_root=self.temp_dir.name,
      )

  def test_add_frame_to_pbtxt_creates_and_updates(self) -> None:
    """Tests adding a new create_frame and updating an existing one."""
    created = add_frame_to_pbtxt(
      filepath=self.pbtxt_path,
      frame_name="new_pick_frame",
      parent_name="root",
      position=(0.11, 0.22, 0.33),
      orientation=(0.0, 0.0, 0.0, 1.0),
      allowed_root=self.temp_dir.name,
    )
    self.assertEqual(created.target_name, "new_pick_frame")
    self.assertEqual(created.parent_name, "root")
    self.assertAlmostEqual(created.position[0], 0.11)

    entries = load_pose_entries(
      [self.pbtxt_path], allowed_root=self.temp_dir.name
    )
    self.assertLen(entries, 3)

    updated = add_frame_to_pbtxt(
      filepath=self.pbtxt_path,
      frame_name="new_pick_frame",
      parent_name="root",
      position=(0.44, 0.55, 0.66),
      orientation=(0.0, 0.0, 0.0, 1.0),
      allowed_root=self.temp_dir.name,
    )
    self.assertEqual(updated.update_index, created.update_index)
    self.assertAlmostEqual(updated.position[0], 0.44)
    entries_after = load_pose_entries(
      [self.pbtxt_path], allowed_root=self.temp_dir.name
    )
    self.assertLen(entries_after, 3)

  def test_unit_quaternion_precision_and_position_rounding_on_save(
    self,
  ) -> None:
    """Tests saved quaternions keep full unit norm while positions round to 4dp."""
    rot = data_types.Rotation3.from_euler_angles(
      rpy_degrees=[30.0, -25.0, 120.0]
    )
    q_xyzw = tuple(float(c) for c in rot.quaternion.xyzw)

    update_pose_in_pbtxt(
      filepath=self.pbtxt_path,
      update_index=0,
      position=(0.123456, -0.654321, 1.987654),
      orientation=q_xyzw,
      allowed_root=self.temp_dir.name,
    )
    add_frame_to_pbtxt(
      filepath=self.pbtxt_path,
      frame_name="precise_frame",
      parent_name="root",
      position=(0.555555, -0.444444, 0.333333),
      orientation=q_xyzw,
      allowed_root=self.temp_dir.name,
    )

    with open(self.pbtxt_path, encoding="utf-8") as f:
      parsed = object_world_updates_pb2.ObjectWorldUpdates()
      text_format.Parse(f.read(), parsed)

    pose_0 = parsed.updates[0].create_frame.parent_t_new_frame
    self.assertAlmostEqual(pose_0.position.x, 0.1235, places=6)
    self.assertAlmostEqual(pose_0.position.y, -0.6543, places=6)
    self.assertAlmostEqual(pose_0.position.z, 1.9877, places=6)
    norm_0 = math.sqrt(
      pose_0.orientation.x**2
      + pose_0.orientation.y**2
      + pose_0.orientation.z**2
      + pose_0.orientation.w**2
    )
    self.assertLess(abs(norm_0 - 1.0), 1e-7)
    converted_0 = proto_conversion.pose_from_proto(pose_0)
    for actual, expected in zip(
      converted_0.rotation.quaternion.xyzw, q_xyzw, strict=True
    ):
      self.assertAlmostEqual(float(actual), expected, places=7)

    pose_added = parsed.updates[3].create_frame.parent_t_new_frame
    self.assertAlmostEqual(pose_added.position.x, 0.5556, places=6)
    self.assertAlmostEqual(pose_added.position.y, -0.4444, places=6)
    self.assertAlmostEqual(pose_added.position.z, 0.3333, places=6)
    norm_added = math.sqrt(
      pose_added.orientation.x**2
      + pose_added.orientation.y**2
      + pose_added.orientation.z**2
      + pose_added.orientation.w**2
    )
    self.assertLess(abs(norm_added - 1.0), 1e-7)
    converted_added = proto_conversion.pose_from_proto(pose_added)
    for actual, expected in zip(
      converted_added.rotation.quaternion.xyzw, q_xyzw, strict=True
    ):
      self.assertAlmostEqual(float(actual), expected, places=7)

  def test_inline_comments_preserved_in_pbtxt_on_save(self) -> None:
    """Tests saving via update_pose_in_pbtxt and add_frame_to_pbtxt preserves # comments."""
    commented_pbtxt = """# proto-file: intrinsic/world/public/proto/object_world_updates.proto
# proto-message: intrinsic_proto.world.ObjectWorldUpdates

# Section comment before view frame
updates {
  # Inline comment inside view update block
  create_frame {
    new_frame_name: "view"
    parent_t_new_frame {
      position {
        x: 0.1753
        y: -0.1008
        z: 1.4011
      }
      orientation {
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
      }
    }
    parent_object_with_filter {
      reference {
        by_name {
          object_name: "root"
        }
      }
    }
  }
}

# Comment between updates blocks
updates {
  # Inline comment inside cnc_enclosure transform
  update_transform {
    node_a {
      by_name {
        object {
          object_name: "enclosure"
        }
      }
    }
    node_b {
      by_name {
        object {
          object_name: "cnc_enclosure"
        }
      }
    }
    node_to_update {
      by_name {
        object {
          object_name: "cnc_enclosure"
        }
      }
    }
    a_t_b {
      position {
        x: -0.57
        y: -0.10
        z: 1.02
      }
      orientation {
        x: 0.0
        y: 0.0
        z: 0.70710678
        w: 0.70710678
      }
    }
  }
}
# Trailing comment at end of file
"""
    with open(self.pbtxt_path, "w", encoding="utf-8") as f:
      f.write(commented_pbtxt)

    update_pose_in_pbtxt(
      filepath=self.pbtxt_path,
      update_index=0,
      position=(0.22, -0.11, 1.44),
      orientation=(0.0, 0.0, 0.0, 1.0),
      allowed_root=self.temp_dir.name,
    )
    with open(self.pbtxt_path, encoding="utf-8") as f:
      after_update = f.read()

    self.assertIn("# Section comment before view frame", after_update)
    self.assertIn("# Inline comment inside view update block", after_update)
    self.assertIn("# Comment between updates blocks", after_update)
    self.assertIn(
      "# Inline comment inside cnc_enclosure transform", after_update
    )
    self.assertIn("# Trailing comment at end of file", after_update)

    # Update existing frame via add_frame_to_pbtxt and append a new frame
    add_frame_to_pbtxt(
      filepath=self.pbtxt_path,
      frame_name="view",
      parent_name="root",
      position=(0.33, -0.22, 1.55),
      orientation=(0.0, 0.0, 0.0, 1.0),
      allowed_root=self.temp_dir.name,
    )
    add_frame_to_pbtxt(
      filepath=self.pbtxt_path,
      frame_name="appended_frame",
      parent_name="root",
      position=(0.10, 0.20, 0.30),
      orientation=(0.0, 0.0, 0.0, 1.0),
      allowed_root=self.temp_dir.name,
    )
    with open(self.pbtxt_path, encoding="utf-8") as f:
      after_add = f.read()

    self.assertIn("# Section comment before view frame", after_add)
    self.assertIn("# Inline comment inside view update block", after_add)
    self.assertIn("# Comment between updates blocks", after_add)
    self.assertIn("# Inline comment inside cnc_enclosure transform", after_add)
    self.assertIn("# Trailing comment at end of file", after_add)

    parsed = object_world_updates_pb2.ObjectWorldUpdates()
    text_format.Parse(after_add, parsed)
    self.assertLen(parsed.updates, 3)
    self.assertAlmostEqual(
      parsed.updates[0].create_frame.parent_t_new_frame.position.x, 0.33
    )
    self.assertEqual(
      parsed.updates[2].create_frame.new_frame_name, "appended_frame"
    )

  def test_apply_pose_entry_to_world_and_snap_to_tool_pose(self) -> None:
    """Tests live world synchronization and snapping an entry to gripper TCP."""
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = ["view"]
    mock_world.root = mock_root
    mock_init_world = mock.MagicMock()
    mock_init_world.root = mock_root

    entry = PoseEntry(
      entry_id=f"{self.pbtxt_path}::0",
      marker_name="root/view",
      file_path=self.pbtxt_path,
      update_index=0,
      entry_type="create_frame",
      parent_name="root",
      target_name="view",
      position=(0.1, 0.2, 0.3),
      orientation=(0.0, 0.0, 0.0, 1.0),
      rpy_deg=(0.0, 0.0, 0.0),
    )
    ok = apply_pose_entry_to_world(
      world=mock_world, entry=entry, init_world=mock_init_world
    )
    self.assertTrue(ok)
    mock_world.batch_update.assert_called_once()
    mock_init_world.batch_update.assert_called_once()

    mock_pose = data_types.Pose3(
      data_types.Rotation3(
        data_types.Quaternion([0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
      ),
      [0.31, 0.42, 0.89],
    )
    mock_world.get_transform.return_value = mock_pose

    snapped = snap_entry_to_tool_pose(
      world=mock_world,
      filepath=self.pbtxt_path,
      update_index=0,
      tool_object="gripper",
      tool_frame="tool_frame",
      init_world=mock_init_world,
      allowed_root=self.temp_dir.name,
    )
    self.assertAlmostEqual(snapped.position[0], 0.31, places=4)
    self.assertAlmostEqual(snapped.position[1], 0.42, places=4)
    self.assertAlmostEqual(snapped.position[2], 0.89, places=4)


class EditFramesRvizMarkerAndControllerTest(absltest.TestCase):
  """Tests for 6-DoF InteractiveMarker construction and RvizFrameEditorController."""

  def setUp(self) -> None:
    super().setUp()
    self.temp_dir = tempfile.TemporaryDirectory()
    self.pbtxt_path = os.path.join(self.temp_dir.name, "scene.updates.pbtxt")
    with open(self.pbtxt_path, "w", encoding="utf-8") as f:
      f.write(SAMPLE_PBTXT)

    self.mock_solution = mock.MagicMock()
    self.mock_solution.is_simulated = True
    self.mock_solution.simulator = mock.MagicMock()
    self.mock_world = mock.MagicMock()
    self.mock_world.get_transform.return_value = None
    self.mock_root = mock.MagicMock()
    self.mock_root.list_frames.return_value = ["view"]
    self.mock_world.root = self.mock_root
    self.mock_init_world = mock.MagicMock()
    self.mock_init_world.root = self.mock_root

    self.controller = RvizFrameEditorController(
      files=[self.pbtxt_path],
      world=self.mock_world,
      init_world=self.mock_init_world,
      solution=self.mock_solution,
      allowed_root=self.temp_dir.name,
      marker_scale=0.20,
      fixed_orientation=False,
      drag_throttle_sec=0.04,
      reset_sim_on_release=False,
    )

  def tearDown(self) -> None:
    self.temp_dir.cleanup()
    super().tearDown()

  def test_build_6dof_interactive_marker_controls_and_modes(self) -> None:
    """Tests 6-DoF InteractiveMarker generation for PoseEntry and dict inputs."""
    entry = self.controller.entries_by_marker["root/view"]
    marker_inherit = build_6dof_interactive_marker(
      entry, scale=0.22, fixed_orientation=False
    )
    self.assertEqual(marker_inherit.header.frame_id, "root")
    self.assertEqual(marker_inherit.name, "root/view")
    self.assertAlmostEqual(marker_inherit.scale, 0.22, places=6)
    self.assertAlmostEqual(marker_inherit.pose.position.x, 0.1753, places=4)
    self.assertAlmostEqual(marker_inherit.pose.orientation.w, 1.0, places=4)

    # 1 MENU control + 6 axis controls = 7 controls
    self.assertLen(marker_inherit.controls, 7)
    menu_controls = [
      c
      for c in marker_inherit.controls
      if c.interaction_mode == int(ControlInteractionMode.MENU)
    ]
    self.assertLen(menu_controls, 1)
    self.assertTrue(menu_controls[0].always_visible)

    axis_controls = [
      c
      for c in marker_inherit.controls
      if c.interaction_mode
      in (
        int(ControlInteractionMode.ROTATE_AXIS),
        int(ControlInteractionMode.MOVE_AXIS),
      )
    ]
    self.assertLen(axis_controls, len(AXIS_CONTROL_SPECS))
    control_names = {c.name for c in axis_controls}
    self.assertEqual(
      control_names,
      {"rotate_x", "move_x", "rotate_z", "move_z", "rotate_y", "move_y"},
    )
    for c in axis_controls:
      self.assertEqual(c.orientation_mode, int(ControlOrientationMode.INHERIT))

    # Verify FIXED orientation mode
    marker_fixed = build_6dof_interactive_marker(
      entry, scale=0.22, fixed_orientation=True
    )
    axis_controls_fixed = [
      c
      for c in marker_fixed.controls
      if c.interaction_mode
      in (
        int(ControlInteractionMode.ROTATE_AXIS),
        int(ControlInteractionMode.MOVE_AXIS),
      )
    ]
    for c in axis_controls_fixed:
      self.assertEqual(c.orientation_mode, int(ControlOrientationMode.FIXED))

    # Verify build_marker_from_dict consolidates cleanly with dict input
    marker_from_dict = build_marker_from_dict(
      item=entry.to_dict(),
      scale=0.22,
      fixed_orientation=True,
      fixed_frame="root",
      marker_pose={"position": [0.1, 0.2, 0.3], "orientation": [0, 0, 0, 1]},
    )
    self.assertEqual(marker_from_dict.header.frame_id, "root")
    self.assertEqual(marker_from_dict.name, "root/view")
    self.assertLen(marker_from_dict.controls, 7)
    self.assertAlmostEqual(marker_from_dict.pose.position.x, 0.1, places=4)

  def test_handle_feedback_pose_update_streams_to_world_without_saving_file(
    self,
  ) -> None:
    """Tests POSE_UPDATE streams live world updates respecting throttle and does not write .pbtxt."""
    res1 = self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.POSE_UPDATE),
      position=(0.30, -0.20, 1.50),
      orientation=(0.0, 0.0, 0.0, 1.0),
      now=100.0,
    )
    self.assertEqual(res1["status"], "pose_updated")
    self.assertTrue(res1["applied_to_world"])
    self.assertFalse(res1["saved_to_file"])
    self.assertEqual(self.mock_world.batch_update.call_count, 1)
    # init_world is not updated on intermediate drag frames
    self.assertEqual(self.mock_init_world.batch_update.call_count, 0)

    # Throttled call within 0.04s should update in-memory pose but skip RPC
    res2 = self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.POSE_UPDATE),
      position=(0.31, -0.21, 1.51),
      orientation=(0.0, 0.0, 0.0, 1.0),
      now=100.01,
    )
    self.assertEqual(res2["status"], "pose_updated")
    self.assertFalse(res2["applied_to_world"])
    self.assertFalse(res2["saved_to_file"])
    self.assertEqual(self.mock_world.batch_update.call_count, 1)

    # Verify disk file still has original x=0.1753 during drag
    disk_entries = load_pose_entries_from_file(
      self.pbtxt_path, allowed_root=self.temp_dir.name
    )
    self.assertAlmostEqual(disk_entries[0].position[0], 0.1753, places=4)

  def test_handle_feedback_mouse_up_saves_to_pbtxt_and_updates_worlds(
    self,
  ) -> None:
    """Tests MOUSE_UP auto-saves the updated pose to .updates.pbtxt and syncs world + init_world."""
    res = self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.MOUSE_UP),
      position=(0.45, -0.25, 1.60),
      orientation=(0.0, 0.0, 0.7071, 0.7071),
      now=200.0,
    )
    self.assertEqual(res["status"], "saved_on_mouse_up")
    self.assertTrue(res["saved_to_file"])
    self.assertTrue(res["applied_to_world"])
    self.mock_world.batch_update.assert_called_once()
    self.mock_init_world.batch_update.assert_called_once()

    disk_entries = load_pose_entries_from_file(
      self.pbtxt_path, allowed_root=self.temp_dir.name
    )
    self.assertAlmostEqual(disk_entries[0].position[0], 0.45, places=4)
    self.assertAlmostEqual(disk_entries[0].position[1], -0.25, places=4)
    self.assertAlmostEqual(disk_entries[0].position[2], 1.60, places=4)

  def test_handle_feedback_menu_entries(self) -> None:
    """Tests MENU_SELECT actions: SAVE_NOW, SNAP_TO_TOOL, TOGGLE_AXES_MODE, REVERT_FROM_FILE, RESET_SIM."""
    # 1. TOGGLE_AXES_MODE
    toggle_res = self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.MENU_SELECT),
      menu_entry_id=int(MenuEntryId.TOGGLE_AXES_MODE),
    )
    self.assertEqual(toggle_res["status"], "axes_mode_toggled")
    self.assertTrue(toggle_res["fixed_orientation"])

    # 2. POSE_UPDATE followed by REVERT_FROM_FILE
    self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.POSE_UPDATE),
      position=(0.99, 0.88, 0.77),
      orientation=(0.0, 0.0, 0.0, 1.0),
      now=300.0,
    )
    self.assertAlmostEqual(
      self.controller.entries_by_marker["root/view"].position[0], 0.99
    )
    revert_res = self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.MENU_SELECT),
      menu_entry_id=int(MenuEntryId.REVERT_FROM_FILE),
    )
    self.assertAlmostEqual(revert_res["entry"]["position"][0], 0.1753, places=4)

    # 3. SAVE_NOW
    self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.POSE_UPDATE),
      position=(0.51, 0.52, 0.53),
      orientation=(0.0, 0.0, 0.0, 1.0),
      now=400.0,
    )
    save_res = self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.MENU_SELECT),
      menu_entry_id=int(MenuEntryId.SAVE_NOW),
    )
    self.assertTrue(save_res["saved_to_file"])
    disk_entries = load_pose_entries_from_file(
      self.pbtxt_path, allowed_root=self.temp_dir.name
    )
    self.assertAlmostEqual(disk_entries[0].position[0], 0.51, places=4)

    # 4. SNAP_TO_TOOL
    mock_pose = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([0.0, 0.0, 0.0, 1.0])),
      [0.61, 0.62, 0.63],
    )
    self.mock_world.get_transform.return_value = mock_pose
    snap_res = self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.MENU_SELECT),
      menu_entry_id=int(MenuEntryId.SNAP_TO_TOOL),
    )
    self.assertAlmostEqual(snap_res["entry"]["position"][0], 0.61, places=4)

    # 5. RESET_SIM
    reset_res = self.controller.handle_feedback(
      marker_name="root/view",
      event_type=int(FeedbackEventType.MENU_SELECT),
      menu_entry_id=int(MenuEntryId.RESET_SIM),
    )
    self.assertTrue(reset_res["reset"])
    self.mock_solution.simulator.reset.assert_called_once()

  def _write_schunk_pbtxt(self) -> str:
    """Writes a child schunk.updates.pbtxt relative to cnc_enclosure."""
    schunk_pbtxt = os.path.join(self.temp_dir.name, "schunk.updates.pbtxt")
    with open(schunk_pbtxt, "w", encoding="utf-8") as f:
      f.write(
        """# proto-file: intrinsic/world/public/proto/object_world_updates.proto
# proto-message: intrinsic_proto.world.ObjectWorldUpdates

updates {
  update_transform {
    node_a {
      by_name {
        object {
          object_name: "cnc_enclosure"
        }
      }
    }
    node_b {
      by_name {
        object {
          object_name: "schunk"
        }
      }
    }
    node_to_update {
      by_name {
        object {
          object_name: "schunk"
        }
      }
    }
    a_t_b {
      position {
        x: -0.0725
        y: 0.005
        z: 0.0
      }
      orientation {
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
      }
    }
  }
}
"""
      )
    return schunk_pbtxt

  def test_build_markers_anchors_all_entries_in_root_and_composes_chained_poses(
    self,
  ) -> None:
    """Tests build_markers sets header.frame_id=='root' and empty sub-marker frame_id for all entries."""
    schunk_pbtxt = self._write_schunk_pbtxt()
    chained_controller = RvizFrameEditorController(
      files=[self.pbtxt_path, schunk_pbtxt],
      world=None,
      allowed_root=self.temp_dir.name,
    )
    markers = chained_controller.build_markers()
    self.assertIn("root/view", markers)
    self.assertIn("enclosure/cnc_enclosure", markers)
    self.assertIn("cnc_enclosure/schunk", markers)

    for marker_name, marker in markers.items():
      self.assertEqual(
        marker.header.frame_id,
        "root",
        msg=f"Marker {marker_name} must be anchored in 'root'",
      )
      menu_ctrl = marker.controls[0]
      self.assertEqual(menu_ctrl.markers[0].header.frame_id, "")

    # Verify chained pose of cnc_enclosure/schunk in 'root':
    # cnc_enclosure is at (-0.57, -0.10, 1.02) with +90 deg Z rotation.
    # schunk relative to cnc_enclosure is (-0.0725, 0.005, 0.0).
    # In root: x = -0.57 - 0.005 = -0.575, y = -0.10 - 0.0725 = -0.1725, z = 1.02.
    schunk_marker = markers["cnc_enclosure/schunk"]
    self.assertAlmostEqual(schunk_marker.pose.position.x, -0.575, places=4)
    self.assertAlmostEqual(schunk_marker.pose.position.y, -0.1725, places=4)
    self.assertAlmostEqual(schunk_marker.pose.position.z, 1.02, places=4)

  def test_handle_feedback_on_chained_child_converts_root_pose_to_parent_relative(
    self,
  ) -> None:
    """Tests dragging a non-root parent entry in 'root' saves parent-relative a_t_b pose to .pbtxt."""
    schunk_pbtxt = self._write_schunk_pbtxt()
    chained_controller = RvizFrameEditorController(
      files=[self.pbtxt_path, schunk_pbtxt],
      world=self.mock_world,
      init_world=self.mock_init_world,
      allowed_root=self.temp_dir.name,
    )

    # Suppose user drags cnc_enclosure/schunk in RViz ('root' frame) to a new
    # target pose corresponding to local offset (0.10, -0.20, 0.05) relative to
    # cnc_enclosure.
    t_root_cnc = chained_controller.get_parent_pose_in_fixed_frame(
      "cnc_enclosure"
    )
    pose_root_cnc = data_types.Pose3(
      rotation=data_types.Rotation3.from_xyzw(
        list(t_root_cnc[1]), normalize=True
      ),
      translation=list(t_root_cnc[0]),
    )
    pose_desired_local = data_types.Pose3(
      rotation=data_types.Rotation3.identity(),
      translation=[0.10, -0.20, 0.05],
    )
    pose_dragged_root = pose_root_cnc.multiply(pose_desired_local)
    dragged_root_pos = (
      float(pose_dragged_root.translation[0]),
      float(pose_dragged_root.translation[1]),
      float(pose_dragged_root.translation[2]),
    )
    q_xyzw = pose_dragged_root.rotation.quaternion.xyzw
    dragged_root_ori = (
      float(q_xyzw[0]),
      float(q_xyzw[1]),
      float(q_xyzw[2]),
      float(q_xyzw[3]),
    )

    res = chained_controller.handle_feedback(
      marker_name="cnc_enclosure/schunk",
      event_type=int(FeedbackEventType.MOUSE_UP),
      position=dragged_root_pos,
      orientation=dragged_root_ori,
      now=500.0,
    )
    self.assertEqual(res["status"], "saved_on_mouse_up")
    self.assertTrue(res["saved_to_file"])

    # Saved entry in .pbtxt must be parent-relative (desired_local), NOT root
    # coordinates!
    disk_entries = load_pose_entries_from_file(
      schunk_pbtxt, allowed_root=self.temp_dir.name
    )
    self.assertLen(disk_entries, 1)
    self.assertAlmostEqual(disk_entries[0].position[0], 0.10, places=4)
    self.assertAlmostEqual(disk_entries[0].position[1], -0.20, places=4)
    self.assertAlmostEqual(disk_entries[0].position[2], 0.05, places=4)
    self.assertAlmostEqual(disk_entries[0].orientation[3], 1.0, places=4)

    # Returned marker_pose must be in 'root' frame for server.setPose()
    for a, b in zip(
      res["marker_pose"]["position"], dragged_root_pos, strict=True
    ):
      self.assertAlmostEqual(a, b, places=3)

  def test_parent_marker_updates_recompute_descendant_marker_poses(
    self,
  ) -> None:
    """Tests updating a parent marker recomputes descendant_marker_poses across all events."""
    schunk_pbtxt = self._write_schunk_pbtxt()
    chained_controller = RvizFrameEditorController(
      files=[self.pbtxt_path, schunk_pbtxt],
      world=self.mock_world,
      init_world=self.mock_init_world,
      allowed_root=self.temp_dir.name,
    )

    # 1. POSE_UPDATE on enclosure/cnc_enclosure to (1.0, 2.0, 3.0) with identity
    # orientation: schunk (-0.0725, 0.005, 0.0) in root becomes (0.9275, 2.005, 3.0)
    res_drag = chained_controller.handle_feedback(
      marker_name="enclosure/cnc_enclosure",
      event_type=int(FeedbackEventType.POSE_UPDATE),
      position=(1.0, 2.0, 3.0),
      orientation=(0.0, 0.0, 0.0, 1.0),
      now=10.0,
    )
    self.assertIn("descendant_marker_poses", res_drag)
    self.assertIn("cnc_enclosure/schunk", res_drag["descendant_marker_poses"])
    schunk_drag_pos = res_drag["descendant_marker_poses"][
      "cnc_enclosure/schunk"
    ]["position"]
    self.assertAlmostEqual(schunk_drag_pos[0], 0.9275, places=4)
    self.assertAlmostEqual(schunk_drag_pos[1], 2.005, places=4)
    self.assertAlmostEqual(schunk_drag_pos[2], 3.0, places=4)

    # 2. REVERT_FROM_FILE restores cnc_enclosure and recomputes schunk in root
    res_revert = chained_controller.handle_feedback(
      marker_name="enclosure/cnc_enclosure",
      event_type=int(FeedbackEventType.MENU_SELECT),
      menu_entry_id=int(MenuEntryId.REVERT_FROM_FILE),
    )
    schunk_revert_pos = res_revert["descendant_marker_poses"][
      "cnc_enclosure/schunk"
    ]["position"]
    self.assertAlmostEqual(schunk_revert_pos[0], -0.575, places=4)
    self.assertAlmostEqual(schunk_revert_pos[1], -0.1725, places=4)
    self.assertAlmostEqual(schunk_revert_pos[2], 1.02, places=4)

    # 3. MOUSE_UP on enclosure/cnc_enclosure
    res_up = chained_controller.handle_feedback(
      marker_name="enclosure/cnc_enclosure",
      event_type=int(FeedbackEventType.MOUSE_UP),
      position=(0.5, 0.5, 1.0),
      orientation=(0.0, 0.0, 0.0, 1.0),
      now=20.0,
    )
    schunk_up_pos = res_up["descendant_marker_poses"]["cnc_enclosure/schunk"][
      "position"
    ]
    self.assertAlmostEqual(schunk_up_pos[0], 0.4275, places=4)
    self.assertAlmostEqual(schunk_up_pos[1], 0.505, places=4)
    self.assertAlmostEqual(schunk_up_pos[2], 1.0, places=4)

    # 4. SAVE_NOW on enclosure/cnc_enclosure
    chained_controller.handle_feedback(
      marker_name="enclosure/cnc_enclosure",
      event_type=int(FeedbackEventType.POSE_UPDATE),
      position=(0.0, 1.0, 2.0),
      orientation=(0.0, 0.0, 0.0, 1.0),
      now=30.0,
    )
    res_save = chained_controller.handle_feedback(
      marker_name="enclosure/cnc_enclosure",
      event_type=int(FeedbackEventType.MENU_SELECT),
      menu_entry_id=int(MenuEntryId.SAVE_NOW),
    )
    schunk_save_pos = res_save["descendant_marker_poses"][
      "cnc_enclosure/schunk"
    ]["position"]
    self.assertAlmostEqual(schunk_save_pos[0], -0.0725, places=4)
    self.assertAlmostEqual(schunk_save_pos[1], 1.005, places=4)
    self.assertAlmostEqual(schunk_save_pos[2], 2.0, places=4)

    # 5. SNAP_TO_TOOL on enclosure/cnc_enclosure
    tool_node = self.mock_world.gripper.tool_frame
    snap_pose = data_types.Pose3(
      data_types.Rotation3.identity(),
      [2.0, 3.0, 4.0],
    )
    self.mock_world.get_transform.side_effect = lambda _a, b: (
      snap_pose if b == tool_node else None
    )
    res_snap = chained_controller.handle_feedback(
      marker_name="enclosure/cnc_enclosure",
      event_type=int(FeedbackEventType.MENU_SELECT),
      menu_entry_id=int(MenuEntryId.SNAP_TO_TOOL),
    )
    schunk_snap_pos = res_snap["descendant_marker_poses"][
      "cnc_enclosure/schunk"
    ]["position"]
    self.assertAlmostEqual(schunk_snap_pos[0], 1.9275, places=4)
    self.assertAlmostEqual(schunk_snap_pos[1], 3.005, places=4)
    self.assertAlmostEqual(schunk_snap_pos[2], 4.0, places=4)

  def test_run_subprocess_ros2_node_raises_on_early_exit_with_stderr(
    self,
  ) -> None:
    """Tests run_subprocess_ros2_node raises RuntimeError with stderr when child exits before ready."""
    fake_proc = mock.MagicMock()
    fake_proc.stdin = io.StringIO()
    fake_proc.stdout = io.StringIO("")
    fake_proc.stderr = io.StringIO("ImportError: libddsc.so not found\n")
    fake_proc.poll.return_value = 1
    fake_proc.wait.return_value = 1
    fake_proc.returncode = 1

    with mock.patch(
      "tools.world.rviz_marker_node.subprocess.Popen", return_value=fake_proc
    ):
      with self.assertRaisesRegex(
        RuntimeError, "ImportError: libddsc.so not found"
      ):
        run_subprocess_ros2_node(
          controller=self.controller,
          topic_ns="omts_frame_editor",
          cmd=["/usr/bin/python3", "-u", "rviz_marker_node.py"],
        )

  def test_parse_args_defaults(self) -> None:
    """Tests CLI argument defaults for edit_frames_rviz."""
    args = parse_args([])
    self.assertEqual(args.address, "localhost:17080")
    self.assertEqual(args.topic_ns, "omts_frame_editor")
    self.assertEqual(args.fixed_frame, "root")
    self.assertAlmostEqual(args.marker_scale, 0.18)
    self.assertFalse(args.fixed_orientation)
    self.assertEqual(args.zenoh_endpoint, "tcp/127.0.0.1:7447")
    self.assertEqual(args.ros_python, "python3")
    self.assertFalse(args.offline)
    self.assertTrue(args.update_init_world)
    self.assertTrue(args.apply_on_start)
    self.assertFalse(args.reset_sim_on_release)

  def test_configure_zenoh_env(self) -> None:
    """Tests _configure_zenoh_env sets RMW_IMPLEMENTATION and client-mode ZENOH_CONFIG_OVERRIDE."""
    with mock.patch.dict(os.environ, {}, clear=True):
      _configure_zenoh_env("tcp/127.0.0.1:7447")
      self.assertEqual(os.environ.get("RMW_IMPLEMENTATION"), "rmw_zenoh_cpp")
      self.assertEqual(
        os.environ.get("ZENOH_CONFIG_OVERRIDE"),
        'mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]',
      )

  def test_resolve_ros2_python_command_validates_active_ros2_python(
    self,
  ) -> None:
    """Tests _resolve_ros2_python_command resolves python3 on PATH or --ros_python and checks rclpy."""
    ok_proc = mock.MagicMock(returncode=0, stderr="")
    with (
      mock.patch(
        "tools.world.edit_frames_rviz.shutil.which",
        return_value="/usr/bin/python3",
      ) as mock_which,
      mock.patch(
        "tools.world.edit_frames_rviz.subprocess.run",
        return_value=ok_proc,
      ) as mock_run,
    ):
      cmd = _resolve_ros2_python_command()
      mock_which.assert_called_once_with("python3")
      mock_run.assert_called_once_with(
        ["/usr/bin/python3", "-c", "import rclpy, interactive_markers"],
        check=False,
        capture_output=True,
        text=True,
      )
      self.assertEqual(cmd[0], "/usr/bin/python3")
      self.assertEqual(cmd[1], "-u")
      self.assertTrue(cmd[2].endswith("rviz_marker_node.py"))

    # Explicit --ros_python path when shutil.which returns None
    custom_py = os.path.join(self.temp_dir.name, "custom_ros_python")
    with open(custom_py, "w", encoding="utf-8") as f:
      f.write("#!/bin/sh\n")
    with (
      mock.patch(
        "tools.world.edit_frames_rviz.shutil.which", return_value=None
      ),
      mock.patch(
        "tools.world.edit_frames_rviz.subprocess.run", return_value=ok_proc
      ),
    ):
      cmd_custom = _resolve_ros2_python_command(ros_python=custom_py)
      self.assertEqual(cmd_custom[0], os.path.abspath(custom_py))
      self.assertEqual(cmd_custom[1], "-u")
      self.assertTrue(cmd_custom[2].endswith("rviz_marker_node.py"))

  def test_resolve_ros2_python_command_raises_when_missing_or_inactive(
    self,
  ) -> None:
    """Tests _resolve_ros2_python_command raises RuntimeError with setup instructions when ROS 2 is inactive."""
    with mock.patch(
      "tools.world.edit_frames_rviz.shutil.which", return_value=None
    ):
      with self.assertRaisesRegex(RuntimeError, "not found on PATH"):
        _resolve_ros2_python_command(ros_python="nonexistent_ros_python")

    fail_proc = mock.MagicMock(
      returncode=1,
      stderr="ModuleNotFoundError: No module named 'rclpy'",
    )
    with (
      mock.patch(
        "tools.world.edit_frames_rviz.shutil.which",
        return_value="/usr/bin/python3",
      ),
      mock.patch(
        "tools.world.edit_frames_rviz.subprocess.run",
        return_value=fail_proc,
      ),
    ):
      with self.assertRaisesRegex(
        RuntimeError,
        "could not import `rclpy` and `interactive_markers`",
      ):
        _resolve_ros2_python_command()


if __name__ == "__main__":
  absltest.main()
