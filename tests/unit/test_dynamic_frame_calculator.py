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

"""Unit tests for dynamic_frame_calculator and math_utils geometric logic."""

import math
from typing import Any
from unittest import mock

from absl.testing import absltest
from intrinsic.math.python import data_types

from src.utils.dynamic_frame_calculator import (
  calculate_and_update_dynamic_frames,
)
from src.utils.execution_utils import (
  create_transform_node_ref,
  describe_motion_types,
  normalize_motion_types,
  object_exists_in_world,
)
from src.utils.math_utils import (
  normalize_angle,
  normalize_joint_angles,
)
from src.utils.script_utils import load_python_script


def _make_params(**overrides: Any) -> Any:
  """Creates a parameter object mimicking SBL PythonScript params proto."""
  defaults = {
    "parent_object": "root",
    "camera_name": "orbbec_camera",
    "pos_x": 0.30,
    "pos_y": -0.20,
    "pos_z": 1.05,
    "ori_x": 0.0,
    "ori_y": 0.0,
    "ori_z": 0.0,
    "ori_w": 1.0,
    "approach_offset_z": 0.08,
    "pregrasp_frame_name": "pre_grasp",
    "grasp_frame_name": "grasp",
    "target_scene_object_id": "raw_stock_2x3x5",
    "min_safe_z": 0.95,
    "tool_object_name": "gripper",
    "tool_frame_name": "tool_frame",
  }
  defaults.update(overrides)
  params = mock.MagicMock()
  for key, val in defaults.items():
    setattr(params, key, val)
  return params


class DynamicFrameCalculatorTest(absltest.TestCase):
  """Tests 3D pose composition, grasp axis alignment, and SO(3) selection."""

  def test_exec_via_load_python_script_in_isolated_namespace(self) -> None:
    """Verifies the script executes hermetically via exec() like bt.PythonScript."""
    from src.utils import math_utils

    script_code = load_python_script(
      calculate_and_update_dynamic_frames, preludes=(math_utils,)
    )
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    mock_world.root = mock_root
    mock_world.get_transform.return_value = None

    context = mock.MagicMock()
    context.object_world = mock_world
    params = _make_params(pos_z=1.02, min_safe_z=0.95)

    isolated_globals = {"context": context, "params": params}
    exec(script_code, isolated_globals)  # noqa: S102

    self.assertEqual(mock_world.create_frame.call_count, 2)
    created_names = {
      call.kwargs["frame_name"]
      for call in mock_world.create_frame.call_args_list
    }
    self.assertEqual(created_names, {"pre_grasp", "grasp"})

  def test_camera_to_root_transform_composition_and_approach_offset(
    self,
  ) -> None:
    """Verifies root_t_target = root_t_camera * cam_pose and Z approach offset."""
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    mock_world.root = mock_root
    mock_camera = mock.MagicMock()
    mock_world.orbbec_camera = mock_camera

    # Camera mounted at (0.1, 0.2, 1.5) in root with identity rotation
    root_t_camera = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([0.0, 0.0, 0.0, 1.0])),
      [0.1, 0.2, 1.5],
    )

    def _get_transform(node_a: Any, node_b: Any) -> data_types.Pose3 | None:
      del node_a
      if node_b is mock_camera.sensor:
        return root_t_camera
      return None

    mock_world.get_transform.side_effect = _get_transform

    context = mock.MagicMock()
    context.object_world = mock_world
    # Part at (0.05, -0.05, -0.45) in camera frame -> (0.15, 0.15, 1.05) in root
    params = _make_params(
      pos_x=0.05,
      pos_y=-0.05,
      pos_z=-0.45,
      approach_offset_z=0.08,
      min_safe_z=0.95,
    )

    calculate_and_update_dynamic_frames(context, params)

    created_poses = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in mock_world.create_frame.call_args_list
    }
    grasp_pose = created_poses["grasp"]
    pregrasp_pose = created_poses["pre_grasp"]

    self.assertAlmostEqual(float(grasp_pose.translation[0]), 0.15, places=5)
    self.assertAlmostEqual(float(grasp_pose.translation[1]), 0.15, places=5)
    self.assertAlmostEqual(float(grasp_pose.translation[2]), 1.05, places=5)

    self.assertAlmostEqual(float(pregrasp_pose.translation[0]), 0.15, places=5)
    self.assertAlmostEqual(float(pregrasp_pose.translation[1]), 0.15, places=5)
    self.assertAlmostEqual(
      float(pregrasp_pose.translation[2]), 1.05 + 0.08, places=5
    )

  def test_updates_existing_frames_and_target_scene_object(self) -> None:
    """Verifies update_transform is used when frames and target object exist."""
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = ["pre_grasp", "grasp"]
    mock_world.root = mock_root
    mock_world.get_transform.return_value = None

    mock_stock = mock.MagicMock()
    mock_stock.parent = mock_root
    mock_world.raw_stock_2x3x5 = mock_stock

    context = mock.MagicMock()
    context.object_world = mock_world
    params = _make_params(
      target_scene_object_id="raw_stock_2x3x5",
      pos_z=1.00,
    )

    calculate_and_update_dynamic_frames(context, params)

    mock_world.create_frame.assert_not_called()
    # 1 update for raw_stock_2x3x5 + 2 updates for pre_grasp and grasp
    self.assertEqual(mock_world.update_transform.call_count, 3)

  def test_upright_workpiece_falls_back_to_medium_axis_az(self) -> None:
    """When local X points along root Z (|ax[2]| >= 0.7), alignment uses local Z."""
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    mock_world.root = mock_root
    mock_world.get_transform.return_value = None

    # Pitch -90 deg around Y so local X [1, 0, 0] -> [0, 0, 1] (vertical)
    # and local Z [0, 0, 1] -> [-1, 0, 0] (along -X in root XY plane, theta = pi)
    half_angle = -math.pi / 4.0
    params = _make_params(
      ori_x=0.0,
      ori_y=math.sin(half_angle),
      ori_z=0.0,
      ori_w=math.cos(half_angle),
      pos_z=1.00,
    )

    context = mock.MagicMock()
    context.object_world = mock_world
    calculate_and_update_dynamic_frames(context, params)

    grasp_pose = next(
      call.kwargs["parent_t_frame"]
      for call in mock_world.create_frame.call_args_list
      if call.kwargs["frame_name"] == "grasp"
    )
    # With psi = pi, half_psi = pi/2, c1 = (0, 1, 0, 0)
    q = grasp_pose.rotation.quaternion
    self.assertAlmostEqual(abs(float(q.y)), 1.0, places=5)
    self.assertAlmostEqual(float(q.x), 0.0, places=5)

  def test_geodesic_so3_selection_picks_closest_quaternion_to_current_tool(
    self,
  ) -> None:
    """Verifies candidate quaternion maximizing dot(c, cur_q) is chosen."""
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    mock_world.root = mock_root

    # With identity target rotation, psi = 0 -> half_psi = 0.
    # Candidates are:
    #   c1  = (1, 0, 0, 0)
    #  -c1  = (-1, 0, 0, 0)
    #   c2  = (0, 1, 0, 0)
    #  -c2  = (0, -1, 0, 0)
    # Set current tool orientation close to -c2 = (0, -1, 0, 0).
    tool_pose = data_types.Pose3(
      data_types.Rotation3(data_types.Quaternion([0.0, -1.0, 0.0, 0.0])),
      [0.3, -0.2, 1.1],
    )

    def _get_transform(node_a: Any, node_b: Any) -> data_types.Pose3 | None:
      del node_a
      if node_b is mock_world.gripper.tool_frame:
        return tool_pose
      return None

    mock_world.get_transform.side_effect = _get_transform

    context = mock.MagicMock()
    context.object_world = mock_world
    params = _make_params(pos_z=1.00)

    calculate_and_update_dynamic_frames(context, params)

    grasp_pose = next(
      call.kwargs["parent_t_frame"]
      for call in mock_world.create_frame.call_args_list
      if call.kwargs["frame_name"] == "grasp"
    )
    q = grasp_pose.rotation.quaternion
    # Must match -c2 = (0, -1, 0, 0)
    self.assertAlmostEqual(float(q.x), 0.0, places=5)
    self.assertAlmostEqual(float(q.y), -1.0, places=5)
    self.assertAlmostEqual(float(q.z), 0.0, places=5)
    self.assertAlmostEqual(float(q.w), 0.0, places=5)


class MathUtilsTest(absltest.TestCase):
  """Unit tests for math_utils helper functions."""

  def test_normalize_angle_and_joint_angles(self) -> None:
    self.assertAlmostEqual(normalize_angle(0.0), 0.0)
    self.assertAlmostEqual(normalize_angle(1.5 * math.pi), -0.5 * math.pi)
    self.assertAlmostEqual(normalize_angle(-1.5 * math.pi), 0.5 * math.pi)

    wrapped = normalize_joint_angles([0.0, 2.5 * math.pi, -2.5 * math.pi])
    self.assertAlmostEqual(wrapped[0], 0.0)
    self.assertAlmostEqual(wrapped[1], 0.5 * math.pi)
    self.assertAlmostEqual(wrapped[2], -0.5 * math.pi)

  def test_normalize_and_describe_motion_types(self) -> None:
    self.assertEqual(normalize_motion_types("LINEAR", 3), ["LINEAR"] * 3)
    self.assertEqual(
      normalize_motion_types(["ANY", "LINEAR"], 2), ["ANY", "LINEAR"]
    )
    with self.assertRaisesRegex(ValueError, "motion_type has 2 entries"):
      normalize_motion_types(["ANY", "LINEAR"], 3)

    self.assertEqual(describe_motion_types(["ANY", "ANY"]), "ANY")
    self.assertEqual(describe_motion_types(["ANY", "LINEAR"]), "ANY/LINEAR")

  def test_object_exists_in_world(self) -> None:
    self.assertFalse(object_exists_in_world(None, "root"))
    self.assertFalse(object_exists_in_world(mock.MagicMock(), None))
    self.assertFalse(object_exists_in_world(mock.MagicMock(), ""))

    mock_solution = mock.MagicMock()
    mock_solution.world.list_object_names.return_value = ["root", "ur_module"]
    self.assertTrue(object_exists_in_world(mock_solution, "ur_module"))
    self.assertFalse(object_exists_in_world(mock_solution, "cnc_enclosure"))

  def test_create_transform_node_ref(self) -> None:
    obj_ref = create_transform_node_ref("ur_module")
    self.assertEqual(obj_ref.by_name.object.object_name, "ur_module")
    self.assertFalse(obj_ref.by_name.HasField("frame"))

    frame_ref = create_transform_node_ref("gripper", "tool_frame")
    self.assertEqual(frame_ref.by_name.frame.object_name, "gripper")
    self.assertEqual(frame_ref.by_name.frame.frame_name, "tool_frame")


if __name__ == "__main__":
  absltest.main()
