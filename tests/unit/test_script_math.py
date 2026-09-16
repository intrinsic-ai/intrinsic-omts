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

"""Unit tests for the grasp orientation math injected into the sandbox."""

import math

from absl.testing import absltest
from intrinsic.math.python import data_types

from src.utils import math_utils


class ScriptMathTest(absltest.TestCase):
  """Tests for extract_in_plane_alignment_axis and its grasp consumer."""

  def test_compute_top_down_grasp_quaternion_minimizes_joint_rotation(self):
    rot = data_types.Rotation3.identity()
    # If current tool quaternion is close to (0, 1, 0, 0)
    tool_q = (0.0, 1.0, 0.0, 0.0)
    grasp_q = math_utils.compute_top_down_grasp_quaternion(
      rot, current_tool_q=tool_q
    )
    self.assertEqual(len(grasp_q), 4)
    # Dot product should be positive (best aligned candidate chosen)
    dot = sum(a * b for a, b in zip(grasp_q, tool_q, strict=False))
    self.assertGreater(dot, 0.5)

  def test_live_world_orientation_snapping(self):
    # Live workpiece: quat [-0.5198, 0.4913, 0.486, -0.5022]
    wp_q = data_types.Quaternion([-0.5198, 0.4913, 0.486, -0.5022])
    wp_rot = data_types.Rotation3(wp_q)
    # Tool frame at view: quat [0.9162, -0.3672, 0.01239, -0.16]
    tool_q = (0.9162, -0.3672, 0.01239, -0.16)
    grasp_q = math_utils.compute_top_down_grasp_quaternion(
      wp_rot, current_tool_q=tool_q
    )
    # Candidate c1 is selected (closer than c2)
    self.assertAlmostEqual(grasp_q[0], 0.7228, places=3)
    self.assertAlmostEqual(grasp_q[1], -0.6911, places=3)
    self.assertAlmostEqual(grasp_q[2], 0.0, places=3)
    self.assertAlmostEqual(grasp_q[3], 0.0, places=3)
    # Dot product with tool orientation is > 0.9 (approx 47 deg difference)
    dot = sum(a * b for a, b in zip(grasp_q, tool_q, strict=False))
    self.assertGreater(dot, 0.90)

  def test_extract_in_plane_alignment_axis_local_x_vertical(self):
    # Rotate 90 deg around Y: local X is [0, 0, -1] (vertical)
    # abs_z = [1.0, 0.0, 0.0] -> selects local Y axis in-plane
    pose = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
    axis = math_utils.extract_in_plane_alignment_axis(pose)
    self.assertAlmostEqual(axis[0], 0.0, places=4)
    self.assertAlmostEqual(axis[1], 1.0, places=4)
    self.assertAlmostEqual(axis[2], 0.0, places=4)

  def test_extract_in_plane_alignment_axis_local_y_vertical_flat(self):
    # Rotate 90 deg around X: local Y is [0, 0, 1] (vertical)
    # abs_z = [0.0, 1.0, 0.0] -> selects local X axis in-plane
    pose = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
    axis = math_utils.extract_in_plane_alignment_axis(pose)
    self.assertAlmostEqual(axis[0], 1.0, places=4)
    self.assertAlmostEqual(axis[1], 0.0, places=4)
    self.assertAlmostEqual(axis[2], 0.0, places=4)

  def test_extract_in_plane_alignment_axis_local_z_vertical(self):
    # Identity rotation: local Z is [0, 0, 1] (vertical)
    # abs_z = [0.0, 0.0, 1.0] -> selects local X axis in-plane
    pose = (0.0, 0.0, 0.0, 1.0)
    axis = math_utils.extract_in_plane_alignment_axis(pose)
    self.assertAlmostEqual(axis[0], 1.0, places=4)
    self.assertAlmostEqual(axis[1], 0.0, places=4)
    self.assertAlmostEqual(axis[2], 0.0, places=4)

  def test_extract_in_plane_alignment_axis_with_pose3_object(self):
    rot_y = data_types.Rotation3.from_axis_angle([0.0, 1.0, 0.0], math.pi / 2.0)
    pose = data_types.Pose3(rot_y, [0.0, 0.0, 0.0])
    axis = math_utils.extract_in_plane_alignment_axis(pose)
    self.assertAlmostEqual(axis[0], 0.0, places=4)
    self.assertAlmostEqual(axis[1], 1.0, places=4)
    self.assertAlmostEqual(axis[2], 0.0, places=4)


if __name__ == "__main__":
  absltest.main()
