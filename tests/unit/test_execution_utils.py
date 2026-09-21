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

"""Unit tests for executive execution mode mapping helpers."""

from absl.testing import absltest
from intrinsic.solutions import execution

from src.core.types import SimulationMode
from src.utils.execution_utils import (
  build_pose_estimator_proto,
  resolve_resource,
  to_executive_simulation_mode,
)


class ToExecutiveSimulationModeTest(absltest.TestCase):
  def test_none_maps_to_none(self):
    self.assertIsNone(to_executive_simulation_mode(None))

  def test_reality_maps_to_reality(self):
    self.assertEqual(
      to_executive_simulation_mode(SimulationMode.REALITY),
      execution.Executive.SimulationMode.REALITY,
    )

  def test_preview_maps_to_preview(self):
    self.assertEqual(
      to_executive_simulation_mode(SimulationMode.PREVIEW),
      execution.Executive.SimulationMode.PREVIEW,
    )

  def test_fast_preview_maps_to_fast_preview(self):
    self.assertEqual(
      to_executive_simulation_mode(SimulationMode.FAST_PREVIEW),
      execution.Executive.SimulationMode.FAST_PREVIEW,
    )

  def test_all_modes_are_mapped(self):
    for mode in SimulationMode:
      self.assertIsNotNone(to_executive_simulation_mode(mode))

  def test_unsupported_mode_raises(self):
    with self.assertRaises(ValueError):
      to_executive_simulation_mode("reality")

  def test_resolve_resource_direct_lookup(self):
    class MockHandle:
      types = ("CameraConfig",)

    class MockSolution:
      resources = {"orbbec_camera": MockHandle()}

    handle = resolve_resource(MockSolution(), "orbbec_camera", "CameraConfig")
    self.assertIsInstance(handle, MockHandle)

  def test_resolve_resource_capability_lookup(self):
    class MockHandle:
      types = ("CameraConfig",)

    class MockSolution:
      resources = {"other_camera": MockHandle()}

    handle = resolve_resource(MockSolution(), "missing", "CameraConfig")
    self.assertIsInstance(handle, MockHandle)

  def test_resolve_resource_not_found(self):
    class MockSolution:
      resources = {}

    with self.assertRaises(ValueError):
      resolve_resource(MockSolution(), "missing", "CameraConfig")

  def test_build_pose_estimator_proto(self):
    proto = build_pose_estimator_proto("ai.intrinsic.test_estimator")
    self.assertEqual(proto.package, "ai.intrinsic")
    self.assertEqual(proto.id, "test_estimator")


if __name__ == "__main__":
  absltest.main()
