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

"""Unit tests for Workpiece domain model state lifecycle."""

from absl.testing import absltest
from src.core.types import PartState, Pose3D
from src.core.workpiece import Workpiece


class WorkpieceTest(absltest.TestCase):

  def test_lifecycle_state_transitions(self):
    part = Workpiece(id="test_part_01", cad_model_name="raw_stock_2x3x5")
    self.assertEqual(part.state, PartState.RAW)
    self.assertIsNone(part.initial_infeed_pose)

    detected_pose = Pose3D(x=0.45, y=0.10, z=0.02)
    part.mark_detected(detected_pose)
    self.assertEqual(part.state, PartState.DETECTED)
    self.assertEqual(part.initial_infeed_pose, detected_pose)

    part.mark_picked()
    self.assertEqual(part.state, PartState.IN_TRANSIT)

    part.mark_loaded()
    self.assertEqual(part.state, PartState.IN_MACHINE)

    part.mark_machined()
    self.assertEqual(part.state, PartState.MACHINED)

    part.mark_finished()
    self.assertEqual(part.state, PartState.INSPECTED_OK)


if __name__ == "__main__":
  absltest.main()
