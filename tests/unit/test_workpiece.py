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

"""Unit tests for Workpiece and WorkcellState domain models."""

from absl.testing import absltest

from src.core.types import PartState, Phase, Pose3D
from src.core.workcell import WorkcellState
from src.core.workpiece import Workpiece


class WorkpieceTest(absltest.TestCase):
  def test_lifecycle_state_transitions(self):
    part = Workpiece(
      id="test_part_01",
      cad_model_name="raw_stock_2x3x5",
      asset_id="ai.intrinsic.raw_stock_2x3x5",
      object_name="raw_stock_2x3x5",
    )
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


class WorkcellStateTest(absltest.TestCase):
  def test_cycle_metrics(self):
    state = WorkcellState.create(total_cycles=2)
    self.assertEqual(state.total_cycles, 2)
    self.assertEqual(state.phase, Phase.PICK)

    wp = Workpiece(object_name="raw_stock_2x3x5")
    state.start_new_cycle(wp)
    self.assertEqual(state.current_workpiece, wp)

    state.record_cycle_success()
    self.assertIsNone(state.current_workpiece)
    self.assertEqual(state.cycles_completed, 1)

  def test_non_pick_phase_clamps_cycles(self):
    state = WorkcellState.create(total_cycles=5, phase=Phase.LOAD)
    self.assertEqual(state.total_cycles, 1)


if __name__ == "__main__":
  absltest.main()
