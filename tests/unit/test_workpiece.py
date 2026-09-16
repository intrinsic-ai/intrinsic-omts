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
    part = Workpiece(
      asset_id="ai.intrinsic.raw_stock_2x3x5", object_name="raw_stock_2x3x5"
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

  def test_scene_object_name_resolution(self):
    wp_asset = Workpiece(
      asset_id="ai.intrinsic.part_x", object_name="part_x", object_id=42
    )
    self.assertEqual(wp_asset.scene_object_name, "ai.intrinsic.part_x")
    self.assertEqual(wp_asset.object_name, "part_x")
    self.assertEqual(wp_asset.object_id, 42)


class WorkcellStateTest(absltest.TestCase):
  def test_workpiece_location_and_holder_tracking(self):
    from src.core.types import WorkpieceLocation
    from src.core.workcell import WorkcellState

    state = WorkcellState.create()
    self.assertEqual(state.workpiece_location, WorkpieceLocation.INFEED)
    self.assertEqual(state.workpiece_parent, "root")
    self.assertEqual(state.total_cycles, 1)
    self.assertEqual(state.cycles_remaining, 1)
    self.assertTrue(state.has_work_remaining)

    wp = Workpiece(
      asset_id="ai.intrinsic.raw_stock_2x3x5", object_name="raw_stock_2x3x5"
    )
    state.start_new_cycle(wp)
    self.assertEqual(state.workpiece_location, WorkpieceLocation.INFEED)
    self.assertEqual(state.workpiece_parent, "root")
    self.assertEqual(state.current_workpiece, wp)

    state.record_infeed_pick()
    self.assertEqual(state.workpiece_location, WorkpieceLocation.GRIPPER)
    self.assertEqual(state.workpiece_parent, "gripper")
    self.assertEqual(state.current_workpiece, wp)

    state.record_machine_load(vise_name="schunk_egp_64nnb")
    self.assertEqual(state.workpiece_location, WorkpieceLocation.VISE)
    self.assertEqual(state.workpiece_parent, "schunk_egp_64nnb")
    self.assertEqual(state.current_workpiece, wp)

    state.record_machine_unload()
    self.assertEqual(state.workpiece_location, WorkpieceLocation.GRIPPER)
    self.assertEqual(state.workpiece_parent, "gripper")
    self.assertEqual(state.current_workpiece, wp)

    state.record_cycle_success()
    self.assertEqual(state.workpiece_location, WorkpieceLocation.RETURNED)
    self.assertEqual(state.workpiece_parent, "root")
    self.assertIsNone(state.current_workpiece)
    self.assertEqual(state.cycles_completed, 1)
    self.assertEqual(state.cycles_remaining, 0)
    self.assertFalse(state.has_work_remaining)

  def test_unbounded_cycles(self):
    from src.core.workcell import WorkcellState

    state = WorkcellState.create(total_cycles=-1)
    self.assertIsNone(state.cycles_remaining)
    self.assertTrue(state.has_work_remaining)

    state = WorkcellState.create(total_cycles=0)
    self.assertIsNone(state.cycles_remaining)
    self.assertTrue(state.has_work_remaining)

  def test_non_pick_phase_clamps_cycles(self):
    from src.core.types import Phase
    from src.core.workcell import WorkcellState

    state = WorkcellState.create(total_cycles=5, phase=Phase.LOAD)
    self.assertEqual(state.total_cycles, 1)


if __name__ == "__main__":
  absltest.main()
