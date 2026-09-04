"""Unit tests for Tray domain model and slot indexing."""

from absl.testing import absltest

from src.core.tray import Tray
from src.core.types import SlotState


class TrayTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.tray = Tray(
      name="test_tray",
      rows=2,
      cols=3,
      pitch_x=0.05,
      pitch_y=0.08,
      origin_frame="test_tray_origin",
    )

  def test_initial_state_empty(self):
    self.assertEqual(self.tray.rows, 2)
    self.assertEqual(self.tray.cols, 3)
    self.assertEqual(self.tray.count_slots_by_state(SlotState.EMPTY), 6)
    self.assertIsNone(self.tray.get_next_available_slot(SlotState.OCCUPIED))

  def test_get_slot_relative_pose(self):
    pose_0_0 = self.tray.get_slot_relative_pose(row=0, col=0)
    self.assertAlmostEqual(pose_0_0.x, 0.0)
    self.assertAlmostEqual(pose_0_0.y, 0.0)

    pose_1_2 = self.tray.get_slot_relative_pose(row=1, col=2)
    self.assertAlmostEqual(pose_1_2.x, 0.10)  # 2 * 0.05
    self.assertAlmostEqual(pose_1_2.y, 0.08)  # 1 * 0.08

  def test_bounds_checking(self):
    with self.assertRaises(IndexError):
      self.tray.get_slot(row=2, col=0)
    with self.assertRaises(IndexError):
      self.tray.get_slot_relative_pose(row=0, col=3)

  def test_populate_all_slots(self):
    self.tray.populate_all_slots(part_cad_model="custom_model")
    self.assertEqual(self.tray.count_slots_by_state(SlotState.OCCUPIED), 6)

    slot_0_0 = self.tray.get_next_available_slot(SlotState.OCCUPIED)
    self.assertIsNotNone(slot_0_0)
    self.assertEqual(slot_0_0.row, 0)
    self.assertEqual(slot_0_0.col, 0)
    self.assertEqual(slot_0_0.workpiece.cad_model_name, "custom_model")


if __name__ == "__main__":
  absltest.main()
