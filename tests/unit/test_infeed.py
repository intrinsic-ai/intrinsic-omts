"""Unit tests for Infeed strategies."""

from absl.testing import absltest
from src.core.infeed import GridInfeedStrategy, PerceptionInfeedStrategy
from src.core.tray import Tray
from src.core.types import InfeedMode, PartState


class InfeedStrategyTest(absltest.TestCase):

  def test_perception_infeed_strategy(self):
    strategy = PerceptionInfeedStrategy(
        camera_name="test_camera",
        estimator_name="test_estimator",
        view_frame_name="view",
    )
    self.assertEqual(strategy.mode, InfeedMode.PERCEPTION)

    part1 = strategy.get_target_part()
    self.assertIsNotNone(part1)
    self.assertEqual(part1.id, "vision_raw_stock_001")
    self.assertEqual(part1.state, PartState.RAW)

    part2 = strategy.get_target_part()
    self.assertEqual(part2.id, "vision_raw_stock_002")

  def test_grid_infeed_strategy(self):
    tray = Tray(
        name="test_infeed",
        rows=1,
        cols=2,
        pitch_x=0.05,
        pitch_y=0.05,
        origin_frame="test_infeed_origin",
    )
    tray.populate_all_slots()

    strategy = GridInfeedStrategy(tray=tray)
    self.assertEqual(strategy.mode, InfeedMode.GRID)

    part = strategy.get_target_part()
    self.assertIsNotNone(part)
    self.assertEqual(part.id, "test_infeed_r0_c0")


if __name__ == "__main__":
  absltest.main()
