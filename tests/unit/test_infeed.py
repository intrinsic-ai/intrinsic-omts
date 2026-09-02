"""Unit tests for Infeed strategies."""

from absl.testing import absltest
from src.core.infeed import GridInfeedStrategy, PerceptionInfeedStrategy
from src.core.tray import Tray
from src.core.types import InfeedMode, PartState


class InfeedStrategyTest(absltest.TestCase):

  def test_perception_infeed_strategy(self):
    strategy = PerceptionInfeedStrategy(
        camera_name="test_camera",
        pose_estimator_id="ai.intrinsic.test_estimator",
        scene_object_id="ai.intrinsic.test_object",
        view_frame_name="view",
    )
    self.assertEqual(strategy.mode, InfeedMode.PERCEPTION)

    part1 = strategy.get_target_part()
    self.assertIsNotNone(part1)
    self.assertEqual(part1.id, "workpiece_0")
    self.assertEqual(part1.state, PartState.RAW)

    part2 = strategy.get_target_part()
    self.assertEqual(part2.id, "workpiece_1")

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
