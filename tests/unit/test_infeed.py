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

"""Unit tests for Infeed strategies."""

from absl.testing import absltest

from src.core.config import VisionConfig
from src.core.infeed import GridInfeedStrategy, PerceptionInfeedStrategy
from src.core.tray import Tray
from src.core.types import InfeedMode, PartState


class InfeedStrategyTest(absltest.TestCase):
  def test_perception_infeed_strategy(self):
    vision_config = VisionConfig(
      camera_name="test_camera",
      perception_service_name="test_perception_service",
      pose_estimator_id="ai.intrinsic.test_estimator",
      scene_object_id="ai.intrinsic.test_object",
      sensor_ids=(1, 4),
      min_num_instances=1,
      infeed_mode="perception",
      min_safe_z=0.95,
    )
    strategy = PerceptionInfeedStrategy(
      config=vision_config,
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
