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

"""Unit tests for apply_scene_updates CLI tool."""

from unittest import mock

from absl.testing import absltest

from tools.world.apply_scene_updates import (
  DEFAULT_UPDATE_FILES,
  main,
  parse_args,
)


class ApplySceneUpdatesTest(absltest.TestCase):
  """Tests for apply_scene_updates script."""

  def test_parse_args_defaults(self) -> None:
    """Tests default command-line argument values."""
    args = parse_args([])
    self.assertEqual(args.address, "localhost:17080")
    self.assertEqual(args.files, DEFAULT_UPDATE_FILES)
    self.assertTrue(args.reset_sim)

  def test_parse_args_no_reset_sim(self) -> None:
    """Tests disabling simulation reset via --no-reset_sim flag."""
    args = parse_args(["--no-reset_sim"])
    self.assertFalse(args.reset_sim)

  def test_parse_args_explicit_reset_sim(self) -> None:
    """Tests explicitly enabling simulation reset via --reset_sim flag."""
    args = parse_args(["--reset_sim"])
    self.assertTrue(args.reset_sim)

  @mock.patch("tools.world.apply_scene_updates.deployments.connect")
  @mock.patch("tools.world.apply_scene_updates.apply_pbtxt_file")
  def test_main_triggers_sim_reset_when_simulated(
    self, mock_apply_file: mock.MagicMock, mock_connect: mock.MagicMock
  ) -> None:
    """Tests that simulation reset is called when running in simulation mode."""
    mock_solution = mock.MagicMock()
    mock_solution.is_simulated = True
    mock_solution.simulator = mock.MagicMock()
    mock_connect.return_value = mock_solution

    main(["--files", "test.pbtxt"])

    mock_apply_file.assert_called_once_with(
      world=mock_solution.world, filepath="test.pbtxt"
    )
    mock_solution.simulator.reset.assert_called_once()

  @mock.patch("tools.world.apply_scene_updates.deployments.connect")
  @mock.patch("tools.world.apply_scene_updates.apply_pbtxt_file")
  def test_main_skips_sim_reset_when_disabled(
    self, mock_apply_file: mock.MagicMock, mock_connect: mock.MagicMock
  ) -> None:
    """Tests that simulation reset is skipped when --no-reset_sim is provided."""
    mock_solution = mock.MagicMock()
    mock_solution.is_simulated = True
    mock_solution.simulator = mock.MagicMock()
    mock_connect.return_value = mock_solution

    main(["--no-reset_sim", "--files", "test.pbtxt"])

    mock_apply_file.assert_called_once_with(
      world=mock_solution.world, filepath="test.pbtxt"
    )
    mock_solution.simulator.reset.assert_not_called()

  @mock.patch("tools.world.apply_scene_updates.deployments.connect")
  @mock.patch("tools.world.apply_scene_updates.apply_pbtxt_file")
  def test_main_skips_sim_reset_on_real_hardware(
    self, mock_apply_file: mock.MagicMock, mock_connect: mock.MagicMock
  ) -> None:
    """Tests that simulation reset is not triggered on real hardware."""
    mock_solution = mock.MagicMock()
    mock_solution.is_simulated = False
    mock_solution.simulator = None
    mock_connect.return_value = mock_solution

    main(["--files", "test.pbtxt"])

    mock_apply_file.assert_called_once_with(
      world=mock_solution.world, filepath="test.pbtxt"
    )


if __name__ == "__main__":
  absltest.main()
