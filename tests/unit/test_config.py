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

"""Unit tests for strict cell YAML configuration loading."""

import dataclasses
import pathlib
import tempfile
from unittest import mock

import yaml
from absl import app
from absl.testing import absltest
from intrinsic.solutions import execution

from src import main as omts_main
from src.core.config import (
  AppConfig,
  GraspConfig,
  GripperConfig,
  load_app_config,
)
from src.core.types import GraspPlannerType, SimulationMode, Touchdown
from src.hardware.grasp_planners import create_grasp_planner


class ConfigTest(absltest.TestCase):
  def test_load_omts_app_config(self):
    config = load_app_config("configs/omts/app_config.yaml")
    self.assertIsInstance(config, AppConfig)
    self.assertEqual(config.cell_name, "omts")
    self.assertEqual(config.robot.arm_part_name, "ur_module")
    self.assertEqual(config.robot.enclosure_object_name, "enclosure")
    self.assertEqual(config.gripper.type, "robotiq")
    self.assertEqual(
      config.gripper.joint_name, "robotiq_hande_left_finger_joint"
    )
    self.assertEqual(config.gripper.open_position, 0.025)
    self.assertEqual(config.gripper.close_position, 0.0)
    self.assertIsNone(config.gripper.action_name)
    self.assertIsNone(config.gripper.dio_open_pin)
    self.assertIsNone(config.gripper.dio_close_pin)
    self.assertIsNone(config.gripper.dio_device_name)
    self.assertIsNone(config.gripper.dio_output_block_name)
    self.assertEqual(config.machine.enclosure_object_name, "cnc_enclosure")
    self.assertEqual(config.machine.vise_object_name, "schunk_egp_64nnb")
    self.assertEqual(config.machine.door_open_joints, (0.4,))
    self.assertEqual(config.machine.door_closed_joints, (0.0,))
    self.assertEqual(config.machine.vise_open_joints, (0.01, 0.01))
    self.assertEqual(config.machine.vise_closed_joints, (0.0, 0.0))
    self.assertEqual(config.machine.output_block_name, "standard_out")
    self.assertEqual(config.machine.input_block_name, "standard_in")
    self.assertEqual(config.frames.transit_frame, "transit")
    self.assertEqual(config.frames.preplace_vise_frame, "vise_pre_place")
    self.assertEqual(config.frames.place_vise_frame, "vise_place")
    self.assertEqual(config.vision.sensor_ids, (1, 4))
    self.assertEqual(config.vision.min_safe_z, 0.95)
    self.assertEqual(config.cycle.workpiece_id, "raw_stock_2x3x5")
    self.assertEqual(config.cycle.pick_touchdown_force_newtons, 10.0)
    self.assertEqual(config.cycle.load_seat_force_newtons, 10.0)
    self.assertEqual(config.cycle.unload_touchdown_force_newtons, 10.0)
    self.assertEqual(config.cycle.return_touchdown_force_newtons, 10.0)
    self.assertEqual(config.cycle.retract_distance_meters, 0.015)
    self.assertEqual(
      config.pick_touchdown,
      Touchdown(
        force_n=10.0, standoff_m=0.02, timeout_s=30.0, retract_after_m=0.015
      ),
    )
    self.assertEqual(
      config.load_touchdown,
      Touchdown(
        force_n=10.0, standoff_m=0.02, timeout_s=30.0, retract_after_m=0.0
      ),
    )
    self.assertEqual(
      config.unload_touchdown,
      Touchdown(
        force_n=10.0, standoff_m=0.035, timeout_s=30.0, retract_after_m=0.015
      ),
    )
    self.assertEqual(
      config.return_touchdown,
      Touchdown(
        force_n=10.0, standoff_m=0.02, timeout_s=30.0, retract_after_m=0.0
      ),
    )
    self.assertEqual(config.robot.enclosure_object_name, "enclosure")

  def test_load_lab_bb_01_app_config(self):
    config = load_app_config("configs/lab_bb_01/app_config.yaml")
    self.assertIsInstance(config, AppConfig)
    self.assertEqual(config.cell_name, "lab_bb_01")
    self.assertEqual(config.gripper.type, "robotiq")
    self.assertIsNone(config.gripper.dio_output_block_name)
    self.assertIsNone(config.machine)
    self.assertIsNone(config.frames.transit_frame)
    self.assertEqual(config.vision.min_safe_z, 0.60)
    self.assertEqual(config.cycle.pick_touchdown_force_newtons, 15.0)
    self.assertIsNone(config.robot.enclosure_object_name)

  def test_shipped_configs_select_the_cuboid_center_planner(self):
    for path in (
      "configs/omts/app_config.yaml",
      "configs/lab_bb_01/app_config.yaml",
    ):
      with self.subTest(path=path):
        config = load_app_config(path)
        self.assertEqual(config.grasp.planner, "cuboid_center")
        self.assertEqual(
          config.grasp.planner_type, GraspPlannerType.CUBOID_CENTER
        )

  def test_omitted_grasp_section_defaults_to_cuboid_center(self):
    raw = yaml.safe_load(
      pathlib.Path("configs/omts/app_config.yaml").read_text(encoding="utf-8")
    )
    del raw["grasp"]
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      yaml.safe_dump(raw, tmp)
      tmp_path = tmp.name

    config = load_app_config(tmp_path)
    self.assertEqual(config.grasp.planner_type, GraspPlannerType.CUBOID_CENTER)

  def test_unsupported_grasp_planner_fails_loudly(self):
    raw = yaml.safe_load(
      pathlib.Path("configs/omts/app_config.yaml").read_text(encoding="utf-8")
    )
    raw["grasp"] = {"planner": "moveit"}
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      yaml.safe_dump(raw, tmp)
      tmp_path = tmp.name

    with self.assertRaisesRegex(
      ValueError, "Unsupported grasp planner 'moveit'"
    ):
      load_app_config(tmp_path)

  def test_missing_required_section_fails_loudly(self):
    yaml_content = """
cell_name: "incomplete_cell"
robot:
  arm_part_name: "ur_module"
  tool_object_name: "gripper"
  tool_frame_name: "tool_frame"
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write(yaml_content)
      tmp_path = tmp.name

    with self.assertRaisesRegex(
      KeyError, "Missing required configuration section 'gripper'"
    ):
      load_app_config(tmp_path)

  def test_missing_required_field_fails_loudly(self):
    yaml_content = """
cell_name: "incomplete_cell"
robot:
  arm_part_name: "ur_module"
  tool_object_name: "gripper"
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write(yaml_content)
      tmp_path = tmp.name

    with self.assertRaisesRegex(
      KeyError, "Missing required configuration field.*tool_frame_name"
    ):
      load_app_config(tmp_path)

  def test_missing_robotiq_gripper_field_fails_loudly(self):
    yaml_content = """
cell_name: "incomplete_cell"
robot:
  arm_part_name: "ur_module"
  tool_object_name: "gripper"
  tool_frame_name: "tool_frame"
gripper:
  type: "robotiq"
  joint_name: "robotiq_hande_left_finger_joint"
  open_position: 0.025
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write(yaml_content)
      tmp_path = tmp.name

    with self.assertRaisesRegex(
      KeyError, "Missing required configuration field.*close_position"
    ):
      load_app_config(tmp_path)

  def test_missing_dio_gripper_fields_and_invalid_yaml_root(self):
    with self.assertRaisesRegex(
      KeyError, "Missing required configuration field.*dio_output_block_name"
    ):
      GripperConfig(type="dio", dio_open_pin=0, dio_close_pin=1)

    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write("- not_a_mapping\n")
      list_path = tmp.name
    with self.assertRaisesRegex(ValueError, "must contain a top-level mapping"):
      load_app_config(list_path)

    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
      tmp.write("robot:\n  arm_part_name: ur_module\n")
      no_cell_path = tmp.name
    with self.assertRaisesRegex(KeyError, "Missing required field 'cell_name'"):
      load_app_config(no_cell_path)

  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_run_machine_tending_pipeline_omts_and_dio_and_errors(
    self, mock_build_bt: mock.MagicMock, mock_connect: mock.MagicMock
  ):
    mock_solution = mock.MagicMock()
    mock_solution.resources = {
      "orbbec_camera": mock.MagicMock(),
      "pose_estimator_service": mock.MagicMock(),
    }
    mock_connect.return_value = mock_solution
    mock_tree = mock.MagicMock()
    mock_build_bt.return_value = mock_tree

    omts_cfg = load_app_config("configs/omts/app_config.yaml")
    omts_main.run_machine_tending_pipeline(
      solution_address="localhost:17080",
      config=omts_cfg,
      simulation_mode=SimulationMode.PREVIEW,
      num_cycles_override=2,
    )
    mock_solution.executive.run.assert_called_once_with(
      mock_tree,
      simulation_mode=execution.Executive.SimulationMode.PREVIEW,
    )

    # Test with DIO gripper and lab_bb_01 (no CNC machine)
    lab_cfg = load_app_config("configs/lab_bb_01/app_config.yaml")
    dio_cfg = dataclasses.replace(
      lab_cfg,
      gripper=GripperConfig(
        type="dio",
        dio_open_pin=0,
        dio_close_pin=1,
        dio_device_name="ur_module",
        dio_output_block_name="standard_out",
      ),
    )
    mock_solution.executive.run.reset_mock()
    omts_main.run_machine_tending_pipeline(
      solution_address="localhost:17080",
      config=dio_cfg,
    )
    self.assertIsNone(mock_build_bt.call_args.kwargs["machine"])
    mock_solution.executive.run.assert_called_once_with(
      mock_tree, simulation_mode=None
    )

    # Unsupported infeed_mode raises ValueError
    grid_cfg = dataclasses.replace(
      lab_cfg,
      vision=dataclasses.replace(lab_cfg.vision, infeed_mode="grid"),
    )
    with self.assertRaisesRegex(ValueError, "Unsupported infeed_mode 'grid'"):
      omts_main.run_machine_tending_pipeline(
        solution_address="localhost:17080",
        config=grid_cfg,
      )

    # Extra CLI positional argument raises UsageError
    with self.assertRaises(app.UsageError):
      omts_main.main(["main.py", "unexpected_arg"])


class GraspConfigTest(absltest.TestCase):
  """Unit tests for the grasp planner selection dataclass."""

  def test_defaults_to_cuboid_center(self):
    self.assertEqual(GraspConfig().planner_type, GraspPlannerType.CUBOID_CENTER)

  def test_planner_type_returns_the_matching_enum_member(self):
    config = GraspConfig(planner="cuboid_center")
    self.assertIs(config.planner_type, GraspPlannerType.CUBOID_CENTER)

  def test_rejects_an_unknown_planner(self):
    with self.assertRaises(ValueError):
      GraspConfig(planner="nonexistent_planner")

  def test_error_names_the_section_and_the_supported_values(self):
    with self.assertRaises(ValueError) as ctx:
      GraspConfig(planner="Cuboid_Center")
    message = str(ctx.exception)
    self.assertIn("'grasp'", message)
    self.assertIn("cuboid_center", message)

  def test_is_immutable(self):
    config = GraspConfig()
    with self.assertRaises(dataclasses.FrozenInstanceError):
      config.planner = "something_else"


class GraspPlannerFactoryTest(absltest.TestCase):
  """Unit tests for `create_grasp_planner`."""

  def test_cuboid_center_needs_no_planner_object(self):
    """Its frames come from the perception pipeline, not from a planner."""
    planner = create_grasp_planner(
      planner_type=GraspPlannerType.CUBOID_CENTER,
      solution=mock.MagicMock(),
      config=GraspConfig(),
    )
    self.assertIsNone(planner)

  def test_unhandled_backend_fails_loudly(self):
    """Guards against adding an enum member without a factory branch."""
    with self.assertRaisesRegex(ValueError, "Unhandled grasp planner"):
      create_grasp_planner(
        planner_type=mock.sentinel.future_planner,
        solution=mock.MagicMock(),
        config=GraspConfig(),
      )


class GraspPlannerSelectionTest(absltest.TestCase):
  """Tests how the entry point resolves which grasp planner to use."""

  def _run_pipeline(self, mock_build_bt, mock_connect, **kwargs):
    mock_solution = mock.MagicMock()
    mock_solution.resources = {
      "orbbec_camera": mock.MagicMock(),
      "pose_estimator_service": mock.MagicMock(),
    }
    mock_connect.return_value = mock_solution
    mock_build_bt.return_value = mock.MagicMock()
    config = load_app_config("configs/omts/app_config.yaml")
    omts_main.run_machine_tending_pipeline(
      solution_address="localhost:17080",
      config=config,
      **kwargs,
    )
    return mock_build_bt.call_args.kwargs

  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_cuboid_center_passes_no_planner_to_the_tree(
    self, mock_build_bt: mock.MagicMock, mock_connect: mock.MagicMock
  ):
    kwargs = self._run_pipeline(mock_build_bt, mock_connect)
    self.assertIsNone(kwargs["grasp_planner"])

  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_cli_override_is_accepted(
    self, mock_build_bt: mock.MagicMock, mock_connect: mock.MagicMock
  ):
    """The override path must work even when it names the current default."""
    kwargs = self._run_pipeline(
      mock_build_bt,
      mock_connect,
      grasp_planner_override=GraspPlannerType.CUBOID_CENTER,
    )
    self.assertIsNone(kwargs["grasp_planner"])

  @mock.patch("src.main.create_grasp_planner")
  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_cli_override_takes_precedence_over_config(
    self,
    mock_build_bt: mock.MagicMock,
    mock_connect: mock.MagicMock,
    mock_factory: mock.MagicMock,
  ):
    """A stand-in for a second backend, which does not exist yet.

    Overriding with `CUBOID_CENTER` would be indistinguishable from reading it
    out of the shipped config, so this asserts on a value the config can never
    produce. The factory is mocked, so it never has to recognize the stand-in;
    it only needs the `.value` that the entry point logs.
    """
    mock_factory.return_value = None
    override = mock.MagicMock()
    override.value = "other_planner"

    self._run_pipeline(
      mock_build_bt, mock_connect, grasp_planner_override=override
    )

    self.assertIs(mock_factory.call_args.kwargs["planner_type"], override)

  @mock.patch("src.main.create_grasp_planner")
  @mock.patch("src.main.deployments.connect")
  @mock.patch("src.main.build_machine_tending_behavior_tree")
  def test_config_is_used_when_no_override_is_given(
    self,
    mock_build_bt: mock.MagicMock,
    mock_connect: mock.MagicMock,
    mock_factory: mock.MagicMock,
  ):
    mock_factory.return_value = None

    self._run_pipeline(mock_build_bt, mock_connect)

    self.assertIs(
      mock_factory.call_args.kwargs["planner_type"],
      GraspPlannerType.CUBOID_CENTER,
    )


if __name__ == "__main__":
  absltest.main()
