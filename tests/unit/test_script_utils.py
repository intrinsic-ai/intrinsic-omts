"""Unit tests for script_utils module."""

from absl.testing import absltest
from src.utils import dynamic_frame_calculator
from src.utils.dynamic_frame_calculator import calculate_and_update_dynamic_frames
from src.utils.script_utils import load_python_script


class ScriptUtilsTest(absltest.TestCase):

  def test_load_python_script_from_function(self):
    code = load_python_script(calculate_and_update_dynamic_frames)
    self.assertIsInstance(code, str)
    self.assertIn("def calculate_and_update_dynamic_frames", code)
    self.assertIn("calculate_and_update_dynamic_frames(context, params)", code)
    self.assertIn("world = context.object_world", code)

  def test_load_python_script_from_module(self):
    code = load_python_script(
        dynamic_frame_calculator,
        function_name="calculate_and_update_dynamic_frames",
    )
    self.assertIsInstance(code, str)
    self.assertIn("def calculate_and_update_dynamic_frames", code)
    self.assertIn("calculate_and_update_dynamic_frames(context, params)", code)

  def test_load_python_script_from_module_string(self):
    code = load_python_script(
        "src.utils.dynamic_frame_calculator",
        function_name="calculate_and_update_dynamic_frames",
    )
    self.assertIsInstance(code, str)
    self.assertIn("def calculate_and_update_dynamic_frames", code)
    self.assertIn("calculate_and_update_dynamic_frames(context, params)", code)

  def test_load_python_script_without_call(self):
    code = load_python_script(
        dynamic_frame_calculator,
        call_args=None,
    )
    self.assertIsInstance(code, str)
    self.assertIn("def calculate_and_update_dynamic_frames", code)
    self.assertNotIn("calculate_and_update_dynamic_frames(context, params)", code)


if __name__ == "__main__":
  absltest.main()
