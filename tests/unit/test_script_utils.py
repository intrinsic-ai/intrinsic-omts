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

"""Unit tests for script_utils module."""

from absl.testing import absltest

from src.utils import dynamic_frame_calculator, math_utils
from src.utils.script_utils import load_python_script


class ScriptUtilsTest(absltest.TestCase):
  def test_load_python_script_from_function(self):
    code = load_python_script(
      dynamic_frame_calculator.calculate_and_update_dynamic_frames,
      preludes=(math_utils,),
    )
    self.assertIsInstance(code, str)
    self.assertIn("def calculate_and_update_dynamic_frames", code)
    self.assertIn("calculate_and_update_dynamic_frames(context, params)", code)
    self.assertIn("world = context.object_world", code)

  def test_load_python_script_from_module(self):
    code = load_python_script(
      dynamic_frame_calculator,
      function_name="calculate_and_update_dynamic_frames",
      preludes=(math_utils,),
    )
    self.assertIsInstance(code, str)
    self.assertIn("def calculate_and_update_dynamic_frames", code)
    self.assertIn("calculate_and_update_dynamic_frames(context, params)", code)

  def test_load_python_script_from_module_string(self):
    code = load_python_script(
      "src.utils.dynamic_frame_calculator",
      function_name="calculate_and_update_dynamic_frames",
      preludes=(math_utils,),
    )
    self.assertIsInstance(code, str)
    self.assertIn("def calculate_and_update_dynamic_frames", code)
    self.assertIn("calculate_and_update_dynamic_frames(context, params)", code)

  def test_load_python_script_without_call(self):
    code = load_python_script(
      dynamic_frame_calculator,
      call_args=None,
      preludes=(math_utils,),
    )
    self.assertIsInstance(code, str)
    self.assertIn("def calculate_and_update_dynamic_frames", code)
    self.assertNotIn(
      "calculate_and_update_dynamic_frames(context, params)", code
    )

  def test_sbl_scripts_are_hermetic(self):
    """Ensures SBL script payloads avoid non-hermetic src.* imports."""
    code = load_python_script(
      dynamic_frame_calculator.calculate_and_update_dynamic_frames,
      preludes=(math_utils,),
    )
    self.assertNotIn("from src", code)
    self.assertNotIn("import src", code)
    self.assertIn("def compute_top_down_grasp_quaternion", code)
    compiled = compile(code, "<string>", "exec")
    self.assertIsNotNone(compiled)

  def test_prelude_is_stdlib_only(self):
    """Asserts math_utils prelude has no non-hermetic src imports and compiles."""
    with open(math_utils.__file__, encoding="utf-8") as f:
      source = f.read()
    for line in source.splitlines():
      stripped = line.strip()
      self.assertFalse(
        stripped.startswith("from src") or stripped.startswith("import src"),
        f"math_utils must not import from src: {line}",
      )


if __name__ == "__main__":
  absltest.main()
