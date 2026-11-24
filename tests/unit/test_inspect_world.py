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

"""Unit tests for tools/world/inspect_world.py."""

import io
import types
from contextlib import redirect_stdout
from unittest import mock

from absl.testing import absltest

from tools.world import inspect_world


class _FakeFrame:
  """Minimal stand-in for an SBL Frame object."""

  def __init__(self, name: str) -> None:
    self.name = name


class _FakeWorldObject:
  """Minimal stand-in for an SBL WorldObject exposing .frames/.frame_names."""

  def __init__(
    self,
    name: str,
    frame_names: list[str] | None = None,
    joint_configurations: dict[str, list[float]] | None = None,
  ) -> None:
    self.name = name
    self._frames = [_FakeFrame(n) for n in (frame_names or [])]
    self.joint_configurations = joint_configurations or {}
    for frame in self._frames:
      setattr(self, frame.name, frame)

  @property
  def frames(self) -> list[_FakeFrame]:
    return list(self._frames)

  @property
  def frame_names(self) -> list[str]:
    return [f.name for f in self._frames]


class InspectWorldTest(absltest.TestCase):
  """Tests for dynamic frame discovery and output in inspect_world."""

  def test_get_object_frames_reads_world_object_frames(self) -> None:
    root = _FakeWorldObject("root", ["view", "inspect", "transit"])
    frames = inspect_world.get_object_frames(root)
    self.assertEqual(
      [name for name, _ in frames], ["inspect", "transit", "view"]
    )

  def test_get_object_frames_supports_frame_names_only(self) -> None:
    obj = types.SimpleNamespace(
      frame_names=["grasp", "inspect"],
      grasp=_FakeFrame("grasp"),
      inspect=_FakeFrame("inspect"),
    )
    frames = inspect_world.get_object_frames(obj)
    self.assertEqual([name for name, _ in frames], ["grasp", "inspect"])

  def test_get_object_frames_supports_legacy_list_frames(self) -> None:
    obj = mock.MagicMock(spec=["list_frames", "inspect"])
    obj.list_frames.return_value = ["inspect"]
    obj.inspect = _FakeFrame("inspect")
    frames = inspect_world.get_object_frames(obj)
    self.assertEqual([name for name, _ in frames], ["inspect"])

  def test_main_prints_all_root_and_object_frames(self) -> None:
    root = _FakeWorldObject("root", ["view", "inspect", "vise_pre_place"])
    gripper = _FakeWorldObject("gripper", ["tool_frame"])
    ur_module = _FakeWorldObject(
      "ur_module",
      ["flange"],
      joint_configurations={"home": [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]},
    )

    world = mock.MagicMock()
    world.root = root
    world.gripper = gripper
    world.ur_module = ur_module
    world.list_objects.return_value = [root, ur_module, gripper]
    world.get_transform.side_effect = lambda _a, b: f"T({b.name})"

    solution = mock.MagicMock()
    solution.world = world
    solution.skills = types.SimpleNamespace(
      ai=types.SimpleNamespace(intrinsic=object())
    )
    solution.resources = types.SimpleNamespace()

    buf = io.StringIO()
    with (
      mock.patch.object(
        inspect_world.deployments, "connect", return_value=solution
      ),
      redirect_stdout(buf),
    ):
      inspect_world.main(["--address=localhost:17080"])

    output = buf.getvalue()
    self.assertIn("inspect", output)
    self.assertIn("root.inspect: T(inspect)", output)
    self.assertIn("root.vise_pre_place: T(vise_pre_place)", output)
    self.assertIn("root.view: T(view)", output)
    self.assertIn("Tool Frame in root: T(tool_frame)", output)
    self.assertIn("Flange in root: T(flange)", output)


if __name__ == "__main__":
  absltest.main()
