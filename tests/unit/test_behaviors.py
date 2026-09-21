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

"""Unit tests for Behavior Tree construction and structure."""

import dataclasses
import os
import re
from typing import Any
from unittest import mock

from absl.testing import absltest
from google.protobuf import descriptor_pb2
from intrinsic.assets.build_defs import asset_pb2
from intrinsic.assets.proto import id_pb2
from intrinsic.executive.proto import behavior_tree_pb2, proto_builder_pb2
from intrinsic.resources.proto import resource_handle_pb2
from intrinsic.skills.proto import equipment_pb2, skill_manifest_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import proto_building as pb
from intrinsic.solutions import provided
from intrinsic.solutions.internal import resources as sbl_resources
from intrinsic.solutions.internal import skill_generation, skill_utils
from intrinsic.world.proto import object_world_service_pb2
from intrinsic.world.python import object_world_resources

from src.behaviors.load_machine import build_load_machine_subtree
from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.behaviors.machining import build_machining_handshake_subtree
from src.behaviors.pick import build_pick_from_infeed_subtree
from src.behaviors.return_infeed import build_return_to_infeed_subtree
from src.behaviors.unload_machine import build_unload_machine_subtree
from src.core.config import load_app_config
from src.core.infeed import PerceptionInfeedStrategy
from src.hardware.gripper import DioGripper, GripperInterface, RobotiqGripper
from src.hardware.machine import CncMachineInterface, DioCncMachine
from src.hardware.robot import RobotInterface, UrRobot
from src.hardware.vision import OrbbecVision, VisionInterface


def _make_mock_node_builder(name_prefix: str):
  def _builder(*args, **kwargs):
    del args
    node_name = kwargs.get("name", name_prefix)
    return bt.Sequence(name=node_name, children=[])

  return _builder


class BehaviorsTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.config = load_app_config("configs/omts/app_config.yaml")

    self.robot = mock.MagicMock(spec=RobotInterface)
    self.robot.build_move_joint_task.side_effect = _make_mock_node_builder(
      "Move Joint"
    )
    self.robot.build_move_cartesian_task.side_effect = _make_mock_node_builder(
      "Move Cartesian"
    )
    self.robot.build_move_blended_cartesian_task.side_effect = (
      _make_mock_node_builder("Move Blended")
    )
    self.robot.build_move_relative_cartesian_task.side_effect = (
      _make_mock_node_builder("Move Relative")
    )
    self.robot.build_move_to_contact_task.side_effect = _make_mock_node_builder(
      "Move Contact"
    )
    self.robot.build_attach_object_task.side_effect = _make_mock_node_builder(
      "Attach Object"
    )
    self.robot.build_detach_object_task.side_effect = _make_mock_node_builder(
      "Detach Object"
    )

    self.gripper = mock.MagicMock(spec=GripperInterface)
    self.gripper.build_open_task.side_effect = _make_mock_node_builder(
      "Open Gripper"
    )
    self.gripper.build_close_task.side_effect = _make_mock_node_builder(
      "Close Gripper"
    )

    self.machine = mock.MagicMock(spec=CncMachineInterface)
    self.machine.build_open_door_task.side_effect = _make_mock_node_builder(
      "Open Door"
    )
    self.machine.build_close_door_task.side_effect = _make_mock_node_builder(
      "Close Door"
    )
    self.machine.build_open_vise_task.side_effect = _make_mock_node_builder(
      "Open Vise"
    )
    self.machine.build_close_vise_task.side_effect = _make_mock_node_builder(
      "Close Vise"
    )
    self.machine.build_trigger_cycle_task.side_effect = _make_mock_node_builder(
      "Trigger Cycle"
    )
    self.machine.build_wait_cycle_complete_task.side_effect = (
      _make_mock_node_builder("Wait Cycle")
    )

    self.vision = mock.MagicMock(spec=VisionInterface)
    self.vision.build_capture_image_task.side_effect = _make_mock_node_builder(
      "Capture Image"
    )
    self.vision.build_perception_and_spawn_task.side_effect = (
      _make_mock_node_builder("Perception Pipeline")
    )

    self.infeed_strategy = PerceptionInfeedStrategy(
      config=self.config.vision,
      view_frame_name=self.config.frames.view_frame,
    )

  def _build_master_tree(self, num_cycles: int, tree_name: str = "Test Tree"):
    return build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      config=self.config,
      num_cycles_override=num_cycles,
      tree_name=tree_name,
    )

  def test_build_master_behavior_tree_single_cycle(self):
    tree = self._build_master_tree(
      num_cycles=1, tree_name="Test Single Cycle Master Tree"
    )
    self.assertIsNotNone(tree)
    self.assertEqual(tree.name, "Test Single Cycle Master Tree")
    self.assertIsInstance(tree.root, bt.Sequence)
    self.assertEqual(len(tree.root.children), 5)

  def test_build_master_behavior_tree_repeat_cycles(self):
    tree = self._build_master_tree(num_cycles=4)
    self.assertIsInstance(tree.root, bt.Loop)
    self.assertEqual(tree.root.max_times, 4)

  def test_build_master_behavior_tree_continuous_loop(self):
    tree = self._build_master_tree(num_cycles=0)
    self.assertIsInstance(tree.root, bt.Loop)
    self.assertEqual(tree.root.max_times, 0)

  def test_build_infeed_pick_subtree_with_machine_prep(self):
    pick_subtree = build_pick_from_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      config=self.config,
      machine=self.machine,
    )

    self.assertIsNotNone(pick_subtree)
    self.assertEqual(len(pick_subtree.children), 11)
    self.machine.build_open_door_task.assert_called_once_with(
      name="Open CNC Door"
    )
    self.machine.build_open_vise_task.assert_called_once_with(
      name="Open CNC Vise"
    )
    self.vision.build_perception_and_spawn_task.assert_called_once_with(
      approach_offset_z=0.08,
      parent_object="root",
      pregrasp_frame_name="pre_grasp",
      grasp_frame_name="grasp",
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
      name="Perception & Dynamic Grasp Frame Update Pipeline",
    )
    self.robot.build_attach_object_task.assert_called_once_with(
      object_name="raw_stock_2x3x5",
      name="Attach raw_stock_2x3x5 to Gripper",
    )
    self.robot.build_move_relative_cartesian_task.assert_has_calls(
      [
        mock.call(
          translation=(0.0, 0.0, -0.015),
          motion_type="LINEAR",
          exclude_collision=True,
          excluded_collision_objects=("raw_stock_2x3x5",),
          name="Linear Retract (1.5 cm, -Z Tool)",
        ),
        mock.call(
          translation=(0.0, 0.0, -0.015),
          motion_type="LINEAR",
          exclude_collision=True,
          excluded_collision_objects=("raw_stock_2x3x5",),
          name="Linear Retract after Attach (1.5 cm, -Z Tool)",
        ),
      ],
      any_order=False,
    )

  def test_build_load_machine_subtree_steps_and_detachment(self):
    load_subtree = build_load_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      config=self.config,
    )

    self.assertIsNotNone(load_subtree)
    self.assertEqual(len(load_subtree.children), 10)
    self.robot.build_move_blended_cartesian_task.assert_called_once()
    self.robot.build_detach_object_task.assert_called_once_with(
      object_name="raw_stock_2x3x5",
      name="Detach raw_stock_2x3x5 from Gripper",
    )
    expected_vise_pairs = [
      ("raw_stock_2x3x5", "schunk_egp_64nnb"),
      ("gripper", "schunk_egp_64nnb"),
      ("gripper", "raw_stock_2x3x5"),
    ]
    self.robot.build_move_cartesian_task.assert_has_calls(
      [
        mock.call(
          target_frame_name="vise_pre_place",
          target_object_name="root",
          motion_type="ANY",
          allow_tool_z_rotation=False,
          cone_opening_half_angle=0.0,
          moving_frame_offset=None,
          target_frame_offset=None,
          excluded_collision_pairs=expected_vise_pairs,
          name="Approach CNC Vise (root/vise_pre_place)",
        ),
        mock.call(
          target_frame_name="vise_pre_place",
          target_object_name="root",
          motion_type="LINEAR",
          allow_tool_z_rotation=False,
          cone_opening_half_angle=0.0,
          moving_frame_offset=None,
          target_frame_offset=None,
          excluded_collision_pairs=expected_vise_pairs,
          name="Retract Arm to Vise Approach (root/vise_pre_place)",
        ),
      ],
      any_order=False,
    )

  def test_build_machining_handshake_subtree_steps(self):
    machining_subtree = build_machining_handshake_subtree(
      robot=self.robot,
      machine=self.machine,
      config=self.config,
    )

    self.assertIsNotNone(machining_subtree)
    self.assertEqual(len(machining_subtree.children), 4)
    self.robot.build_move_cartesian_task.assert_called_once_with(
      target_frame_name="machine_approach",
      target_object_name="root",
      motion_type="ANY",
      allow_tool_z_rotation=False,
      cone_opening_half_angle=0.0,
      moving_frame_offset=None,
      target_frame_offset=None,
      excluded_collision_pairs=None,
      name="Move to Safe Standby (root/machine_approach)",
    )
    self.machine.build_close_door_task.assert_called_once_with(
      name="Close CNC Door"
    )
    self.machine.build_trigger_cycle_task.assert_called_once_with(
      name="Trigger CNC Cycle Start"
    )
    self.machine.build_wait_cycle_complete_task.assert_called_once_with(
      timeout_seconds=self.config.cycle.machining_timeout_seconds,
      name="Wait for CNC Cycle Complete",
    )

  def test_build_unload_machine_subtree_steps_and_attachment(self):
    unload_subtree = build_unload_machine_subtree(
      robot=self.robot,
      gripper=self.gripper,
      machine=self.machine,
      config=self.config,
    )

    self.assertIsNotNone(unload_subtree)
    self.assertEqual(len(unload_subtree.children), 10)
    self.robot.build_attach_object_task.assert_called_once_with(
      object_name="raw_stock_2x3x5",
      name="Attach raw_stock_2x3x5 to Gripper",
    )
    self.robot.build_move_relative_cartesian_task.assert_has_calls(
      [
        mock.call(
          translation=(0.0, 0.0, -0.015),
          motion_type="LINEAR",
          exclude_collision=True,
          excluded_collision_objects=("raw_stock_2x3x5",),
          name="Linear Retract (1.5 cm, -Z Tool)",
        ),
        mock.call(
          translation=(0.0, 0.0, -0.015),
          motion_type="LINEAR",
          exclude_collision=True,
          excluded_collision_objects=("raw_stock_2x3x5",),
          name="Linear Retract Clear of Vise (1.5 cm, -Z Tool)",
        ),
      ],
      any_order=False,
    )
    expected_vise_pairs = [
      ("raw_stock_2x3x5", "schunk_egp_64nnb"),
      ("gripper", "schunk_egp_64nnb"),
      ("gripper", "raw_stock_2x3x5"),
    ]
    self.robot.build_move_cartesian_task.assert_has_calls(
      [
        mock.call(
          target_frame_name="vise_pre_place",
          target_object_name="root",
          motion_type="ANY",
          allow_tool_z_rotation=False,
          cone_opening_half_angle=0.0,
          moving_frame_offset=None,
          target_frame_offset=None,
          excluded_collision_pairs=expected_vise_pairs,
          name="Approach Machined Part (root/vise_pre_place)",
        ),
      ],
      any_order=False,
    )

  def test_build_return_to_infeed_subtree_steps_and_detachment(self):
    return_subtree = build_return_to_infeed_subtree(
      robot=self.robot,
      gripper=self.gripper,
      config=self.config,
    )

    self.assertIsNotNone(return_subtree)
    self.assertEqual(len(return_subtree.children), 6)
    self.robot.build_move_blended_cartesian_task.assert_called_once()
    self.robot.build_detach_object_task.assert_called_once_with(
      object_name="raw_stock_2x3x5",
      name="Detach raw_stock_2x3x5 from Gripper",
    )
    self.robot.build_move_cartesian_task.assert_has_calls(
      [
        mock.call(
          target_frame_name="pre_grasp",
          target_object_name="root",
          motion_type="LINEAR",
          allow_tool_z_rotation=False,
          cone_opening_half_angle=0.0,
          moving_frame_offset=None,
          target_frame_offset=None,
          excluded_collision_pairs=[("gripper", "raw_stock_2x3x5")],
          name="Retract Arm from Table (root/pre_grasp)",
        ),
      ],
      any_order=False,
    )

  def test_build_master_behavior_tree_without_cnc_machine(self):
    lab_config = load_app_config("configs/lab_bb_01/app_config.yaml")
    tree = build_machine_tending_behavior_tree(
      robot=self.robot,
      gripper=self.gripper,
      machine=None,
      vision=self.vision,
      infeed_strategy=self.infeed_strategy,
      config=lab_config,
      num_cycles_override=1,
    )
    self.assertIsNotNone(tree)
    self.assertIsInstance(tree.root, bt.Sequence)
    self.assertEqual(len(tree.root.children), 5)
    self.vision.build_perception_and_spawn_task.assert_called_once_with(
      approach_offset_z=0.08,
      parent_object="root",
      pregrasp_frame_name="pre_grasp",
      grasp_frame_name="grasp",
      tool_object_name="gripper",
      tool_frame_name="tool_frame",
      name="Perception & Dynamic Grasp Frame Update Pipeline",
    )
    self.machine.build_open_door_task.assert_not_called()
    self.machine.build_close_door_task.assert_not_called()
    self.machine.build_open_vise_task.assert_not_called()
    self.machine.build_close_vise_task.assert_not_called()
    self.machine.build_trigger_cycle_task.assert_not_called()
    self.machine.build_wait_cycle_complete_task.assert_not_called()

  def test_all_subtrees_have_unique_child_names_and_no_legacy_prefixes(self):
    tree = self._build_master_tree(num_cycles=1)
    for subtree in tree.root.children:
      child_names = [child.name for child in subtree.children]
      self.assertEqual(
        len(child_names),
        len(set(child_names)),
        f"Duplicate child task names in {subtree.name}: {child_names}",
      )
      for name in child_names:
        self.assertNotIn("Step ", name)
        self.assertNotIn("Prep:", name)


# ---------------------------------------------------------------------------
# Tier 1 + Tier 2 Hermetic Proto Serialization & Solution Contract Tests
# ---------------------------------------------------------------------------

_SCALAR_PROTO_TYPES: dict[str, int] = {
  "double": descriptor_pb2.FieldDescriptorProto.TYPE_DOUBLE,
  "float": descriptor_pb2.FieldDescriptorProto.TYPE_FLOAT,
  "int64": descriptor_pb2.FieldDescriptorProto.TYPE_INT64,
  "uint64": descriptor_pb2.FieldDescriptorProto.TYPE_UINT64,
  "int32": descriptor_pb2.FieldDescriptorProto.TYPE_INT32,
  "fixed64": descriptor_pb2.FieldDescriptorProto.TYPE_FIXED64,
  "fixed32": descriptor_pb2.FieldDescriptorProto.TYPE_FIXED32,
  "bool": descriptor_pb2.FieldDescriptorProto.TYPE_BOOL,
  "string": descriptor_pb2.FieldDescriptorProto.TYPE_STRING,
  "bytes": descriptor_pb2.FieldDescriptorProto.TYPE_BYTES,
  "uint32": descriptor_pb2.FieldDescriptorProto.TYPE_UINT32,
  "sfixed32": descriptor_pb2.FieldDescriptorProto.TYPE_SFIXED32,
  "sfixed64": descriptor_pb2.FieldDescriptorProto.TYPE_SFIXED64,
  "sint32": descriptor_pb2.FieldDescriptorProto.TYPE_SINT32,
  "sint64": descriptor_pb2.FieldDescriptorProto.TYPE_SINT64,
}


def _find_runfile(suffix: str) -> str:
  """Resolves a data dependency file from the Bazel test runfiles tree."""
  candidates = [
    suffix,
    os.path.join("..", suffix),
    os.path.join("..", "intrinsic-core+", suffix),
  ]
  test_srcdir = os.environ.get("TEST_SRCDIR") or os.environ.get(
    "PYTHON_RUNFILES"
  )
  if test_srcdir:
    candidates.extend(
      [
        os.path.join(test_srcdir, "_main", suffix),
        os.path.join(test_srcdir, "intrinsic-core+", suffix),
        os.path.join(test_srcdir, suffix),
      ]
    )
  for candidate in candidates:
    if os.path.exists(candidate):
      return candidate

  search_roots = [r for r in (test_srcdir, "..", ".") if r and os.path.isdir(r)]
  for root in search_roots:
    for dirpath, _, filenames in os.walk(root):
      for fname in filenames:
        full_path = os.path.join(dirpath, fname)
        if full_path.endswith(suffix):
          return full_path
  raise FileNotFoundError(f"Could not find runfile matching '{suffix}'.")


def _find_all_runfiles(suffixes: tuple[str, ...]) -> list[str]:
  """Finds all files in the Bazel runfiles tree matching any of `suffixes`."""
  test_srcdir = os.environ.get("TEST_SRCDIR") or os.environ.get(
    "PYTHON_RUNFILES"
  )
  search_roots = [r for r in (test_srcdir, "..", ".") if r and os.path.isdir(r)]
  found: dict[str, str] = {}
  for root in search_roots:
    for dirpath, _, filenames in os.walk(root):
      for fname in filenames:
        if fname.endswith(suffixes):
          full_path = os.path.join(dirpath, fname)
          found[os.path.realpath(full_path)] = full_path
  return list(found.values())


def _extract_top_level_proto_messages(schema: str) -> list[tuple[str, str]]:
  """Extracts `(message_name, message_body)` pairs respecting balanced `{}` braces."""
  messages: list[tuple[str, str]] = []
  idx = 0
  for match in re.finditer(r"\bmessage\s+(\w+)\s*\{", schema):
    if match.start() < idx:
      continue
    msg_name = match.group(1)
    body_start = match.end()
    depth = 1
    pos = body_start
    while pos < len(schema) and depth > 0:
      if schema[pos] == "{":
        depth += 1
      elif schema[pos] == "}":
        depth -= 1
      pos += 1
    if depth == 0:
      messages.append((msg_name, schema[body_start : pos - 1]))
      idx = pos
  return messages


def _set_proto_field_type(
  field_desc: descriptor_pb2.FieldDescriptorProto, ftype: str
) -> None:
  if ftype in _SCALAR_PROTO_TYPES:
    field_desc.type = _SCALAR_PROTO_TYPES[ftype]
  else:
    field_desc.type = descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE
    field_desc.type_name = ftype


class FakeProtoBuilderStub:
  """In-process gRPC stub replacement for ProtoBuilder.Compile, Compose & GetWellKnownTypes."""

  def GetWellKnownTypes(
    self, request: proto_builder_pb2.GetWellKnownTypesRequest
  ) -> proto_builder_pb2.GetWellKnownTypesResponse:
    del request
    return proto_builder_pb2.GetWellKnownTypesResponse()

  def Compose(
    self, request: proto_builder_pb2.ProtoComposeRequest
  ) -> proto_builder_pb2.ProtoComposeResponse:
    file_proto = descriptor_pb2.FileDescriptorProto(
      name=request.proto_filename,
      package=request.proto_package,
      syntax="proto3",
      message_type=request.input_descriptor,
    )
    fds = descriptor_pb2.FileDescriptorSet(file=[file_proto])
    return proto_builder_pb2.ProtoComposeResponse(file_descriptor_set=fds)

  def Compile(
    self, request: proto_builder_pb2.ProtoCompileRequest
  ) -> proto_builder_pb2.ProtoCompileResponse:
    schema = request.proto_schema
    pkg_match = re.search(r"package\s+([a-zA-Z0-9_.]+)\s*;", schema)
    package_name = pkg_match.group(1) if pkg_match else ""

    file_proto = descriptor_pb2.FileDescriptorProto(
      name=request.proto_filename,
      package=package_name,
      syntax="proto3",
    )
    for msg_name, body in _extract_top_level_proto_messages(schema):
      msg_desc = file_proto.message_type.add(name=msg_name)
      # Strip bracketed field option blocks `[...] ` before parsing declarations
      clean_body = re.sub(r"\[[^\];]*\]", "", body)
      for raw_stmt in clean_body.split(";"):
        lines = [
          ln.strip()
          for ln in raw_stmt.splitlines()
          if ln.strip() and not ln.strip().startswith("//")
        ]
        if not lines:
          continue
        stmt = " ".join(lines)

        map_match = re.match(
          r"map<\s*([a-zA-Z0-9_.]+)\s*,\s*([a-zA-Z0-9_.]+)\s*>\s+(\w+)\s*=\s*(\d+)",
          stmt,
        )
        if map_match:
          key_type, val_type, fname, fnum = map_match.groups()
          entry_name = (
            "".join(part.capitalize() for part in fname.split("_")) + "Entry"
          )
          entry_desc = msg_desc.nested_type.add(name=entry_name)
          entry_desc.options.map_entry = True
          key_field = entry_desc.field.add(
            name="key",
            number=1,
            label=descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL,
          )
          _set_proto_field_type(key_field, key_type)
          val_field = entry_desc.field.add(
            name="value",
            number=2,
            label=descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL,
          )
          _set_proto_field_type(val_field, val_type)

          field_desc = msg_desc.field.add(
            name=fname,
            number=int(fnum),
            label=descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED,
            type=descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE,
            type_name=entry_name,
          )
          continue

        field_match = re.match(
          r"(?:(repeated|optional)\s+)?([a-zA-Z0-9_.]+)\s+(\w+)\s*=\s*(\d+)",
          stmt,
        )
        if not field_match:
          continue
        qualifier, ftype, fname, fnum = field_match.groups()
        field_desc = msg_desc.field.add(
          name=fname,
          number=int(fnum),
        )
        if qualifier == "repeated":
          field_desc.label = descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED
        else:
          field_desc.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
        _set_proto_field_type(field_desc, ftype)

    fds = descriptor_pb2.FileDescriptorSet(file=[file_proto])
    return proto_builder_pb2.ProtoCompileResponse(file_descriptor_set=fds)


def _make_resource_handle(
  name: str, capabilities: tuple[str, ...]
) -> provided.ResourceHandle:
  proto = resource_handle_pb2.ResourceHandle(name=name)
  for cap in capabilities:
    proto.resource_data[cap].SetInParent()
  return provided.ResourceHandle(proto)


class _FakeWorld:
  """Hermetic SBL object world populated from LocalSolution instances."""

  def __init__(self, object_names: tuple[str, ...]) -> None:
    ur_proto = object_world_service_pb2.Object(
      id="ur_module_id",
      name="ur_module",
      name_is_global_alias=True,
      type=object_world_service_pb2.ObjectType.KINEMATIC_OBJECT,
      object_component=object_world_service_pb2.ObjectComponent(),
      kinematic_object_component=object_world_service_pb2.KinematicObjectComponent(),
    )
    ur_proto.kinematic_object_component.named_joint_configurations.add(
      name="home",
      joint_positions=[0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    )
    self._objects: dict[str, object_world_resources.WorldObject] = {}
    for obj_name in object_names:
      if obj_name == "ur_module":
        self._objects["ur_module"] = object_world_resources.KinematicObject(
          ur_proto, stub=None
        )
        continue
      obj_proto = object_world_service_pb2.Object(
        id=f"{obj_name}_id",
        name=obj_name,
        name_is_global_alias=True,
        type=object_world_service_pb2.ObjectType.PHYSICAL_OBJECT,
        object_component=object_world_service_pb2.ObjectComponent(),
      )
      self._objects[obj_name] = object_world_resources.WorldObject(
        obj_proto, stub=None
      )

  def __getattr__(self, name: str) -> object_world_resources.WorldObject:
    if name in self._objects:
      return self._objects[name]
    raise AttributeError(f"World object '{name}' not found in FakeWorld.")

  def __getitem__(self, name: str) -> object_world_resources.WorldObject:
    if name in self._objects:
      return self._objects[name]
    raise KeyError(f"World object '{name}' not found in FakeWorld.")

  def __contains__(self, name: str) -> bool:
    return name in self._objects


class _FakeResources:
  """Mimics intrinsic.solutions.internal.resources.Resources (raises KeyError on missing)."""

  def __init__(self, handles: dict[str, provided.ResourceHandle]) -> None:
    self._resources = dict(handles)

  def __getattr__(self, name: str) -> provided.ResourceHandle:
    return self._resources[name]

  def __getitem__(self, name: str) -> provided.ResourceHandle:
    return self._resources[name]


class _SkillNamespace:
  """Nested skill namespace (e.g., solution.skills.ai.intrinsic.<skill>)."""

  def __init__(self, skill_classes: dict[str, type[Any]]) -> None:
    self.ai = mock.MagicMock()
    self.ai.intrinsic = mock.MagicMock()
    # Explicitly set dio_wait_for_input to None because DioCncMachine falls back
    # to dio_read_input when dio_wait_for_input is not in omts_solution.
    self.ai.intrinsic.dio_wait_for_input = None
    for skill_id, cls in skill_classes.items():
      _, _, short_name = skill_id.rpartition(".")
      setattr(self.ai.intrinsic, short_name, cls)


class FakeSolution:
  """Hermetic SBL Solution backed by real protobuf descriptors and GeneratedSkill classes."""

  def __init__(
    self,
    skill_infos: dict[str, skill_generation.SkillInfoImpl],
    world_objects: tuple[str, ...],
  ) -> None:
    self.proto_builder = pb.ProtoBuilder(stub=FakeProtoBuilderStub())
    self.world = _FakeWorld(world_objects)

    # Note: ur_module intentionally has RobotCalibrationDataService (NOT Icon2AdioPart)
    # to mirror omts_solution in production, while icon provides Icon2AdioPart.
    all_capabilities = {
      "ur_module": ("intrinsic_proto.world.RobotCalibrationDataService",),
      "icon": (
        "Icon2AdioPart",
        "intrinsic_proto.icon.v1.IconApi",
        "Icon2PositionPart",
        "Icon2ForceControlPart",
        "Icon2ForceTorqueSensorPart",
      ),
      "motion_planner_service": (
        "intrinsic_proto.motion_planning.MotionPlannerService",
      ),
      "world_service": ("intrinsic_proto.world.ObjectWorldService",),
      "orbbec_camera": ("CameraConfig",),
      "pose_estimator_service": (
        "intrinsic_proto.perception.v1.PoseEstimationService",
      ),
    }
    gripper_caps: set[str] = set()
    if "ai.intrinsic.gripper_cmd_skill" in skill_infos:
      for selector in skill_infos[
        "ai.intrinsic.gripper_cmd_skill"
      ].resource_selectors.values():
        gripper_caps.update(selector.capability_names)
    if gripper_caps:
      all_capabilities["hande_gripper"] = tuple(sorted(gripper_caps))

    active_instances = set(world_objects) | {"world_service"}
    handle_map = {
      name: _make_resource_handle(name, caps)
      for name, caps in all_capabilities.items()
      if name in active_instances
    }
    self.resources = _FakeResources(handle_map)

    skill_classes: dict[str, type[Any]] = {}
    for skill_id, info in skill_infos.items():
      compat_map: dict[str, provided.ResourceList] = {}
      for slot, selector in info.resource_selectors.items():
        slot_param = (
          slot + skill_utils.RESOURCE_SLOT_DECONFLICT_SUFFIX
          if slot in info.field_names
          else slot
        )
        req_caps = set(selector.capability_names)
        matching = [
          h for h in handle_map.values() if req_caps.issubset(set(h.types))
        ]
        compat_map[slot_param] = sbl_resources.ResourceListImpl(matching)
      skill_classes[skill_id] = skill_generation.gen_skill_class(
        info, compat_map
      )

    self.skills = _SkillNamespace(skill_classes)


_INTRINSIC_CORE_SKILL_SPECS: dict[
  str, tuple[str, str, dict[str, list[str]]]
] = {
  "ai.intrinsic.move_robot": (
    "MoveRobotParams",
    "MoveRobotResult",
    {
      "arm_part": ["Icon2PositionPart"],
      "motion_planner_service": [
        "intrinsic_proto.motion_planning.MotionPlannerService"
      ],
      "world_service": ["intrinsic_proto.world.ObjectWorldService"],
    },
  ),
  "ai.intrinsic.move_to_contact": (
    "MoveToContactParams",
    "MoveToContactResult",
    {"icon2": ["Icon2ForceControlPart", "Icon2PositionPart"]},
  ),
  "ai.intrinsic.attach_object_to_robot": (
    "AttachObjectToRobotParams",
    "Empty",
    {},
  ),
  "ai.intrinsic.detach_object": (
    "DetachObjectParams",
    "Empty",
    {},
  ),
  "ai.intrinsic.dio_set_output": (
    "DioSetOutputParams",
    "Empty",
    {"adio_part": ["Icon2AdioPart"]},
  ),
  "ai.intrinsic.dio_read_input": (
    "DioReadInputParams",
    "DioReadInputResult",
    {"adio_part": ["Icon2AdioPart"]},
  ),
  "ai.intrinsic.update_world": (
    "UpdateWorldParams",
    "Empty",
    {},
  ),
  "ai.intrinsic.capture_images": (
    "CaptureImagesParams",
    "CaptureImagesResult",
    {"camera": ["CameraConfig"]},
  ),
  "ai.intrinsic.estimate_pose_multi_view": (
    "EstimatePoseMultiViewParams",
    "EstimatePoseMultiViewResult",
    {
      "camera_1": ["CameraConfig"],
      "camera_2": ["CameraConfig"],
      "camera_3": ["CameraConfig"],
      "camera_4": ["CameraConfig"],
      "perception": ["intrinsic_proto.perception.v1.PoseEstimationService"],
    },
  ),
}


def _resolve_full_message_name(
  fds: descriptor_pb2.FileDescriptorSet, short_or_full_name: str
) -> str | None:
  """Resolves the exact package-qualified protobuf message name inside a FileDescriptorSet."""
  short_name = short_or_full_name.rpartition(".")[2] or short_or_full_name
  if not short_name or short_name == "Empty":
    return None
  for fd in reversed(fds.file):
    for msg in fd.message_type:
      if msg.name == short_name:
        return f"{fd.package}.{msg.name}" if fd.package else msg.name
  return None


def _load_real_skill_infos() -> dict[str, skill_generation.SkillInfoImpl]:
  """Builds real SkillInfoImpl objects from Bazel-compiled AssetInfo and SkillManifest protos."""
  asset_infos: dict[str, asset_pb2.AssetInfo] = {}
  for path in _find_all_runfiles((".asset_info.binpb",)):
    with open(path, "rb") as f:
      info = asset_pb2.AssetInfo.FromString(f.read())
    if info.id.package and info.id.name:
      asset_infos[f"{info.id.package}.{info.id.name}"] = info

  skill_manifests: dict[str, skill_manifest_pb2.SkillManifest] = {}
  for path in _find_all_runfiles(
    ("_augmented_manifest.pbbin", "_manifest.binpb")
  ):
    with open(path, "rb") as f:
      manifest = skill_manifest_pb2.SkillManifest.FromString(f.read())
    if manifest.id.package and manifest.id.name:
      skill_manifests[f"{manifest.id.package}.{manifest.id.name}"] = manifest

  skill_infos: dict[str, skill_generation.SkillInfoImpl] = {}
  for skill_id, asset_info in asset_infos.items():
    if skill_id in skill_manifests:
      manifest = skill_manifests[skill_id]
      param_full_name = manifest.parameter.message_full_name
      return_full_name = manifest.return_type.message_full_name
      default_params = (
        manifest.parameter.default_value
        if manifest.parameter.HasField("default_value")
        else None
      )
      resource_selectors = {
        slot: equipment_pb2.ResourceSelector(
          capability_names=list(sel.capability_names)
        )
        for slot, sel in manifest.dependencies.required_equipment.items()
      }
    elif skill_id in _INTRINSIC_CORE_SKILL_SPECS:
      param_short, return_short, req_equip = _INTRINSIC_CORE_SKILL_SPECS[
        skill_id
      ]
      param_full_name = _resolve_full_message_name(
        asset_info.file_descriptor_set, param_short
      )
      return_full_name = _resolve_full_message_name(
        asset_info.file_descriptor_set, return_short
      )
      default_params = None
      resource_selectors = {
        slot: equipment_pb2.ResourceSelector(capability_names=caps)
        for slot, caps in req_equip.items()
      }
    else:
      continue

    id_version = id_pb2.IdVersion(
      id=id_pb2.Id(package=asset_info.id.package, name=asset_info.id.name),
      version="0.0.1",
    )
    skill_infos[skill_id] = skill_generation.SkillInfoImpl(
      id_version=id_version,
      description=f"Hermetic skill {skill_id}",
      parameter_message_full_name=param_full_name,
      return_value_message_full_name=return_full_name,
      file_descriptor_set=asset_info.file_descriptor_set,
      default_params=default_params,
      recommended_config=None,
      resource_selectors=resource_selectors,
      proto_comments={},
      skill_type=provided.SkillType.REGULAR_SKILL,
      type_url_area=skill_utils.INTRINSIC_TYPE_URL_AREA_ASSETS,
    )
  return skill_infos


def _parse_build_solution_instances(
  build_content: str,
) -> dict[str, tuple[str, ...]]:
  """Parses intrinsic_asset_instance definitions and omts_solution.instances from BUILD."""
  if "assets = _OMTS_SKILL_ASSETS" not in build_content:
    raise AssertionError(
      "omts_solution in BUILD must include _OMTS_SKILL_ASSETS"
    )
  if "skills = _OMTS_SKILL_ASSETS" not in build_content:
    raise AssertionError(
      "omts_solution_manifest in BUILD must reference _OMTS_SKILL_ASSETS"
    )

  instance_names: dict[str, str] = {}
  for match in re.finditer(
    r"intrinsic_asset_instance\(\s*name\s*=\s*\"([^\"]+)\"(.*?)^\)",
    build_content,
    flags=re.DOTALL | re.MULTILINE,
  ):
    target_name, body = match.group(1), match.group(2)
    inst_match = re.search(r"instance_name\s*=\s*\"([^\"]+)\"", body)
    instance_names[f":{target_name}"] = (
      inst_match.group(1) if inst_match else target_name
    )

  sol_match = re.search(
    r"intrinsic_solution\(\s*name\s*=\s*\"omts_solution\"(.*?)^\)",
    build_content,
    flags=re.DOTALL | re.MULTILINE,
  )
  if not sol_match:
    raise AssertionError(
      "intrinsic_solution 'omts_solution' not found in BUILD"
    )
  sol_body = sol_match.group(1)

  inst_block_match = re.search(
    r"instances\s*=\s*\[(.*?)\]\s*\+\s*select\(\{\s*\":is_lab_bb_01\"\s*:\s*\[(.*?)\]\s*,\s*\"//conditions:default\"\s*:\s*\[(.*?)\]",
    sol_body,
    flags=re.DOTALL,
  )
  if not inst_block_match:
    raise AssertionError("Could not parse instances block of omts_solution")

  def _extract_names(block: str) -> list[str]:
    resolved = []
    for label in re.findall(r"\"(:[^\"]+)\"", block):
      if label not in instance_names:
        raise AssertionError(
          f"omts_solution references undeclared instance target '{label}'"
        )
      resolved.append(instance_names[label])
    return resolved

  common = ["root"] + _extract_names(inst_block_match.group(1))
  lab_extra = _extract_names(inst_block_match.group(2))
  omts_extra = _extract_names(inst_block_match.group(3))

  return {
    "omts": tuple(common + omts_extra),
    "lab_bb_01": tuple(common + lab_extra),
  }


def _iter_tree_nodes(
  node: behavior_tree_pb2.BehaviorTree.Node,
) -> list[behavior_tree_pb2.BehaviorTree.Node]:
  """Recursively yields all BehaviorTree.Node protos in a serialized BehaviorTree."""
  nodes = [node]
  if node.HasField("sequence"):
    for child in node.sequence.children:
      nodes.extend(_iter_tree_nodes(child))
  if node.HasField("parallel"):
    for child in node.parallel.children:
      nodes.extend(_iter_tree_nodes(child))
  if node.HasField("selector"):
    for child in node.selector.children:
      nodes.extend(_iter_tree_nodes(child))
    for branch in node.selector.branches:
      if branch.HasField("node"):
        nodes.extend(_iter_tree_nodes(branch.node))
  if node.HasField("fallback"):
    for child in node.fallback.children:
      nodes.extend(_iter_tree_nodes(child))
    for try_branch in node.fallback.tries:
      if try_branch.HasField("node"):
        nodes.extend(_iter_tree_nodes(try_branch.node))
  if node.HasField("loop"):
    if node.loop.HasField("do"):
      nodes.extend(_iter_tree_nodes(node.loop.do))
  if node.HasField("retry"):
    if node.retry.HasField("child"):
      nodes.extend(_iter_tree_nodes(node.retry.child))
    if node.retry.HasField("recovery"):
      nodes.extend(_iter_tree_nodes(node.retry.recovery))
  if node.HasField("branch"):
    if node.branch.HasField("then"):
      nodes.extend(_iter_tree_nodes(node.branch.then))
    if node.branch.HasField("else"):
      nodes.extend(_iter_tree_nodes(getattr(node.branch, "else")))
  if node.HasField("sub_tree") and node.sub_tree.HasField("tree"):
    if node.sub_tree.tree.HasField("root"):
      nodes.extend(_iter_tree_nodes(node.sub_tree.tree.root))
  return nodes


def _assert_platform_feature_flags_support_tree(
  bt_proto: behavior_tree_pb2.BehaviorTree,
  global_values_template_content: str,
) -> None:
  """Verifies platform Helm feature flags satisfy executive requirements of bt_proto."""
  all_nodes = _iter_tree_nodes(bt_proto.root)
  execute_code_nodes = [
    n
    for n in all_nodes
    if n.HasField("task") and n.task.HasField("execute_code")
  ]
  if execute_code_nodes:
    if not re.search(
      r'"enable_python_task_node"\s+true\b', global_values_template_content
    ):
      raise AssertionError(
        f"BehaviorTree '{bt_proto.name}' contains {len(execute_code_nodes)} "
        "bt.PythonScript (execute_code) task node(s), but "
        "@intrinsic-core//intrinsic/kubernetes:global-values-core.yaml.template "
        "does not enable '\"enable_python_task_node\" true' "
        "(regression from intrinsic-ai/insrc/pull/55094)."
      )


def _assert_no_parallel_universe_lock_conflicts(
  bt_proto: behavior_tree_pb2.BehaviorTree,
) -> None:
  """Verifies no bt.Parallel node pairs lock_the_universe skills with robot motion."""
  universe_locking_skills = {"ai.intrinsic.update_world"}
  motion_skills = {"ai.intrinsic.move_robot", "ai.intrinsic.move_to_contact"}

  for node in _iter_tree_nodes(bt_proto.root):
    if not node.HasField("parallel"):
      continue
    branch_skill_sets: list[set[str]] = []
    for child in node.parallel.children:
      branch_skills = {
        descendant.task.call_behavior.skill_id
        for descendant in _iter_tree_nodes(child)
        if descendant.HasField("task")
        and descendant.task.HasField("call_behavior")
      }
      branch_skill_sets.append(branch_skills)

    for i, skills_a in enumerate(branch_skill_sets):
      for j, skills_b in enumerate(branch_skill_sets):
        if i == j:
          continue
        if (skills_a & universe_locking_skills) and (skills_b & motion_skills):
          raise AssertionError(
            f"Executive StatusCode 18201 conflict in Parallel node '{node.name}': "
            f"branch {i} executes {skills_a & universe_locking_skills} "
            "(lock_the_universe: true) concurrently with branch "
            f"{j} executing {skills_b & motion_skills}."
          )


class HermeticSolutionAndBehaviorTreeContractTest(absltest.TestCase):
  """Tier 1 + Tier 2 hermetic validation of tree.proto, solution skills, and feature flags."""

  @classmethod
  def setUpClass(cls) -> None:
    super().setUpClass()
    cls.skill_infos = _load_real_skill_infos()
    build_path = _find_runfile("_main/BUILD")
    with open(build_path, encoding="utf-8") as f:
      cls.build_content = f.read()
    cls.cell_world_objects = _parse_build_solution_instances(cls.build_content)
    cls.global_values_content = '    "enable_python_task_node" true\n'

  def _build_real_tree_for_config(
    self,
    config_path: str,
    use_dio_gripper: bool = False,
    num_cycles_override: int | None = 1,
  ) -> tuple[bt.BehaviorTree, behavior_tree_pb2.BehaviorTree]:
    config = load_app_config(config_path)
    cell_key = "lab_bb_01" if "lab_bb_01" in config_path else "omts"
    world_objects = self.cell_world_objects[cell_key]
    solution = FakeSolution(self.skill_infos, world_objects=world_objects)
    robot = UrRobot(solution=solution, config=config.robot)
    if use_dio_gripper:
      dio_gripper_cfg = dataclasses.replace(
        config.gripper,
        dio_open_pin=0,
        dio_close_pin=1,
        dio_output_block_name="digital_out",
        dio_device_name="ur_module",
      )
      gripper: GripperInterface = DioGripper(
        solution=solution, config=dio_gripper_cfg
      )
    else:
      gripper = RobotiqGripper(solution=solution, config=config.gripper)
    machine = (
      DioCncMachine(solution=solution, config=config.machine)
      if config.machine is not None
      else None
    )
    vision = OrbbecVision(solution=solution, config=config.vision)
    infeed_strategy = PerceptionInfeedStrategy(
      config=config.vision,
      view_frame_name=config.frames.view_frame,
    )
    tree = build_machine_tending_behavior_tree(
      robot=robot,
      gripper=gripper,
      machine=machine,
      vision=vision,
      infeed_strategy=infeed_strategy,
      config=config,
      num_cycles_override=num_cycles_override,
      tree_name=f"Hermetic Tree ({config_path})",
    )
    tree.validate_id_uniqueness()
    tree._validate_signatures()
    return tree, tree.proto

  def test_tier1_omts_and_lab_bb_01_tree_proto_serialization_and_descriptors(
    self,
  ) -> None:
    expected_skills = {
      "ai.intrinsic.move_robot",
      "ai.intrinsic.move_to_contact",
      "ai.intrinsic.attach_object_to_robot",
      "ai.intrinsic.detach_object",
      "ai.intrinsic.dio_set_output",
      "ai.intrinsic.dio_read_input",
      "ai.intrinsic.update_world",
      "ai.intrinsic.gripper_cmd_skill",
      "ai.intrinsic.capture_images",
      "ai.intrinsic.estimate_pose_multi_view",
    }
    self.assertTrue(expected_skills.issubset(set(self.skill_infos.keys())))

    for config_path, use_dio, num_cycles in (
      ("configs/omts/app_config.yaml", False, 1),
      ("configs/omts/app_config.yaml", True, 2),
      ("configs/lab_bb_01/app_config.yaml", False, 0),
    ):
      with self.subTest(
        config_path=config_path, use_dio=use_dio, num_cycles=num_cycles
      ):
        _, bt_proto = self._build_real_tree_for_config(
          config_path,
          use_dio_gripper=use_dio,
          num_cycles_override=num_cycles,
        )
        all_nodes = _iter_tree_nodes(bt_proto.root)
        self.assertNotEmpty(all_nodes)
        if num_cycles != 1:
          self.assertTrue(bt_proto.root.HasField("loop"))

        # Verify OrbbecVision._create_frame_calc_script_task generated a real
        # FileDescriptorSet, packed parameters Any, and CEL blackboard assignments.
        calc_nodes = [
          n
          for n in all_nodes
          if n.HasField("task")
          and n.task.HasField("execute_code")
          and "Calculate & Update Dynamic Grasp" in n.name
        ]
        self.assertLen(calc_nodes, 1)
        exec_code = calc_nodes[0].task.execute_code
        self.assertNotEmpty(exec_code.parameter_message_full_name)
        self.assertNotEmpty(exec_code.file_descriptor_set.file)

        params_msg = skill_utils.create_message_from_file_descriptor_set(
          exec_code.file_descriptor_set,
          exec_code.parameter_message_full_name,
        )
        self.assertTrue(exec_code.parameters.proto.Unpack(params_msg))
        self.assertAlmostEqual(params_msg.approach_offset_z, 0.08, places=5)
        self.assertEqual(params_msg.parent_object, "root")
        self.assertEqual(params_msg.pregrasp_frame_name, "pre_grasp")
        self.assertEqual(params_msg.grasp_frame_name, "grasp")
        self.assertEqual(params_msg.camera_name, "orbbec_camera")
        self.assertEqual(params_msg.tool_object_name, "gripper")
        self.assertEqual(params_msg.tool_frame_name, "tool_frame")

        assignment_map = {
          a.path: a.cel_expression for a in exec_code.parameters.assign
        }
        for pose_field in (
          "pos_x",
          "pos_y",
          "pos_z",
          "ori_x",
          "ori_y",
          "ori_z",
          "ori_w",
        ):
          self.assertIn(pose_field, assignment_map)
          self.assertIn(
            "estimates[0].root_t_target.", assignment_map[pose_field]
          )

  def test_tier1_proto_builder_handles_field_metadata_options_and_maps(
    self,
  ) -> None:
    builder = pb.ProtoBuilder(stub=FakeProtoBuilderStub())
    sig = builder.create_signature_with_args(
      parameters=pb.MessageSpec(
        fields=[
          pb.FieldSpec(
            type="float",
            name="offset_m",
            number=1,
            unit="m",
            arg=0.05,
          ),
          pb.MapFieldSpec(
            key_type="string",
            value_type="int32",
            name="pin_map",
            number=2,
            arg={"open": 0, "close": 1},
          ),
          pb.FieldSpec(
            type="string",
            name="tail_field",
            number=3,
            arg="still_present",
          ),
        ]
      )
    )
    self.assertIsNotNone(sig.params_message)
    assert sig.params_message is not None
    self.assertAlmostEqual(sig.params_message.offset_m, 0.05, places=5)
    self.assertEqual(dict(sig.params_message.pin_map), {"open": 0, "close": 1})
    self.assertEqual(sig.params_message.tail_field, "still_present")

  def test_tier2_platform_feature_flag_contract_catches_disabled_python_node(
    self,
  ) -> None:
    _, bt_proto = self._build_real_tree_for_config(
      "configs/omts/app_config.yaml"
    )
    _assert_platform_feature_flags_support_tree(
      bt_proto, self.global_values_content
    )

    broken_template = self.global_values_content.replace(
      '"enable_python_task_node" true',
      '"enable_python_task_node" false',
    )
    with self.assertRaisesRegex(
      AssertionError, "enable_python_task_node.*insrc/pull/55094"
    ):
      _assert_platform_feature_flags_support_tree(bt_proto, broken_template)

  def test_tier2_solution_manifest_skill_and_parameter_schema_completeness(
    self,
  ) -> None:
    omts_instances = set(self.cell_world_objects["omts"])
    lab_instances = set(self.cell_world_objects["lab_bb_01"])
    self.assertIn("cnc_enclosure", omts_instances)
    self.assertIn("schunk_egp_64nnb", omts_instances)
    self.assertNotIn("cnc_enclosure", lab_instances)
    self.assertNotIn("schunk_egp_64nnb", lab_instances)

    registered_skill_ids = set(self.skill_infos.keys())

    for cell_key, config_path in (
      ("omts", "configs/omts/app_config.yaml"),
      ("lab_bb_01", "configs/lab_bb_01/app_config.yaml"),
    ):
      _, bt_proto = self._build_real_tree_for_config(config_path)
      behavior_calls = [
        n.task.call_behavior
        for n in _iter_tree_nodes(bt_proto.root)
        if n.HasField("task") and n.task.HasField("call_behavior")
      ]
      self.assertNotEmpty(behavior_calls)

      for call in behavior_calls:
        self.assertIn(
          call.skill_id,
          registered_skill_ids,
          f"Skill '{call.skill_id}' invoked by BehaviorTree is missing from "
          f"omts_solution ({cell_key})!",
        )
        skill_info = self.skill_infos[call.skill_id]
        param_msg = skill_info.create_param_message()
        self.assertTrue(
          call.parameters.Unpack(param_msg),
          f"Failed to unpack BehaviorCall.parameters for skill '{call.skill_id}' "
          f"into '{skill_info.parameter_message_full_name}'",
        )
        param_fields = skill_info.parameter_descriptor().fields_by_name
        for assignment in call.assignments:
          root_field = assignment.parameter_path.split("[")[0].split(".")[0]
          self.assertIn(
            root_field,
            param_fields,
            f"Invalid parameter_path '{assignment.parameter_path}' on "
            f"'{skill_info.parameter_message_full_name}'",
          )

  def test_tier2_executive_concurrency_contract_catches_status_18201(
    self,
  ) -> None:
    for config_path in (
      "configs/omts/app_config.yaml",
      "configs/lab_bb_01/app_config.yaml",
    ):
      _, bt_proto = self._build_real_tree_for_config(config_path)
      _assert_no_parallel_universe_lock_conflicts(bt_proto)

    config = load_app_config("configs/omts/app_config.yaml")
    solution = FakeSolution(
      self.skill_infos,
      world_objects=self.cell_world_objects["omts"],
    )
    robot = UrRobot(solution=solution, config=config.robot)
    assert config.machine is not None
    machine = DioCncMachine(solution=solution, config=config.machine)
    illegal_parallel_tree = bt.BehaviorTree(
      name="Illegal Parallel Tree",
      root=bt.Parallel(
        name="Concurrent Door & Motion",
        children=[
          machine.build_open_door_task(),
          robot.build_move_joint_task("home"),
        ],
      ),
    )
    with self.assertRaisesRegex(
      AssertionError, "Executive StatusCode 18201 conflict"
    ):
      _assert_no_parallel_universe_lock_conflicts(illegal_parallel_tree.proto)


if __name__ == "__main__":
  absltest.main()
