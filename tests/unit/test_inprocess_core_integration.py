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

"""In-process @intrinsic-core world, skill, and motion planning integration tests.

Exercises the serialized OMTS and lab_bb_01 Behavior Trees against the real
@intrinsic-core C++ ObjectWorldService (FakeWorldService),
ComposeResourceInstancesIntoWorld, MotionPlannerServiceInProcess,
MoveRobot::PlanTrajectory, PreplanMotionSkill, AttachObjectToRobotSkill,
DetachObjectSkill, and ObjectWorldClient without requiring k3s or Docker.
"""

import base64
import dataclasses
import json
import random
import subprocess
from typing import Any

import grpc
from absl.testing import absltest
from intrinsic.executive.proto import behavior_tree_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions.internal import skill_utils
from intrinsic.world.proto import object_world_service_pb2_grpc
from intrinsic.world.python import (
  object_world_client,
  object_world_ids,
  object_world_resources,
)

from src.behaviors.machine_tending_bt import build_machine_tending_behavior_tree
from src.core.config import load_app_config
from src.core.infeed import PerceptionInfeedStrategy
from src.hardware.gripper import DioGripper, GripperInterface, RobotiqGripper
from src.hardware.machine import DioCncMachine
from src.hardware.robot import Robot
from src.hardware.vision import OrbbecVision
from src.utils.dynamic_frame_calculator import (
  calculate_and_update_dynamic_frames,
)
from src.utils.random_placement import randomize_placement_frame
from tests.unit import test_behaviors

_CELL_SCENE_OBJECT_FIXTURES: dict[str, tuple[tuple[str, str], ...]] = {
  "omts": (
    ("ur_module", "tests/unit/fixtures/scene_objects/ur5e.pbtxt"),
    ("gripper", "tests/unit/fixtures/scene_objects/robotiq_gripper.pbtxt"),
    (
      "raw_stock_2x3x5",
      "tests/unit/fixtures/scene_objects/raw_stock_2x3x5.pbtxt",
    ),
    ("enclosure", "tests/unit/fixtures/scene_objects/omts_enclosure.pbtxt"),
    ("cnc_enclosure", "tests/unit/fixtures/scene_objects/cnc_enclosure.pbtxt"),
    (
      "schunk_egp_64nnb",
      "tests/unit/fixtures/scene_objects/schunk_egp_64nnb.pbtxt",
    ),
    ("camera_mount", "tests/unit/fixtures/scene_objects/camera_mount.pbtxt"),
    (
      "orbbec_camera",
      "tests/unit/fixtures/scene_objects/orbbec_gemini_335le.pbtxt",
    ),
  ),
  "lab_bb_01": (
    ("ur_module", "tests/unit/fixtures/scene_objects/ur3e.pbtxt"),
    ("gripper", "tests/unit/fixtures/scene_objects/robotiq_gripper.pbtxt"),
    (
      "raw_stock_2x3x5",
      "tests/unit/fixtures/scene_objects/raw_stock_2x3x5.pbtxt",
    ),
    ("enclosure", "tests/unit/fixtures/scene_objects/caw_enclosure.pbtxt"),
    (
      "orbbec_camera",
      "tests/unit/fixtures/scene_objects/orbbec_gemini_335le.pbtxt",
    ),
  ),
}

_CELL_WORLD_UPDATE_FILES: dict[str, tuple[str, ...]] = {
  "omts": (
    "configs/omts/ur_module.limits.updates.pbtxt",
    "configs/omts/ur_module.attachments.updates.pbtxt",
    "configs/omts/scene.updates.pbtxt",
    "configs/omts/align_robot.updates.pbtxt",
    "configs/omts/cnc_enclosure.updates.pbtxt",
    "configs/omts/schunk.updates.pbtxt",
    "configs/omts/camera_mount.updates.pbtxt",
    "configs/omts/orbbec_gemini.updates.pbtxt",
  ),
  "lab_bb_01": (
    "configs/lab_bb_01/ur_module.attachments.updates.pbtxt",
    "configs/lab_bb_01/scene.updates.pbtxt",
    "configs/lab_bb_01/align_robot.updates.pbtxt",
    "configs/lab_bb_01/orbbec_gemini.updates.pbtxt",
  ),
}


def _encode_len_delimited(field_number: int, payload: bytes) -> bytes:
  """Encodes a protobuf length-delimited field for `PreplanMotionParams`."""
  tag = (field_number << 3) | 2
  length = len(payload)
  varint = bytearray()
  while length >= 0x80:
    varint.append((length & 0x7F) | 0x80)
    length >>= 7
  varint.append(length)
  return bytes([tag]) + bytes(varint) + payload


class _InProcessWorldClient(object_world_client.ObjectWorldClient):
  """ObjectWorldClient subclass exposing `.root` for BT PythonScript nodes."""

  @property
  def root(self) -> object_world_resources.WorldObject:
    return self.get_object(object_world_ids.ROOT_OBJECT_NAME)


@dataclasses.dataclass
class _ScriptExecutionContext:
  """Minimal SBL PythonScript execution context backed by ObjectWorldClient."""

  object_world: _InProcessWorldClient


class _InProcessSkillRunner:
  """Subprocess client for `//tests/unit:inprocess_skill_runner`."""

  def __init__(self) -> None:
    runner_path = test_behaviors._find_runfile(
      "tests/unit/inprocess_skill_runner"
    )
    self._proc = subprocess.Popen(
      [runner_path],
      stdin=subprocess.PIPE,
      stdout=subprocess.PIPE,
      text=True,
      bufsize=1,
    )
    self._channel: grpc.Channel | None = None
    self.world_client: _InProcessWorldClient | None = None

  def _send_command(self, *parts: str) -> dict[str, Any]:
    if self._proc.stdin is None or self._proc.stdout is None:
      raise RuntimeError("inprocess_skill_runner stdio is closed.")
    line = "\t".join(parts) + "\n"
    self._proc.stdin.write(line)
    self._proc.stdin.flush()
    raw_resp = self._proc.stdout.readline()
    if not raw_resp:
      rc = self._proc.poll()
      raise RuntimeError(
        f"inprocess_skill_runner terminated unexpectedly (returncode={rc}) "
        f"on command: {parts[0]}"
      )
    return json.loads(raw_resp)

  def init_world(self, cell_key: str) -> _InProcessWorldClient:
    """Initializes the C++ FakeWorldService + MotionPlannerServiceInProcess."""
    if self._channel is not None:
      self._channel.close()
      self._channel = None

    obj_specs = [
      f"{name}={test_behaviors._find_runfile(rel_path)}"
      for name, rel_path in _CELL_SCENE_OBJECT_FIXTURES[cell_key]
    ]
    upd_paths = [
      test_behaviors._find_runfile(rel_path)
      for rel_path in _CELL_WORLD_UPDATE_FILES[cell_key]
    ]
    resp = self._send_command(
      "INIT_WORLD",
      ";".join(obj_specs),
      ";".join(upd_paths),
    )
    if not resp.get("ok"):
      raise RuntimeError(f"INIT_WORLD ({cell_key}) failed: {resp}")

    address = str(resp["address"])
    self._channel = grpc.insecure_channel(address)
    stub = object_world_service_pb2_grpc.ObjectWorldServiceStub(self._channel)
    self.world_client = _InProcessWorldClient("world", stub)
    return self.world_client

  def attach_object(self, params_bytes: bytes) -> dict[str, Any]:
    b64 = base64.b64encode(params_bytes).decode("ascii")
    return self._send_command("ATTACH_OBJECT", b64)

  def detach_object(self, params_bytes: bytes) -> dict[str, Any]:
    b64 = base64.b64encode(params_bytes).decode("ascii")
    return self._send_command("DETACH_OBJECT", b64)

  def update_world(self, params_bytes: bytes) -> dict[str, Any]:
    b64 = base64.b64encode(params_bytes).decode("ascii")
    return self._send_command("UPDATE_WORLD", b64)

  def move_robot(self, robot_name: str, params_bytes: bytes) -> dict[str, Any]:
    b64 = base64.b64encode(params_bytes).decode("ascii")
    return self._send_command("MOVE_ROBOT", robot_name, b64)

  def preplan_motion(
    self, robot_name: str, params_bytes: bytes
  ) -> dict[str, Any]:
    b64 = base64.b64encode(params_bytes).decode("ascii")
    return self._send_command("PREPLAN_MOTION", robot_name, b64)

  def close(self) -> None:
    if self._channel is not None:
      self._channel.close()
      self._channel = None
    if self._proc.poll() is None:
      try:
        self._send_command("QUIT")
      except Exception:  # pylint: disable=broad-exception-caught
        self._proc.kill()
      self._proc.wait(timeout=5)


@dataclasses.dataclass
class _CycleExecutionReport:
  """Summary of an in-process BehaviorTree execution against @intrinsic-core."""

  move_robot_calls: int = 0
  total_trajectory_states: int = 0
  attach_calls: int = 0
  detach_calls: int = 0
  update_world_calls: int = 0
  python_script_calls: int = 0
  last_end_joints: list[float] | None = None


class InProcessCoreIntegrationTest(absltest.TestCase):
  """Integration tests against in-process @intrinsic-core C++ services."""

  runner: _InProcessSkillRunner

  @classmethod
  def setUpClass(cls) -> None:
    super().setUpClass()
    cls.skill_infos = test_behaviors._load_real_skill_infos()
    build_path = test_behaviors._find_runfile("_main/BUILD")
    with open(build_path, encoding="utf-8") as f:
      cls.build_content = f.read()
    cls.cell_world_objects = test_behaviors._parse_build_solution_instances(
      cls.build_content
    )
    cls.runner = _InProcessSkillRunner()

  @classmethod
  def tearDownClass(cls) -> None:
    cls.runner.close()
    super().tearDownClass()

  def _build_tree_for_config(
    self,
    config_path: str,
    use_dio_gripper: bool = False,
    num_cycles_override: int | None = 1,
  ) -> tuple[bt.BehaviorTree, behavior_tree_pb2.BehaviorTree]:
    config = load_app_config(config_path)
    cell_key = "lab_bb_01" if "lab_bb_01" in config_path else "omts"
    world_objects = self.cell_world_objects[cell_key]
    solution = test_behaviors.FakeSolution(
      self.skill_infos, world_objects=world_objects
    )
    robot = Robot(solution=solution, config=config.robot)
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
      tree_name=f"InProcess Core Integration Tree ({config_path})",
    )
    tree.validate_id_uniqueness()
    tree._validate_signatures()
    return tree, tree.proto

  def _set_gripper_joint_positions(self, position: float) -> None:
    """Updates both prismatic finger joints on `gripper` in the belief world."""
    update_info = self.skill_infos["ai.intrinsic.update_world"]
    msg = update_info.create_param_message()
    msg.update.update_object_joints.object.by_name.object_name = "gripper"
    msg.update.update_object_joints.joint_positions.extend(
      [float(position), float(position)]
    )
    resp = self.runner.update_world(msg.SerializeToString())
    self.assertTrue(resp.get("ok"), f"Failed to update gripper joints: {resp}")

  def test_world_composition_and_updates_for_omts_and_lab_bb_01(self) -> None:
    expected_frames_by_cell = {
      "omts": {
        "view",
        "work_view",
        "grasp",
        "pre_grasp",
        "transit",
        "machine_approach",
        "vise_pre_place",
        "vise_place",
        "vise_view",
      },
      "lab_bb_01": {
        "view",
        "grasp",
        "pre_grasp",
        "machine_approach",
        "pre_place_vise",
        "place_vise",
      },
    }
    for cell_key, expected_frames in expected_frames_by_cell.items():
      with self.subTest(cell_key=cell_key):
        world = self.runner.init_world(cell_key)
        object_names = set(world.list_object_names())
        for obj_name, _ in _CELL_SCENE_OBJECT_FIXTURES[cell_key]:
          self.assertIn(obj_name, object_names)

        root_frames = set(world.root.frame_names)
        self.assertTrue(
          expected_frames.issubset(root_frames),
          f"Missing frames in {cell_key}: {expected_frames - root_frames}",
        )

        # Verify kinematic chain parenting applied by *.updates.pbtxt.
        self.assertEqual(world.gripper.parent_name, "ur_module")
        self.assertIn("tool_frame", set(world.gripper.frame_names))
        if cell_key == "omts":
          self.assertEqual(world.camera_mount.parent_name, "ur_module")
        self.assertEqual(world.orbbec_camera.parent_name, "ur_module")
        self.assertIn("sensor", set(world.orbbec_camera.frame_names))
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "root")

        ur_robot = world.get_kinematic_object("ur_module")
        self.assertLen(ur_robot.joint_positions, 6)
        if cell_key == "omts":
          app_limits = ur_robot.joint_application_limits
          self.assertAlmostEqual(
            app_limits.min_position.values[5], -3.6651914, places=5
          )
          self.assertAlmostEqual(
            app_limits.max_position.values[5], 3.8397244, places=5
          )

        # Verify ObjectWorldService rejects an invalid update referencing a
        # nonexistent frame (mirroring ComposeWorldUtil_InvalidUpdate).
        update_info = self.skill_infos["ai.intrinsic.update_world"]
        invalid_update = update_info.create_param_message()
        invalid_update.update.delete_frame.frame.by_name.object_name = "root"
        invalid_update.update.delete_frame.frame.by_name.frame_name = (
          "unknown_frame_xyz"
        )
        invalid_resp = self.runner.update_world(
          invalid_update.SerializeToString()
        )
        self.assertFalse(invalid_resp["ok"])
        self.assertEqual(invalid_resp["code"], 5)  # StatusCode::kNotFound
        self.assertIn("unknown_frame_xyz", invalid_resp["message"])

  def test_attach_and_detach_object_entity_filter_and_collision_invariants(
    self,
  ) -> None:
    for cell_key, config_path in (
      ("omts", "configs/omts/app_config.yaml"),
      ("lab_bb_01", "configs/lab_bb_01/app_config.yaml"),
    ):
      with self.subTest(cell_key=cell_key):
        config = load_app_config(config_path)
        world = self.runner.init_world(cell_key)
        _, bt_proto = self._build_tree_for_config(config_path)
        all_nodes = test_behaviors._iter_tree_nodes(bt_proto.root)

        attach_nodes = [
          n
          for n in all_nodes
          if n.HasField("task")
          and n.task.HasField("call_behavior")
          and n.task.call_behavior.skill_id
          == "ai.intrinsic.attach_object_to_robot"
        ]
        detach_nodes = [
          n
          for n in all_nodes
          if n.HasField("task")
          and n.task.HasField("call_behavior")
          and n.task.call_behavior.skill_id == "ai.intrinsic.detach_object"
        ]
        self.assertLen(attach_nodes, 2)
        self.assertLen(detach_nodes, 2)

        attach_info = self.skill_infos["ai.intrinsic.attach_object_to_robot"]
        detach_info = self.skill_infos["ai.intrinsic.detach_object"]

        # 1. Verify all BT attach/detach calls target tool_object_name
        # ("gripper") and cycle.workpiece_id ("raw_stock_2x3x5").
        for attach_node in attach_nodes:
          attach_msg = attach_info.create_param_message()
          self.assertTrue(
            attach_node.task.call_behavior.parameters.Unpack(attach_msg)
          )
          self.assertEqual(
            attach_msg.gripper_entity.by_name.object_name,
            config.robot.tool_object_name,
          )
          self.assertEqual(
            attach_msg.object_entity.by_name.object_name,
            config.cycle.workpiece_id,
          )
        for detach_node in detach_nodes:
          detach_msg = detach_info.create_param_message()
          self.assertTrue(
            detach_node.task.call_behavior.parameters.Unpack(detach_msg)
          )
          self.assertEqual(
            detach_msg.gripper_entity.by_name.object_name,
            config.robot.tool_object_name,
          )
          self.assertEqual(
            detach_msg.object_entity.by_name.object_name,
            config.cycle.workpiece_id,
          )

        # 2. Verify multi-finger gripper attachment attaches to "gripper"'s
        # base entity ("gripper_body"), disables collisions between "gripper"
        # and "raw_stock_2x3x5", and preserves relative transform
        # (matching AttachObjectToRobotTest.ExecuteWithMultiFingerGripper).
        gripper_t_stock_before = world.get_transform(
          world.gripper, world.raw_stock_2x3x5
        )
        attach_resp = self.runner.attach_object(
          attach_nodes[0].task.call_behavior.parameters.value
        )
        self.assertTrue(attach_resp["ok"], f"Attach failed: {attach_resp}")
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "gripper")
        self.assertEqual(attach_resp["parent_entity"], "gripper_body")
        self.assertTrue(attach_resp["collisions_disabled"])
        gripper_t_stock_after = world.get_transform(
          world.gripper, world.raw_stock_2x3x5
        )
        for i in range(3):
          self.assertAlmostEqual(
            float(gripper_t_stock_before.translation[i]),
            float(gripper_t_stock_after.translation[i]),
            places=5,
          )

        # Detaching with a mismatched parent ("ur_module" while attached to
        # "gripper") must be rejected by DetachObjectSkill::ExecuteImpl.
        wrong_detach_msg = detach_info.create_param_message()
        self.assertTrue(
          detach_nodes[0].task.call_behavior.parameters.Unpack(wrong_detach_msg)
        )
        wrong_detach_msg.gripper_entity.by_name.object_name = (
          config.robot.arm_part_name
        )
        wrong_detach_resp = self.runner.detach_object(
          wrong_detach_msg.SerializeToString()
        )
        self.assertFalse(wrong_detach_resp["ok"])
        self.assertEqual(wrong_detach_resp["code"], 3)
        self.assertIn(
          "not a child of the 'gripper_entity'",
          wrong_detach_resp["message"],
        )

        # Detaching with skip_collision_enable=True preserves the collision
        # exclusion rule between "gripper" and "raw_stock_2x3x5" (matching
        # DetachObjectTest.Execute with SkipCollisionEnable=true).
        skip_enable_detach_msg = detach_info.create_param_message()
        self.assertTrue(
          detach_nodes[0].task.call_behavior.parameters.Unpack(
            skip_enable_detach_msg
          )
        )
        skip_enable_detach_msg.skip_collision_enable = True
        skip_enable_resp = self.runner.detach_object(
          skip_enable_detach_msg.SerializeToString()
        )
        self.assertTrue(skip_enable_resp["ok"])
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "root")
        self.assertTrue(skip_enable_resp["collisions_disabled"])

        # Re-attach and detach with the BT's DetachObjectParams
        # (skip_collision_enable=False) and verify parenting returns to "root"
        # and collisions between "gripper" and "raw_stock_2x3x5" are re-enabled.
        self.assertTrue(
          self.runner.attach_object(
            attach_nodes[0].task.call_behavior.parameters.value
          )["ok"]
        )
        detach_resp = self.runner.detach_object(
          detach_nodes[0].task.call_behavior.parameters.value
        )
        self.assertTrue(detach_resp["ok"], f"Detach failed: {detach_resp}")
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "root")
        self.assertFalse(detach_resp["collisions_disabled"])

        # 3. Verify attaching directly to a robot arm ("ur_module", which has
        # ISO flange frame "flange") attaches to its final link ("wrist_3_link")
        # rather than "base_link" (matching AttachObjectToRobotTest.Execute).
        robot_attach_msg = attach_info.create_param_message()
        self.assertTrue(
          attach_nodes[0].task.call_behavior.parameters.Unpack(robot_attach_msg)
        )
        robot_attach_msg.gripper_entity.by_name.object_name = (
          config.robot.arm_part_name
        )
        robot_attach_resp = self.runner.attach_object(
          robot_attach_msg.SerializeToString()
        )
        self.assertTrue(
          robot_attach_resp["ok"], f"Robot attach failed: {robot_attach_resp}"
        )
        self.assertEqual(
          world.raw_stock_2x3x5.parent_name, config.robot.arm_part_name
        )
        self.assertEqual(robot_attach_resp["parent_entity"], "wrist_3_link")
        self.assertTrue(robot_attach_resp["collisions_disabled"])

        robot_detach_resp = self.runner.detach_object(
          wrong_detach_msg.SerializeToString()
        )
        self.assertTrue(robot_detach_resp["ok"])
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "root")
        self.assertFalse(robot_detach_resp["collisions_disabled"])

        # 4. Verify ObjectWorldService rejects reparenting to a multi-final-
        # entity gripper when include_final_entity=True, while succeeding when
        # include_base_entity=True.
        update_info = self.skill_infos["ai.intrinsic.update_world"]
        final_entity_reparent = update_info.create_param_message()
        reparent_req = final_entity_reparent.update.reparent_object
        reparent_req.object.by_name.object_name = config.cycle.workpiece_id
        reparent_req.new_parent.reference.by_name.object_name = (
          config.robot.tool_object_name
        )
        reparent_req.new_parent.entity_filter.include_final_entity = True
        final_entity_resp = self.runner.update_world(
          final_entity_reparent.SerializeToString()
        )
        self.assertFalse(final_entity_resp["ok"])
        self.assertEqual(final_entity_resp["code"], 3)
        self.assertIn(
          "can only be parented to a single entity",
          final_entity_resp["message"],
        )
        self.assertIn("gripper_finger1", final_entity_resp["message"])
        self.assertIn("gripper_finger2", final_entity_resp["message"])

        # 5. Verify world collision state interaction with MotionPlannerService
        # at the seated grasp pose when gripper fingers close on
        # raw_stock_2x3x5:
        # - While detached (collisions enabled between "gripper" and
        #   "raw_stock_2x3x5"), retracting without segment collision_rules
        #   fails collision checking in MotionPlannerServiceInProcess.
        # - Once attached to "gripper" via AttachObjectToRobotSkill (which
        #   disables world collisions between "gripper" and "raw_stock_2x3x5"),
        #   the exact same retract move without segment collision_rules
        #   succeeds.
        move_by_name = {
          n.name: n.task.call_behavior.parameters.value
          for n in all_nodes
          if n.HasField("task")
          and n.task.HasField("call_behavior")
          and n.task.call_behavior.skill_id == "ai.intrinsic.move_robot"
        }
        script_nodes = [
          n
          for n in all_nodes
          if n.HasField("task")
          and n.task.HasField("execute_code")
          and "Calculate & Update Dynamic Grasp" in n.name
        ]
        self.assertNotEmpty(script_nodes)

        self.assertTrue(
          self.runner.move_robot(
            "ur_module",
            move_by_name["Move to View Frame (root/view)"],
          )["ok"]
        )
        self._execute_bt_node(script_nodes[0], world, _CycleExecutionReport())
        self.assertTrue(
          self.runner.move_robot(
            "ur_module",
            move_by_name["Move to Dynamic Pre-Grasp (root/pre_grasp)"],
          )["ok"]
        )
        self.assertTrue(
          self.runner.move_robot(
            "ur_module",
            move_by_name[
              "Infeed Pick: Linear Approach to Standoff (root/grasp)"
            ],
          )["ok"]
        )
        self.assertTrue(
          self.runner.move_robot(
            "ur_module",
            move_by_name["Infeed Pick: Fallback Linear Seat (root/grasp)"],
          )["ok"]
        )

        # Simulate closing the gripper fingers on the workpiece at the seated
        # grasp pose (`Close Gripper (Grasp Part)`).
        self._set_gripper_joint_positions(0.025)

        move_info = self.skill_infos["ai.intrinsic.move_robot"]
        retract_msg = move_info.create_param_message()
        self.assertTrue(
          retract_msg.ParseFromString(
            move_by_name["Retract to Dynamic Pre-Grasp (root/pre_grasp)"]
          )
        )
        for seg in retract_msg.motion_segments:
          seg.collision_settings.ClearField("collision_rules")
        retract_relying_on_world_attach = retract_msg.SerializeToString()

        # Before attaching to "gripper", world collisions between "gripper" and
        # "raw_stock_2x3x5" are enabled, so retracting without segment
        # collision_rules fails in MotionPlannerServiceInProcess.
        retract_while_unattached = self.runner.move_robot(
          "ur_module", retract_relying_on_world_attach
        )
        self.assertFalse(
          retract_while_unattached["ok"],
          "Expected retract move without collision rules to fail before "
          f"attaching to gripper: {retract_while_unattached}",
        )

        # If AttachObjectToRobot + DetachObject(skip_collision_enable=True) is
        # called with "ur_module" instead of "gripper", only ("ur_module",
        # "raw_stock_2x3x5") collisions are disabled in the world while
        # ("gripper", "raw_stock_2x3x5") collisions remain enabled, so
        # retracting without segment collision_rules still fails.
        self.assertTrue(
          self.runner.attach_object(robot_attach_msg.SerializeToString())["ok"]
        )
        robot_skip_detach_msg = detach_info.create_param_message()
        self.assertTrue(
          detach_nodes[0].task.call_behavior.parameters.Unpack(
            robot_skip_detach_msg
          )
        )
        robot_skip_detach_msg.gripper_entity.by_name.object_name = (
          config.robot.arm_part_name
        )
        robot_skip_detach_msg.skip_collision_enable = True
        self.assertTrue(
          self.runner.detach_object(robot_skip_detach_msg.SerializeToString())[
            "ok"
          ]
        )
        retract_with_only_robot_excluded = self.runner.move_robot(
          "ur_module", retract_relying_on_world_attach
        )
        self.assertFalse(
          retract_with_only_robot_excluded["ok"],
          "Expected retract move to fail when only ur_module (not gripper) "
          f"collisions are disabled: {retract_with_only_robot_excluded}",
        )
        # Clean up the ur_module collision exclusion.
        self.assertTrue(
          self.runner.attach_object(robot_attach_msg.SerializeToString())["ok"]
        )
        self.assertTrue(
          self.runner.detach_object(wrong_detach_msg.SerializeToString())["ok"]
        )

        # Attach to "gripper" (as the BT does) and verify the exact same
        # retract move succeeds because AttachObjectToRobotSkill disabled world
        # collisions between "gripper" and "raw_stock_2x3x5".
        self.assertTrue(
          self.runner.attach_object(
            attach_nodes[0].task.call_behavior.parameters.value
          )["ok"]
        )
        retract_with_gripper_parent = self.runner.move_robot(
          "ur_module", retract_relying_on_world_attach
        )
        self.assertTrue(
          retract_with_gripper_parent["ok"],
          f"Expected retract move to succeed when attached to gripper: "
          f"{retract_with_gripper_parent}",
        )

  def _execute_bt_node(
    self,
    node: behavior_tree_pb2.BehaviorTree.Node,
    world: _InProcessWorldClient,
    report: _CycleExecutionReport,
    min_limits: list[float] | None = None,
    max_limits: list[float] | None = None,
  ) -> None:
    if node.HasField("sequence"):
      for child in node.sequence.children:
        self._execute_bt_node(child, world, report, min_limits, max_limits)
      return

    if node.HasField("parallel"):
      for child in node.parallel.children:
        self._execute_bt_node(child, world, report, min_limits, max_limits)
      return

    if node.HasField("loop"):
      if node.loop.HasField("do"):
        self._execute_bt_node(
          node.loop.do, world, report, min_limits, max_limits
        )
      return

    if node.HasField("retry"):
      if node.retry.HasField("child"):
        self._execute_bt_node(
          node.retry.child, world, report, min_limits, max_limits
        )
      return

    if node.HasField("fallback"):
      # In create_seated_approach_tasks, tries[0] is bt.Retry(move_to_contact,
      # recovery=reapproach_standoff_task) and tries[1] is fallback_seat_task.
      # Walking tries[0]'s recovery MoveRobot task first and then tries[1]'s
      # fallback linear seat MoveRobot task verifies both motion plans while
      # leaving the robot at the final seated pose before attach/detach.
      for try_branch in node.fallback.tries:
        if not try_branch.HasField("node"):
          continue
        branch_node = try_branch.node
        if branch_node.HasField("retry"):
          if branch_node.retry.HasField("recovery"):
            self._execute_bt_node(
              branch_node.retry.recovery,
              world,
              report,
              min_limits,
              max_limits,
            )
          continue
        self._execute_bt_node(
          branch_node, world, report, min_limits, max_limits
        )
        break
      return

    if node.HasField("sub_tree") and node.sub_tree.HasField("tree"):
      if node.sub_tree.tree.HasField("root"):
        self._execute_bt_node(
          node.sub_tree.tree.root, world, report, min_limits, max_limits
        )
      return

    if not node.HasField("task"):
      return

    if node.task.HasField("execute_code"):
      exec_code = node.task.execute_code
      if not exec_code.parameter_message_full_name:
        report.python_script_calls += 1
        return
      params_msg = skill_utils.create_message_from_file_descriptor_set(
        exec_code.file_descriptor_set,
        exec_code.parameter_message_full_name,
      )
      self.assertTrue(exec_code.parameters.proto.Unpack(params_msg))
      ctx = _ScriptExecutionContext(object_world=world)

      if "Calculate & Update Dynamic Grasp" in node.name:
        cam_t_stock = world.get_transform(
          world.orbbec_camera.sensor,
          world.raw_stock_2x3x5,
        )
        params_msg.pos_x = float(cam_t_stock.translation[0])
        params_msg.pos_y = float(cam_t_stock.translation[1])
        params_msg.pos_z = float(cam_t_stock.translation[2])
        quat = cam_t_stock.rotation.quaternion
        params_msg.ori_x = float(quat.x)
        params_msg.ori_y = float(quat.y)
        params_msg.ori_z = float(quat.z)
        params_msg.ori_w = float(quat.w)
        calculate_and_update_dynamic_frames(ctx, params_msg)
        report.python_script_calls += 1
      elif "Shift Return Placement Frame" in node.name:
        random.seed(42)
        randomize_placement_frame(ctx, params_msg)
        report.python_script_calls += 1
      return

    if not node.task.HasField("call_behavior"):
      return

    call = node.task.call_behavior
    skill_id = call.skill_id
    params_bytes = call.parameters.value

    if skill_id == "ai.intrinsic.move_robot":
      resp = self.runner.move_robot("ur_module", params_bytes)
      self.assertTrue(
        resp.get("ok"),
        f"MoveRobot failed on task '{node.name}': {resp}",
      )
      states = int(resp.get("states", 0))
      self.assertGreaterEqual(states, 2)
      start_joints = [float(q) for q in resp["start_joints"]]
      end_joints = [float(q) for q in resp["end_joints"]]
      min_joints = [float(q) for q in resp["min_joints"]]
      max_joints = [float(q) for q in resp["max_joints"]]

      if report.last_end_joints is not None:
        for idx, (prev_q, start_q) in enumerate(
          zip(report.last_end_joints, start_joints, strict=True)
        ):
          self.assertAlmostEqual(
            prev_q,
            start_q,
            places=3,
            msg=f"Start-state discontinuity on joint {idx} at '{node.name}'",
          )

      if min_limits is not None and max_limits is not None:
        for idx in range(len(min_joints)):
          self.assertGreaterEqual(
            min_joints[idx],
            min_limits[idx] - 1e-3,
            f"Joint {idx} violated min limit on '{node.name}'",
          )
          self.assertLessEqual(
            max_joints[idx],
            max_limits[idx] + 1e-3,
            f"Joint {idx} violated max limit on '{node.name}'",
          )

      report.last_end_joints = end_joints
      report.move_robot_calls += 1
      report.total_trajectory_states += states
    elif skill_id == "ai.intrinsic.attach_object_to_robot":
      resp = self.runner.attach_object(params_bytes)
      self.assertTrue(
        resp.get("ok"),
        f"AttachObject failed on task '{node.name}': {resp}",
      )
      self.assertEqual(world.raw_stock_2x3x5.parent_name, "gripper")
      self.assertEqual(resp.get("parent_entity"), "gripper_body")
      report.attach_calls += 1
    elif skill_id == "ai.intrinsic.detach_object":
      resp = self.runner.detach_object(params_bytes)
      self.assertTrue(
        resp.get("ok"),
        f"DetachObject failed on task '{node.name}': {resp}",
      )
      self.assertEqual(world.raw_stock_2x3x5.parent_name, "root")
      report.detach_calls += 1
    elif skill_id == "ai.intrinsic.update_world":
      resp = self.runner.update_world(params_bytes)
      self.assertTrue(
        resp.get("ok"),
        f"UpdateWorld failed on task '{node.name}': {resp}",
      )
      report.update_world_calls += 1

  def test_full_cycle_behavior_tree_execution_in_process(self) -> None:
    for (
      cell_key,
      config_path,
      expected_update_world_calls,
      expected_python_scripts,
    ) in (
      ("omts", "configs/omts/app_config.yaml", 8, 8),
      ("lab_bb_01", "configs/lab_bb_01/app_config.yaml", 0, 2),
    ):
      with self.subTest(cell_key=cell_key):
        world = self.runner.init_world(cell_key)
        ur_robot = world.get_kinematic_object("ur_module")
        app_limits = ur_robot.joint_application_limits
        min_limits = [float(x) for x in app_limits.min_position.values]
        max_limits = [float(x) for x in app_limits.max_position.values]

        _, bt_proto = self._build_tree_for_config(config_path)
        report = _CycleExecutionReport(
          last_end_joints=[float(q) for q in ur_robot.joint_positions]
        )
        self._execute_bt_node(
          bt_proto.root, world, report, min_limits, max_limits
        )

        self.assertGreaterEqual(report.move_robot_calls, 18)
        self.assertGreater(report.total_trajectory_states, 30)
        self.assertEqual(report.attach_calls, 2)
        self.assertEqual(report.detach_calls, 2)
        self.assertEqual(report.update_world_calls, expected_update_world_calls)
        self.assertEqual(report.python_script_calls, expected_python_scripts)

        # At the end of the cycle, the workpiece has been returned to the
        # infeed table ("root") and detached from the gripper.
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "root")

  def test_collision_rules_joint_limits_and_preplan_motion_regressions(
    self,
  ) -> None:
    world = self.runner.init_world("omts")
    _, bt_proto = self._build_tree_for_config("configs/omts/app_config.yaml")
    all_nodes = test_behaviors._iter_tree_nodes(bt_proto.root)
    move_nodes = [
      n
      for n in all_nodes
      if n.HasField("task")
      and n.task.HasField("call_behavior")
      and n.task.call_behavior.skill_id == "ai.intrinsic.move_robot"
    ]
    self.assertNotEmpty(move_nodes)
    move_by_name = {
      n.name: n.task.call_behavior.parameters.value for n in move_nodes
    }
    move_info = self.skill_infos["ai.intrinsic.move_robot"]

    # 1. Verify PreplanMotionSkill::Execute succeeds on a valid UpdateWorld
    # (create_frame) + MoveRobot sequence without mutating the live execution
    # world (matching PreplanMotionFixtureTest.ExecuteWorksWithValidMotionSegment).
    initial_joints = list(
      world.get_kinematic_object("ur_module").joint_positions
    )
    update_info = self.skill_infos["ai.intrinsic.update_world"]
    create_frame_msg = update_info.create_param_message()
    create_req = create_frame_msg.update.create_frame
    create_req.new_frame_name = "preplan_temp_frame"
    create_req.parent_object_with_filter.reference.by_name.object_name = "root"
    create_req.parent_t_new_frame.position.x = 0.2
    create_req.parent_t_new_frame.position.y = 0.2
    create_req.parent_t_new_frame.position.z = 1.2
    create_req.parent_t_new_frame.orientation.w = 1.0

    first_move_bytes = move_nodes[0].task.call_behavior.parameters.value
    skill_call_update = _encode_len_delimited(
      4, create_frame_msg.SerializeToString()
    )
    skill_call_move = _encode_len_delimited(1, first_move_bytes)
    preplan_params_bytes = _encode_len_delimited(
      1, skill_call_update
    ) + _encode_len_delimited(1, skill_call_move)
    preplan_resp = self.runner.preplan_motion("ur_module", preplan_params_bytes)
    self.assertTrue(
      preplan_resp.get("ok"), f"PreplanMotionSkill failed: {preplan_resp}"
    )

    # Confirm the live world was not mutated by PreplanMotionSkill::Execute.
    self.assertNotIn("preplan_temp_frame", set(world.root.frame_names))
    joints_after_preplan = list(
      world.get_kinematic_object("ur_module").joint_positions
    )
    for q_before, q_after in zip(
      initial_joints, joints_after_preplan, strict=True
    ):
      self.assertAlmostEqual(q_before, q_after, places=6)

    # 2. Verify per-segment collision rules are enforced by
    # MotionPlannerServiceInProcess during grasp touchdown when gripper fingers
    # contact the workpiece: advance to standoff above grasp, close the gripper
    # fingers so they overlap the raw_stock_2x3x5 footprint at the grasp pose,
    # then verify that planning the touchdown seat to grasp fails with a
    # collision error if collision_rules are stripped, and succeeds when the
    # BT's collision_rules (excluding gripper <-> raw_stock_2x3x5) are preserved.
    script_nodes = [
      n
      for n in all_nodes
      if n.HasField("task")
      and n.task.HasField("execute_code")
      and "Calculate & Update Dynamic Grasp" in n.name
    ]
    self.assertTrue(
      self.runner.move_robot(
        "ur_module",
        move_by_name["Move to View Frame (root/view)"],
      )["ok"]
    )
    self._execute_bt_node(script_nodes[0], world, _CycleExecutionReport())
    self.assertTrue(
      self.runner.move_robot(
        "ur_module",
        move_by_name["Move to Dynamic Pre-Grasp (root/pre_grasp)"],
      )["ok"]
    )
    self.assertTrue(
      self.runner.move_robot(
        "ur_module",
        move_by_name["Infeed Pick: Linear Approach to Standoff (root/grasp)"],
      )["ok"]
    )
    self._set_gripper_joint_positions(0.025)

    seat_bytes = move_by_name["Infeed Pick: Fallback Linear Seat (root/grasp)"]
    seat_without_rules_msg = move_info.create_param_message()
    self.assertTrue(seat_without_rules_msg.ParseFromString(seat_bytes))
    for seg in seat_without_rules_msg.motion_segments:
      seg.collision_settings.ClearField("collision_rules")
    seat_no_rules_resp = self.runner.move_robot(
      "ur_module", seat_without_rules_msg.SerializeToString()
    )
    self.assertFalse(
      seat_no_rules_resp.get("ok"),
      "Expected touchdown seat without CollisionRule(gripper, raw_stock_2x3x5) "
      f"to fail collision checking: {seat_no_rules_resp}",
    )
    seat_with_rules_resp = self.runner.move_robot("ur_module", seat_bytes)
    self.assertTrue(
      seat_with_rules_resp.get("ok"),
      f"Expected touchdown seat with BT collision rules to succeed: "
      f"{seat_with_rules_resp}",
    )

    # 3. Verify application joint limits from configs/omts/ur_module.limits.updates.pbtxt
    # are enforced by MoveRobot::PlanTrajectory: wrist_3_joint = 4.5 rad is
    # within the UR5e hardware limit (6.22 rad) but exceeds the OMTS application
    # limit (+3.8397 rad).
    joint_limit_msg = move_info.create_param_message()
    current_joints = list(
      world.get_kinematic_object("ur_module").joint_positions
    )
    seg = joint_limit_msg.motion_segments.add()
    for q in current_joints[:5]:
      seg.joint_position.joints.append(float(q))
    seg.joint_position.joints.append(4.5)
    joint_limit_resp = self.runner.move_robot(
      "ur_module", joint_limit_msg.SerializeToString()
    )
    self.assertFalse(
      joint_limit_resp.get("ok"),
      f"Expected wrist_3_joint=4.5 rad to violate application limits: "
      f"{joint_limit_resp}",
    )

    # 4. Verify MoveRobot::PlanTrajectory rejects a non-unit quaternion
    # (matching MoveRobotFixtureTest.ComputePlanFailsForInvalidQuaternion).
    bad_quat_msg = move_info.create_param_message()
    self.assertTrue(
      move_nodes[0].task.call_behavior.parameters.Unpack(bad_quat_msg)
    )
    bad_quat_msg.motion_segments[
      0
    ].cartesian_pose.target_frame_offset.orientation.z = 1.7062
    bad_quat_resp = self.runner.move_robot(
      "ur_module", bad_quat_msg.SerializeToString()
    )
    self.assertFalse(
      bad_quat_resp.get("ok"),
      f"Expected non-unit quaternion to fail planning: {bad_quat_resp}",
    )
    self.assertIn("non-unit quaternion", bad_quat_resp.get("message", ""))

    # 5. Verify MotionPlannerServiceInProcess rejects an unreachable target
    # pose (10 meters above root) both in MoveRobot and PreplanMotionSkill.
    unreachable_msg = move_info.create_param_message()
    self.assertTrue(
      move_nodes[0].task.call_behavior.parameters.Unpack(unreachable_msg)
    )
    unreachable_msg.motion_segments[
      0
    ].cartesian_pose.target_frame_offset.position.z = 10.0
    unreachable_bytes = unreachable_msg.SerializeToString()
    unreachable_resp = self.runner.move_robot("ur_module", unreachable_bytes)
    self.assertFalse(
      unreachable_resp.get("ok"),
      f"Expected unreachable pose to fail planning: {unreachable_resp}",
    )

    preplan_unreachable_bytes = _encode_len_delimited(
      1, _encode_len_delimited(1, unreachable_bytes)
    )
    preplan_unreachable_resp = self.runner.preplan_motion(
      "ur_module", preplan_unreachable_bytes
    )
    self.assertFalse(
      preplan_unreachable_resp.get("ok"),
      f"Expected PreplanMotionSkill to reject unreachable pose: "
      f"{preplan_unreachable_resp}",
    )

    # 6. Verify MotionPlannerServiceInProcess rejects a nonexistent target frame.
    missing_frame_msg = move_info.create_param_message()
    self.assertTrue(
      move_nodes[0].task.call_behavior.parameters.Unpack(missing_frame_msg)
    )
    missing_frame_msg.motion_segments[
      0
    ].cartesian_pose.target_frame.by_name.frame.frame_name = (
      "nonexistent_frame_xyz"
    )
    missing_resp = self.runner.move_robot(
      "ur_module", missing_frame_msg.SerializeToString()
    )
    self.assertFalse(
      missing_resp.get("ok"),
      f"Expected nonexistent frame to fail planning: {missing_resp}",
    )


if __name__ == "__main__":
  absltest.main()
