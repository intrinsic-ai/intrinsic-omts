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

"""Tier 3B in-process @intrinsic-core world, skill, and motion planning tests.

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

  def attach_object(
    self, params_bytes: bytes, *, fallback_base_entity: bool = False
  ) -> dict[str, Any]:
    b64 = base64.b64encode(params_bytes).decode("ascii")
    return self._send_command(
      "ATTACH_OBJECT", b64, "1" if fallback_base_entity else "0"
    )

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
  strict_attach_errors: list[tuple[int, str]] = dataclasses.field(
    default_factory=list
  )


class InProcessCoreIntegrationTest(absltest.TestCase):
  """Tier 3B integration tests against real @intrinsic-core C++ services."""

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
      tree_name=f"Tier 3B InProcess Tree ({config_path})",
    )
    tree.validate_id_uniqueness()
    tree._validate_signatures()
    return tree, tree.proto

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

  def test_attach_object_to_robot_reproduces_pr160_multi_final_entity_error(
    self,
  ) -> None:
    for cell_key, config_path in (
      ("omts", "configs/omts/app_config.yaml"),
      ("lab_bb_01", "configs/lab_bb_01/app_config.yaml"),
    ):
      with self.subTest(cell_key=cell_key):
        world = self.runner.init_world(cell_key)
        _, bt_proto = self._build_tree_for_config(config_path)
        attach_nodes = [
          n
          for n in test_behaviors._iter_tree_nodes(bt_proto.root)
          if n.HasField("task")
          and n.task.HasField("call_behavior")
          and n.task.call_behavior.skill_id
          == "ai.intrinsic.attach_object_to_robot"
        ]
        self.assertLen(attach_nodes, 2)

        for attach_node in attach_nodes:
          params_bytes = attach_node.task.call_behavior.parameters.value
          resp = self.runner.attach_object(
            params_bytes, fallback_base_entity=False
          )
          self.assertFalse(resp["ok"])
          # AttachObjectToRobotSkill::kInvalidParentError == 3
          self.assertEqual(resp["code"], 3)
          self.assertIn(
            "can only be parented to a single entity", resp["message"]
          )
          self.assertIn("gripper_finger1", resp["message"])
          self.assertIn("gripper_finger2", resp["message"])

        # Verify that attaching to a single-final-entity parent (`ur_module`,
        # whose single final entity is `tool0`) succeeds with strict
        # AttachObjectToRobotSkill::ExecuteImpl, and DetachObjectSkill
        # restores parenting to `root`.
        attach_info = self.skill_infos["ai.intrinsic.attach_object_to_robot"]
        valid_attach_msg = attach_info.create_param_message()
        self.assertTrue(
          attach_nodes[0].task.call_behavior.parameters.Unpack(valid_attach_msg)
        )
        valid_attach_msg.gripper_entity.by_name.object_name = "ur_module"
        valid_resp = self.runner.attach_object(
          valid_attach_msg.SerializeToString(), fallback_base_entity=False
        )
        self.assertTrue(valid_resp["ok"], f"Expected success: {valid_resp}")
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "ur_module")

        detach_nodes = [
          n
          for n in test_behaviors._iter_tree_nodes(bt_proto.root)
          if n.HasField("task")
          and n.task.HasField("call_behavior")
          and n.task.call_behavior.skill_id == "ai.intrinsic.detach_object"
        ]
        self.assertLen(detach_nodes, 2)

        # Detaching from `gripper` while attached to `ur_module` must be
        # rejected by DetachObjectSkill::ExecuteImpl.
        wrong_parent_detach_resp = self.runner.detach_object(
          detach_nodes[0].task.call_behavior.parameters.value
        )
        self.assertFalse(wrong_parent_detach_resp["ok"])
        self.assertEqual(wrong_parent_detach_resp["code"], 3)
        self.assertIn(
          "not a child of the 'gripper_entity'",
          wrong_parent_detach_resp["message"],
        )

        # Detaching from the actual parent (`ur_module`) succeeds and restores
        # parenting to `root`.
        detach_info = self.skill_infos["ai.intrinsic.detach_object"]
        valid_detach_msg = detach_info.create_param_message()
        self.assertTrue(
          detach_nodes[0].task.call_behavior.parameters.Unpack(valid_detach_msg)
        )
        valid_detach_msg.gripper_entity.by_name.object_name = "ur_module"
        detach_resp = self.runner.detach_object(
          valid_detach_msg.SerializeToString()
        )
        self.assertTrue(detach_resp["ok"], f"Detach failed: {detach_resp}")
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "root")

  def _execute_bt_node(
    self,
    node: behavior_tree_pb2.BehaviorTree.Node,
    world: _InProcessWorldClient,
    report: _CycleExecutionReport,
  ) -> None:
    if node.HasField("sequence"):
      for child in node.sequence.children:
        self._execute_bt_node(child, world, report)
      return

    if node.HasField("parallel"):
      for child in node.parallel.children:
        self._execute_bt_node(child, world, report)
      return

    if node.HasField("loop"):
      if node.loop.HasField("do"):
        self._execute_bt_node(node.loop.do, world, report)
      return

    if node.HasField("retry"):
      if node.retry.HasField("child"):
        self._execute_bt_node(node.retry.child, world, report)
      return

    if node.HasField("fallback"):
      # In create_seated_approach_tasks, tries[0] is bt.Retry(move_to_contact,
      # recovery=reapproach_standoff_task) and tries[1] is fallback_seat_task.
      # Execute the retry recovery MoveRobot task first to verify its motion
      # plan, then execute the fallback linear seat MoveRobot task.
      for try_branch in node.fallback.tries:
        if not try_branch.HasField("node"):
          continue
        branch_node = try_branch.node
        if branch_node.HasField("retry"):
          if branch_node.retry.HasField("recovery"):
            self._execute_bt_node(branch_node.retry.recovery, world, report)
          continue
        self._execute_bt_node(branch_node, world, report)
        break
      return

    if node.HasField("sub_tree") and node.sub_tree.HasField("tree"):
      if node.sub_tree.tree.HasField("root"):
        self._execute_bt_node(node.sub_tree.tree.root, world, report)
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
      self.assertGreater(int(resp.get("states", 0)), 0)
      report.move_robot_calls += 1
      report.total_trajectory_states += int(resp["states"])
    elif skill_id == "ai.intrinsic.attach_object_to_robot":
      resp = self.runner.attach_object(params_bytes, fallback_base_entity=True)
      self.assertTrue(
        resp.get("ok"),
        f"AttachObject failed on task '{node.name}': {resp}",
      )
      if resp.get("fallback_applied"):
        report.strict_attach_errors.append(
          (int(resp["strict_code"]), str(resp["strict_message"]))
        )
      self.assertEqual(world.raw_stock_2x3x5.parent_name, "gripper")
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
        _, bt_proto = self._build_tree_for_config(config_path)
        report = _CycleExecutionReport()
        self._execute_bt_node(bt_proto.root, world, report)

        self.assertGreaterEqual(report.move_robot_calls, 18)
        self.assertGreater(report.total_trajectory_states, 30)
        self.assertEqual(report.attach_calls, 2)
        self.assertEqual(report.detach_calls, 2)
        self.assertEqual(report.update_world_calls, expected_update_world_calls)
        self.assertEqual(report.python_script_calls, expected_python_scripts)
        self.assertLen(report.strict_attach_errors, 2)
        for code, msg in report.strict_attach_errors:
          self.assertEqual(code, 3)
          self.assertIn("multiple final entities", msg)

        # At the end of the cycle, the workpiece has been returned to the
        # infeed table (`root`) and detached from the gripper.
        self.assertEqual(world.raw_stock_2x3x5.parent_name, "root")

  def test_preplan_motion_and_motion_planner_negative_checks(self) -> None:
    self.runner.init_world("omts")
    _, bt_proto = self._build_tree_for_config("configs/omts/app_config.yaml")
    all_nodes = test_behaviors._iter_tree_nodes(bt_proto.root)
    move_nodes = [
      n
      for n in all_nodes
      if n.HasField("task")
      and n.task.HasField("call_behavior")
      and n.task.call_behavior.skill_id == "ai.intrinsic.move_robot"
    ]
    attach_nodes = [
      n
      for n in all_nodes
      if n.HasField("task")
      and n.task.HasField("call_behavior")
      and n.task.call_behavior.skill_id == "ai.intrinsic.attach_object_to_robot"
    ]
    self.assertNotEmpty(move_nodes)
    self.assertNotEmpty(attach_nodes)

    def _encode_len_delimited(field_number: int, payload: bytes) -> bytes:
      tag = (field_number << 3) | 2
      length = len(payload)
      varint = bytearray()
      while length >= 0x80:
        varint.append((length & 0x7F) | 0x80)
        length >>= 7
      varint.append(length)
      return bytes([tag]) + bytes(varint) + payload

    # 1. Verify PreplanMotionSkill::Execute succeeds when preplanning the
    # initial MoveRobot call inside a cloned world.
    first_move_bytes = move_nodes[0].task.call_behavior.parameters.value
    skill_call_move = _encode_len_delimited(1, first_move_bytes)
    preplan_params_bytes = _encode_len_delimited(1, skill_call_move)
    preplan_resp = self.runner.preplan_motion("ur_module", preplan_params_bytes)
    self.assertTrue(
      preplan_resp.get("ok"), f"PreplanMotionSkill failed: {preplan_resp}"
    )

    # 2. Verify PreplanMotionSkill::Execute catches PR #160 when an
    # AttachObjectToRobotParams step for multi-final-entity `gripper` is
    # included in the preplan sequence.
    attach_bytes = attach_nodes[0].task.call_behavior.parameters.value
    skill_call_attach = _encode_len_delimited(2, attach_bytes)
    preplan_with_attach_bytes = _encode_len_delimited(
      1, skill_call_move
    ) + _encode_len_delimited(1, skill_call_attach)
    preplan_attach_resp = self.runner.preplan_motion(
      "ur_module", preplan_with_attach_bytes
    )
    self.assertFalse(preplan_attach_resp.get("ok"))
    self.assertEqual(preplan_attach_resp.get("code"), 3)
    self.assertIn(
      "multiple final entities", str(preplan_attach_resp.get("message", ""))
    )

    # 3. Verify MotionPlannerServiceInProcess rejects an unreachable target
    # pose (10 meters above root).
    move_info = self.skill_infos["ai.intrinsic.move_robot"]
    unreachable_msg = move_info.create_param_message()
    self.assertTrue(
      move_nodes[0].task.call_behavior.parameters.Unpack(unreachable_msg)
    )
    seg = unreachable_msg.motion_segments[0]
    seg.cartesian_pose.target_frame_offset.position.z = 10.0
    unreachable_resp = self.runner.move_robot(
      "ur_module", unreachable_msg.SerializeToString()
    )
    self.assertFalse(
      unreachable_resp.get("ok"),
      f"Expected unreachable pose to fail planning: {unreachable_resp}",
    )

    # 4. Verify MotionPlannerServiceInProcess rejects a nonexistent target frame.
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
