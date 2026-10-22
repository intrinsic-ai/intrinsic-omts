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

"""Shared test harness for in-process @intrinsic-core C++ integration tests."""

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

CELL_SCENE_OBJECT_FIXTURES: dict[str, tuple[tuple[str, str], ...]] = {
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

CELL_WORLD_UPDATE_FILES: dict[str, tuple[str, ...]] = {
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


def encode_len_delimited(field_number: int, payload: bytes) -> bytes:
  """Encodes a protobuf length-delimited field for `PreplanMotionParams`."""
  tag = (field_number << 3) | 2
  length = len(payload)
  varint = bytearray()
  while length >= 0x80:
    varint.append((length & 0x7F) | 0x80)
    length >>= 7
  varint.append(length)
  return bytes([tag]) + bytes(varint) + payload


class InProcessWorldClient(object_world_client.ObjectWorldClient):
  """ObjectWorldClient subclass exposing `.root` for BT PythonScript nodes."""

  @property
  def root(self) -> object_world_resources.WorldObject:
    return self.get_object(object_world_ids.ROOT_OBJECT_NAME)


@dataclasses.dataclass
class ScriptExecutionContext:
  """Minimal SBL PythonScript execution context backed by ObjectWorldClient."""

  object_world: InProcessWorldClient


class InProcessSkillRunner:
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
    self.world_client: InProcessWorldClient | None = None

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

  def init_world(self, cell_key: str) -> InProcessWorldClient:
    """Initializes the C++ FakeWorldService + MotionPlannerServiceInProcess."""
    if self._channel is not None:
      self._channel.close()
      self._channel = None

    obj_specs = [
      f"{name}={test_behaviors._find_runfile(rel_path)}"
      for name, rel_path in CELL_SCENE_OBJECT_FIXTURES[cell_key]
    ]
    upd_paths = [
      test_behaviors._find_runfile(rel_path)
      for rel_path in CELL_WORLD_UPDATE_FILES[cell_key]
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
    self.world_client = InProcessWorldClient("world", stub)
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
class CycleExecutionReport:
  """Summary of an in-process BehaviorTree execution against @intrinsic-core."""

  move_robot_calls: int = 0
  total_trajectory_states: int = 0
  attach_calls: int = 0
  detach_calls: int = 0
  update_world_calls: int = 0
  python_script_calls: int = 0
  last_end_joints: list[float] | None = None


class InProcessCoreTestCase(absltest.TestCase):
  """Base test case for in-process @intrinsic-core C++ integration tests."""

  runner: InProcessSkillRunner

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
    cls.runner = InProcessSkillRunner()

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

  def _execute_bt_node(
    self,
    node: behavior_tree_pb2.BehaviorTree.Node,
    world: InProcessWorldClient,
    report: CycleExecutionReport,
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
      ctx = ScriptExecutionContext(object_world=world)

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
