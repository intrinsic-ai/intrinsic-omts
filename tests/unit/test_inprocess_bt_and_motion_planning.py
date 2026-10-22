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

"""In-process @intrinsic-core Behavior Tree and motion planning integration tests."""

from absl.testing import absltest

from tests.unit import inprocess_core_test_harness, test_behaviors


class InProcessBtAndMotionPlanningTest(
  inprocess_core_test_harness.InProcessCoreTestCase
):
  """Verifies full-cycle BT execution and MotionPlannerServiceInProcess rules."""

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
        report = inprocess_core_test_harness.CycleExecutionReport(
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
    skill_call_update = inprocess_core_test_harness.encode_len_delimited(
      4, create_frame_msg.SerializeToString()
    )
    skill_call_move = inprocess_core_test_harness.encode_len_delimited(
      1, first_move_bytes
    )
    preplan_params_bytes = inprocess_core_test_harness.encode_len_delimited(
      1, skill_call_update
    ) + inprocess_core_test_harness.encode_len_delimited(1, skill_call_move)
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
    self._execute_bt_node(
      script_nodes[0],
      world,
      inprocess_core_test_harness.CycleExecutionReport(),
    )
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

    preplan_unreachable_bytes = (
      inprocess_core_test_harness.encode_len_delimited(
        1,
        inprocess_core_test_harness.encode_len_delimited(1, unreachable_bytes),
      )
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
