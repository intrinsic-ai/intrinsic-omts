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

"""In-process @intrinsic-core world composition and skill integration tests."""

from absl.testing import absltest

from src.core.config import load_app_config
from tests.unit import inprocess_core_test_harness, test_behaviors


class InProcessWorldAndSkillsTest(
  inprocess_core_test_harness.InProcessCoreTestCase
):
  """Verifies world composition, UpdateWorld, and Attach/DetachObject skills."""

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
        for (
          obj_name,
          _,
        ) in inprocess_core_test_harness.CELL_SCENE_OBJECT_FIXTURES[cell_key]:
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


if __name__ == "__main__":
  absltest.main()
