"""Unit tests for Building Block Pick and Place entry point and helpers."""

from unittest import mock
from absl.testing import absltest
from intrinsic.math.python import data_types
from intrinsic.world.public.proto import object_world_updates_pb2
from src.behaviors.building_block_bt import build_building_block_pick_place_tree
from src.core.types import Pose3D
from src.hardware.gripper import MockGripper
from src.hardware.robot import MockRobot
from src.pick_and_place_building_block import compute_dynamic_frame_poses
from src.pick_and_place_building_block import extract_pose_from_estimate
from src.pick_and_place_building_block import get_camera_resource
from src.pick_and_place_building_block import get_camera_transform_in_root
from src.pick_and_place_building_block import get_perception_service_resource
from src.pick_and_place_building_block import inject_dynamic_frames
from src.pick_and_place_building_block import run_pick_and_place_loop
from src.pick_and_place_building_block import select_best_detection
from src.pick_and_place_building_block import select_best_estimate
from src.pick_and_place_building_block import update_workpiece_pose


class PickAndPlaceBuildingBlockTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.robot = MockRobot()
    self.gripper = MockGripper()

  def test_compute_dynamic_frame_poses_tuples(self):
    pos = (0.10, 0.20, 0.30)
    ori = (0.0, 0.0, 0.70710678, 0.70710678)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
        approach_height_m=0.10,
        place_offset_x=0.20,
        place_offset_y=0.05,
    )

    self.assertIn("dynamic_pregrasp", poses)
    self.assertIn("dynamic_grasp", poses)
    self.assertIn("dynamic_preplace", poses)
    self.assertIn("dynamic_place", poses)

    # Grasp pose
    self.assertAlmostEqual(poses["dynamic_grasp"][0][0], 0.10)
    self.assertAlmostEqual(poses["dynamic_grasp"][0][1], 0.20)
    self.assertAlmostEqual(poses["dynamic_grasp"][0][2], 0.30)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][0], 0.70710678)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][1], 0.70710678)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][2], 0.0)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][3], 0.0)

    # Pregrasp pose
    self.assertAlmostEqual(poses["dynamic_pregrasp"][0][0], 0.10)
    self.assertAlmostEqual(poses["dynamic_pregrasp"][0][1], 0.20)
    self.assertAlmostEqual(poses["dynamic_pregrasp"][0][2], 0.40)
    self.assertAlmostEqual(poses["dynamic_pregrasp"][1][0], 0.70710678)
    self.assertAlmostEqual(poses["dynamic_pregrasp"][1][1], 0.70710678)

    # Place pose
    self.assertAlmostEqual(poses["dynamic_place"][0][0], 0.30)
    self.assertAlmostEqual(poses["dynamic_place"][0][1], 0.25)
    self.assertAlmostEqual(poses["dynamic_place"][0][2], 0.30)
    self.assertAlmostEqual(poses["dynamic_place"][1][0], 0.70710678)
    self.assertAlmostEqual(poses["dynamic_place"][1][1], 0.70710678)

    # Preplace pose
    self.assertAlmostEqual(poses["dynamic_preplace"][0][0], 0.30)
    self.assertAlmostEqual(poses["dynamic_preplace"][0][1], 0.25)
    self.assertAlmostEqual(poses["dynamic_preplace"][0][2], 0.40)
    self.assertAlmostEqual(poses["dynamic_preplace"][1][0], 0.70710678)
    self.assertAlmostEqual(poses["dynamic_preplace"][1][1], 0.70710678)

  def test_compute_dynamic_frame_poses_pose3d(self):
    pose = Pose3D(x=0.15, y=0.25, z=0.35, qx=1.0, qy=0.0, qz=0.0, qw=0.0)
    poses = compute_dynamic_frame_poses(
        position=pose,
        approach_height_m=0.12,
        place_offset_x=0.18,
        place_offset_y=0.00,
    )

    self.assertAlmostEqual(poses["dynamic_grasp"][0][0], 0.15)
    self.assertAlmostEqual(poses["dynamic_grasp"][0][1], 0.25)
    self.assertAlmostEqual(poses["dynamic_grasp"][0][2], 0.35)
    self.assertAlmostEqual(poses["dynamic_pregrasp"][0][2], 0.47)
    self.assertAlmostEqual(poses["dynamic_place"][0][0], 0.33)
    self.assertAlmostEqual(poses["dynamic_place"][0][2], 0.35)
    self.assertAlmostEqual(poses["dynamic_preplace"][0][0], 0.33)
    self.assertAlmostEqual(poses["dynamic_preplace"][0][2], 0.47)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][0], 1.0)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][1], 0.0)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][2], 0.0)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][3], 0.0)

  def test_compute_dynamic_frame_poses_top_down_orientation(self):
    pos = (0.10, 0.20, 0.30)
    # 90-degree yaw workpiece on table
    ori = (0.0, 0.0, 0.70710678, 0.70710678)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
        approach_height_m=0.10,
        place_offset_x=0.20,
        place_offset_y=0.00,
    )

    expected_ori = (0.7071067811865476, 0.7071067811865475, 0.0, 0.0)
    for k in [
        "dynamic_grasp",
        "dynamic_pregrasp",
        "dynamic_place",
        "dynamic_preplace",
    ]:
      for i in range(4):
        self.assertAlmostEqual(poses[k][1][i], expected_ori[i], places=6)

  def test_compute_dynamic_frame_poses_identity_top_down(self):
    pos = (0.10, 0.20, 0.30)
    ori = (0.0, 0.0, 0.0, 1.0)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
    )
    downward_canonical = (1.0, 0.0, 0.0, 0.0)
    self.assertEqual(poses["dynamic_grasp"][1], downward_canonical)

  def test_compute_dynamic_frame_poses_45_deg_yaw(self):
    pos = (0.10, 0.20, 0.30)
    # 45 deg around Z: qz = sin(pi/8), qw = cos(pi/8)
    ori = (0.0, 0.0, 0.38268343, 0.92387953)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
    )
    # Expected top-down: qx = cos(pi/8), qy = sin(pi/8)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][0], 0.92387953, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][1], 0.38268343, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][2], 0.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][3], 0.0, places=5)

  def test_compute_dynamic_frame_poses_180_deg_yaw(self):
    pos = (0.10, 0.20, 0.30)
    # 180 deg around Z: qz = 1.0, qw = 0.0
    ori = (0.0, 0.0, 1.0, 0.0)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
    )
    # Expected top-down: qx = cos(pi/2) = 0.0, qy = sin(pi/2) = 1.0
    self.assertAlmostEqual(poses["dynamic_grasp"][1][0], 0.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][1], 1.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][2], 0.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][3], 0.0, places=5)

  def test_compute_dynamic_frame_poses_negative_90_deg_yaw(self):
    pos = (0.10, 0.20, 0.30)
    # -90 deg around Z: qz = -0.70710678, qw = 0.70710678
    ori = (0.0, 0.0, -0.70710678, 0.70710678)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
    )
    # Expected top-down: qx = cos(-pi/4) = 0.70710678,
    #                    qy = sin(-pi/4) = -0.70710678
    self.assertAlmostEqual(poses["dynamic_grasp"][1][0], 0.70710678, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][1], -0.70710678, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][2], 0.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][3], 0.0, places=5)

  def test_compute_dynamic_frame_poses_vertical_local_x_axis(self):
    pos = (0.34769688, 0.26536308, 1.05280108)
    # Detected workpiece pose where local X points along +Z
    ori = (0.52664544, 0.4644981, 0.53018673, -0.47517168)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
    )
    # Local X is vertical (+Z). Primary horizontal axis is local Y
    # with direction (0.99311, -0.11690), yielding yaw = -0.11721 rad (-6.716 deg).
    # Expected top-down: qx = cos(yaw/2) = 0.99828, qy = sin(yaw/2) = -0.05856
    self.assertAlmostEqual(poses["dynamic_grasp"][1][0], 0.9982827, places=4)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][1], -0.0585787, places=4)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][2], 0.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][3], 0.0, places=5)

  def test_compute_dynamic_frame_poses_vertical_local_y_axis(self):
    pos = (0.10, 0.20, 0.30)
    # 90 deg around X: local Y points along +Z, local Z points along -Y
    ori = (0.70710678, 0.0, 0.0, 0.70710678)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
    )
    # Local Y is vertical. Primary horizontal axis is local Z (0, -1, 0),
    # yielding yaw = -pi/2 (-90 deg).
    # Expected top-down: qx = cos(-pi/4) = 0.70710678, qy = sin(-pi/4) = -0.70710678
    self.assertAlmostEqual(poses["dynamic_grasp"][1][0], 0.70710678, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][1], -0.70710678, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][2], 0.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][3], 0.0, places=5)

  def test_compute_dynamic_frame_poses_inverted_vertical_local_z_axis(self):
    pos = (0.10, 0.20, 0.30)
    # 180 deg around X: local Z points along -Z, local X points along +X
    ori = (1.0, 0.0, 0.0, 0.0)
    poses = compute_dynamic_frame_poses(
        position=pos,
        orientation=ori,
    )
    # Local Z is vertical. Primary horizontal axis is local X (1, 0, 0),
    # yielding yaw = 0.
    # Expected top-down: qx = 1.0, qy = 0.0
    self.assertAlmostEqual(poses["dynamic_grasp"][1][0], 1.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][1], 0.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][2], 0.0, places=5)
    self.assertAlmostEqual(poses["dynamic_grasp"][1][3], 0.0, places=5)

  def test_select_best_estimate(self):
    est1 = mock.MagicMock(score=-19.2566)
    est2 = mock.MagicMock(score=-17.4622)
    est3 = mock.MagicMock(score=8.4320)

    best = select_best_detection([est1, est2, est3])
    self.assertEqual(best, est1)

    best_alias = select_best_estimate([est1, est2, est3])
    self.assertEqual(best_alias, est1)

    self.assertIsNone(select_best_detection([]))

  def test_select_best_estimate_dict(self):
    est1 = {"score": -19.2566, "position": (0, 0, 0)}
    est2 = {"score": 8.4320, "position": (1, 1, 1)}
    best = select_best_detection([est1, est2])
    self.assertEqual(best, est1)

  def test_extract_pose_from_estimate(self):
    # Test tuple extraction from object
    est_obj = mock.MagicMock()
    est_obj.position = (0.1, 0.2, 0.3)
    est_obj.orientation = (0.0, 0.0, 0.0, 1.0)
    del est_obj.root_t_target
    del est_obj.pose
    pos, ori = extract_pose_from_estimate(est_obj)
    self.assertEqual(pos, (0.1, 0.2, 0.3))
    self.assertEqual(ori, (0.0, 0.0, 0.0, 1.0))

    # Test Pose3D
    est_pose = mock.MagicMock()
    est_pose.pose = Pose3D(x=0.4, y=0.5, z=0.6, qx=0, qy=0, qz=0, qw=1)
    del est_pose.root_t_target
    del est_pose.position
    pos, ori = extract_pose_from_estimate(est_pose)
    self.assertEqual(pos, (0.4, 0.5, 0.6))
    self.assertEqual(ori, (0.0, 0.0, 0.0, 1.0))

    # Test dict
    est_dict = {"position": (0.7, 0.8, 0.9), "orientation": (1, 0, 0, 0)}
    pos, ori = extract_pose_from_estimate(est_dict)
    self.assertEqual(pos, (0.7, 0.8, 0.9))
    self.assertEqual(ori, (1.0, 0.0, 0.0, 0.0))

  def test_extract_pose_from_estimate_with_camera_transform(self):
    # Camera at (0.6, 0.7, 1.0), identity rotation
    r_cam = data_types.Rotation3(data_types.Quaternion([0.0, 0.0, 0.0, 1.0]))
    root_t_cam = data_types.Pose3(r_cam, [0.6, 0.7, 1.0])

    # Workpiece detection in camera frame: (0.1, -0.05, -0.4)
    est_dict = {"position": (0.1, -0.05, -0.4), "orientation": (0, 0, 0, 1)}
    pos, _ = extract_pose_from_estimate(est_dict, root_t_camera=root_t_cam)
    self.assertAlmostEqual(pos[0], 0.7)
    self.assertAlmostEqual(pos[1], 0.65)
    self.assertAlmostEqual(pos[2], 0.6)

  def test_extract_pose_root_t_target_with_camera_transform(self):
    # Camera at (0.6, 0.7, 1.0)
    r_cam = data_types.Rotation3(data_types.Quaternion([0.0, 0.0, 0.0, 1.0]))
    root_t_cam = data_types.Pose3(r_cam, [0.6, 0.7, 1.0])

    # Workpiece detection in camera frame: (0.1, -0.05, -0.4)
    est_proto = mock.MagicMock()
    est_proto.root_t_target.position.x = 0.10
    est_proto.root_t_target.position.y = -0.05
    est_proto.root_t_target.position.z = -0.40
    est_proto.root_t_target.orientation.x = 0.0
    est_proto.root_t_target.orientation.y = 0.0
    est_proto.root_t_target.orientation.z = 0.0
    est_proto.root_t_target.orientation.w = 1.0

    pos, ori = extract_pose_from_estimate(est_proto, root_t_camera=root_t_cam)
    self.assertAlmostEqual(pos[0], 0.70)
    self.assertAlmostEqual(pos[1], 0.65)
    self.assertAlmostEqual(pos[2], 0.60)
    self.assertEqual(ori, (0.0, 0.0, 0.0, 1.0))

  def test_extract_pose_root_t_target_without_camera_transform(self):
    est_proto = mock.MagicMock()
    est_proto.root_t_target.position.x = 0.15
    est_proto.root_t_target.position.y = 0.25
    est_proto.root_t_target.position.z = 0.70
    est_proto.root_t_target.orientation.x = 0.0
    est_proto.root_t_target.orientation.y = 0.0
    est_proto.root_t_target.orientation.z = 0.0
    est_proto.root_t_target.orientation.w = 1.0

    pos, ori = extract_pose_from_estimate(est_proto, root_t_camera=None)
    self.assertAlmostEqual(pos[0], 0.15)
    self.assertAlmostEqual(pos[1], 0.25)
    self.assertAlmostEqual(pos[2], 0.70)
    self.assertEqual(ori, (0.0, 0.0, 0.0, 1.0))

  def test_get_camera_transform_in_root(self):
    mock_world = mock.MagicMock(spec=[])
    mock_root = mock.MagicMock(spec=[])
    mock_cam = mock.MagicMock(spec=[])
    mock_sensor = mock.MagicMock(spec=[])

    setattr(mock_world, "root", mock_root)
    setattr(mock_world, "orbbec_camera", mock_cam)
    setattr(mock_cam, "sensor", mock_sensor)

    expected_pose = data_types.Pose3(
        data_types.Rotation3(data_types.Quaternion([0.0, 0.0, 0.0, 1.0])),
        [0.8, 0.3, 0.9],
    )
    mock_world.get_transform = mock.MagicMock(return_value=expected_pose)

    tf = get_camera_transform_in_root(
        mock_world, camera_name="orbbec_camera", parent_object_name="root"
    )
    self.assertEqual(tf, expected_pose)
    mock_world.get_transform.assert_called_once_with(mock_root, mock_sensor)

  def test_inject_dynamic_frames_existing_frames(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = [
        "dynamic_pregrasp",
        "dynamic_grasp",
        "dynamic_preplace",
        "dynamic_place",
    ]
    setattr(mock_world, "root", mock_root)
    setattr(mock_root, "dynamic_pregrasp", mock.MagicMock())
    setattr(mock_root, "dynamic_grasp", mock.MagicMock())
    setattr(mock_root, "dynamic_preplace", mock.MagicMock())
    setattr(mock_root, "dynamic_place", mock.MagicMock())

    frame_poses = {
        "dynamic_pregrasp": ((0.10, 0.20, 0.40), (0.0, 0.0, 0.0, 1.0)),
        "dynamic_grasp": ((0.10, 0.20, 0.30), (0.0, 0.0, 0.0, 1.0)),
        "dynamic_preplace": ((0.30, 0.20, 0.40), (0.0, 0.0, 0.0, 1.0)),
        "dynamic_place": ((0.30, 0.20, 0.30), (0.0, 0.0, 0.0, 1.0)),
    }

    inject_dynamic_frames(mock_world, frame_poses, parent_object_name="root")
    self.assertEqual(mock_world.update_transform.call_count, 4)
    mock_world.batch_update.assert_not_called()

  def test_inject_dynamic_frames_new_frames(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    # Clear attribute access for dynamic frames
    del mock_root.dynamic_pregrasp
    del mock_root.dynamic_grasp
    del mock_root.dynamic_preplace
    del mock_root.dynamic_place
    setattr(mock_world, "root", mock_root)

    frame_poses = {
        "dynamic_pregrasp": ((0.10, 0.20, 0.40), (0.0, 0.0, 0.0, 1.0)),
        "dynamic_grasp": ((0.10, 0.20, 0.30), (0.0, 0.0, 0.0, 1.0)),
        "dynamic_preplace": ((0.30, 0.20, 0.40), (0.0, 0.0, 0.0, 1.0)),
        "dynamic_place": ((0.30, 0.20, 0.30), (0.0, 0.0, 0.0, 1.0)),
    }

    inject_dynamic_frames(mock_world, frame_poses, parent_object_name="root")
    mock_world.batch_update.assert_called_once()
    updates = mock_world.batch_update.call_args[0][0]
    self.assertIsInstance(updates, object_world_updates_pb2.ObjectWorldUpdates)
    self.assertEqual(len(updates.updates), 4)

  def test_update_workpiece_pose_existing_object(self):
    mock_world = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_raw_stock = mock.MagicMock()
    setattr(mock_world, "root", mock_root)
    setattr(mock_world, "raw_stock_2x3x5", mock_raw_stock)

    update_workpiece_pose(
        world=mock_world,
        position=(0.15, 0.25, 0.71),
        orientation=(1.0, 0.0, 0.0, 0.0),
        workpiece_name="raw_stock_2x3x5",
        parent_object_name="root",
    )
    mock_world.update_transform.assert_called_once()
    call_kwargs = mock_world.update_transform.call_args.kwargs
    self.assertEqual(call_kwargs["node_a"], mock_root)
    self.assertEqual(call_kwargs["node_b"], mock_raw_stock)

  def test_update_workpiece_pose_fallback_name(self):
    mock_world = mock.MagicMock(spec=["update_transform", "root", "raw_stock"])
    mock_root = mock.MagicMock()
    mock_raw_stock = mock.MagicMock()
    setattr(mock_world, "root", mock_root)
    setattr(mock_world, "raw_stock", mock_raw_stock)

    update_workpiece_pose(
        world=mock_world,
        position=(0.15, 0.25, 0.71),
        orientation=(1.0, 0.0, 0.0, 0.0),
        workpiece_name="raw_stock_2x3x5",
        parent_object_name="root",
    )
    mock_world.update_transform.assert_called_once()
    call_kwargs = mock_world.update_transform.call_args.kwargs
    self.assertEqual(call_kwargs["node_a"], mock_root)
    self.assertEqual(call_kwargs["node_b"], mock_raw_stock)

  def test_update_workpiece_pose_missing_object_noops(self):
    mock_world = mock.MagicMock(spec=["update_transform", "root"])
    mock_root = mock.MagicMock()
    setattr(mock_world, "root", mock_root)

    update_workpiece_pose(
        world=mock_world,
        position=(0.15, 0.25, 0.71),
        orientation=(1.0, 0.0, 0.0, 0.0),
        workpiece_name="raw_stock_2x3x5",
        parent_object_name="root",
    )
    mock_world.update_transform.assert_not_called()

  def test_build_cycle_tree(self):
    tree = build_building_block_pick_place_tree(
        robot=self.robot,
        gripper=self.gripper,
        parent_object="root",
        pregrasp_frame_name="dynamic_pregrasp",
        grasp_frame_name="dynamic_grasp",
        preplace_frame_name="dynamic_preplace",
        place_frame_name="dynamic_place",
        view_frame_name="view",
    )

    self.assertIsNotNone(tree)
    self.assertEqual(tree.name, "Building Block Pick & Place Cycle")
    self.assertEqual(len(tree.root.children), 10)

  def test_resource_resolution(self):
    mock_solution = mock.MagicMock()
    mock_cam = mock.MagicMock()
    mock_svc = mock.MagicMock()
    mock_solution.resources = {
        "orbbec_camera": mock_cam,
        "pose_estimator_service": mock_svc,
    }

    cam = get_camera_resource(mock_solution, "orbbec_camera")
    self.assertEqual(cam, mock_cam)

    svc = get_perception_service_resource(
        mock_solution, "pose_estimator_service"
    )
    self.assertEqual(svc, mock_svc)

  def test_run_pick_and_place_loop_mock_single_cycle(self):
    mock_solution = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    del mock_root.dynamic_pregrasp
    del mock_root.dynamic_grasp
    del mock_root.dynamic_preplace
    del mock_root.dynamic_place
    setattr(mock_solution.world, "root", mock_root)

    # Mock estimation skill result
    mock_estimate = mock.MagicMock()
    mock_estimate.score = 0.96
    mock_estimate.position = (0.15, 0.25, 0.70)
    mock_estimate.orientation = (1.0, 0.0, 0.0, 0.0)
    del mock_estimate.root_t_target
    del mock_estimate.pose

    mock_result = mock.MagicMock()
    mock_result.estimates = [mock_estimate]
    mock_solution.executive.get_value.return_value = mock_result

    completed_cycles = run_pick_and_place_loop(
        solution_address="localhost:17080",
        mock_hardware=True,
        move_to_view_first=True,
        num_cycles=1,
        solution=mock_solution,
    )

    self.assertEqual(completed_cycles, 1)
    # Verify executive ran: move_view, perception pipeline, master tree
    self.assertGreaterEqual(mock_solution.executive.run.call_count, 3)
    mock_solution.world.batch_update.assert_called_once()

  def test_run_pick_and_place_loop_alternate_offsets(self):
    mock_solution = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    del mock_root.dynamic_pregrasp
    del mock_root.dynamic_grasp
    del mock_root.dynamic_preplace
    del mock_root.dynamic_place
    setattr(mock_solution.world, "root", mock_root)

    mock_estimate = mock.MagicMock()
    mock_estimate.score = 0.96
    mock_estimate.position = (0.15, 0.25, 0.70)
    mock_estimate.orientation = (1.0, 0.0, 0.0, 0.0)
    del mock_estimate.root_t_target
    del mock_estimate.pose

    mock_result = mock.MagicMock()
    mock_result.estimates = [mock_estimate]
    mock_solution.executive.get_value.return_value = mock_result

    completed_cycles = run_pick_and_place_loop(
        solution_address="localhost:17080",
        place_offset_x=0.20,
        place_offset_y=0.05,
        alternate_place_offset=True,
        mock_hardware=True,
        move_to_view_first=False,
        num_cycles=2,
        solution=mock_solution,
    )

    self.assertEqual(completed_cycles, 2)
    self.assertEqual(mock_solution.world.batch_update.call_count, 2)

  def test_run_pick_and_place_loop_settling_timeout(self):
    mock_solution = mock.MagicMock()
    mock_root = mock.MagicMock()
    mock_root.list_frames.return_value = []
    setattr(mock_solution.world, "root", mock_root)

    mock_estimate = mock.MagicMock()
    mock_estimate.score = 0.99
    mock_estimate.position = (0.1, 0.2, 0.3)
    mock_estimate.orientation = (1.0, 0.0, 0.0, 0.0)
    del mock_estimate.root_t_target
    del mock_estimate.pose

    mock_result = mock.MagicMock()
    mock_result.estimates = [mock_estimate]
    mock_solution.executive.get_value.return_value = mock_result

    with (
        mock.patch("src.pick_and_place_building_block.UrRobot") as mock_ur,
        mock.patch(
            "src.pick_and_place_building_block.get_camera_resource"
        ),
        mock.patch(
            "src.pick_and_place_building_block.get_perception_service_resource"
        ),
        mock.patch(
            "src.pick_and_place_building_block.SideloadedGripperCmd"
        ),
    ):
      mock_robot_inst = mock.MagicMock()
      mock_ur.return_value = mock_robot_inst
      run_pick_and_place_loop(
          mock_hardware=False,
          move_to_view_first=True,
          num_cycles=1,
          settling_timeout_seconds=20.0,
          solution=mock_solution,
      )
      mock_ur.assert_called_once()
      self.assertEqual(
          mock_ur.call_args.kwargs.get("default_settling_timeout_seconds"),
          20.0,
      )


if __name__ == "__main__":
  absltest.main()

