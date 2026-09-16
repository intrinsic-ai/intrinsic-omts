"""Unit tests for dynamic_frame_calculator module."""

import types
from unittest import mock

from absl.testing import absltest
from intrinsic.math.python import data_types

from src.utils import dynamic_frame_calculator


class DummyPoint:
  """Simple 3D point mock."""

  def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    self.x = x
    self.y = y
    self.z = z


class DummyQuaternion:
  """Simple quaternion mock."""

  def __init__(
    self,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    w: float = 1.0,
  ):
    self.x = x
    self.y = y
    self.z = z
    self.w = w


class DummyPoseNode:
  """Simple pose mock containing position and orientation."""

  def __init__(
    self,
    position: DummyPoint,
    orientation: DummyQuaternion | None = None,
  ):
    self.position = position
    self.orientation = orientation or DummyQuaternion()


class DummyEstimate:
  """Mock vision detection estimate."""

  def __init__(
    self,
    position: DummyPoint,
    score: float = 1.0,
    orientation: DummyQuaternion | None = None,
  ):
    self.root_t_target = DummyPoseNode(position, orientation)
    self.score = score


class DummyParent:
  """Mock parent object implementing list_frames."""

  def __init__(self, frames: list[str] | None = None):
    self._frames = frames or []

  def list_frames(self) -> list[str]:
    return list(self._frames)


class DynamicFrameCalculatorTest(absltest.TestCase):
  def setUp(self):
    super().setUp()
    self.world = mock.MagicMock()
    self.context = mock.MagicMock()
    self.context.object_world = self.world
    self.parent = DummyParent()
    self.world.root = self.parent
    self.world.get_transform.return_value = data_types.Pose3(
      data_types.Rotation3.identity(), [0.0, 0.0, 0.0]
    )

    self.params = types.SimpleNamespace()
    self.params.parent_object = "root"
    self.params.camera_name = "camera"
    self.params.approach_offset_z = 0.08

  def test_missing_camera_transform_raises_value_error(self):
    self.world.get_transform.return_value = None
    est = DummyEstimate(DummyPoint(1.0, 1.0, 0.10), score=-20.0)
    self.params.estimate_result = types.SimpleNamespace(estimates=[est])

    with self.assertRaises(ValueError):
      dynamic_frame_calculator.calculate_and_update_dynamic_frames(
        self.context, self.params
      )

  def test_negative_scores_accepted_for_foundation_pose(self):
    # FoundationPose produces negative log-likelihood/loss scores.
    # More negative score = higher confidence.
    est_target = DummyEstimate(DummyPoint(3.0, 3.0, 0.10), score=-22.0)
    est_mid = DummyEstimate(DummyPoint(2.0, 2.0, 0.10), score=-16.0)
    est_low = DummyEstimate(DummyPoint(1.0, 1.0, 0.10), score=-13.0)
    self.params.estimate_result = types.SimpleNamespace(
      estimates=[est_low, est_mid, est_target]
    )

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    created = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in self.world.create_frame.call_args_list
    }
    self.assertIn("infeed_grasp", created)
    self.assertAlmostEqual(created["infeed_grasp"].translation[0], 3.0)
    self.assertAlmostEqual(created["infeed_grasp"].translation[1], 3.0)

  def test_mixed_negative_and_positive_scores_selects_most_negative(self):
    # FoundationPose produces negative scores for confident fits (most negative is best),
    # while scores >= 0 are unconfident/bad fits that must be filtered out.
    est_target = DummyEstimate(DummyPoint(3.0, 3.0, 0.10), score=-20.69)
    est_mid = DummyEstimate(DummyPoint(2.0, 2.0, 0.10), score=-16.19)
    est_low = DummyEstimate(DummyPoint(1.0, 1.0, 0.10), score=-12.08)
    est_bad1 = DummyEstimate(DummyPoint(9.0, 9.0, 0.10), score=24.10)
    est_bad2 = DummyEstimate(DummyPoint(8.0, 8.0, 0.10), score=24.10)
    self.params.estimate_result = types.SimpleNamespace(
      estimates=[est_bad1, est_low, est_mid, est_target, est_bad2]
    )

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    created = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in self.world.create_frame.call_args_list
    }
    self.assertIn("infeed_grasp", created)
    # Most negative (-20.69) is selected as target!
    self.assertAlmostEqual(created["infeed_grasp"].translation[0], 3.0)
    self.assertAlmostEqual(created["infeed_grasp"].translation[1], 3.0)

  def test_min_safe_z_raises_when_target_too_low(self):
    est = DummyEstimate(DummyPoint(1.0, 1.0, 0.90), score=-20.0)
    self.params.estimate_result = types.SimpleNamespace(estimates=[est])
    self.params.min_safe_z = 1.00

    with self.assertRaises(ValueError):
      dynamic_frame_calculator.calculate_and_update_dynamic_frames(
        self.context, self.params
      )

  def test_min_safe_z_allows_target_above_the_floor(self):
    est = DummyEstimate(DummyPoint(1.0, 1.0, 1.10), score=-20.0)
    self.params.estimate_result = types.SimpleNamespace(estimates=[est])
    self.params.min_safe_z = 1.00

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    created = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in self.world.create_frame.call_args_list
    }
    self.assertIn("infeed_grasp", created)
    self.assertAlmostEqual(created["infeed_grasp"].translation[2], 1.10)

  def test_empty_estimates_discards_and_creates_no_frames(self):
    # When estimates list is empty, returns cleanly and creates no frames.
    self.params.estimate_result = types.SimpleNamespace(estimates=[])
    self.params.pos_x = 0.5
    self.params.pos_y = 0.5
    self.params.pos_z = 0.5

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    self.world.create_frame.assert_not_called()
    self.world.update_transform.assert_not_called()

  def test_target_selection_argmax_score(self):
    # Detections: argmax score should be selected as target candidate
    est_low = DummyEstimate(DummyPoint(1.0, 1.0, 0.05), score=-0.2)
    est_target = DummyEstimate(DummyPoint(2.0, 3.0, 0.10), score=-0.95)
    est_mid = DummyEstimate(DummyPoint(4.0, 5.0, 0.15), score=-0.6)
    self.params.estimate_result = types.SimpleNamespace(
      estimates=[est_low, est_target, est_mid]
    )

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    created = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in self.world.create_frame.call_args_list
    }
    self.assertIn("infeed_grasp", created)
    grasp_tf = created["infeed_grasp"]
    self.assertAlmostEqual(grasp_tf.translation[0], 2.0)
    self.assertAlmostEqual(grasp_tf.translation[1], 3.0)
    self.assertAlmostEqual(grasp_tf.translation[2], 0.10)

    self.assertIn("infeed_pre_grasp", created)
    pregrasp_tf = created["infeed_pre_grasp"]
    self.assertAlmostEqual(pregrasp_tf.translation[0], 2.0)
    self.assertAlmostEqual(pregrasp_tf.translation[1], 3.0)
    self.assertAlmostEqual(pregrasp_tf.translation[2], 0.18)

  def test_transform_chain_with_root_t_camera(self):
    # Verify root_t_target = root_t_camera * cam_pose
    camera_sensor = mock.MagicMock()
    camera_obj = mock.MagicMock()
    camera_obj.sensor = camera_sensor
    self.world.camera = camera_obj

    # Camera at [10.0, 20.0, 30.0]
    cam_rot = data_types.Rotation3.identity()
    root_t_camera = data_types.Pose3(cam_rot, [10.0, 20.0, 30.0])

    def mock_get_transform(node_a, node_b):
      if node_b == camera_sensor:
        return root_t_camera
      return None

    self.world.get_transform.side_effect = mock_get_transform

    est = DummyEstimate(DummyPoint(1.0, 2.0, 3.0), score=-0.8)
    self.params.estimate_result = types.SimpleNamespace(estimates=[est])

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    self.world.get_transform.assert_any_call(self.parent, camera_sensor)
    created = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in self.world.create_frame.call_args_list
    }
    grasp_tf = created["infeed_grasp"]
    self.assertAlmostEqual(grasp_tf.translation[0], 11.0)
    self.assertAlmostEqual(grasp_tf.translation[1], 22.0)
    self.assertAlmostEqual(grasp_tf.translation[2], 33.0)

  def test_frame_creation_all_four_frames_created(self):
    est = DummyEstimate(DummyPoint(0.5, 0.6, 0.1), score=-0.9)
    self.params.estimate_result = types.SimpleNamespace(estimates=[est])
    self.params.approach_offset_z = 0.05

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    created = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in self.world.create_frame.call_args_list
    }
    self.assertIn("infeed_pre_grasp", created)
    self.assertIn("infeed_grasp", created)

    self.assertAlmostEqual(created["infeed_pre_grasp"].translation[0], 0.5)
    self.assertAlmostEqual(created["infeed_pre_grasp"].translation[1], 0.6)
    self.assertAlmostEqual(created["infeed_pre_grasp"].translation[2], 0.15)

    self.assertAlmostEqual(created["infeed_grasp"].translation[0], 0.5)
    self.assertAlmostEqual(created["infeed_grasp"].translation[1], 0.6)
    self.assertAlmostEqual(created["infeed_grasp"].translation[2], 0.10)

  def test_frame_creation_updates_existing_frames(self):
    dynamic_frames = [
      "infeed_pre_grasp",
      "infeed_grasp",
    ]
    # The static scene frames are hand-taught and must survive perception.
    existing_frames = dynamic_frames + ["pre_grasp", "grasp"]
    self.parent = DummyParent(frames=existing_frames)
    handles = {}
    for fname in existing_frames:
      handles[fname] = mock.MagicMock()
      setattr(self.parent, fname, handles[fname])
    self.world.root = self.parent

    est = DummyEstimate(DummyPoint(0.5, 0.6, 0.1), score=-0.9)
    self.params.estimate_result = types.SimpleNamespace(estimates=[est])

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    self.world.create_frame.assert_not_called()
    updated = {
      call.kwargs.get("node_b")
      for call in self.world.update_transform.mock_calls
    }
    for fname in dynamic_frames:
      self.assertIn(handles[fname], updated)
    for fname in ("pre_grasp", "grasp"):
      self.assertNotIn(handles[fname], updated)
    # The two dynamic frames.
    self.assertEqual(self.world.update_transform.call_count, 2)

  def test_target_object_updated_with_camera_transform_and_direct_estimates(
    self,
  ):
    camera_sensor = mock.MagicMock()
    camera_obj = mock.MagicMock()
    camera_obj.sensor = camera_sensor
    self.world.camera = camera_obj

    # Camera at [10.0, 20.0, 30.0] with identity rotation.
    cam_rot = data_types.Rotation3.identity()
    root_t_camera = data_types.Pose3(cam_rot, [10.0, 20.0, 30.0])

    def mock_get_transform(node_a, node_b):
      if node_b == camera_sensor:
        return root_t_camera
      return None

    self.world.get_transform.side_effect = mock_get_transform

    # Provide direct params.estimates with two DummyEstimate objects.
    est_target = DummyEstimate(DummyPoint(1.0, 2.0, 3.0), score=-0.90)
    est_obstacle = DummyEstimate(DummyPoint(4.0, 5.0, 6.0), score=-0.70)
    self.params.estimates = [est_target, est_obstacle]
    self.params.target_scene_object_id = "ai.intrinsic.raw_stock_2x3x5"
    mock_target = mock.MagicMock()
    setattr(self.world, "ai.intrinsic.raw_stock_2x3x5", mock_target)

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    # Verify target_scene_object_id updated with root_t_camera * cam_pose.
    target_updates = [
      call
      for call in self.world.update_transform.call_args_list
      if call.kwargs.get("node_b") == mock_target
    ]
    self.assertEqual(len(target_updates), 1)
    self.assertAlmostEqual(
      target_updates[0].kwargs["a_t_b"].translation[0], 11.0
    )
    self.assertAlmostEqual(
      target_updates[0].kwargs["a_t_b"].translation[1], 22.0
    )
    self.assertAlmostEqual(
      target_updates[0].kwargs["a_t_b"].translation[2], 33.0
    )

  def test_direct_estimates_empty_discards_and_creates_no_frames(self):
    self.params.estimates = []
    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )
    self.world.create_frame.assert_not_called()
    self.world.update_transform.assert_not_called()

  def test_multi_detection_primary_block_updated_in_object_world(self):
    est1 = DummyEstimate(DummyPoint(0.15, 0.05, 0.05), score=-0.95)
    est2 = DummyEstimate(DummyPoint(0.15, 0.15, 0.05), score=-0.85)
    est3 = DummyEstimate(DummyPoint(0.15, 0.25, 0.05), score=-0.75)
    self.params.estimates = [est1, est2, est3]
    self.params.target_scene_object_id = "ai.intrinsic.raw_stock_2x3x5"

    mock_b1 = mock.MagicMock()
    setattr(self.world, "ai.intrinsic.raw_stock_2x3x5", mock_b1)

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    # Primary detected block must be updated in ObjectWorld at detected pose
    updated_nodes = {
      call.kwargs.get("node_b"): call.kwargs.get("a_t_b")
      for call in self.world.update_transform.call_args_list
    }
    self.assertIn(mock_b1, updated_nodes)
    self.assertAlmostEqual(updated_nodes[mock_b1].translation[1], 0.05)

  def test_single_active_workpiece_updated_in_world(self):
    # Single block detected by vision
    est1 = DummyEstimate(DummyPoint(0.15, 0.05, 0.05), score=-0.95)
    self.params.estimates = [est1]
    self.params.target_scene_object_id = "ai.intrinsic.raw_stock_2x3x5"

    mock_b1 = mock.MagicMock()
    setattr(self.world, "ai.intrinsic.raw_stock_2x3x5", mock_b1)

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    updated_nodes = {
      call.kwargs.get("node_b"): call.kwargs.get("a_t_b")
      for call in self.world.update_transform.call_args_list
    }
    self.assertIn(mock_b1, updated_nodes)
    self.assertAlmostEqual(updated_nodes[mock_b1].translation[2], 0.05)

  def test_grasp_offset_z_ignored_in_infeed_grasp_and_pregrasp(self):
    est = DummyEstimate(DummyPoint(0.20, 0.10, 0.05), score=-0.95)
    self.params.estimates = [est]
    self.params.approach_offset_z = 0.08
    self.params.grasp_offset_z = 0.015

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    created = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in self.world.create_frame.call_args_list
    }
    self.assertIn("infeed_grasp", created)
    self.assertIn("infeed_pre_grasp", created)
    self.assertAlmostEqual(created["infeed_grasp"].translation[2], 0.05)
    self.assertAlmostEqual(created["infeed_pre_grasp"].translation[2], 0.13)

  def test_place_frames_mirror_the_pick_frames(self):
    # The part goes back where it came from, and other detections in the
    # scene must not perturb that.
    est = DummyEstimate(DummyPoint(0.20, 0.10, 0.05), score=-0.95)
    est_other = DummyEstimate(DummyPoint(0.40, 0.10, 0.05), score=-0.80)
    self.params.estimates = [est, est_other]
    self.params.approach_offset_z = 0.08
    self.params.grasp_offset_z = 0.015

    dynamic_frame_calculator.calculate_and_update_dynamic_frames(
      self.context, self.params
    )

    created = {
      call.kwargs["frame_name"]: call.kwargs["parent_t_frame"]
      for call in self.world.create_frame.call_args_list
    }
    for place, pick in (
      ("infeed_grasp", "infeed_grasp"),
      ("infeed_pre_grasp", "infeed_pre_grasp"),
    ):
      self.assertIn(place, created)
      self.assertIn(pick, created)
      for axis in range(3):
        self.assertAlmostEqual(
          created[place].translation[axis],
          created[pick].translation[axis],
        )
    self.assertAlmostEqual(created["infeed_grasp"].translation[0], 0.20)
    self.assertAlmostEqual(created["infeed_grasp"].translation[1], 0.10)
    self.assertAlmostEqual(created["infeed_grasp"].translation[2], 0.05)
    self.assertAlmostEqual(created["infeed_pre_grasp"].translation[2], 0.13)


if __name__ == "__main__":
  absltest.main()
