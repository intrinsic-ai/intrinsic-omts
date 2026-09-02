"""Unit tests for evaluate_calibration summary formatting functions."""

from absl.testing import absltest
from intrinsic.perception.skills.calibration import (
    calibration_evaluation_pb2 as eval_pb2,
)
from tools.calibration import evaluate_calibration


class EvaluateCalibrationSummaryTest(absltest.TestCase):

  def test_format_human_readable_validation_summary_full_output(self):
    result = eval_pb2.CalibrationEvaluationResult()

    # Epipolar metrics
    ep0 = result.epipolar_metrics.add()
    ep0.camera_0 = "cam0"
    ep0.camera_1 = "cam1"
    ep0.num_common_corners = 25
    ep0.mean_epipolar_error_px = 0.12
    ep0.rms_epipolar_error_px = 0.15
    ep0.max_epipolar_error_px = 0.35

    ep1 = result.epipolar_metrics.add()
    ep1.camera_0 = "cam0"
    ep1.camera_1 = "cam2"
    ep1.num_common_corners = 20
    ep1.mean_epipolar_error_px = 0.10
    ep1.rms_epipolar_error_px = 0.13
    ep1.max_epipolar_error_px = 0.30

    result.mean_epipolar_error_px = 0.11
    result.mean_angular_epipolar_error_mrad = 0.11
    result.mean_subpixel_noise_ratio = 0.7333
    result.mean_tangent_drift_mm_per_m = 0.11

    # Pose consistency metrics
    pm0 = result.pose_consistency_metrics.add()
    pm0.camera_0 = "cam0"
    pm0.camera_1 = "cam1"
    pm0.translation_error_m = 0.001
    pm0.rotation_error_deg = 0.05
    pm0.add_error_m = 0.0015
    pm0.cross_reprojection_error_px = 0.20

    result.mean_pose_translation_error_m = 0.001
    result.mean_pose_rotation_error_deg = 0.05
    result.mean_pose_add_error_m = 0.0015
    result.mean_pose_cross_reprojection_error_px = 0.20

    # Triangulation metrics
    tm0 = result.triangulation_metrics.add()
    tm0.triangulated_by_camera_0 = "cam0"
    tm0.triangulated_by_camera_1 = "cam1"
    tm0.reprojected_to_camera = "cam2"
    tm0.num_evaluated_corners = 20
    tm0.mean_reprojection_error_px = 0.18
    tm0.rms_reprojection_error_px = 0.22
    tm0.max_reprojection_error_px = 0.40
    tm0.mean_angular_reprojection_error_mrad = 0.18
    tm0.subpixel_noise_ratio = 1.20
    tm0.mean_tangent_drift_mm_per_m = 0.18

    result.mean_triangulation_reprojection_error_px = 0.18
    result.mean_angular_triangulation_reprojection_error_mrad = 0.18

    # Camera diagnostics
    d0 = result.camera_diagnostics.add()
    d0.camera_name = "cam0"
    d0.num_detected_corners = 30
    d0.mean_mono_reprojection_error_px = 0.15
    d0.mean_angular_mono_reprojection_error_mrad = 0.15
    d0.mono_subpixel_noise_ratio = 1.00

    d1 = result.camera_diagnostics.add()
    d1.camera_name = "cam1"
    d1.num_detected_corners = 25

    d2 = result.camera_diagnostics.add()
    d2.camera_name = "cam2"
    d2.num_detected_corners = 20

    result.mean_mono_reprojection_error_px = 0.15
    result.mean_angular_mono_reprojection_error_mrad = 0.15

    summary = evaluate_calibration.format_human_readable_validation_summary(
        result
    )

    # 1. Detections per camera
    self.assertIn("=== Detections per camera ===", summary)
    self.assertIn("Camera 'cam0': 30 corners detected", summary)
    self.assertIn("Camera 'cam1': 25 corners detected", summary)
    self.assertIn("Camera 'cam2': 20 corners detected", summary)

    # 2. Pairwise Epipolar Section
    self.assertIn("=== Pairwise Epipolar Section ===", summary)
    self.assertIn("Pair cam0 - cam1:", summary)
    self.assertIn("  Common corners: 25", summary)
    self.assertIn("  Mean epipolar error: 0.1200 px", summary)
    self.assertIn("  Max epipolar error:  0.3500 px", summary)
    self.assertIn("  RMS epipolar error:  0.1500 px", summary)
    self.assertIn("  Sensor cam0 PnP error: 0.1500 px", summary)
    self.assertIn("Pair cam0 - cam2:", summary)

    # 3. Test 1: Board Pose Consistency (Aggregated)
    self.assertIn(
        "=== Test 1: Board Pose Consistency (Aggregated) ===", summary
    )
    self.assertIn(
        "  Sensor cam0 Average Errors (against all other sensors):", summary
    )
    self.assertIn("    Translation Error: 1.0000 mm", summary)
    self.assertIn("    Rotation Error:    0.0500 deg", summary)
    self.assertIn("    ADD Error:         1.5000 mm", summary)
    self.assertIn("    Pose Reproj Error: 0.2000 px", summary)

    # 4. Test 2: Triangulation & Reprojection (All Pairs)
    self.assertIn(
        "=== Test 2: Triangulation & Reprojection (All Pairs) ===", summary
    )
    self.assertIn(
        "Triangulating using pair cam0 - cam1 (Common corners: 20)", summary
    )
    self.assertIn("Reprojection onto Sensor cam2:", summary)
    self.assertIn("Compared corners: 20", summary)
    self.assertIn("Mean error:       0.1800 px", summary)
    self.assertIn("Max error:        0.4000 px", summary)
    self.assertIn("RMS error:        0.2200 px", summary)

    # 5. Calibration Validation Summary Table
    self.assertIn("=== Calibration Validation Summary ===", summary)
    self.assertIn(
        "Test                         | Best       | Mean       | Worst     ",
        summary,
    )
    self.assertIn(
        "Epipolar Error (px)          | 0.1000     | 0.1100     | 0.3500    ",
        summary,
    )
    self.assertIn(
        "Board Pose ADD (mm)          | 1.5000     | 1.5000     | 1.5000    ",
        summary,
    )
    self.assertIn(
        "Board Pose Reproj (px)       | 0.2000     | 0.2000     | 0.2000    ",
        summary,
    )
    self.assertIn(
        "Triangulation Reproj (px)    | 0.1800     | 0.1800     | 0.4000    ",
        summary,
    )
    self.assertIn(
        "Mono Reprojection (px)       | 0.1500     | 0.1500     | 0.1500    ",
        summary,
    )

    # 6. Advanced Diagnostics Table
    self.assertIn(
        "=== Advanced Physical & Normalized Diagnostics ===", summary
    )
    self.assertIn(
        "Metric                       | Angular (mrad)   | Noise Ratio    |"
        " Tangent Drift @ 1.0m  ",
        summary,
    )
    self.assertIn(
        "Epipolar Error               | 0.1100 mrad      | 0.73x          |"
        " 0.1100 mm             ",
        summary,
    )
    self.assertIn(
        "Triangulation Reproj         | 0.1800 mrad      | 1.20x          |"
        " 0.1800 mm             ",
        summary,
    )
    self.assertIn(
        "Mono Reprojection            | 0.1500 mrad      | 1.00x          |"
        " 0.1500 mm             ",
        summary,
    )

  def test_format_human_readable_validation_summary_empty(self):
    result = eval_pb2.CalibrationEvaluationResult()
    summary = evaluate_calibration.format_human_readable_validation_summary(
        result
    )

    self.assertIn("=== Detections per camera ===", summary)
    self.assertIn("  (No camera detections)", summary)
    self.assertIn("=== Pairwise Epipolar Section ===", summary)
    self.assertIn("  (No epipolar metrics available)", summary)
    self.assertIn(
        "=== Test 1: Board Pose Consistency (Aggregated) ===", summary
    )
    self.assertIn("  (No pose consistency metrics available)", summary)
    self.assertIn(
        "=== Test 2: Triangulation & Reprojection (All Pairs) ===", summary
    )
    self.assertIn("  (No triangulation metrics available)", summary)
    self.assertIn("=== Calibration Validation Summary ===", summary)
    self.assertIn(
        "Test                         | Best       | Mean       | Worst     ",
        summary,
    )
    self.assertIn(
        "Epipolar Error (px)          | 0.0000     | 0.0000     | 0.0000    ",
        summary,
    )
    self.assertIn(
        "Board Pose ADD (mm)          | 0.0000     | 0.0000     | 0.0000    ",
        summary,
    )
    self.assertIn(
        "Board Pose Reproj (px)       | 0.0000     | 0.0000     | 0.0000    ",
        summary,
    )
    self.assertIn(
        "Triangulation Reproj (px)    | 0.0000     | 0.0000     | 0.0000    ",
        summary,
    )
    self.assertIn(
        "Mono Reprojection (px)       | 0.0000     | 0.0000     | 0.0000    ",
        summary,
    )
    # Advanced section must be omitted when empty
    self.assertNotIn(
        "=== Advanced Physical & Normalized Diagnostics ===", summary
    )


if __name__ == "__main__":
  absltest.main()
