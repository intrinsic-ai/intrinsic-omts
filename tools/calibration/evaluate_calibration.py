"""SBL script to execute calibration_evaluation skill via Behavior Tree."""

import datetime
import os
import sys
from typing import Any

from absl import app
from absl import flags
from google.protobuf import text_format

from intrinsic.math.python import proto_conversion
from intrinsic.motion_planning.public.proto.v1 import geometric_constraints_pb2
from intrinsic.perception.public.proto.v1 import camera_to_robot_calibration_pb2 as calibration_type_pb2
from intrinsic.perception.public.proto.v1 import charuco_pattern_pb2
from intrinsic.perception.skills.calibration import calibration_evaluation_pb2 as eval_pb2
from intrinsic.perception.skills.calibration import sample_calibration_poses_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments
from intrinsic.solutions import execution
from intrinsic.solutions import provided
from intrinsic.util.status import extended_status_pb2
from intrinsic.util.status import status_exception

# Command line flags
_ADDRESS = flags.DEFINE_string(
    'address',
    'localhost:17080',
    'gRPC address of the running SBL solution deployment.',
)
_ROBOT = flags.DEFINE_string(
    'robot', 'icon', 'Robot / controller resource name in the workcell.'
)
_CAMERA = flags.DEFINE_string(
    'camera', 'orbbec_camera', 'Camera name in the workcell.'
)
_CALIBRATION_OBJECT = flags.DEFINE_string(
    'calibration_object',
    'charuco_22x30_25mm_18mm_dict_5x5',
    'Calibration pattern / object name (e.g.'
    ' charuco_9x14_20mm_15mm_dict_5x5,'
    ' charuco_22x30_25mm_18mm_dict_5x5, or'
    ' charuco_11x15_35mm_26mm_dict_4x4).',
)
_POSE_ESTIMATOR = flags.DEFINE_string(
    'pose_estimator',
    'charuco_22x30_25mm_18mm_dict_5x5_estimator',
    'Pose estimator name (e.g.'
    ' charuco_9x14_20mm_15mm_dict_5x5_estimator,'
    ' charuco_22x30_25mm_18mm_dict_5x5_estimator, or'
    ' charuco_11x15_35mm_26mm_dict_4x4_estimator).',
)
_MOVING_CAMERA = flags.DEFINE_bool(
    'moving_camera',
    True,
    'Whether it is a moving camera calibration. If false, it is stationary.',
)
_IMPORT_WAYPOINTS_FILE = flags.DEFINE_string(
    'import_waypoints_file',
    '',
    'Import manual waypoints from this local file path.',
)
_SKIP_MOTION = flags.DEFINE_bool(
    'skip_motion',
    False,
    'If true, skip robot motion and evaluate on current camera view.',
)
_DISABLE_COLLISION_CHECKING = flags.DEFINE_bool(
    'disable_collision_checking',
    True,
    'Whether to disable collision checking during evaluation motions (useful for known safe calibration waypoints).',
)
_FAIL_IF_INVALID = flags.DEFINE_bool(
    'fail_if_invalid',
    False,
    'Whether the skill should fail with an error if quality gates fail.',
)
_MIN_COMMON_CORNERS = flags.DEFINE_integer(
    'min_common_corners',
    4,
    'Minimum common corners required to evaluate a camera pair.',
)
_SAVE_REPORT_FILE = flags.DEFINE_string(
    'save_report_file',
    '',
    'Optional local file path to save the evaluation result proto (.pbtxt).',
)

# Threshold flags (Quality Gates)
_MAX_MEAN_EPIPOLAR_ERROR_PX = flags.DEFINE_float(
    'max_mean_epipolar_error_px',
    1.5,
    'Maximum allowed mean epipolar error in pixels.',
)
_MAX_RMS_EPIPOLAR_ERROR_PX = flags.DEFINE_float(
    'max_rms_epipolar_error_px',
    2.0,
    'Maximum allowed RMS epipolar error in pixels.',
)
_MAX_MEAN_POSE_TRANS_ERROR_M = flags.DEFINE_float(
    'max_mean_pose_translation_error_m',
    0.015,
    'Maximum allowed mean board pose translation error in meters.',
)
_MAX_MEAN_POSE_ROT_ERROR_DEG = flags.DEFINE_float(
    'max_mean_pose_rotation_error_deg',
    1.5,
    'Maximum allowed mean board pose rotation error in degrees.',
)
_MAX_MEAN_REPROJ_ERROR_PX = flags.DEFINE_float(
    'max_mean_reprojection_error_px',
    1.5,
    'Maximum allowed mean triangulation reprojection error in pixels.',
)
_MAX_RMS_REPROJ_ERROR_PX = flags.DEFINE_float(
    'max_rms_reprojection_error_px',
    2.0,
    'Maximum allowed RMS triangulation reprojection error in pixels.',
)
_PUBLISH_ANNOTATED_IMAGES = flags.DEFINE_bool(
    'publish_annotated_images',
    True,
    'Whether to render and publish annotated visual diagnostic artifacts.',
)
_ARROW_MAGNIFICATION = flags.DEFINE_float(
    'arrow_magnification',
    10.0,
    'Magnification scale for residual error quiver arrows.',
)


def log_task(message: str) -> bt.Task:
  safe_name = ''.join([c if c.isalnum() else '_' for c in message]).lower()
  return bt.Task(
      action=bt.PythonScript(function_body=f'print("{message}")'),
      name=f'log_{safe_name}',
  )


def print_extended_status(err: status_exception.ExtendedStatusError, indent: str = '') -> None:
  status_code = err._extended_status.status_code
  print(f'{indent}StatusCode: {status_code.component}:{status_code.code}')
  if err._extended_status.title:
    print(f'{indent}Title: {err._extended_status.title}')
  if err._extended_status.HasField('timestamp'):
    print(
        f'{indent}Timestamp:'
        f' {err._extended_status.timestamp.ToDatetime().strftime("%c")}'
    )
  if (
      err._extended_status.HasField('user_report')
      and err._extended_status.user_report.message
  ):
    print(f'{indent}User Report: {err._extended_status.user_report.message}')
  if (
      err._extended_status.HasField('debug_report')
      and err._extended_status.debug_report.message
  ):
    print(f'{indent}Debug Report:'
          f' {err._extended_status.debug_report.message}')
  if err._extended_status.context:
    print(f'{indent}Context:')
    for j, ctx in enumerate(err._extended_status.context):
      print(f'{indent}  Context [{j}]:')
      ctx_err = status_exception.ExtendedStatusError.create_from_proto(ctx)
      print_extended_status(ctx_err, indent + '    ')


def format_detections_summary_section(
    result: eval_pb2.CalibrationEvaluationResult,
) -> str:
  """Formats corner detections per camera."""
  summary = '=== Detections per camera ===\n'
  if not result.camera_diagnostics:
    summary += '  (No camera detections)\n'
  else:
    for diag in result.camera_diagnostics:
      summary += (
          f"Camera '{diag.camera_name}': {diag.num_detected_corners} corners"
          ' detected\n'
      )
  return summary


def format_pairwise_epipolar_summary_section(
    result: eval_pb2.CalibrationEvaluationResult,
) -> str:
  """Formats pairwise epipolar error metrics and sensor PnP errors."""
  mono_pnp_errors = {}
  for diag in result.camera_diagnostics:
    if diag.HasField('mean_mono_reprojection_error_px'):
      mono_pnp_errors[diag.camera_name] = diag.mean_mono_reprojection_error_px

  summary = '=== Pairwise Epipolar Section ===\n'
  if not result.epipolar_metrics:
    summary += '  (No epipolar metrics available)\n'
  else:
    for ep in result.epipolar_metrics:
      summary += f'Pair {ep.camera_0} - {ep.camera_1}:\n'
      summary += f'  Common corners: {ep.num_common_corners}\n'
      summary += f'  Mean epipolar error: {ep.mean_epipolar_error_px:.4f} px\n'
      summary += f'  Max epipolar error:  {ep.max_epipolar_error_px:.4f} px\n'
      summary += f'  RMS epipolar error:  {ep.rms_epipolar_error_px:.4f} px\n'
      if ep.camera_0 in mono_pnp_errors:
        summary += (
            f'  Sensor {ep.camera_0} PnP error:'
            f' {mono_pnp_errors[ep.camera_0]:.4f} px\n'
        )
      if ep.camera_1 in mono_pnp_errors:
        summary += (
            f'  Sensor {ep.camera_1} PnP error:'
            f' {mono_pnp_errors[ep.camera_1]:.4f} px\n'
        )
  return summary


def format_pose_consistency_summary_section(
    result: eval_pb2.CalibrationEvaluationResult,
) -> str:
  """Formats aggregated board pose consistency error metrics per sensor."""
  summary = '=== Test 1: Board Pose Consistency (Aggregated) ===\n'
  pose_cam_names = []
  pose_stats = {}
  for pm in result.pose_consistency_metrics:
    for cam in (pm.camera_0, pm.camera_1):
      if cam not in pose_stats:
        pose_cam_names.append(cam)
        pose_stats[cam] = {
            'count': 0,
            'sum_trans_m': 0.0,
            'sum_rot_deg': 0.0,
            'sum_add_m': 0.0,
            'sum_cross_reproj_px': 0.0,
        }
      st = pose_stats[cam]
      st['count'] += 1
      st['sum_trans_m'] += pm.translation_error_m
      st['sum_rot_deg'] += pm.rotation_error_deg
      st['sum_add_m'] += pm.add_error_m
      st['sum_cross_reproj_px'] += pm.cross_reprojection_error_px

  if not pose_cam_names:
    summary += '  (No pose consistency metrics available)\n'
  else:
    for cam_name in pose_cam_names:
      st = pose_stats[cam_name]
      count = st['count']
      summary += (
          f'  Sensor {cam_name} Average Errors (against all other sensors):\n'
      )
      summary += (
          '    Translation Error:'
          f' {(st["sum_trans_m"] / count) * 1000.0:.4f} mm\n'
      )
      summary += f'    Rotation Error:    {st["sum_rot_deg"] / count:.4f} deg\n'
      summary += (
          f'    ADD Error:         {(st["sum_add_m"] / count) * 1000.0:.4f}'
          ' mm\n'
      )
      summary += (
          f'    Pose Reproj Error: {st["sum_cross_reproj_px"] / count:.4f} px\n'
      )
  return summary


def format_triangulation_summary_section(
    result: eval_pb2.CalibrationEvaluationResult,
) -> str:
  """Formats multi-view triangulation and reprojection errors."""
  summary = '=== Test 2: Triangulation & Reprojection (All Pairs) ===\n'
  pair_order = []
  grouped_triangulation = {}
  common_corners_by_pair = {}

  for tm in result.triangulation_metrics:
    key = (tm.triangulated_by_camera_0, tm.triangulated_by_camera_1)
    if key not in grouped_triangulation:
      pair_order.append(key)
      grouped_triangulation[key] = []
      common_corners_by_pair[key] = tm.num_evaluated_corners
    grouped_triangulation[key].append(tm)

  if not pair_order:
    summary += '  (No triangulation metrics available)\n'
  else:
    for pair in pair_order:
      corners = common_corners_by_pair[pair]
      summary += (
          f'\n  Triangulating using pair {pair[0]} - {pair[1]} (Common corners:'
          f' {corners})\n'
      )
      for tm in grouped_triangulation[pair]:
        summary += f'    Reprojection onto Sensor {tm.reprojected_to_camera}:\n'
        summary += f'      Compared corners: {tm.num_evaluated_corners}\n'
        summary += (
            f'      Mean error:       {tm.mean_reprojection_error_px:.4f} px\n'
        )
        summary += (
            f'      Max error:        {tm.max_reprojection_error_px:.4f} px\n'
        )
        summary += (
            f'      RMS error:        {tm.rms_reprojection_error_px:.4f} px\n'
        )
  return summary


def format_validation_table_summary_section(
    result: eval_pb2.CalibrationEvaluationResult,
) -> str:
  """Formats the calibration validation summary table (Best, Mean, Worst)."""
  summary = '=== Calibration Validation Summary ===\n'
  summary += f'{"Test":<28} | {"Best":<10} | {"Mean":<10} | {"Worst":<10}\n'
  summary += '-' * 67 + '\n'

  def format_row(name: str, best: str, mean: str, worst: str) -> str:
    return f'{name:<28} | {best:<10} | {mean:<10} | {worst:<10}\n'

  # Row 1: Epipolar Error (px)
  ep_best = '0.0000'
  ep_mean = '0.0000'
  ep_worst = '0.0000'
  if result.epipolar_metrics:
    min_v = float('inf')
    max_v = float('-inf')
    for ep in result.epipolar_metrics:
      if ep.HasField('mean_epipolar_error_px'):
        min_v = min(min_v, ep.mean_epipolar_error_px)
      if ep.HasField('max_epipolar_error_px'):
        max_v = max(max_v, ep.max_epipolar_error_px)
      elif ep.HasField('mean_epipolar_error_px'):
        max_v = max(max_v, ep.mean_epipolar_error_px)
    if min_v != float('inf'):
      ep_best = f'{min_v:.4f}'
    if max_v != float('-inf'):
      ep_worst = f'{max_v:.4f}'
  if result.HasField('mean_epipolar_error_px'):
    ep_mean = f'{result.mean_epipolar_error_px:.4f}'
    if ep_best == '0.0000':
      ep_best = ep_mean
    if ep_worst == '0.0000':
      ep_worst = ep_mean
  summary += format_row('Epipolar Error (px)', ep_best, ep_mean, ep_worst)

  # Row 2: Board Pose ADD (mm)
  pose_add_best = '0.0000'
  pose_add_mean = '0.0000'
  pose_add_worst = '0.0000'
  if result.pose_consistency_metrics:
    min_v = float('inf')
    max_v = float('-inf')
    for pm in result.pose_consistency_metrics:
      if pm.HasField('add_error_m'):
        min_v = min(min_v, pm.add_error_m * 1000.0)
        max_v = max(max_v, pm.add_error_m * 1000.0)
    if min_v != float('inf'):
      pose_add_best = f'{min_v:.4f}'
    if max_v != float('-inf'):
      pose_add_worst = f'{max_v:.4f}'
  if result.HasField('mean_pose_add_error_m'):
    pose_add_mean = f'{result.mean_pose_add_error_m * 1000.0:.4f}'
    if pose_add_best == '0.0000':
      pose_add_best = pose_add_mean
    if pose_add_worst == '0.0000':
      pose_add_worst = pose_add_mean
  summary += format_row(
      'Board Pose ADD (mm)', pose_add_best, pose_add_mean, pose_add_worst
  )

  # Row 3: Board Pose Reproj (px)
  pose_reproj_best = '0.0000'
  pose_reproj_mean = '0.0000'
  pose_reproj_worst = '0.0000'
  if result.pose_consistency_metrics:
    min_v = float('inf')
    max_v = float('-inf')
    for pm in result.pose_consistency_metrics:
      if pm.HasField('cross_reprojection_error_px'):
        min_v = min(min_v, pm.cross_reprojection_error_px)
        max_v = max(max_v, pm.cross_reprojection_error_px)
    if min_v != float('inf'):
      pose_reproj_best = f'{min_v:.4f}'
    if max_v != float('-inf'):
      pose_reproj_worst = f'{max_v:.4f}'
  if result.HasField('mean_pose_cross_reprojection_error_px'):
    pose_reproj_mean = f'{result.mean_pose_cross_reprojection_error_px:.4f}'
    if pose_reproj_best == '0.0000':
      pose_reproj_best = pose_reproj_mean
    if pose_reproj_worst == '0.0000':
      pose_reproj_worst = pose_reproj_mean
  summary += format_row(
      'Board Pose Reproj (px)',
      pose_reproj_best,
      pose_reproj_mean,
      pose_reproj_worst,
  )

  # Row 4: Triangulation Reproj (px)
  tri_best = '0.0000'
  tri_mean = '0.0000'
  tri_worst = '0.0000'
  if result.triangulation_metrics:
    min_v = float('inf')
    max_v = float('-inf')
    for tm in result.triangulation_metrics:
      if tm.HasField('mean_reprojection_error_px'):
        min_v = min(min_v, tm.mean_reprojection_error_px)
      if tm.HasField('max_reprojection_error_px'):
        max_v = max(max_v, tm.max_reprojection_error_px)
      elif tm.HasField('mean_reprojection_error_px'):
        max_v = max(max_v, tm.mean_reprojection_error_px)
    if min_v != float('inf'):
      tri_best = f'{min_v:.4f}'
    if max_v != float('-inf'):
      tri_worst = f'{max_v:.4f}'
  if result.HasField('mean_triangulation_reprojection_error_px'):
    tri_mean = f'{result.mean_triangulation_reprojection_error_px:.4f}'
    if tri_best == '0.0000':
      tri_best = tri_mean
    if tri_worst == '0.0000':
      tri_worst = tri_mean
  summary += format_row(
      'Triangulation Reproj (px)', tri_best, tri_mean, tri_worst
  )

  # Row 5: Mono Reprojection (px)
  mono_best = '0.0000'
  mono_mean = '0.0000'
  mono_worst = '0.0000'
  if result.camera_diagnostics:
    min_v = float('inf')
    max_v = float('-inf')
    for diag in result.camera_diagnostics:
      if diag.HasField('mean_mono_reprojection_error_px'):
        min_v = min(min_v, diag.mean_mono_reprojection_error_px)
      if diag.HasField('max_mono_reprojection_error_px'):
        max_v = max(max_v, diag.max_mono_reprojection_error_px)
      elif diag.HasField('mean_mono_reprojection_error_px'):
        max_v = max(max_v, diag.mean_mono_reprojection_error_px)
    if min_v != float('inf'):
      mono_best = f'{min_v:.4f}'
    if max_v != float('-inf'):
      mono_worst = f'{max_v:.4f}'
  if result.HasField('mean_mono_reprojection_error_px'):
    mono_mean = f'{result.mean_mono_reprojection_error_px:.4f}'
    if mono_best == '0.0000':
      mono_best = mono_mean
    if mono_worst == '0.0000':
      mono_worst = mono_mean
  summary += format_row(
      'Mono Reprojection (px)', mono_best, mono_mean, mono_worst
  )

  return summary


def format_advanced_diagnostics_summary_section(
    result: eval_pb2.CalibrationEvaluationResult,
) -> str | None:
  """Formats physical and normalized advanced metrics."""
  has_advanced = (
      result.HasField('mean_angular_epipolar_error_mrad')
      or result.HasField('mean_subpixel_noise_ratio')
      or result.HasField('mean_tangent_drift_mm_per_m')
      or result.HasField('mean_angular_triangulation_reprojection_error_mrad')
      or result.HasField('mean_angular_mono_reprojection_error_mrad')
  )
  if not has_advanced:
    return None

  summary = '=== Advanced Physical & Normalized Diagnostics ===\n'
  summary += (
      f'{"Metric":<28} | {"Angular (mrad)":<16} | {"Noise Ratio":<14} |'
      ' Tangent Drift @ 1.0m  \n'
  )
  summary += '-' * 89 + '\n'

  def format_advanced_row(
      metric: str, angular: str, noise: str, drift: str
  ) -> str:
    return f'{metric:<28} | {angular:<16} | {noise:<14} | {drift:<22}\n'

  # Row 1: Epipolar Error
  ep_ang = (
      f'{result.mean_angular_epipolar_error_mrad:.4f} mrad'
      if result.HasField('mean_angular_epipolar_error_mrad')
      else 'N/A'
  )
  ep_noi = (
      f'{result.mean_subpixel_noise_ratio:.2f}x'
      if result.HasField('mean_subpixel_noise_ratio')
      else 'N/A'
  )
  ep_drf = (
      f'{result.mean_tangent_drift_mm_per_m:.4f} mm'
      if result.HasField('mean_tangent_drift_mm_per_m')
      else 'N/A'
  )
  summary += format_advanced_row('Epipolar Error', ep_ang, ep_noi, ep_drf)

  # Row 2: Triangulation Reproj
  tri_ang = (
      f'{result.mean_angular_triangulation_reprojection_error_mrad:.4f} mrad'
      if result.HasField('mean_angular_triangulation_reprojection_error_mrad')
      else 'N/A'
  )
  tri_noi = 'N/A'
  total_tri_noise = 0.0
  total_tri_corners = 0
  for tm in result.triangulation_metrics:
    if tm.HasField('subpixel_noise_ratio') and tm.num_evaluated_corners > 0:
      total_tri_noise += tm.subpixel_noise_ratio * tm.num_evaluated_corners
      total_tri_corners += tm.num_evaluated_corners
  if total_tri_corners > 0:
    tri_noi = f'{total_tri_noise / total_tri_corners:.2f}x'
  elif result.HasField('mean_triangulation_reprojection_error_px'):
    tri_noi = f'{result.mean_triangulation_reprojection_error_px / 0.15:.2f}x'

  tri_drf = 'N/A'
  total_tri_drift = 0.0
  total_tri_corners_drift = 0
  for tm in result.triangulation_metrics:
    if (
        tm.HasField('mean_tangent_drift_mm_per_m')
        and tm.num_evaluated_corners > 0
    ):
      total_tri_drift += (
          tm.mean_tangent_drift_mm_per_m * tm.num_evaluated_corners
      )
      total_tri_corners_drift += tm.num_evaluated_corners
  if total_tri_corners_drift > 0:
    tri_drf = f'{total_tri_drift / total_tri_corners_drift:.4f} mm'
  elif result.HasField('mean_angular_triangulation_reprojection_error_mrad'):
    tri_drf = (
        f'{result.mean_angular_triangulation_reprojection_error_mrad:.4f} mm'
    )
  summary += format_advanced_row(
      'Triangulation Reproj', tri_ang, tri_noi, tri_drf
  )

  # Row 3: Mono Reprojection
  mono_ang = (
      f'{result.mean_angular_mono_reprojection_error_mrad:.4f} mrad'
      if result.HasField('mean_angular_mono_reprojection_error_mrad')
      else 'N/A'
  )
  mono_noi = 'N/A'
  total_mono_noise = 0.0
  num_mono_diag = 0
  for diag in result.camera_diagnostics:
    if diag.HasField('mono_subpixel_noise_ratio'):
      total_mono_noise += diag.mono_subpixel_noise_ratio
      num_mono_diag += 1
  if num_mono_diag > 0:
    mono_noi = f'{total_mono_noise / num_mono_diag:.2f}x'
  elif result.HasField('mean_mono_reprojection_error_px'):
    mono_noi = f'{result.mean_mono_reprojection_error_px / 0.15:.2f}x'

  mono_drf = (
      f'{result.mean_angular_mono_reprojection_error_mrad:.4f} mm'
      if result.HasField('mean_angular_mono_reprojection_error_mrad')
      else 'N/A'
  )
  summary += format_advanced_row(
      'Mono Reprojection', mono_ang, mono_noi, mono_drf
  )

  return summary


def format_human_readable_validation_summary_sections(
    result: eval_pb2.CalibrationEvaluationResult,
) -> list[str]:
  """Returns formatted human-readable summary sections matching skill output."""
  sections = [
      format_detections_summary_section(result),
      format_pairwise_epipolar_summary_section(result),
      format_pose_consistency_summary_section(result),
      format_triangulation_summary_section(result),
      format_validation_table_summary_section(result),
  ]
  adv = format_advanced_diagnostics_summary_section(result)
  if adv is not None:
    sections.append(adv)
  return sections


def format_human_readable_validation_summary(
    result: eval_pb2.CalibrationEvaluationResult,
) -> str:
  """Returns a single string with all human-readable summary sections."""
  sections = format_human_readable_validation_summary_sections(result)
  return '\n'.join(sections)


def print_evaluation_summary(
    result: eval_pb2.CalibrationEvaluationResult,
) -> None:
  """Prints human-readable summary of calibration evaluation metrics."""
  print('\n' + '=' * 78)
  print('                   CALIBRATION EVALUATION REPORT')
  print('=' * 78)

  verdict = 'PASSED' if result.is_valid else 'FAILED / QUALITY GATES NOT MET'
  print(f'Overall Quality Gate Status : {verdict}')

  if result.quality_gate_failures:
    print('\n[Quality Gate Failures]:')
    for failure in result.quality_gate_failures:
      print(f'  [FAIL] {failure}')

  if result.warnings:
    print('\n[Warnings]:')
    for warning in result.warnings:
      print(f'  [WARN] {warning}')

  for section in format_human_readable_validation_summary_sections(result):
    print('\n' + section.rstrip('\n'))

  print('=' * 78 + '\n')


def main(argv) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  import_path = _IMPORT_WAYPOINTS_FILE.value
  if not import_path and not _SKIP_MOTION.value:
    user_resp = input(
        'Enter file path to import waypoints from (or leave blank to evaluate current pose): '
    ).strip()
    if user_resp:
      import_path = user_resp

  print(f'Connecting to deployment at {_ADDRESS.value}...')
  solution = deployments.connect(address=_ADDRESS.value)

  executive = solution.executive
  skills = solution.skills
  world = solution.world

  try:
    robot_ref = solution.resources[_ROBOT.value]
  except KeyError:
    raise ValueError(
        f"Robot '{_ROBOT.value}' not found in resources. Available:"
        f' {dir(solution.resources)}'
    )

  try:
    camera_ref = solution.resources[_CAMERA.value]
  except KeyError:
    raise ValueError(
        f"Camera '{_CAMERA.value}' not found in resources. Available:"
        f' {dir(solution.resources)}'
    )

  calibration_object_ref = world.get_object(_CALIBRATION_OBJECT.value)
  if not calibration_object_ref:
    raise ValueError(
        f"Calibration object '{_CALIBRATION_OBJECT.value}' not found in world."
    )

  # Extract ChArUco pattern parameters
  charuco_pattern = charuco_pattern_pb2.CharucoPattern()
  if '22x30' in _CALIBRATION_OBJECT.value:
    charuco_pattern.squares_x = 30
    charuco_pattern.squares_y = 22
    charuco_pattern.square_length = 0.025
    charuco_pattern.marker_length = 0.018
    charuco_pattern.dictionary = charuco_pattern_pb2.DICT_5X5_1000
  elif '9x14' in _CALIBRATION_OBJECT.value:
    charuco_pattern.squares_x = 14
    charuco_pattern.squares_y = 9
    charuco_pattern.square_length = 0.020
    charuco_pattern.marker_length = 0.015
    charuco_pattern.dictionary = charuco_pattern_pb2.DICT_5X5_250
  elif '11x15' in _CALIBRATION_OBJECT.value:
    charuco_pattern.squares_x = 15
    charuco_pattern.squares_y = 11
    charuco_pattern.square_length = 0.035
    charuco_pattern.marker_length = 0.026
    charuco_pattern.dictionary = charuco_pattern_pb2.DICT_4X4_100
  else:
    charuco_pattern.squares_x = 30
    charuco_pattern.squares_y = 22
    charuco_pattern.square_length = 0.025
    charuco_pattern.marker_length = 0.018
    charuco_pattern.dictionary = charuco_pattern_pb2.DICT_5X5_1000

  # Load waypoints if provided
  waypoints = []
  if import_path:
    workspace_dir = os.environ.get('BUILD_WORKSPACE_DIRECTORY')
    if workspace_dir and not os.path.isabs(import_path):
      import_path = os.path.normpath(os.path.join(workspace_dir, import_path))

    result_proto = sample_calibration_poses_pb2.SampleCalibrationPosesResult()
    with open(import_path, 'r') as f:
      text_format.Parse(f.read(), result_proto)
    waypoints = list(result_proto.sample_calibration_poses_result)
    print(f'Successfully loaded {len(waypoints)} waypoints from {import_path}')

  # Prepare evaluation params proto
  eval_params = eval_pb2.CalibrationEvaluationParams()
  eval_params.fail_if_invalid = _FAIL_IF_INVALID.value
  eval_params.min_common_corners = _MIN_COMMON_CORNERS.value
  eval_params.charuco_pattern.CopyFrom(charuco_pattern)

  # Acceptance quality thresholds
  eval_params.epipolar_thresholds.max_mean_epipolar_error_px = (
      _MAX_MEAN_EPIPOLAR_ERROR_PX.value
  )
  eval_params.epipolar_thresholds.max_rms_epipolar_error_px = (
      _MAX_RMS_EPIPOLAR_ERROR_PX.value
  )
  eval_params.pose_thresholds.max_mean_translation_error_m = (
      _MAX_MEAN_POSE_TRANS_ERROR_M.value
  )
  eval_params.pose_thresholds.max_mean_rotation_error_deg = (
      _MAX_MEAN_POSE_ROT_ERROR_DEG.value
  )
  eval_params.triangulation_thresholds.max_mean_reprojection_error_px = (
      _MAX_MEAN_REPROJ_ERROR_PX.value
  )
  eval_params.triangulation_thresholds.max_rms_reprojection_error_px = (
      _MAX_RMS_REPROJ_ERROR_PX.value
  )
  eval_params.visualization_options.publish_annotated_images = (
      _PUBLISH_ANNOTATED_IMAGES.value
  )
  eval_params.visualization_options.arrow_magnification = (
      _ARROW_MAGNIFICATION.value
  )

  # Construct Behavior Tree Sequence
  bt_children = []

  if waypoints and not _SKIP_MOTION.value:
    print(f'Building Multi-View observation set across {len(waypoints)} waypoints...')
    capture_set = eval_params.capture_sets.add()
    capture_set.capture_id = 'eval_multiview_capture_set'

    bt_children.append(log_task(f'Starting evaluation motion sequence across {len(waypoints)} waypoints...'))

    for i, wp in enumerate(waypoints):
      view_key = f'eval_cap_view_{i + 1}'
      view_name = f'{_CAMERA.value}_view_{i + 1}'

      obs_input = capture_set.camera_observations.add()
      obs_input.camera_name = view_name
      obs_input.camera_resource_handle.CopyFrom(camera_ref.proto)
      obs_input.capture_result_location.store = 'capture_results'
      obs_input.capture_result_location.key = view_key

      motion_segment = (
          skills.ai.intrinsic.move_robot.intrinsic_proto.skills.MotionSegment(
              joint_position=wp.joint_position,
              motion_type=skills.ai.intrinsic.move_robot.intrinsic_proto.skills.MotionSegment.JOINT,
          )
      )
      if _DISABLE_COLLISION_CHECKING.value:
        motion_segment.wrapped_message.collision_settings.disable_collision_checking = True
      move_task = skills.ai.intrinsic.move_robot(
          robot=robot_ref,
          motion_segments=[motion_segment],
      )

      capture_task = skills.ai.intrinsic.capture_images(
          camera=camera_ref,
          capture_result_location=obs_input.capture_result_location,
      )

      bt_children.extend([
          log_task(f'Moving robot to evaluation waypoint {i + 1}/{len(waypoints)}...'),
          move_task,
          log_task(f'Capturing sensor frame {i + 1}/{len(waypoints)}...'),
          capture_task,
      ])

  else:
    print('Evaluating single camera frame at current pose...')
    capture_set = eval_params.capture_sets.add()
    capture_set.capture_id = 'eval_current_pose_capture_set'

    obs_input = capture_set.camera_observations.add()
    obs_input.camera_name = _CAMERA.value
    obs_input.camera_resource_handle.CopyFrom(camera_ref.proto)
    obs_input.capture_result_location.store = 'capture_results'
    obs_input.capture_result_location.key = 'eval_cap_view_current'

    capture_task = skills.ai.intrinsic.capture_images(
        camera=camera_ref,
        capture_result_location=obs_input.capture_result_location,
    )

    bt_children.extend([
        log_task('Capturing frame at current robot pose...'),
        capture_task,
    ])

  # Instantiate sideloaded calibration_evaluation skill
  try:
    calibration_eval_skill = skills.ai.intrinsic.calibration_evaluation
  except AttributeError:
    try:
      calibration_eval_skill = skills.ai.intrinsic.calibration_evaluation_skill
    except AttributeError:
      raise RuntimeError(
          "Could not locate 'calibration_evaluation' in solution.skills. "
          f'Available skills: {dir(skills.ai.intrinsic)}'
      )

  eval_task = calibration_eval_skill(
      fail_if_invalid=eval_params.fail_if_invalid,
      min_common_corners=eval_params.min_common_corners,
      charuco_pattern=eval_params.charuco_pattern,
      epipolar_thresholds=eval_params.epipolar_thresholds,
      pose_thresholds=eval_params.pose_thresholds,
      triangulation_thresholds=eval_params.triangulation_thresholds,
      observability_thresholds=eval_params.observability_thresholds,
      visualization_options=eval_params.visualization_options,
      capture_sets=list(eval_params.capture_sets),
  )

  bt_children.extend([
      log_task('Executing calibration evaluation analysis...'),
      eval_task,
      log_task('Calibration evaluation analysis complete.'),
  ])

  evaluation_tree = bt.SubTree(
      name='CalibrationEvaluation',
      behavior_tree=bt.Sequence(children=bt_children),
  )

  print(f'Executing calibration evaluation behavior tree ({len(bt_children)} steps) via executive...')
  try:
    executive.run(evaluation_tree)
    print('Behavior tree execution finished successfully.')
  except execution.ExecutionFailedError as e:
    print('\n=== Behavior Tree Execution Failed ===')
    print('Error message:', e)
    if executive.operation and executive.operation.proto.HasField('error'):
      print('Raw status message:', executive.operation.proto.error.message)
      for i, detail in enumerate(executive.operation.proto.error.details):
        print(f'Error Detail [{i}] Type URL: {detail.type_url}')
        if detail.Is(extended_status_pb2.ExtendedStatus.DESCRIPTOR):
          try:
            ext_status = extended_status_pb2.ExtendedStatus()
            detail.Unpack(ext_status)
            ext_err = status_exception.ExtendedStatusError.create_from_proto(ext_status)
            print(f'--- Unpacked ExtendedStatus [{i}] ---')
            print_extended_status(ext_err)
            print('----------------------------------')
          except Exception as ex:
            print(f'  Failed to unpack ExtendedStatus: {ex}')
    print(executive.get_errors())
    return

  # Retrieve result from blackboard
  eval_result = None
  try:
    eval_result = executive.operation.blackboard.get_value(eval_task.result)
  except Exception:
    try:
      eval_result = executive.get_value(eval_task.result)
    except Exception as ex:
      print(f'Could not retrieve eval_task result: {ex}')
      return

  if eval_result is not None:
    print_evaluation_summary(eval_result)

    if _SAVE_REPORT_FILE.value:
      save_path = _SAVE_REPORT_FILE.value
      workspace_dir = os.environ.get('BUILD_WORKSPACE_DIRECTORY')
      if workspace_dir and not os.path.isabs(save_path):
        save_path = os.path.normpath(os.path.join(workspace_dir, save_path))
      with open(save_path, 'w') as f:
        f.write(text_format.MessageToString(eval_result))
      print(f'Saved evaluation report to {save_path}')


if __name__ == '__main__':
  app.run(main)

