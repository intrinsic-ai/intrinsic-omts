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

"""SBL script to calibrate camera to robot using imported waypoints."""

import datetime
import os
import sys

from absl import app
from absl import flags
from google.protobuf import text_format

from intrinsic.math.proto import pose_pb2
from intrinsic.math.python import proto_conversion
from intrinsic.perception.public.proto.v1 import camera_to_robot_calibration_pb2 as calibration_type_pb2
from intrinsic.perception.skills.calibration import sample_calibration_poses_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments
from intrinsic.solutions import execution
from intrinsic.solutions import provided
from intrinsic.world.public.proto import object_world_updates_pb2
from intrinsic.world.python import object_world_ids

# Command line input flags
_ADDRESS = flags.DEFINE_string(
    'address', 'localhost:17080', 'Solution address to connect to.'
)
_ROBOT = flags.DEFINE_string(
    'robot', 'icon', 'Robot / controller resource name in the workcell.'
)
_CAMERA = flags.DEFINE_string(
    'camera', 'orbbec_camera', 'Camera name in the workcell.'
)
_CALIBRATION_OBJECT = flags.DEFINE_string(
    'calibration_object',
    'charuco_9x14_20mm_15mm_dict_5x5',
    'Calibration pattern / object name.',
)
_POSE_ESTIMATOR = flags.DEFINE_string(
    'pose_estimator',
    'charuco_9x14_20mm_15mm_dict_5x5_estimator',
    'Pose estimator name.',
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


def read_input(prompt: str, choices: list[str]) -> str:
  sys.stdout.write(prompt)
  sys.stdout.flush()
  while True:
    val = sys.stdin.readline().strip().lower()
    if val in choices:
      return val


def log_task(message: str) -> bt.Task:
  safe_name = ''.join([c if c.isalnum() else '_' for c in message]).lower()
  return bt.Task(
      action=bt.PythonScript(function_body=f'print("{message}")'),
      name=f'log_{safe_name}',
  )


def main(argv) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  import_path = _IMPORT_WAYPOINTS_FILE.value
  if not import_path:
    import_path = input('Enter file path to import waypoints from: ').strip()
    if not import_path:
      raise ValueError('An import waypoint file must be specified.')

  solution = deployments.connect(address=_ADDRESS.value)

  executive = solution.executive
  skills = solution.skills
  world = solution.world

  try:
    robot_ref = solution.resources[_ROBOT.value]

  except KeyError:
    raise ValueError(
        f"Robot '{_ROBOT.value}' not found in resources. Available resources:"
        f' {dir(solution.resources)}'
    )

  try:
    camera_ref = solution.resources[_CAMERA.value]
  except KeyError:
    raise ValueError(
        f"Camera '{_CAMERA.value}' not found in resources. Available resources:"
        f' {dir(solution.resources)}'
    )

  calibration_object_ref = world.get_object(_CALIBRATION_OBJECT.value)
  if not calibration_object_ref:
    raise ValueError(
        f"Calibration object '{_CALIBRATION_OBJECT.value}' not found in the"
        ' world model.'
    )

  pose_estimator = solution.pose_estimators[_POSE_ESTIMATOR.value]
  if not pose_estimator:
    raise ValueError(
        f"Pose estimator '{_POSE_ESTIMATOR.value}' not found in the solution."
    )

  if _MOVING_CAMERA.value:
    calibration_type = (
        calibration_type_pb2.CAMERA_TO_ROBOT_CALIBRATION_TYPE_MOVING_CAMERA
    )
  else:
    calibration_type = (
        calibration_type_pb2.CAMERA_TO_ROBOT_CALIBRATION_TYPE_STATIONARY_CAMERA
    )

  # Import waypoints
  try:
    workspace_dir = os.environ.get('BUILD_WORKSPACE_DIRECTORY')
    if workspace_dir and not os.path.isabs(import_path):
      import_path = os.path.normpath(
          os.path.join(workspace_dir, import_path)
      )

    result_proto = (
        sample_calibration_poses_pb2.SampleCalibrationPosesResult()
    )
    with open(import_path, 'r') as f:
      text_format.Parse(f.read(), result_proto)
    waypoints = list(result_proto.sample_calibration_poses_result)
    print(
        f'Successfully imported {len(waypoints)} waypoints from'
        f' {import_path}'
    )
  except Exception as e:
    raise ValueError(f'Failed to import waypoints from {import_path}: {e}')

  # Initialize calibration service
  initialize_calibration = skills.ai.intrinsic.initialize_calibration(
      pose_estimator=pose_estimator,
      calibration_object=calibration_object_ref,
      camera_1=camera_ref,
      camera_2=camera_ref,
      camera_3=camera_ref,
      camera_4=camera_ref,
      data_assets_service=provided.ResourceHandle.create(
          name='intrinsic_runtime',
          capabilities=['intrinsic_proto.data.v1.DataAssets'],
      ),
      robot=robot_ref,
  )
  collect_calibration_data_skill = skills.ai.intrinsic.collect_calibration_data
  calibrate_camera_to_robot_skill = (
      skills.ai.intrinsic.calibrate_camera_to_robot
  )

  # Setup calibration subtree
  collect_calibration_data = collect_calibration_data_skill(
      calibration_type=calibration_type,
      calibration_object=calibration_object_ref,
      waypoints=waypoints,
      robot=robot_ref,
      disable_collision_checking=True,
      motion_type=collect_calibration_data_skill.intrinsic_proto.skills.MotionType.MOTION_TYPE_JOINT,
      skip_return_to_base_between_waypoints=True,
  )
  collect_calibration_data.execute_timeout = datetime.timedelta(seconds=600)

  calibrate = calibrate_camera_to_robot_skill(
      calibration_type=calibration_type,
  )

  calibration_children = [
      log_task('Initializing main calibration session...'),
      initialize_calibration,
      log_task('Collecting calibration data...'),
      collect_calibration_data,
      log_task('Running main calibration solver...'),
      calibrate,
      log_task('Main calibration completed successfully.'),
  ]

  calibration = bt.SubTree(
      name='Calibration',
      behavior_tree=bt.Sequence(children=calibration_children),
  )

  try:
    executive.run(calibration, silence_outputs=True)
  except execution.ExecutionFailedError as e:
    print('=== Execution Failed ===')
    print('Error message:', e)
    if executive.operation and executive.operation.proto.HasField('error'):
      print('Raw status message:', executive.operation.proto.error.message)
      from intrinsic.util.status import extended_status_pb2
      from intrinsic.util.status import status_exception

      def print_extended_status(err, indent=''):
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
          print(
              f'{indent}User Report: {err._extended_status.user_report.message}'
          )
        if (
            err._extended_status.HasField('debug_report')
            and err._extended_status.debug_report.message
        ):
          print(
              f'{indent}Debug Report:'
              f' {err._extended_status.debug_report.message}'
          )
        if err._extended_status.context:
          print(f'{indent}Context:')
          for j, ctx in enumerate(err._extended_status.context):
            print(f'{indent}  Context [{j}]:')
            ctx_err = status_exception.ExtendedStatusError.create_from_proto(
                ctx
            )
            print_extended_status(ctx_err, indent + '    ')

      for i, detail in enumerate(executive.operation.proto.error.details):
        print(f'Error Detail [{i}] Type URL: {detail.type_url}')
        if detail.Is(extended_status_pb2.ExtendedStatus.DESCRIPTOR):
          try:
            ext_status = extended_status_pb2.ExtendedStatus()
            detail.Unpack(ext_status)
            ext_err = status_exception.ExtendedStatusError.create_from_proto(
                ext_status
            )
            print(f'--- Unpacked ExtendedStatus [{i}] ---')
            print_extended_status(ext_err)
            print('----------------------------------')
          except Exception as ex:
            print(f'  Failed to unpack ExtendedStatus: {ex}')
    print(executive.get_errors())
    return

  # Retrieve results from blackboard
  res = executive.get_value(calibrate.result).calibration_results[0]

  # Print results
  print('=== Calibration Results ===')
  print(
      'Translation RMS Error:'
      f' {res.translation_root_mean_square_error * 1000.0:.4f} mm'
  )
  print(
      f'Translation Max Error: {res.translation_maximum_error * 1000.0:.4f} mm'
  )
  print(
      'Rotation RMS Error:'
      f' {res.rotation_root_mean_square_error_in_degrees:.4f} deg'
  )
  print(f'Rotation Max Error: {res.rotation_maximum_error_in_degrees:.4f} deg')

  if res.HasField('stationary_camera_result_poses'):
    print('Stationary Camera Result Poses:')
    print(res.stationary_camera_result_poses)
  elif res.HasField('moving_camera_result_poses'):
    print('Moving Camera Result Poses:')
    print(res.moving_camera_result_poses)

  save_pose = read_input(
      '\nDo you want to persist the new camera pose? [y/n]: ', ['y', 'n']
  )
  if save_pose == 'y':
    try:
      object_world_updates = object_world_updates_pb2.ObjectWorldUpdates()
      objects_list = world.list_object_names()
      camera_name = _CAMERA.value
      if camera_name not in objects_list:
        raise ValueError(f'{camera_name} not in list of available objects.')
      scene_object = world.get_object(
          object_world_ids.WorldObjectName(camera_name)
      )
      if _MOVING_CAMERA.value:
        new_pose_proto = res.moving_camera_result_poses.flange_t_camera
      else:
        new_pose_proto = res.stationary_camera_result_poses.base_t_camera

      a_t_b_pose = pose_pb2.Pose()
      a_t_b_pose.ParseFromString(new_pose_proto.SerializeToString())

      update_request = object_world_updates_pb2.UpdateTransformRequest(
          node_a=scene_object.parent.transform_node_reference,
          node_b=scene_object.transform_node_reference,
          a_t_b=a_t_b_pose,
          node_to_update=scene_object.transform_node_reference,
      )

      object_world_update = object_world_updates_pb2.ObjectWorldUpdate(
          update_transform=update_request
      )
      object_world_updates.updates.append(object_world_update)

      print(
          'You need to copy-paste the following proto to the appropriate'
          ' world_updates.pbtxt file of your solution.\n\n'
      )
      print(object_world_updates)
    except Exception as e:
      print(f'Error fetching camera pose updates: {e}')


if __name__ == '__main__':
  app.run(main)
