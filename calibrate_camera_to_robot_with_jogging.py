"""SBL script for testing camera-to-robot calibration on solution."""

import datetime
import sys
import termios
import tty

from absl import app
from absl import flags
from google.protobuf import text_format
import grpc

from intrinsic.icon.proto import joint_space_pb2
from intrinsic.icon.proto.v1 import service_pb2_grpc
from intrinsic.icon.python import create_action_utils
from intrinsic.icon.python import errors as icon_errors
from intrinsic.icon.python import icon_api
from intrinsic.math.python import proto_conversion
from intrinsic.motion_planning.public.proto.v1 import geometric_constraints_pb2
from intrinsic.perception.public.proto.v1 import camera_to_robot_calibration_pb2 as calibration_type_pb2
from intrinsic.perception.skills.calibration import sample_calibration_poses_pb2
from intrinsic.skills.proto import skills_pb2
from intrinsic.solutions import behavior_tree as bt
from intrinsic.solutions import deployments
from intrinsic.solutions import execution
from intrinsic.solutions import provided
from intrinsic.util.grpc import connection
from intrinsic.util.grpc import interceptor
from intrinsic.world.public.proto import object_world_updates_pb2
from intrinsic.world.python import object_world_ids

# Command line input flags
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
_SAMPLE_BOX_HALFSIZE_X = flags.DEFINE_float(
    'sample_box_halfsize_x',
    0.20,
    'Sampling box half-size X dimension (meters).',
)
_SAMPLE_BOX_HALFSIZE_Y = flags.DEFINE_float(
    'sample_box_halfsize_y',
    0.20,
    'Sampling box half-size Y dimension (meters).',
)
_SAMPLE_BOX_HALFSIZE_Z = flags.DEFINE_float(
    'sample_box_halfsize_z',
    0.20,
    'Sampling box half-size Z dimension (meters).',
)
_RAND_ANGLE = flags.DEFINE_float(
    'rand_angle', 10.0, 'Randomization angle in degrees.'
)
_RAND_ROLL_ANGLE = flags.DEFINE_float(
    'rand_roll_angle', 45.0, 'Randomization roll angle in degrees.'
)
_NUM_SAMPLES = flags.DEFINE_integer(
    'num_samples', 25, 'Number of calibration samples.'
)
_RUN_PRE_CALIBRATION = flags.DEFINE_bool(
    'run_pre_calibration', False, 'Whether to run pre-calibration.'
)
_MANUAL_WAYPOINTS = flags.DEFINE_bool(
    'manual_waypoints',
    True,
    'If True, collect waypoints manually instead of sampling automatically.',
)
_EXPORT_WAYPOINTS_FILE = flags.DEFINE_string(
    'export_waypoints_file',
    '',
    'If set, export the manual waypoints to this local file path.',
)
_IMPORT_WAYPOINTS_FILE = flags.DEFINE_string(
    'import_waypoints_file',
    '',
    'If set, import manual waypoints from this local file path instead of'
    ' collecting them interactively.',
)
_ICON_PORT = flags.DEFINE_integer(
    'icon_port', 17080, 'Local port mapped to ICON service via port-forward.'
)


def get_key() -> str:
  """Reads a single character or escape sequence from standard input."""
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    if ch == '\x1b':
      # It's an escape sequence
      ch += sys.stdin.read(2)
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch


def read_input(prompt: str, choices: list[str]) -> str:
  """Reads characters from stdin until they uniquely match one of the choices."""
  sys.stdout.write(prompt)
  sys.stdout.flush()
  current_input = ''
  while True:
    ch = get_key()
    if ch in ('\x03', '\x04', ''):  # Ctrl+C, Ctrl+D, EOF
      raise KeyboardInterrupt
    if ch in ('\r', '\n'):
      if current_input in choices:
        sys.stdout.write('\n')
        sys.stdout.flush()
        return current_input
      continue
    # Handle backspace
    if ch in ('\x7f', '\x08'):
      if current_input:
        current_input = current_input[:-1]
        sys.stdout.write('\b \b')
        sys.stdout.flush()
      continue
    # Ignore escape sequences
    if len(ch) > 1:
      continue

    ch = ch.lower()
    next_input = current_input + ch
    matching_choices = [c for c in choices if c.startswith(next_input)]
    if not matching_choices:
      continue
    current_input = next_input
    sys.stdout.write(ch)
    sys.stdout.flush()
    if len(matching_choices) == 1 and matching_choices[0] == current_input:
      sys.stdout.write('\n')
      sys.stdout.flush()
      return current_input


def log_task(message: str) -> bt.Task:
  # Replace spaces/special chars for a safe node name
  safe_name = ''.join([c if c.isalnum() else '_' for c in message]).lower()
  # These messages are printed in the executive container's logs (visible with
  # k9s). For printing them locally, we'd need to introduce a polling mechanism
  # to check the state of each skill node in the behavior tree.
  return bt.Task(
      action=bt.PythonScript(function_body=f'print("{message}")'),
      name=f'log_{safe_name}',
  )


def run_manual_waypoint_loop(
    world,
    robot_ref,
    manual_waypoints,
    session,
    ndof: int | None,
    part_name: str | None,
    icon_client,
) -> None:
  """Runs the interactive loop to record waypoints or jog the robot."""
  print(
      'To record waypoints: jog the robot to a pose (either via Flowstate'
      " jogging panel, or by entering 'j' below to jog via terminal), and"
      " press 'r' to record the waypoint. Press 's' to stop recording"
      " waypoints. Close the Flowstate jogging panel before pressing 's'."
  )
  action_id_counter = 0
  while True:
    user_input = read_input(
        "\nEnter 'r' to record, 'j' to enter jogging mode, 's' to stop: ",
        ['r', 'j', 's'],
    )
    if user_input == 'r':
      try:
        kinematic_robot = world.get_kinematic_object(robot_ref.proto)
        joints = kinematic_robot.joint_positions
        waypoint = geometric_constraints_pb2.GeometricConstraint(
            joint_position=joint_space_pb2.JointVec(joints=joints)
        )
        manual_waypoints.append(waypoint)
        print(f'Recorded waypoint #{len(manual_waypoints)}: {joints}')
      except Exception as e:
        print(f'Error reading robot pose: {e}')
    elif user_input == 's':
      confirm = read_input(
          '\nConfirm robot poses are collected [y/n]: ', ['y', 'n']
      )
      if confirm == 'y':
        flowstate_confirmed = False
        while not flowstate_confirmed:
          confirm_jogging = read_input(
              '\nConfirm that, if you are using Flowstate, you have closed'
              " the robot's jogging panel [y/n]: ",
              ['y', 'n'],
          )
          if confirm_jogging == 'y':
            flowstate_confirmed = True
          elif confirm_jogging == 'n':
            pass  # loops and asks again

        if len(manual_waypoints) < 3:
          print(
              'Warning: Calibration typically requires at least 3 waypoints.'
              f' You have only recorded {len(manual_waypoints)}.'
          )
        break
      elif confirm == 'n':
        print('Continuing waypoint collection...')
    elif user_input == 'j':
      if not session or not ndof or not part_name or not icon_client:
        print(
            'Error: Jogging is unavailable because ICON initialization failed.'
        )
        continue

      while True:
        try:
          print('-' * 40)
          choices = [str(i) for i in range(ndof)] + ['q']
          joint_input = read_input(
              'Enter joint index to rotate (0 to'
              f" {ndof-1}) or 'q' to quit jogging: ",
              choices,
          )
          if joint_input == 'q':
            print('Exiting jogging mode...')
            break

          joint_idx = int(joint_input)

          print(f'\n--- Live control for joint {joint_idx} ---')
          print(
              'Use Left/Right arrow keys to jog by 0.01 rad. Press'
              " 'x' to return."
          )

          while True:
            key = get_key()
            if key.lower() == 'x' or key == '\x03':
              print('\nExiting live control for this joint.\n')
              break

            delta_value = 0.0
            if key == '\x1b[C':  # Right arrow
              delta_value = 0.04
            elif key == '\x1b[D':  # Left arrow
              delta_value = -0.04
            else:
              continue  # Ignore other keys

            # Fetch current status to perform relative rotation
            status = icon_client.get_status()
            part_status = status.part_status[part_name]

            # Extract current positions
            current_positions = [
                j.position_sensed for j in part_status.joint_states
            ]

            if not current_positions or len(current_positions) != ndof:
              sys.stdout.write(
                  '\rError: Could not retrieve current joint positions'
                  ' from the robot.'
              )
              sys.stdout.flush()
              continue

            # Calculate goal positions
            goal_position = list(current_positions)
            goal_position[joint_idx] += delta_value
            goal_velocity = [0.0] * ndof

            # Send move action
            action = session.add_action(
                create_action_utils.create_point_to_point_move_action(
                    action_id=action_id_counter,
                    joint_position_part_name=part_name,
                    goal_position=goal_position,
                    goal_velocity=goal_velocity,
                )
            )
            action_id_counter += 1

            session.start_action(action_id=action.id)
            sys.stdout.write(
                f'\rJogged joint {joint_idx} by {delta_value:+.2f} rad.'
                f' Current: {goal_position[joint_idx]:.3f}   '
            )
            sys.stdout.flush()

        except ValueError:
          print('Error: Invalid input. Please enter numbers.')
        except icon_errors.Session.ActionError as e:
          print(f'Action error: {e}')
        except grpc.RpcError as e:
          print(f'\nICON RPC error during jogging: {e}')
          print('Exiting jogging mode...\n')
          break
        except Exception as e:
          print(f'\nUnexpected error during jogging: {e}')
          print('Exiting jogging mode...\n')
          break
    else:
      print("Invalid input. Please enter 'r', 'j', or 's'.")


def main(argv) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  solution = deployments.connect(address='localhost:17080')

  executive = solution.executive
  skills = solution.skills
  world = solution.world

  # Find references to resources and world objects
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

  # Find pose estimator
  pose_estimator = solution.pose_estimators[_POSE_ESTIMATOR.value]
  if not pose_estimator:
    raise ValueError(
        f"Pose estimator '{_POSE_ESTIMATOR.value}' not found in the solution."
    )

  # Determine calibration type
  if _MOVING_CAMERA.value:
    calibration_type = (
        calibration_type_pb2.CAMERA_TO_ROBOT_CALIBRATION_TYPE_MOVING_CAMERA
    )
  else:
    calibration_type = (
        calibration_type_pb2.CAMERA_TO_ROBOT_CALIBRATION_TYPE_STATIONARY_CAMERA
    )

  # Collect manual waypoints if selected
  manual_waypoints = []
  imported_waypoints = False
  if _MANUAL_WAYPOINTS.value:
    if _IMPORT_WAYPOINTS_FILE.value:
      try:
        import os

        import_path = _IMPORT_WAYPOINTS_FILE.value
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
        manual_waypoints = list(result_proto.sample_calibration_poses_result)
        imported_waypoints = True
        print(
            f'Successfully imported {len(manual_waypoints)} waypoints from'
            f' {import_path}'
        )
      except Exception as e:
        raise ValueError(f'Failed to import waypoints from {import_path}: {e}')
    else:
      print('\n=== Manual Waypoint Collection ===')
      # Initialize ICON client
      icon_client = None
      part_name = None
      ndof = None
      session_context = None

      try:
        print(f'\nConnecting to ICON on localhost:{_ICON_PORT.value}...')
        icon_client = icon_api.Client.connect_with_params(
            connection.ConnectionParams(f'localhost:{_ICON_PORT.value}', 'icon')
        )

        parts = icon_client.list_parts()
        if not parts:
          raise ValueError('No parts found on the ICON server.')

        # Find correct part name matching the robot resource name
        for part in parts:
          if part == _ROBOT.value:
            part_name = part
            break
        if part_name is None:
          part_name = parts[1] if len(parts) > 1 else parts[0]
          print(
              f"Warning: Could not find part matching '{_ROBOT.value}'."
              f" Using fallback part '{part_name}'."
          )

        # Get part config to know the number of DoFs
        part_configs = icon_client.get_config().part_configs
        for config in part_configs:
          if config.name == part_name:
            if config.HasField('generic_config'):
              ndof = config.generic_config.joint_position_config.num_joints
              print(f"Part '{part_name}' has {ndof} DoFs.")
            break
        if ndof is None:
          raise ValueError(
              f"Could not retrieve configuration for part '{part_name}'."
          )

        print(
            f"Connected! Controlling part '{part_name}' with {ndof} joints.\n"
        )
        session_context = icon_client.start_session([part_name])
      except Exception as e:
        print(
            f"Warning: Failed to initialize ICON client: {e}. Jogging ('j')"
            ' will be unavailable.'
        )

      if session_context:
        with session_context as session:
          run_manual_waypoint_loop(
              world,
              robot_ref,
              manual_waypoints,
              session,
              ndof,
              part_name,
              icon_client,
          )
      else:
        run_manual_waypoint_loop(
            world,
            robot_ref,
            manual_waypoints,
            None,
            None,
            None,
            None,
        )

    print(
        'Completed manual waypoint collection. Total waypoints:'
        f' {len(manual_waypoints)}'
    )
    print('==================================\n')

    if manual_waypoints and not imported_waypoints:
      if _EXPORT_WAYPOINTS_FILE.value:
        export_path = _EXPORT_WAYPOINTS_FILE.value
      else:
        export_choice = read_input(
            '\nDo you want to export the manual waypoints to a file? [y/n]: ',
            ['y', 'n'],
        )
        if export_choice == 'y':
          default_filename = f'waypoints_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.pbtxt'
          export_path = input(
              f'Enter file path to export [default: {default_filename}]: '
          ).strip()
          if not export_path:
            export_path = default_filename
        else:
          export_path = None

      if export_path:
        try:
          import os

          workspace_dir = os.environ.get('BUILD_WORKSPACE_DIRECTORY')
          if workspace_dir and not os.path.isabs(export_path):
            export_path = os.path.normpath(
                os.path.join(workspace_dir, export_path)
            )

          result_proto = (
              sample_calibration_poses_pb2.SampleCalibrationPosesResult()
          )
          result_proto.sample_calibration_poses_result.extend(manual_waypoints)
          with open(export_path, 'w') as f:
            f.write(text_format.MessageToString(result_proto))
          print(f'Successfully exported manual waypoints to {export_path}')
        except Exception as e:
          print(f'Failed to export manual waypoints: {e}')

  # Initialize calibration skills
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
  sample_calibration_poses_skill = skills.ai.intrinsic.sample_calibration_poses
  collect_calibration_data_skill = skills.ai.intrinsic.collect_calibration_data
  calibrate_camera_to_robot_skill = (
      skills.ai.intrinsic.calibrate_camera_to_robot
  )

  calibration_sequence = []

  # 1. Precalibration step
  if _RUN_PRE_CALIBRATION.value:
    print('Setting up Pre-calibration subtree...')
    pcp = sample_calibration_poses_skill.intrinsic_proto.skills.PreCalibrationParams(
        max_distance=0.02,
        max_angle_degrees=1.0,
    )
    sample_precalibration_poses = sample_calibration_poses_skill(
        calibration_type=calibration_type,
        calibration_object=calibration_object_ref,
        camera=camera_ref,
        robot=robot_ref,
        minimum_margin=0.001,
        pre_calibration_params=pcp,
    )
    collect_precalibration_data = collect_calibration_data_skill(
        calibration_type=calibration_type,
        calibration_object=calibration_object_ref,
        waypoints=sample_precalibration_poses.result.sample_calibration_poses_result,
        disable_collision_checking=True,
        robot=robot_ref,
        motion_type=collect_calibration_data_skill.intrinsic_proto.skills.MotionType.MOTION_TYPE_JOINT,
        skip_return_to_base_between_waypoints=True,
    )
    collect_precalibration_data.execute_timeout = datetime.timedelta(
        seconds=600
    )

    precalibrate = calibrate_camera_to_robot_skill(
        calibration_type=calibration_type,
        translation_root_mean_square_error_threshold=0.03,
        rotation_root_mean_square_error_threshold=1.5,
    )

    pre_calibration = bt.SubTree(
        name='Precalibration',
        behavior_tree=bt.Sequence(
            children=[
                log_task('Starting pre-calibration...'),
                sample_precalibration_poses,
                log_task('Initializing pre-calibration session...'),
                initialize_calibration,
                log_task('Collecting pre-calibration data...'),
                collect_precalibration_data,
                log_task('Running pre-calibration solver...'),
                precalibrate,
                log_task('Pre-calibration completed successfully.'),
            ]
        ),
    )
    calibration_sequence.append(pre_calibration)

  # 2. Main Calibration step
  print('Setting up Main Calibration subtree...')
  if _MANUAL_WAYPOINTS.value:
    collect_calibration_data = collect_calibration_data_skill(
        calibration_type=calibration_type,
        calibration_object=calibration_object_ref,
        waypoints=manual_waypoints,
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
        log_task('Collecting manual calibration data...'),
        collect_calibration_data,
        log_task('Running main calibration solver...'),
        calibrate,
        log_task('Main calibration completed successfully.'),
    ]
  else:
    rbp = sample_calibration_poses_skill.intrinsic_proto.skills.RandomizedBoxParams(
        num_samples=_NUM_SAMPLES.value,
        sample_box_halfsize=skills_pb2.VectorNdValue(
            value=[
                _SAMPLE_BOX_HALFSIZE_X.value,
                _SAMPLE_BOX_HALFSIZE_Y.value,
                _SAMPLE_BOX_HALFSIZE_Z.value,
            ]
        ),
        rotation_randomization_angle_degrees=float(_RAND_ANGLE.value),
        rotation_randomization_roll_angle_degrees=float(_RAND_ROLL_ANGLE.value),
    )

    sample_calibration_poses = sample_calibration_poses_skill(
        calibration_type=calibration_type,
        calibration_object=calibration_object_ref,
        camera=camera_ref,
        robot=robot_ref,
        randomized_box_params=rbp,
    )

    collect_calibration_data = collect_calibration_data_skill(
        calibration_type=calibration_type,
        calibration_object=calibration_object_ref,
        waypoints=sample_calibration_poses.result.sample_calibration_poses_result,
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
        log_task('Sampling calibration poses...'),
        sample_calibration_poses,
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
  calibration_sequence.append(calibration)

  # Execute sequence
  calibration_bt = bt.Sequence(children=calibration_sequence)

  print('Running behavior tree sequence via executive...')
  try:
    executive.run(calibration_bt)
    print('Execution completed successfully.')
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
      update_request = object_world_updates_pb2.UpdateTransformRequest(
          node_a=scene_object.parent.transform_node_reference,
          node_b=scene_object.transform_node_reference,
          a_t_b=proto_conversion.pose_to_proto(scene_object.parent_t_this),
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
